"""Independent RGB-D measurement node. No arm messages, audit or task state."""
import hashlib
import json
import os
import signal
import threading
import time
import uuid
from collections import Counter, deque
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from chess_robot_vision.msg import FrameInfo

from benchmark_lease import BenchmarkLease
from measurement_contract import CAMERA_FRAME, ContractError, RoiHint, camera_measurement, decode_json, encode
from shared_pipeline import Pipeline
from spin_guard import SpinGuard

MEASUREMENTS_TOPIC = '/perception/xiangqi/measurements'
ROI_TOPIC = '/perception/xiangqi/roi_hint'


def stamp(message):
    return message.header.stamp.sec*10**9+message.header.stamp.nanosec


def image_qualified(image, depth, info, frame):
    return bool(frame.stream_ok and image.encoding == 'bgr8' and depth.encoding == '32FC1'
                and image.width == depth.width == info.width == 1280
                and image.height == depth.height == info.height == 720 and not any(info.d)
                and all(m.header.frame_id == CAMERA_FRAME for m in (image, depth, info, frame))
                and np.isfinite(info.k).all() and info.k[0] > 0 and info.k[4] > 0
                and np.allclose(np.array(info.k).reshape(3, 3), np.array(info.p).reshape(3, 4)[:, :3]))


def main():
    depth_config_path=None
    if os.environ.get('XIANGQI_CANDIDATE_MODE','yolo') == 'depth_shadow':
        if not os.environ.get('XIANGQI_DEPTH_CONFIG'):
            raise ValueError('depth_shadow requires XIANGQI_DEPTH_CONFIG')
        depth_config_path=Path(os.environ['XIANGQI_DEPTH_CONFIG']).resolve()
    out = Path(os.environ.get('XIANGQI_RUN_DIR', str(Path(__file__).resolve().parent)))
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('xiangqi_perception')
    bridge = CvBridge()
    pipeline = Pipeline(camera_only=True)
    lease = BenchmarkLease()
    lock = threading.RLock()
    cache = {k: {} for k in ('color', 'depth', 'info', 'frame')}
    hints = deque(maxlen=1)
    source_epoch = uuid.uuid4().hex
    hint = RoiHint()
    published = deque(maxlen=64)
    last_rgbd = None
    last = 0
    sequence = detection_count = count = 0
    rejects = Counter()
    running = True
    config_check = written = 0
    initial_identity = None
    config_changed = False
    begin = time.monotonic()
    qos = QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT)
    publisher = node.create_publisher(String, MEASUREMENTS_TOPIC, qos)
    image_pub = node.create_publisher(Image, '/pose/xiangqi/slow_debug_image', qos_profile_sensor_data)

    def receive(kind, message):
        with lock:
            cache[kind][stamp(message)] = message
            while len(cache[kind]) > 8:
                del cache[kind][min(cache[kind])]

    def receive_hint(message):
        try:
            data = decode_json(message.data, max_bytes=4096)
        except ContractError:
            with lock:
                rejects['invalid ROI JSON'] += 1
            return
        with lock:
            hints.append(data)

    subscriptions = [node.create_subscription(cls, topic, lambda m, k=k: receive(k, m), qos_profile_sensor_data)
                     for k, cls, topic in [('color', Image, '/vision/color/image_rect'),
                                           ('depth', Image, '/vision/aligned_depth/image_rect'),
                                           ('info', CameraInfo, '/vision/color/camera_info'),
                                           ('frame', FrameInfo, '/vision/frame_info')]]
    subscriptions.append(node.create_subscription(String, ROI_TOPIC, receive_hint, 1))

    def stop(*args):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    spin = SpinGuard(lambda: rclpy.spin(node))

    def publish(record, metadata):
        nonlocal sequence, count, written, config_changed
        now = node.get_clock().now().nanoseconds
        if depth_config_path is not None and record.get('detection_valid'):
            try:
                loaded_sha=(record.get('identity') or {}).get('depth_config_sha256')
                # Check the worker's loaded bytes as well as monitor history:
                # a file change before the parent's first poll is still a
                # configuration fault and cannot become its new baseline.
                config_changed = config_changed or loaded_sha != hashlib.sha256(depth_config_path.read_bytes()).hexdigest()
            except OSError:
                config_changed=True
        try:
            packet = camera_measurement(record, metadata, source_epoch, sequence, now, not config_changed)
            wire = encode(packet)
        except (ContractError, ValueError, TypeError, OverflowError, RecursionError) as error:
            rejects['camera payload rejected'] += 1
            if rejects['camera payload rejected'] == 1:
                print(json.dumps(dict(node='xiangqi_perception', rejected='camera payload', reason=str(error)[:256])), flush=True)
            return False
        publisher.publish(String(data=wire))
        published.append((sequence, packet['stamp_ns']))
        sequence += 1
        count += 1
        if time.monotonic()-written > 3:
            packet.update(published_frames=count, running_seconds=time.monotonic()-begin,
                          perception_rejection_counts=dict(rejects))
            tmp = out/'perception-latest.json.tmp'
            tmp.write_text(json.dumps(packet, indent=2, allow_nan=False))
            tmp.replace(out/'perception-latest.json')
            written = time.monotonic()
            print(json.dumps(dict(node='xiangqi_perception', frames=count, source_epoch=source_epoch,
                                  configuration_valid=not config_changed, rejects=dict(rejects))), flush=True)
        return True

    try:
        while running and rclpy.ok():
            spin.ensure_alive()
            now = node.get_clock().now().nanoseconds
            if time.monotonic()-config_check > 1:
                from recognize_tracked import WEIGHTS, MODEL_ARTIFACT
                paths = [WEIGHTS, MODEL_ARTIFACT, Path(__file__), Path(__file__).with_name('recognize_tracked.py'),
                         Path(__file__).with_name('shared_pipeline.py'), Path(__file__).with_name('measurement_contract.py'),
                         Path(__file__).with_name('candidate_roi.py')]
                if depth_config_path is not None:
                    paths.append(Path(__file__).with_name('depth_candidates.py'))
                    paths.append(depth_config_path)
                try:
                    current = tuple((str(p), p.stat().st_mtime_ns, p.stat().st_size) for p in paths)
                    if depth_config_path is not None:
                        current += (('depth_config_sha256',hashlib.sha256(paths[-1].read_bytes()).hexdigest()),)
                except OSError:
                    current = ('configuration file missing',)
                if initial_identity is None:
                    initial_identity = current
                config_changed = config_changed or current != initial_identity
                config_check = time.monotonic()
            with lock:
                latest_hint = hints.pop() if hints else None
                hints.clear()
            if latest_hint is not None:
                hint.receive(latest_hint, now, source_epoch, published)
            for job, record, overlay in pipeline.poll():
                context = job['context']
                if context['source_epoch'] != source_epoch:
                    rejects['in-flight result from retired camera epoch'] += 1
                    continue
                now = node.get_clock().now().nanoseconds
                if not 0 <= (now-record['stamp_ns'])*1e-9 <= .5:
                    rejects['image observation expired during pipeline'] += 1
                    record.update(detection_valid=False, camera_geometry_valid=False, detections=[])
                    overlay = None
                record['live_stage_ms'] = dict(image_decode=context['decode_ms'], shared_memory_copy=context['shared_copy_ms'],
                                               image_age_at_processing_start=context['image_age_ms'],
                                               analysis=record['processing_s']*1000, gpu_stage=record['detector_ms'],
                                               geometry_stage=record['geometry_ms'], tracking=0.,
                                               frame_pipeline_elapsed=(time.monotonic()-job['started'])*1000,
                                               debug_image_generated=job['draw'])
                publish(record, job['metadata'])
                if overlay is not None:
                    message = bridge.cv2_to_imgmsg(overlay, 'bgr8')
                    t = record['stamp_ns']
                    message.header.stamp.sec, message.header.stamp.nanosec = t//10**9, t % 10**9
                    message.header.frame_id = CAMERA_FRAME
                    image_pub.publish(message)
            if not lease.can_submit(pipeline.busy()) or not pipeline.can_submit():
                time.sleep(.001)
                continue
            chosen = None
            with lock:
                now = node.get_clock().now().nanoseconds
                common = set.intersection(*(set(cache[k]) for k in cache))
                eligible = [t for t in common if t > last and 0 <= (now-t)*1e-9 <= .2]
                if eligible:
                    t = max(eligible)
                    last = t
                    image, depth, info, frame = (cache[k][t] for k in ('color', 'depth', 'info', 'frame'))
                    rgbd = (int(frame.color_frame_number), int(frame.depth_frame_number))
                    repeated = last_rgbd is not None and (rgbd[0] == last_rgbd[0] or rgbd[1] == last_rgbd[1])
                    if last_rgbd is not None and (rgbd[0] < last_rgbd[0] or rgbd[1] < last_rgbd[1]):
                        source_epoch = uuid.uuid4().hex
                        sequence = 0
                        hint = RoiHint()
                        published.clear()
                    last_rgbd = rgbd
                    metadata = dict(stamp_ns=t, K=[float(v) for v in info.k], base_eligible=False,
                                    rgbd_frame_key=[source_epoch, *rgbd], camera_epoch=source_epoch)
                    if image_qualified(image, depth, info, frame) and not repeated:
                        chosen = (image, depth, metadata)
                    else:
                        rejects['image/RGB-D/intrinsics qualification failed'] += 1
                        publish(dict(stamp_ns=t, detection_valid=False, camera_geometry_valid=False, detections=[],
                                     image_size=[1280, 720], reason='image/RGB-D/intrinsics qualification failed'), metadata)
            if chosen is None:
                time.sleep(.002)
                continue
            image, depth, metadata = chosen
            decode_start = time.monotonic()
            color = bridge.imgmsg_to_cv2(image, 'bgr8')
            depth_array = bridge.imgmsg_to_cv2(depth, '32FC1')
            context = dict(source_epoch=source_epoch, decode_ms=(time.monotonic()-decode_start)*1000,
                           image_age_ms=(node.get_clock().now().nanoseconds-metadata['stamp_ns'])*1e-6,
                           shared_copy_ms=0.)
            # Optional hint chooses classification context only. Every submitted
            # frame still receives its own full-image candidate scan.
            current_roi = hint.roi(node.get_clock().now().nanoseconds, source_epoch)
            copy_start = time.monotonic()
            pipeline.submit(color, depth_array, metadata, current_roi, draw=bool(image_pub.get_subscription_count()), context=context)
            context['shared_copy_ms'] = (time.monotonic()-copy_start)*1000
            detection_count += 1
    finally:
        try:
            pipeline.close()
        finally:
            lease.close()
            rclpy.shutdown()
            spin.join(timeout=2)
            node.destroy_node()


if __name__ == '__main__':
    main()
