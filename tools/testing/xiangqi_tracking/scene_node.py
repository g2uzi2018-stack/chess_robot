"""Scene/identity/observation ROS adapter. No image subscription, model or motion API."""
import json
import os
import signal
import threading
import time
import uuid
from collections import Counter, deque
from pathlib import Path

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict as convert
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from chess_robot_interfaces.msg import ArmState

from base_geometry import BaseGeometry, CALIB, URDF
from measurement_contract import ContractError, MAX_MESSAGE_BYTES, decode, decode_json, encode_json
from scene_state import SceneState
from spin_guard import SpinGuard
from stable_observation import AuditEpoch, FeedbackWindow, Policy


def stamp(message):
    return message.header.stamp.sec*10**9+message.header.stamp.nanosec


def main():
    out = Path(os.environ.get('XIANGQI_RUN_DIR', str(Path(__file__).resolve().parent)))
    out.mkdir(parents=True, exist_ok=True)
    session = uuid.uuid4().hex
    policy_path = os.environ.get('XIANGQI_OBSERVATION_POLICY')
    policy = Policy(**json.loads(Path(policy_path).read_text())) if policy_path else Policy()
    feedback = FeedbackWindow(policy)
    audit = AuditEpoch(os.environ.get('XIANGQI_FEEDBACK_AUDIT', '/home/kuang/workspace/chess_robot/.runtime/audit/feedback.bin'))
    rclpy.init()
    node = rclpy.create_node('xiangqi_scene')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    data_pub = node.create_publisher(String, '/pose/xiangqi/detections', 10)
    response_pub = node.create_publisher(String, '/pose/xiangqi/observation/response', 32)
    status_pub = node.create_publisher(String, '/pose/xiangqi/observation/status', qos)
    snapshot_pub = node.create_publisher(String, '/pose/xiangqi/observation/snapshot', qos)
    hint_pub = node.create_publisher(String, '/perception/xiangqi/roi_hint', 1)

    def save(event):
        event_wire = encode_json(event)
        with (out/'snapshot-audit.jsonl').open('a') as stream:
            stream.write(event_wire+'\n')
        if event['event'] == 'locked':
            snapshot = event['snapshot']
            snapshot_wire = encode_json(snapshot)
            (out/('snapshot-'+snapshot['snapshot_id']+'.json')).write_text(snapshot_wire)
            snapshot_pub.publish(String(data=snapshot_wire))

    from piece_tracker import PieceTracker
    state = SceneState(policy=policy, base=BaseGeometry(),
                       tracker=PieceTracker(out/('track-state-v2-'+session+'.json')), save=save, session=session)
    lock = threading.RLock()
    cache = {k: {} for k in ('arm', 'joint')}
    pairs = deque(maxlen=512)
    measurements = deque(maxlen=4)
    requests = deque(maxlen=32)
    input_rejections = Counter()
    running = True
    overflow = False
    measurement_overflow = False

    def receive_feedback(kind, message):
        nonlocal overflow
        with lock:
            t = stamp(message)
            cache[kind][t] = message
            while len(cache[kind]) > 120:
                del cache[kind][min(cache[kind])]
            if t in cache['arm'] and t in cache['joint']:
                if len(pairs) == pairs.maxlen:
                    overflow = True
                pairs.append((t, convert(cache['arm'][t]), convert(cache['joint'][t])))

    def receive_measurement(message):
        nonlocal measurement_overflow
        # The byte bound also prevents arbitrary nested detector payloads.
        if len(message.data) > MAX_MESSAGE_BYTES:
            with lock:
                measurement_overflow = True
            return
        with lock:
            if len(measurements) == measurements.maxlen:
                measurement_overflow = True
            measurements.append(message.data)

    def receive_request(message):
        try:
            command = decode_json(message.data, max_bytes=4096)
        except ContractError:
            with lock:
                input_rejections['invalid request JSON'] += 1
            return
        with lock:
            if isinstance(command, dict) and len(requests) < requests.maxlen:
                requests.append(command)

    subscriptions = [node.create_subscription(cls, topic, lambda m, k=k: receive_feedback(k, m), 100)
                     for k, cls, topic in [('arm', ArmState, '/arm/state'), ('joint', JointState, '/arm/joint_states')]]
    subscriptions.append(node.create_subscription(String, '/perception/xiangqi/measurements', receive_measurement,
                                                 QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT)))
    subscriptions.append(node.create_subscription(String, '/pose/xiangqi/observation/request', receive_request, 32))

    def stop(*args):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    spin = SpinGuard(lambda: rclpy.spin(node))
    last_status = last_hint = written = config_check = 0
    initial_identity = None
    stale_sent = False
    count = 0
    begin = time.monotonic()

    def publish(record):
        nonlocal count, written
        now = node.get_clock().now().nanoseconds
        # This is the final publisher; preserve acquisition-based total latency.
        record['publish_latency_s'] = (now-record['stamp_ns'])*1e-9
        try:
            wire = encode_json(record, max_bytes=2*MAX_MESSAGE_BYTES)
        except ContractError:
            input_rejections['output measurement JSON rejected'] += 1
            state.observation.invalidate('output measurement JSON rejected')
            state.observation.requested = True
            return
        data_pub.publish(String(data=wire))
        count += 1
        if time.monotonic()-written > 3:
            record.update(published_frames=count, running_seconds=time.monotonic()-begin)
            tmp = out/'live-latest.json.tmp'
            tmp.write_text(json.dumps(record, indent=2, allow_nan=False))
            tmp.replace(out/'live-latest.json')
            written = time.monotonic()
            print(json.dumps(dict(node='xiangqi_scene', frames=count, base_valid=record['base_transform_valid'],
                                  observation=record['observation'], packet_rejections=dict(state.packet_rejections))), flush=True)

    try:
        while running and rclpy.ok():
            spin.ensure_alive()
            now = node.get_clock().now().nanoseconds
            audit.poll(now)
            with lock:
                pending = list(pairs)
                pairs.clear()
                incoming = list(measurements)
                measurements.clear()
                commands = list(requests)
                requests.clear()
                dropped, measurement_dropped = overflow, measurement_overflow
                overflow = measurement_overflow = False
            if dropped:
                feedback._break('feedback queue overflow', True)
            for t, arm, joint in sorted(pending, key=lambda row: row[0]):
                if audit.controller is not None and t >= audit.epoch_start_ns:
                    feedback.ingest(t, arm, joint, now, audit.token)
            if time.monotonic()-config_check > 1:
                paths = [CALIB, URDF, Path(__file__), Path(__file__).with_name('base_geometry.py'),
                         Path(__file__).with_name('scene_state.py'), Path(__file__).with_name('stable_observation.py')]
                if policy_path:
                    paths.append(Path(policy_path))
                try:
                    current = tuple((str(p), p.stat().st_mtime_ns, p.stat().st_size) for p in paths)
                except OSError:
                    current = ('configuration file missing',)
                if initial_identity is None:
                    initial_identity = current
                state.configuration_fault = state.configuration_fault or current != initial_identity
                config_check = time.monotonic()
            raw_good = bool(audit.ok and 0 <= (now-audit.host)*1e-9 <= policy.state_age_s)
            state.tick(now, feedback, raw_good, audit.token)
            if measurement_dropped:
                state.observation.invalidate('bounded scene measurement queue overflow')
                state.observation.requested = True
            for wire in incoming:
                now = node.get_clock().now().nanoseconds
                try:
                    packet = decode(wire)
                    record = state.process(packet, now, feedback, raw_good, audit.token)
                except (ContractError, KeyError, TypeError, ValueError, OverflowError, RecursionError) as error:
                    record = state.reject_packet(str(error), now)
                if record is not None:
                    publish(record)
                stale_sent = False
            now = node.get_clock().now().nanoseconds
            state.tick(now, feedback, raw_good, audit.token)
            if not state.source_fresh(now) and not stale_sent:
                record = state.expired_record(now)
                if record is not None:
                    publish(record)
                stale_sent = True
            for command in commands:
                response = state.observation.request(command, now)
                try:
                    response_wire = encode_json(response)
                except ContractError:
                    input_rejections['response JSON rejected'] += 1
                    continue
                response_pub.publish(String(data=response_wire))
            if time.monotonic()-last_status > .1:
                status = state.observation.status(now)
                status['transport_rejections'] = dict(input_rejections)
                status_pub.publish(String(data=encode_json(status)))
                last_status = time.monotonic()
            if time.monotonic()-last_hint > .1:
                hint = state.hint(now)
                if hint is not None:
                    hint_pub.publish(String(data=encode_json(hint, max_bytes=4096)))
                last_hint = time.monotonic()
            time.sleep(.002)
    finally:
        audit.close()
        rclpy.shutdown()
        spin.join(timeout=2)
        node.destroy_node()


if __name__ == '__main__':
    main()
