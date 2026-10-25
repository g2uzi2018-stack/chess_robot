#!/usr/bin/env python3
"""Real D405 / ROS smoke test. Owns only its child vision_node process."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from chess_robot_vision.msg import PieceCandidates

output = Path(sys.argv[1]).resolve()
output.mkdir(parents=True, exist_ok=True)
rclpy.init()
node = rclpy.create_node('vision_smoke_observer')
seen = {k: set() for k in ('color', 'depth', 'info', 'debug', 'candidates')}
counts = {k: 0 for k in seen}
errors = []
valid_depth_pixels = []
depth_frames = {}
candidate_frames = {}
ages = []
frame_numbers = []
valid_candidates = 0
invalid_candidates = 0
depth_comparisons = 0

def stamp(msg):
    return msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec

def callback(kind, msg):
    global valid_candidates, invalid_candidates
    counts[kind] += 1
    key = stamp(msg)
    seen[kind].add(key)
    if msg.header.frame_id != 'd405_color_optical_frame':
        errors.append('wrong frame_id')
    if kind == 'color':
        if (msg.width, msg.height, msg.encoding) != (640, 480, 'bgr8'):
            errors.append('wrong RGB format')
    elif kind == 'depth':
        if (msg.width, msg.height, msg.encoding) != (640, 480, '32FC1'):
            errors.append('wrong depth format')
        else:
            arr = np.ndarray((msg.height, msg.width), dtype='<f4', buffer=msg.data,
                             strides=(msg.step, 4)).copy()
            valid_depth_pixels.append(float(np.isfinite(arr).mean()))
            depth_frames[key] = arr
    elif kind == 'info':
        if not (msg.k[0] > 0 and msg.k[4] > 0 and msg.width == 640 and msg.height == 480):
            errors.append('bad calibration')
    elif kind == 'debug':
        if not bytes(msg.data).startswith(b'\xff\xd8'):
            errors.append('bad JPEG')
    else:
        if not msg.stream_ok:
            errors.append('unexpected invalid stream: ' + msg.diagnostic)
        frame_numbers.append(msg.frame_number)
        ages.append((node.get_clock().now().nanoseconds - key) / 1e9)
        candidate_frames[key] = msg
        for p in msg.candidates:
            if p.label != 'unknown_circle':
                errors.append('unsupported semantic claim')
            xyz = np.array([p.surface_point.x, p.surface_point.y, p.surface_point.z])
            if p.depth_valid:
                valid_candidates += 1
                if not np.isfinite(xyz).all() or not 0.07 <= xyz[2] <= 0.5:
                    errors.append('bad valid 3D point')
            else:
                invalid_candidates += 1
                if not np.isnan(xyz).all():
                    errors.append('invalid point must be NaN')

subscriptions = []
for kind, cls, topic in [
    ('color', Image, '/vision/color/image_rect'),
    ('depth', Image, '/vision/aligned_depth/image_rect'),
    ('info', CameraInfo, '/vision/color/camera_info'),
    ('debug', CompressedImage, '/vision/debug/image/compressed'),
    ('candidates', PieceCandidates, '/vision/candidates')]:
    subscriptions.append(node.create_subscription(cls, topic,
        lambda msg, kind=kind: callback(kind, msg), qos_profile_sensor_data))

log = (output / 'node.log').open('w')
proc = subprocess.Popen(['ros2', 'run', 'chess_robot_vision', 'vision_node', '--ros-args',
    '-p', 'max_frames:=180', '-p', f'evidence_dir:={output}'],
    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
started = time.monotonic()
try:
    while time.monotonic()-started < 60 and proc.poll() is None:
        rclpy.spin_once(node, timeout_sec=0.05)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.02)
    if proc.poll() is None:
        errors.append('node exceeded 60 second timeout')
finally:
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
    log.close()

for key in depth_frames.keys() & candidate_frames.keys():
    arr = depth_frames[key]
    for p in candidate_frames[key].candidates:
        if not p.depth_valid:
            continue
        radius = max(2, min(12, int(p.radius_px*0.25)))
        # Reconstruct central-disk robust depth independently from published metre image.
        cx, cy = int(np.rint(p.pixel_u)), int(np.rint(p.pixel_v))
        values = [arr[y, x] for y in range(max(0, cy-radius), min(arr.shape[0], cy+radius+1))
                  for x in range(max(0, cx-radius), min(arr.shape[1], cx+radius+1))
                  if (x-cx)**2+(y-cy)**2 <= radius**2 and np.isfinite(arr[y, x]) and 0.07 <= arr[y, x] <= 0.5]
        if values:
            depth_comparisons += 1
            if abs(float(np.median(values))-p.surface_point.z) > 2e-6:
                errors.append('candidate Z disagrees with aligned metric depth')

if proc.returncode != 0:
    errors.append(f'node exit {proc.returncode}')
for name in ('color.png', 'depth_z16.png', 'candidates.jpg', 'capture.json'):
    if not (output / name).is_file() or (output / name).stat().st_size == 0:
        errors.append(f'missing capture artifact: {name}')
for key, count in counts.items():
    if count < 20:
        errors.append(f'too few {key} messages: {count}')
common = set.intersection(*seen.values())
if len(common) < 20:
    errors.append('too few synchronized RGB-D/info/result frames')
if not valid_depth_pixels or max(valid_depth_pixels) <= 0:
    errors.append('all depth pixels missing')
if any(b <= a for a, b in zip(frame_numbers, frame_numbers[1:])):
    errors.append('nonmonotonic hardware frame number')
if ages and (min(ages) < -0.01 or float(np.quantile(ages, .99)) > 0.5):
    errors.append('candidate receipt age outside freshness budget')
summary = dict(passed=not errors, errors=sorted(set(errors)), message_counts=counts,
    synchronized_frames=len(common), valid_candidate_observations=valid_candidates,
    invalid_candidate_observations=invalid_candidates, candidate_z_comparisons=depth_comparisons,
    median_finite_depth_fraction=float(np.median(valid_depth_pixels)) if valid_depth_pixels else None,
    candidate_receive_age_s_p50=float(np.median(ages)) if ages else None,
    candidate_receive_age_s_p99=float(np.quantile(ages, .99)) if ages else None,
    elapsed_s=time.monotonic()-started, node_exit=proc.returncode)
(output / 'smoke.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
print(json.dumps(summary, ensure_ascii=False, indent=2))
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if summary['passed'] else 1)
