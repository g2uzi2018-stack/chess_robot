#!/usr/bin/env python3
"""Validate RGB-D transport/calibration metadata without any detection/pose output."""
import argparse
import json
from pathlib import Path
import os
import signal
import subprocess
import time
import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from chess_robot_vision.msg import FrameInfo

parser=argparse.ArgumentParser()
parser.add_argument('output')
parser.add_argument('--observe-live',action='store_true')
parser.add_argument('--frames',type=int,default=180)
args=parser.parse_args()
output=Path(args.output).resolve();output.mkdir(parents=True,exist_ok=True)
rclpy.init();node=rclpy.create_node('camera_io_smoke_observer')
kinds=('color','depth','info','frame','color_jpeg','depth_jpeg')
seen={k:set() for k in kinds};counts={k:0 for k in kinds}
errors=[];ages=[];finite_fractions=[];numbers=[];scales=[];startup_diagnostics=[];first_valid_delay=None
def callback(kind,msg):
    global first_valid_delay
    counts[kind]+=1
    key=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
    seen[kind].add(key)
    if msg.header.frame_id!='d405_color_optical_frame':errors.append('wrong frame id')
    if kind=='color':
        if (msg.width,msg.height,msg.encoding)!=(640,480,'bgr8'):errors.append('bad color image format')
    elif kind=='depth':
        if (msg.width,msg.height,msg.encoding)!=(640,480,'32FC1'):errors.append('bad metric depth format')
        else:
            arr=np.ndarray((msg.height,msg.width),dtype='<f4',buffer=msg.data,strides=(msg.step,4))
            finite_fractions.append(float(np.isfinite(arr).mean()))
            if np.any(arr==0) or np.any(arr[np.isfinite(arr)]<0):errors.append('invalid metric depth encoding')
    elif kind=='info':
        if msg.k[0]<=0 or msg.k[4]<=0 or any(abs(x)>1e-9 for x in msg.d):errors.append('invalid rectified calibration')
    elif kind=='frame':
        if not msg.stream_ok:
            if not numbers:startup_diagnostics.append(msg.diagnostic)
            else:errors.append('unexpected camera error after valid frames: '+msg.diagnostic)
        else:
            if first_valid_delay is None:first_valid_delay=time.monotonic()-started
            numbers.append((msg.color_frame_number,msg.depth_frame_number))
            if not msg.timestamp_domain or not np.isfinite(msg.device_timestamp_ms):errors.append('missing SDK timestamp')
            if not 0<msg.depth_scale_m<0.01:errors.append('invalid SDK depth scale')
            scales.append(msg.depth_scale_m)
            ages.append((node.get_clock().now().nanoseconds-key)/1e9)
    else:
        image=cv2.imdecode(np.frombuffer(msg.data,dtype=np.uint8),cv2.IMREAD_COLOR)
        if image is None or image.shape!=(480,640,3):errors.append('bad preview JPEG')

subscriptions=[]
for kind,cls,topic in [
    ('color',Image,'/vision/color/image_rect'),
    ('depth',Image,'/vision/aligned_depth/image_rect'),
    ('info',CameraInfo,'/vision/color/camera_info'),
    ('frame',FrameInfo,'/vision/frame_info'),
    ('color_jpeg',CompressedImage,'/vision/color/image_rect/compressed'),
    ('depth_jpeg',CompressedImage,'/vision/aligned_depth/preview/compressed')]:
    subscriptions.append(node.create_subscription(cls,topic,lambda m,k=kind:callback(k,m),qos_profile_sensor_data))

proc=None;log=None;started=time.monotonic()
try:
    if not args.observe_live:
        log=(output/'node.log').open('w')
        proc=subprocess.Popen(['ros2','run','chess_robot_vision','vision_node','--ros-args',
            '-p',f'max_frames:={args.frames}','-p',f'evidence_dir:={output}'],
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    while time.monotonic()-started<45:
        rclpy.spin_once(node,timeout_sec=.02)
        if args.observe_live and len(numbers)>=args.frames:break
        if proc is not None and proc.poll() is not None:break
    else:errors.append('camera test timeout')
    for _ in range(20):rclpy.spin_once(node,timeout_sec=.01)
    obsolete={topic:len(node.get_publishers_info_by_topic(topic)) for topic in
              ('/vision/candidates','/vision/debug/image/compressed')}
    if any(obsolete.values()):errors.append('obsolete algorithm topic still has a publisher')
finally:
    if proc is not None:
        if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=6)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        if proc.returncode!=0:errors.append('camera node failed: '+str(proc.returncode))
        log.close()
    node.destroy_node();rclpy.shutdown()

for kind,count in counts.items():
    if count<30:errors.append('too few '+kind+' messages')
common=set.intersection(*seen.values())
if len(common)<30:errors.append('too few complete synchronized RGB-D/calibration/metadata/JPEG groups')
if any(b[0]<=a[0] or b[1]<=a[1] for a,b in zip(numbers,numbers[1:])):errors.append('nonmonotonic frame numbers')
if not finite_fractions or max(finite_fractions)==0:errors.append('no finite depth pixels')
if first_valid_delay is None or first_valid_delay>5:errors.append('first valid frame did not arrive within 5 seconds')
if ages and (min(ages)<-.01 or np.quantile(ages,.99)>.5):errors.append('frame info outside receipt freshness budget')
if not args.observe_live:
    for name in ('color.png','depth_z16.png','capture.json'):
        if not (output/name).is_file():errors.append('missing capture '+name)
    if (output/'candidates.jpg').exists():errors.append('obsolete detection artifact generated')
summary=dict(passed=not errors,errors=sorted(set(errors)),observe_live=args.observe_live,message_counts=counts,
    synchronized_groups=len(common),obsolete_topic_publishers=obsolete,
    sdk_depth_scale_m=scales[-1] if scales else None,
    startup_diagnostics=startup_diagnostics,first_valid_frame_delay_s=first_valid_delay,
    median_finite_depth_fraction=float(np.median(finite_fractions)) if finite_fractions else None,
    frame_receive_age_s_p99=float(np.quantile(ages,.99)) if ages else None,elapsed_s=time.monotonic()-started)
(output/'camera-io-smoke.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(summary,ensure_ascii=False,indent=2))
raise SystemExit(0 if summary['passed'] else 1)
