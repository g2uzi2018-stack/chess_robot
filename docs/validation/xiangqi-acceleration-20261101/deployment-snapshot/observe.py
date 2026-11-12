import argparse
import collections
import json
from pathlib import Path
import threading
import time
import rclpy
from rclpy.qos import DurabilityPolicy,QoSProfile,qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from rosidl_runtime_py.convert import message_to_ordereddict as convert
from measure_lib import Observer,environment,ROOT

p=argparse.ArgumentParser();p.add_argument('out');p.add_argument('--seconds',type=int,default=30);p.add_argument('--capture',action='store_true');args=p.parse_args()
out=Path(args.out);out.mkdir(parents=True,exist_ok=False)
rclpy.init();o=Observer(out);statuses=[];snapshots=[]
def receive(k,m):
    data=json.loads(m.data)
    with o.lock:(statuses if k=='status' else snapshots).append((time.monotonic(),data))
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
subs=[o.node.create_subscription(String,'/pose/xiangqi/observation/'+k,lambda m,k=k:receive(k,m),qos) for k in ['status','snapshot']]
try:
    (out/'environment.json').write_text(json.dumps(environment(),indent=2))
    if args.capture:
        cache={k:{} for k in ['color','depth','info']}
        def raw(k,m):
            stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
            with o.lock:
                cache[k][stamp]=m
                while len(cache[k])>5:cache[k].pop(min(cache[k]))
        raw_subs=[o.node.create_subscription(t,topic,lambda m,k=k:raw(k,m),qos_profile_sensor_data) for k,t,topic in [('color',Image,'/vision/color/image_rect'),('depth',Image,'/vision/aligned_depth/image_rect'),('info',CameraInfo,'/vision/color/camera_info')]]
        deadline=time.monotonic()+8;chosen=None
        while time.monotonic()<deadline:
            time.sleep(.05)
            with o.lock:
                common=set(cache['color'])&set(cache['depth'])&set(cache['info'])
                if common and o.arms and o.joints:
                    stamp=max(common);arm_by_stamp={m.header.stamp.sec*10**9+m.header.stamp.nanosec:m for _,m in o.arms};joint_by_stamp={m.header.stamp.sec*10**9+m.header.stamp.nanosec:m for _,m in o.joints};pairs=set(arm_by_stamp)&set(joint_by_stamp)
                    if pairs:
                        a=min(pairs,key=lambda t:abs(t-stamp))
                        if abs(a-stamp)<25_000_000:chosen=(stamp,cache['color'][stamp],cache['depth'][stamp],cache['info'][stamp],arm_by_stamp[a],joint_by_stamp[a]);break
        for sub in raw_subs:o.node.destroy_subscription(sub)
        assert chosen,'No matching RGB-D/metadata capture'
        captured_at_ns=time.time_ns()
        import cv2,numpy as np
        stamp,color,depth,ci,arm,joint=chosen;capture=out/'capture';capture.mkdir()
        cv2.imwrite(str(capture/'color.png'),np.frombuffer(color.data,np.uint8).reshape(color.height,color.width,3))
        np.save(capture/'depth_m.npy',np.frombuffer(depth.data,np.float32).reshape(depth.height,depth.width))
        metadata=dict(stamp_ns=stamp,K=list(ci.k),arm=convert(arm),joint=convert(joint),base_eligible=True,joint_age_s=max(0,(captured_at_ns-(arm.header.stamp.sec*10**9+arm.header.stamp.nanosec))*1e-9),offline_replay_only=True)
        (capture/'metadata.json').write_text(json.dumps(metadata,indent=2))
    o.wait(6);assert o.stationary(),'Measured arm not stationary/fresh'
    start=time.monotonic();summary=o.measure(out/'measurement',args.seconds);end=time.monotonic()
    with o.lock:
        sts=[dict(receive_monotonic=t,record=r) for t,r in statuses if start<=t<=end]
        snaps=[dict(receive_monotonic=t,record=r) for t,r in snapshots if start<=t<=end]
    (out/'status.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in sts));(out/'snapshots.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in snaps))
    rows=[json.loads(s)['record'] for s in (out/'measurement/records.jsonl').read_text().splitlines()];rows=[r for r in rows if r.get('detection_valid')]
    ids={d['track_id'] for r in rows for d in r.get('detections',[]) if d.get('track_id') is not None}
    summary.update(track_ids=len(ids),confirmed_frames=sum(any(d.get('track_confirmed') for d in r.get('detections',[])) for r in rows),four_complete_center_frames=sum(sum(d.get('complete_center_target',False) for d in r.get('detections',[]))==4 for r in rows),detection_counts=dict(collections.Counter(len(r.get('detections',[])) for r in rows)),status_counts=dict(collections.Counter(s['record']['state'] for s in sts)),snapshots=[dict(snapshot_id=s['record']['snapshot_id'],targets=len(s['record']['targets']),unique_frames=s['record']['unique_frames'],span_s=(s['record']['sample_end_ns']-s['record']['sample_start_ns'])*1e-9,identity=s['record'].get('identity')) for s in snaps],base_rejections=dict(collections.Counter(r.get('base_transform_reason') for r in rows if not r.get('base_transform_valid'))),backend_samples=[r['compute'] for r in rows[:2]])
    (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps({k:summary[k] for k in ['completion_hz','base_transform_frames','track_ids','confirmed_frames','four_complete_center_frames','http_jpeg_hz','status_counts','snapshots']},ensure_ascii=False),flush=True)
finally:o.close();rclpy.shutdown()
