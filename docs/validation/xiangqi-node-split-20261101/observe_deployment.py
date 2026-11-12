"""Read-only visual topology observation; no requests or device commands."""
import collections,json,time,sys
from pathlib import Path
import numpy as np
import rclpy
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

rclpy.init();node=rclpy.create_node('xiangqi_split_deployment_observer')
rows=collections.defaultdict(list)
def receive(topic,msg):
    try:
        record=json.loads(msg.data)
    except (ValueError,TypeError):
        return
    rows[topic].append((time.time_ns(),record))
subscriptions=[]
for topic in ['/perception/xiangqi/measurements','/pose/xiangqi/detections']:
    subscriptions.append(node.create_subscription(String,topic,lambda msg,t=topic:receive(t,msg),qos_profile_sensor_data))
start=time.monotonic()
while time.monotonic()-start<30:
    rclpy.spin_once(node,timeout_sec=.1)
window=time.monotonic()-start
result=dict(read_only=True,physical_commands=0,window_s=window,nodes=node.get_node_names_and_namespaces(),topics={})
for topic,items in rows.items():
    unique={}
    for received,record in items:
        key=(record.get('source_epoch'),record.get('sequence'),record.get('stamp_ns'))
        unique[key]=(received,record)
    valid=[(received,record) for received,record in unique.values() if record.get('detection_valid')]
    ages=[(received-record['stamp_ns'])/1e6 for received,record in valid]
    last=items[-1][1]
    result['topics'][topic]=dict(messages=len(items),unique=len(unique),detection_valid_unique=len(valid),
        valid_completion_hz=len(valid)/window,base_valid_unique=sum(bool(r.get('base_transform_valid')) for _,r in valid),
        observer_receive_age_ms={str(p):float(np.percentile(ages,p)) for p in [50,95]} if ages else {},
        last_compute=last.get('compute'),last_reason=last.get('reason'),last_source_epoch=last.get('source_epoch'),
        last_detection_count=len(last.get('detections',[])),last_camera_geometry_valid=last.get('camera_geometry_valid'),
        last_motion_ready=last.get('motion_ready'),last_grasp_ready=last.get('grasp_ready'))
Path(sys.argv[1]).write_text(json.dumps(result,indent=2));print(json.dumps(result))
node.destroy_node();rclpy.shutdown()
