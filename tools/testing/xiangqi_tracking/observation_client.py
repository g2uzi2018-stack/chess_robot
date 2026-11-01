"""Client for visual observations only. No arm control topics or services."""
import argparse,json,time,uuid
import rclpy
from std_msgs.msg import String
p=argparse.ArgumentParser();p.add_argument('action',choices=['collect','query','consume']);p.add_argument('--snapshot-id');p.add_argument('--operation-id');p.add_argument('--timeout',type=float,default=5.)
a=p.parse_args();rclpy.init();n=rclpy.create_node('xiangqi_observation_client');responses=[]
rid=uuid.uuid4().hex
sub=n.create_subscription(String,'/pose/xiangqi/observation/response',lambda m:responses.append(json.loads(m.data)),32)
pub=n.create_publisher(String,'/pose/xiangqi/observation/request',10)
try:
    end=time.monotonic()+a.timeout
    while not pub.get_subscription_count() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
    if not pub.get_subscription_count():raise TimeoutError('visual observation server unavailable')
    payload=dict(action=a.action,request_id=rid)
    if a.snapshot_id is not None:payload['snapshot_id']=a.snapshot_id
    if a.operation_id is not None:payload['operation_id']=a.operation_id
    pub.publish(String(data=json.dumps(payload)))
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.02)
        response=next((r for r in responses if r.get('request_id')==rid),None)
        if response:
            print(json.dumps(response,ensure_ascii=False,indent=2));break
    else:raise TimeoutError('visual observation response timeout')
finally:n.destroy_node();rclpy.shutdown()
