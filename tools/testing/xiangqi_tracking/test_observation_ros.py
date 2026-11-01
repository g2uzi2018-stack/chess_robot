"""Isolated ROS116 integration. All joints/images are fixtures; no live topics."""
import argparse,os,sys,time,json,signal,subprocess,copy,hashlib
from pathlib import Path
os.environ.update(ROS_DOMAIN_ID='116',ROS_LOCALHOST_ONLY='1')
HERE=Path(__file__).resolve().parent
if '--worker' in sys.argv:
    import live_tracked as live
    from test_stable_observation import frame
    class FixtureAudit:
        def __init__(self,*a):self.controller=1;self.epoch_start_ns=0;self.token='fixture-epoch';self.ok=True;self.reason='isolated fixture'
        def poll(self,now):self.host=now;return True
        def close(self):pass
    class FixturePipeline:
        def __init__(self):self.ready=[];self.count=0
        def can_submit(self):return not self.ready
        def busy(self):return bool(self.ready)
        def submit(self,color,depth,metadata,roi=None,draw=False,context=None):
            r=frame(self.count);r.update(stamp_ns=metadata['stamp_ns'],identity={'fixture':'four-independent-targets'},
                source_joint_position_rad=metadata.get('joint',{}).get('position',[]),source_joint_names=['joint'+str(i) for i in range(1,7)],
                base_transform_valid=metadata['base_eligible'],base_transform_reason=metadata['base_reason'],processing_s=.001,detector_ms=0.,geometry_ms=0.,camera_geometry_valid=True)
            for i,d in enumerate(r['detections']):d.update(box_xyxy=[250+i*120,300,310+i*120,360],class_name='red_pawn',camera_geometry_valid=True)
            self.ready.append((dict(metadata=metadata,context=context,started=time.monotonic(),draw=False),r,None));self.count+=1
        def poll(self):r=self.ready;self.ready=[];return r
        def close(self):pass
    live.Pipeline=FixturePipeline;live.AuditEpoch=FixtureAudit;live.main();sys.exit()
import numpy as np,rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from sensor_msgs.msg import Image,CameraInfo,JointState
from chess_robot_interfaces.msg import ArmState
from chess_robot_vision.msg import FrameInfo
from std_msgs.msg import String
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
rclpy.init();n=rclpy.create_node('isolated_stable_observation_integration');bridge=CvBridge()
pubs=[n.create_publisher(c,t,qos_profile_sensor_data) for c,t in [(Image,'/vision/color/image_rect'),(Image,'/vision/aligned_depth/image_rect'),(CameraInfo,'/vision/color/camera_info'),(FrameInfo,'/vision/frame_info')]]
ap=n.create_publisher(ArmState,'/arm/state',100);jp=n.create_publisher(JointState,'/arm/joint_states',100);request=n.create_publisher(String,'/pose/xiangqi/observation/request',10)
statuses=[];responses=[]
subs=[n.create_subscription(String,'/pose/xiangqi/observation/status',lambda m:statuses.append(json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
      n.create_subscription(String,'/pose/xiangqi/observation/response',lambda m:responses.append(json.loads(m.data)),32)]
env=os.environ.copy();env['XIANGQI_RUN_DIR']=str(out/'worker');env.pop('XIANGQI_BENCHMARK_LOCK',None)
log=(out/'worker.log').open('w');child=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'--worker'],env=env,stdout=log,stderr=subprocess.STDOUT)
last_image=last_joint=0.;q=0.;frame_number=1;feedback=True;frames=True
def spin(seconds):
    global last_image,last_joint,frame_number
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        now=time.monotonic();t=n.get_clock().now().nanoseconds;stamp=Time(sec=t//10**9,nanosec=t%10**9)
        if feedback and now-last_joint>=.05:
            j=JointState();j.header.stamp=stamp;j.header.frame_id='base_link';j.name=['joint'+str(i) for i in range(1,7)];j.position=[q]*6;j.velocity=[0.]*6
            arm=ArmState();arm.header=j.header;arm.backend='isolated-fixture';arm.feedback_fresh=True;arm.simulated=False;arm.busy=False;arm.controller_mode=4
            ap.publish(arm);jp.publish(j);last_joint=now
        if frames and now-last_image>=1/30:
            c=bridge.cv2_to_imgmsg(np.zeros((720,1280,3),np.uint8),'bgr8');d=bridge.cv2_to_imgmsg(np.full((720,1280),.2,np.float32),'32FC1')
            ci=CameraInfo();ci.width=1280;ci.height=720;ci.k=[600.,0.,640.,0.,600.,360.,0.,0.,1.];ci.p=[600.,0.,640.,0.,0.,600.,360.,0.,0.,0.,1.,0.]
            fi=FrameInfo();fi.stream_ok=True;fi.color_frame_number=frame_number;fi.depth_frame_number=frame_number
            for m,pub in zip([c,d,ci,fi],pubs):m.header.stamp=stamp;m.header.frame_id='d405_color_optical_frame';pub.publish(m)
            frame_number+=1;last_image=now
        rclpy.spin_once(n,timeout_sec=.001)
        assert child.poll() is None,'worker exited'
def rpc(action,**kw):
    rid=str(len(responses));request.publish(String(data=json.dumps(dict(action=action,request_id=rid,**kw))))
    end=time.monotonic()+2
    while time.monotonic()<end:
        spin(.03)
        result=next((r for r in responses if r['request_id']==rid),None)
        if result:return result
    raise AssertionError('response timeout')
try:
    deadline=time.monotonic()+12
    while not statuses or statuses[-1]['state']!='locked':
        spin(.05);assert time.monotonic()<deadline,(statuses[-1] if statuses else 'no status')
    first=rpc('query');second=rpc('query');snapshot=first['snapshot'];sid=snapshot['snapshot_id']
    assert first['ok'] and snapshot==second['snapshot'] and len(snapshot['targets'])==4
    assert snapshot['sample_end_ns']-snapshot['sample_start_ns']>=2e9 and snapshot['unique_frames']>=20
    feedback=False;spin(.35);assert not statuses[-1]['observation_eligible']
    assert rpc('query')['snapshot']==snapshot
    assert not rpc('consume',snapshot_id=sid,operation_id='blocked-during-outage')['ok']
    feedback=True;deadline=time.monotonic()+4
    while not statuses[-1]['observation_eligible']:spin(.05);assert time.monotonic()<deadline
    assert rpc('query')['snapshot']==snapshot
    assert rpc('consume',snapshot_id=sid,operation_id='fixture-op')['ok']
    assert rpc('consume',snapshot_id=sid,operation_id='fixture-op')['idempotent']
    assert not rpc('consume',snapshot_id=sid,operation_id='other-op')['ok']
    q=.01;spin(.15);assert not statuses[-1]['observation_eligible']
    (out/'result.json').write_text(json.dumps(dict(passed=True,domain=116,snapshot=snapshot,responses=responses,statuses=statuses,physical_commands=0),indent=2))
    print(json.dumps(dict(passed=True,domain=116,targets=4,unique_frames=snapshot['unique_frames'],unique_feedback_states=snapshot['unique_feedback_states'],physical_commands=0)))
finally:
    child.send_signal(signal.SIGTERM)
    try:child.wait(timeout=5)
    except subprocess.TimeoutExpired:child.kill();child.wait(timeout=2)
    log.close();n.destroy_node();rclpy.shutdown()
