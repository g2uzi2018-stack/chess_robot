"""Isolated ROS admission regression with deterministic geometry (no GPU/driver)."""
import os,sys,json,time,subprocess,signal,argparse
from pathlib import Path
os.environ['ROS_DOMAIN_ID']='117';os.environ['ROS_LOCALHOST_ONLY']='1'
HERE=Path(__file__).resolve().parent
if len(sys.argv)>1 and sys.argv[1]=='--worker':
 sys.path.insert(0,str(HERE));import perception_node as live
 from recognize_tracked import surface_position
 import numpy as np
 class FixturePipeline:
  def __init__(self,**kwargs):self.ready=[];self.count=0
  def can_submit(self):return not self.ready
  def busy(self):return bool(self.ready)
  def submit(self,color,depth,metadata,roi=None,draw=False,context=None):
   mask=np.zeros(depth.shape,np.uint8);mask[300:400,600:700]=1
   d=surface_position(mask,depth,np.array(metadata['K']).reshape(3,3));d.update(id=0,class_id=0,class_name='fixture',confidence=1.,box_xyxy=[600,300,700,400],detection_valid=True,camera_geometry_valid=bool(d['valid']),valid=False,camera_complete_center_target=bool(d['valid']),touches_image_edge=False,touches_processing_edge=False,grasp_ready=False,motion_ready=False)
   record=dict(stamp_ns=metadata['stamp_ns'],detections=[d],detection_valid=True,camera_geometry_valid=d['camera_geometry_valid'],base_transform_valid=False,detector_ms=0.,geometry_ms=0.,processing_s=0.,fixture=True,image_size=[1280,720],inference_roi_xyxy=roi or [0,0,1280,720],identity={'model':'deterministic-fixture'},compute={'inference_backend':'fixture','fp16':False})
   self.ready.append((dict(metadata=metadata,context=context,started=time.monotonic(),draw=False),record,None));self.count+=1;return self.count
  def poll(self):out=self.ready;self.ready=[];return out
  def close(self):pass
 live.Pipeline=FixturePipeline;sys.argv=[sys.argv[0]];live.main();sys.exit(0)

import rclpy,numpy as np
from sensor_msgs.msg import Image,CameraInfo
from chess_robot_vision.msg import FrameInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
p=argparse.ArgumentParser();p.add_argument('--output',required=True);args=p.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
rclpy.init();n=rclpy.create_node('isolated_image_admission_regression');bridge=CvBridge();received=[]
pubs=[n.create_publisher(c,t,qos_profile_sensor_data) for c,t in [(Image,'/vision/color/image_rect'),(Image,'/vision/aligned_depth/image_rect'),(CameraInfo,'/vision/color/camera_info'),(FrameInfo,'/vision/frame_info')]]
sub=n.create_subscription(String,'/perception/xiangqi/measurements',lambda m:received.append(json.loads(m.data)),qos_profile_sensor_data)
hint_pub=n.create_publisher(String,'/perception/xiangqi/roi_hint',1)
env=os.environ.copy();env['XIANGQI_RUN_DIR']=str(out);env.pop('XIANGQI_BENCHMARK_LOCK',None)
log=(out/'fixture.log').open('w');child=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'--worker'],env=env,stdout=log,stderr=subprocess.STDOUT)
def spin(s):
 end=time.monotonic()+s
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.005)
def observe(name):
 start=len(received);end=time.monotonic()+.65
 while time.monotonic()<end:
  t=n.get_clock().now().nanoseconds-(300000000 if name=='stale' else 0);stamp=__import__('builtin_interfaces.msg',fromlist=['Time']).Time(sec=t//10**9,nanosec=t%10**9)
  color=bridge.cv2_to_imgmsg(np.zeros((720,1280,3),np.uint8),'bgr8');depth=bridge.cv2_to_imgmsg(np.full((720,1280),np.nan if name=='invalid-depth' else .2,np.float32),'32FC1')
  ci=CameraInfo();ci.width=1280;ci.height=720;ci.k=[600.,0.,640.,0.,600.,360.,0.,0.,1.];ci.p=[600.,0.,640.,0.,0.,600.,360.,0.,0.,0.,1.,0.]
  if name=='bad-intrinsics':ci.k[0]=0.
  if name=='nan-intrinsics':ci.k[0]=float('nan')
  fi=FrameInfo();fi.stream_ok=name!='stream-invalid';fi.color_frame_number=t;fi.depth_frame_number=t
  for m,pub in zip([color,depth,ci,fi],pubs):m.header.stamp=stamp;m.header.frame_id='d405_color_optical_frame'
  if name=='rgbd-mismatch':
   v=t+1000000;depth.header.stamp=type(stamp)(sec=v//10**9,nanosec=v%10**9)
  for m,pub in zip([color,depth,ci,fi],pubs):pub.publish(m)
  spin(.05)
 spin(.25);return received[start:]
try:
 deadline=time.monotonic()+8
 while any(p.get_subscription_count()==0 for p in pubs):
  assert child.poll() is None and time.monotonic()<deadline;spin(.05)
 spin(.2);assert not received,'Missing sources must wait without fabricated results'
 for malformed in ['['*1100+'0'+']'*1100,'{"schema":"bad","value":1e999}']:
  hint_pub.publish(String(data=malformed));spin(.1)
 results=[]
 for name in ['rgbd-mismatch','stale','bad-intrinsics','nan-intrinsics','good-no-arm','invalid-depth','stream-invalid','source-resumed']:
  frames=observe(name);qualified=[r for r in frames if r.get('camera_geometry_valid')]
  if name in ('rgbd-mismatch','stale'):assert not frames,(name,'unexpected admitted frame')
  elif name in ('bad-intrinsics','nan-intrinsics','stream-invalid'):assert frames and not any(r.get('detection_valid') for r in frames),name
  elif name=='invalid-depth':assert frames and not qualified and all(r['detection_valid'] for r in frames),name
  else:assert len(qualified)>=2,name
  assert all(not r.get('base_transform_valid') for r in frames)
  results.append(dict(case=name,passed=True,records=len(frames),camera_geometry_frames=len(qualified)))
 result=dict(passed=True,domain=117,deterministic_pipeline_fixture=True,malformed_roi_did_not_kill_spin=True,invalid_nan_K_published_then_recovered=True,real_model_test='not tested in this fixture; saved-input comparison reuses cached detector outputs',cases=results,physical_commands=0)
 (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
finally:
 if child.poll() is None:child.send_signal(signal.SIGTERM)
 try:child.wait(timeout=5)
 except subprocess.TimeoutExpired:child.kill();child.wait(timeout=3)
 log.close();n.destroy_node();rclpy.shutdown()
