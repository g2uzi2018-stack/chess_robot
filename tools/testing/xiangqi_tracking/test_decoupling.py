"""CPU regression; no ROS, model inference or controller connection."""
import unittest,sys,ast,hashlib,tempfile
from pathlib import Path
import numpy as np
from recognize_tracked import Experiment,surface_position
from qualification import measured_stopped,revoke_base,expire_observation
from piece_tracker import PieceTracker
from shared_pipeline import FrameSlots,Pipeline,geometry_slot
from concurrent.futures import Future

class Decoupling(unittest.TestCase):
 def setUp(self):
  self.e=Experiment.__new__(Experiment);self.e.imgsz=640;self.e.X=np.eye(4);self.e.robot=None
  self.e.fk=lambda *a:dict(Link6=np.eye(4),grasp_link=np.eye(4));self.e.base_model_reason='fixture unavailable'
  self.color=np.zeros((120,160,3),np.uint8);self.depth=np.full((120,160),.2,np.float32)
  self.K=[300.,0,80.,0,300.,60.,0,0,1.];self.mask=np.zeros((120,160),np.uint8);self.mask[35:85,55:105]=1
  self.packet=dict(predictions=np.array([[55,35,105,85,.9,0]]),masks=self.mask[None],names={0:'red_pawn'},compute={},identity={},roi_xyxy=[0,0,160,120],image_size=[160,120],inference_speed_ms={},detector_s=.01)
  self.joint=dict(name=['joint'+str(i) for i in range(1,7)],position=[0.]*6,velocity=[0.]*6)
  self.arm=dict(feedback_fresh=True,reference_valid=False,simulated=False,busy=False,controller_mode=4,tcp=dict(header=dict(frame_id='base_link'),pose=dict(position=dict(x=0,y=0,z=0))))
 def run_frame(self,base=False,**extra):
  m=dict(K=self.K,stamp_ns=100,base_eligible=base,arm=self.arm,joint=self.joint);m.update(extra)
  return self.e.run(self.color,self.depth,m,draw=False,detection=self.packet)[0]
 def test_no_arm_and_model_still_produce_camera_geometry(self):
  self.e.X=None;r=self.run_frame(arm={},joint={});d=r['detections'][0]
  self.assertTrue(r['detection_valid'] and d['camera_geometry_valid']);self.assertFalse(d['valid'])
  self.assertNotIn('base_surface_center_m',d);self.assertEqual(r['frame_id'],'base_link');self.assertFalse(d['grasp_ready'])
 def test_reference_and_mode_independent_measured_base(self):
  for mode,ref in [(4,False),(5,False),(5,True)]:
   self.arm.update(controller_mode=mode,reference_valid=ref);r=self.run_frame(True);d=r['detections'][0]
   self.assertTrue(d['valid']);np.testing.assert_array_equal(d['base_surface_center_m'],d['camera_surface_center_m'])
   self.assertFalse(r['motion_ready'] or d['grasp_ready'])
 def test_feedback_missing_moving_busy_simulated_reject_base_only(self):
  for update in [dict(feedback_fresh=False),dict(controller_mode=7),dict(busy=True),dict(simulated=True)]:
   saved=dict(self.arm);self.arm.update(update);r=self.run_frame(True)
   self.assertFalse(r['base_transform_valid']);self.assertTrue(r['detections'][0]['camera_geometry_valid']);self.arm=saved
  self.joint['velocity'][0]=.003;self.assertFalse(self.run_frame(True)['base_transform_valid'])
 def test_model_mismatch_invalid_depth_and_generation_revocation(self):
  self.arm['tcp']['pose']['position']['x']=.001;r=self.run_frame(True)
  self.assertFalse(r['base_transform_valid']);self.assertTrue(r['camera_geometry_valid'])
  self.arm['tcp']['pose']['position']['x']=0;r=self.run_frame(True);revoke_base(r,'generation changed')
  self.assertTrue(r['detections'][0]['camera_geometry_valid']);self.assertNotIn('base_surface_center_m',r['detections'][0])
  expire_observation(r);self.assertFalse(r['detection_valid']);self.assertNotIn('camera_surface_center_m',r['detections'][0])
  self.depth[:]=np.nan;r=self.run_frame(True);self.assertFalse(r['detections'][0]['valid']);self.assertFalse(r['camera_geometry_valid'])
 def test_tracker_no_camera_pollution_or_old_state_write(self):
  with tempfile.TemporaryDirectory() as tmp:
   p=Path(tmp)/'production.json';p.write_text('production-state-sentinel');tracker=PieceTracker()
   r=self.run_frame();tracker.update(r['detections'],100);self.assertEqual(tracker.tracks,{});self.assertEqual(p.read_text(),'production-state-sentinel')
 def test_production_lease_drains_and_yields_to_shared_benchmark_lock(self):
  import os,fcntl
  from benchmark_lease import BenchmarkLease
  previous=os.environ.get('XIANGQI_BENCHMARK_LOCK')
  with tempfile.TemporaryDirectory() as tmp:
   os.environ['XIANGQI_BENCHMARK_LOCK']=str(Path(tmp)/'benchmark.lock');lease=BenchmarkLease()
   try:
    self.assertTrue(lease.can_submit(False))
    with open(lease.path,'a') as other:
     with self.assertRaises(BlockingIOError):fcntl.flock(other,fcntl.LOCK_EX|fcntl.LOCK_NB)
     lease.until=0;self.assertFalse(lease.can_submit(True))
     with self.assertRaises(BlockingIOError):fcntl.flock(other,fcntl.LOCK_EX|fcntl.LOCK_NB)
     self.assertFalse(lease.can_submit(False));fcntl.flock(other,fcntl.LOCK_EX|fcntl.LOCK_NB)
     self.assertFalse(lease.can_submit(False));fcntl.flock(other,fcntl.LOCK_UN)
     self.assertTrue(lease.can_submit(False))
   finally:
    lease.close()
    if previous is None:os.environ.pop('XIANGQI_BENCHMARK_LOCK',None)
    else:os.environ['XIANGQI_BENCHMARK_LOCK']=previous
 def test_surface_geometry_unchanged(self):
  # Compare the actual geometry function to the preserved pre-edit source.
  import tarfile
  with tarfile.open(sys.argv[1]) as t:old=t.extractfile('tools/testing/xiangqi_tracking/recognize_tracked.py').read().decode()
  tree=ast.parse(old);fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='surface_position')
  namespace={};exec(compile(ast.Module(body=[fn],type_ignores=[]),'original','exec'),dict(np=np,cv2=__import__('cv2')),namespace)
  a=namespace['surface_position'](self.mask,self.depth,np.array(self.K).reshape(3,3));b=surface_position(self.mask,self.depth,np.array(self.K).reshape(3,3));self.assertEqual(a,b)
 def test_slots_reject_wrong_stamp_and_free_on_submit_exception(self):
  s=FrameSlots(160,120,count=3,max_masks=2)
  try:
   s.write(0,1,100,self.color,self.depth);s.check(dict(slot=0,job_id=1,metadata=dict(stamp_ns=100)))
   with self.assertRaises(AssertionError):s.check(dict(slot=0,job_id=2,metadata=dict(stamp_ns=100)))
   p=Pipeline.__new__(Pipeline);p.slots=s;p.free=[0,1,2];p.gpu_job=None;p.next_id=0
   class Broken:
    def submit(self,*a):raise RuntimeError('worker error')
   p.gpu=Broken()
   with self.assertRaises(RuntimeError):p.submit(self.color,self.depth,dict(stamp_ns=101))
   self.assertEqual(sorted(p.free),[0,1,2]);self.assertIsNone(p.gpu_job)
   with self.assertRaises(AssertionError):geometry_slot(dict(slot=0,job_id=1,metadata=dict(stamp_ns=100)),dict(job_id=1,stamp_ns=101))
  finally:s.close()
 def test_future_exception_and_order_release(self):
  p=Pipeline.__new__(Pipeline);p.free=[];p.geometry_jobs=[];p.last_returned=-1
  f=Future();f.set_exception(RuntimeError('gpu'));p.gpu_job=({'slot':0},f)
  with self.assertRaises(RuntimeError):p.poll()
  self.assertEqual(p.free,[0]);self.assertIsNone(p.gpu_job)
  f=Future();f.set_exception(RuntimeError('geometry'));p.geometry_jobs=[({'slot':1},f)]
  with self.assertRaises(RuntimeError):p.poll()
  self.assertEqual(p.free,[0,1])

if __name__=='__main__':
 # Keep the tar path outside unittest's argument parser.
 unittest.main(argv=[sys.argv[0]],verbosity=2)
