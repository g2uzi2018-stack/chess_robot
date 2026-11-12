import copy,json,struct,tempfile,unittest
from pathlib import Path
import numpy as np
from stable_observation import FeedbackWindow,StableObservation,Policy,AuditEpoch,observation_roi
from piece_tracker import PieceTracker

T=100_000_000_000
def arm():return dict(backend='hardware',reference_valid=False,feedback_fresh=True,simulated=False,busy=False,controller_mode=4)
def joint(q=0):return dict(name=['joint'+str(i) for i in range(1,7)],position=[q]*6,velocity=[0.]*6)
def frame(i,offset=0):
    t=T+round(i*1e9/30)
    ds=[dict(valid=True,complete_center_target=True,camera_complete_center_target=True,track_confirmed=True,
             track_id=n+1,class_name='red_pawn',confidence=.9,
             camera_surface_center_m=[.1+n*.08+offset,.1,.2],base_surface_center_m=[.1+n*.08+offset,.1,.2],
             camera_surface_normal=[0,0,1.],base_surface_normal=[0,0,1.],plane_rms_m=.0005) for n in range(4)]
    return dict(stamp_ns=t,detection_valid=True,base_transform_valid=True,detections=ds,source_joint_position_rad=[0.]*6,
                feedback_stamp_ns=T+(i//2)*50_000_000,identity=dict(model='fixture'),source_K=[1,0,0,0,1,0,0,0,1],
                camera_frame_id='d405_color_optical_frame',base_frame_id='base_link',image_size=[1280,720],observation_frame_index=i)

class StableTests(unittest.TestCase):
    def test_wide_scene_is_not_forced_into_720_square(self):
        self.assertIsNone(observation_roi([[100,300,200,400],[950,300,1050,400]]))
        roi=observation_roi([[400,300,460,360],[500,300,560,360]])
        self.assertLessEqual(roi[0],400);self.assertGreaterEqual(roi[2],560)

    def locked(self):
        s=StableObservation()
        for i in range(62):s.observe(frame(i),frame(i)['stamp_ns'],True,1,'epoch1')
        self.assertEqual(s.state,'locked');return s

    def test_20hz_feedback_30hz_images_reuses_measured_state(self):
        f=FeedbackWindow();used=[]
        for i in range(121):
            t=T+round(i*1e9/30)
            for k in range(81):
                ft=T+k*50_000_000
                if f.last<ft<=t:f.ingest(ft,arm(),joint(),t,'one')
            h=f.at(t,t)
            if i>=20:self.assertTrue(h['ok']);used.append(h['feedback_stamp_ns'])
        self.assertLess(len(set(used)),len(used))

    def test_duplicate_reorder_does_not_refresh_or_clear_ids(self):
        f=FeedbackWindow()
        for i in range(20):f.ingest(T+i*50_000_000,arm(),joint(),T+i*50_000_000,'one')
        g=f.generation;last=f.last
        f.ingest(last-50_000_000,arm(),joint(),last,'one')
        self.assertEqual(f.generation,g)
        for i in range(10):f.ingest(last,arm(),joint(),last+100_000_000,'one')
        self.assertFalse(f.at(last+250_000_000,last+250_000_000)['ok'])
        f.ingest(last+300_000_000,arm(),joint(),last+300_000_000,'one')
        self.assertEqual(f.generation,g);self.assertEqual(f.retire_serial,0)
        for i in range(1,15):f.ingest(last+300_000_000+i*50_000_000,arm(),joint(),last+300_000_000+i*50_000_000,'one')
        self.assertTrue(f.at(f.last,f.last)['ok'])

    def test_processing_delay_and_delayed_callbacks_do_not_fake_motion(self):
        f=FeedbackWindow()
        for i in range(30):f.ingest(T+i*50_000_000,arm(),joint(),T+i*50_000_000,'one')
        self.assertTrue(f.at(f.last-150_000_000,f.last+100_000_000)['ok'])
        g=f.generation;retired=f.retire_serial
        f.ingest(f.last+1_000_000,arm(),joint(),f.last,'one')
        f.ingest(f.last,arm(),joint(),f.last+500_000_000,'one')
        self.assertEqual((f.generation,f.retire_serial),(g,retired))

    def test_device_duplicate_and_partial_geometry(self):
        s=StableObservation()
        for i in range(80):
            r=frame(i);r['rgbd_frame_key']=['session',1,1];s.observe(r,r['stamp_ns'],True,1,'epoch1')
        self.assertIsNone(s.snapshot)
        s=StableObservation()
        for i in range(65):
            r=frame(i)
            if i%10==5:r['detections'][0]['valid']=False
            s.observe(r,r['stamp_ns'],True,1,'epoch1')
        self.assertIsNotNone(s.snapshot);self.assertEqual(len(s.snapshot['targets']),4)

    def test_motion_epoch_and_long_outage_retire(self):
        for change in ('pose','epoch','long'):
            f=FeedbackWindow()
            for i in range(20):f.ingest(T+i*50_000_000,arm(),joint(),T+i*50_000_000,'one')
            t=f.last+(3_000_000_000 if change=='long' else 50_000_000)
            f.ingest(t,arm(),joint(.01 if change=='pose' else 0),t,'two' if change=='epoch' else 'one')
            self.assertGreater(f.retire_serial,0);self.assertFalse(f.at(t,t)['ok'])

    def test_immutable_query_and_one_operation_consume(self):
        s=self.locked();now=frame(61)['stamp_ns'];sid=s.snapshot['snapshot_id'];raw=json.dumps(s.snapshot,sort_keys=True)
        for i in range(62,70):s.observe(frame(i,.0001),frame(i)['stamp_ns'],True,1,'epoch1')
        self.assertEqual(json.dumps(s.snapshot,sort_keys=True),raw)
        now=frame(69)['stamp_ns']
        q=s.request(dict(action='query'),now);q['snapshot']['targets'].clear()
        self.assertEqual(len(s.snapshot['targets']),4)
        c=dict(action='consume',snapshot_id=sid,operation_id='op1')
        self.assertTrue(s.request(c,now)['ok']);self.assertTrue(s.request(c,now)['idempotent'])
        self.assertFalse(s.request(dict(c,operation_id='op2'),now)['ok'])
        self.assertFalse(s.status(now)['observation_eligible'])
        self.assertFalse(s.snapshot['motion_ready'])

    def test_few_duplicate_and_bad_depth_frames_cannot_fill_window(self):
        s=StableObservation()
        for i in [0]*80:s.observe(frame(i),T,True,1,'epoch1')
        self.assertIsNone(s.snapshot)
        for i in range(1,90):
            r=frame(i)
            if i%2:r['detections'][0]['valid']=False
            s.observe(r,r['stamp_ns'],True,1,'epoch1')
        self.assertIsNone(s.snapshot)
        s=StableObservation()
        for i in range(0,100,10):s.observe(frame(i),frame(i)['stamp_ns'],True,1,'epoch1')
        self.assertIsNone(s.snapshot)

    def test_feedback_uncertainty_preserves_snapshot_and_reconfirms(self):
        s=self.locked();saved=copy.deepcopy(s.snapshot);now=frame(62)['stamp_ns']
        s.tick(now,False,1,'epoch1');self.assertFalse(s.status(now)['observation_eligible']);self.assertEqual(saved,s.snapshot)
        s.observe(frame(63),frame(63)['stamp_ns'],True,1,'epoch1')
        self.assertTrue(s.status(frame(63)['stamp_ns'])['observation_eligible'])
        s.tick(frame(64)['stamp_ns'],True,2,'epoch2');self.assertTrue(s.revoked);self.assertEqual(saved,s.snapshot)

    def test_scene_change_blocks_consume_immediately_and_resamples(self):
        s=self.locked();sid=s.snapshot['snapshot_id']
        r=frame(62,.02);s.observe(r,r['stamp_ns'],True,1,'epoch1')
        self.assertFalse(s.status(r['stamp_ns'])['observation_eligible'])
        for i in range(63,130):s.observe(frame(i,.02),frame(i)['stamp_ns'],True,1,'epoch1')
        self.assertNotEqual(s.snapshot['snapshot_id'],sid)

    def test_stale_scene_lifetime_and_configuration_change(self):
        s=self.locked();now=frame(61)['stamp_ns']
        self.assertFalse(s.status(now+600_000_000)['observation_eligible'])
        r=frame(62);r['identity']['model']='changed';s.observe(r,r['stamp_ns'],True,1,'epoch1')
        self.assertTrue(s.revoked)
        s=self.locked();s.tick(now+61_000_000_000,True,1,'epoch1');self.assertTrue(s.revoked)

    def test_unstable_normal_is_reported_without_upgrading_orientation(self):
        s=StableObservation()
        for i in range(65):
            r=frame(i);angle=np.radians(6 if i%2 else -6)
            for d in r['detections']:
                d['base_surface_normal']=d['camera_surface_normal']=[float(np.sin(angle)),0.,float(np.cos(angle))]
            s.observe(r,r['stamp_ns'],True,1,'epoch1')
        self.assertIsNotNone(s.snapshot)
        self.assertFalse(s.snapshot['targets'][0]['base_surface_normal_temporally_stable'])
        self.assertFalse(s.snapshot['targets'][0]['orientation_valid_for_motion'])

    def test_expired_history_cannot_repeatedly_clear_new_collection(self):
        s=self.locked();old_id=s.snapshot['snapshot_id'];now=frame(2100)['stamp_ns']
        s.tick(now,True,1,'epoch1')
        self.assertTrue(s.revoked)
        self.assertTrue(s.request(dict(action='collect'),now)['ok'])
        for i in range(2100,2165):s.observe(frame(i),frame(i)['stamp_ns'],True,1,'epoch1')
        self.assertNotEqual(s.snapshot['snapshot_id'],old_id)
        self.assertEqual(s.state,'locked')
        self.assertTrue(s.status(frame(2164)['stamp_ns'])['observation_eligible'])

    def test_brief_missing_detection_uses_existing_valid_fraction_gate(self):
        s=StableObservation()
        for i in range(65):
            r=frame(i)
            if i%10==5:r['detections'].pop()
            s.observe(r,r['stamp_ns'],True,1,'epoch1')
        self.assertIsNotNone(s.snapshot)
        self.assertEqual(len(s.snapshot['targets']),4)
        self.assertGreaterEqual(min(t['valid_fraction'] for t in s.snapshot['targets']),.8)

    def test_tracker_suspend_resume_and_order(self):
        tracker=PieceTracker();a=tracker.update(frame(0)['detections'],T)
        ids=[d['track_id'] for d in a]
        # No update during an unqualified image: unchanged state is historical.
        b=tracker.update(list(reversed(frame(5)['detections'])),T+166_666_667)
        self.assertEqual([d['track_id'] for d in b],list(reversed(ids)))
        self.assertEqual(len(tracker.tracks),4)

    def test_passive_audit_epoch_duplicate_rotation_and_motion(self):
        def raw(host,controller,q=0):
            b=bytearray(1448);struct.pack_into('<q',b,0,host);struct.pack_into('<H',b,8,1440)
            struct.pack_into('<Q',b,32,4);struct.pack_into('<Q',b,40,controller);struct.pack_into('<Q',b,56,0x0123456789ABCDEF)
            struct.pack_into('<6d',b,440,*([q]*6));return b
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'feedback.bin';p.write_bytes(raw(T,100));a=AuditEpoch(p)
            self.assertTrue(a.poll(T));token=a.token
            with p.open('ab') as f:f.write(raw(T+50_000_000,100))
            self.assertFalse(a.poll(T+250_000_000));self.assertEqual(a.host,T)
            p.rename(p.with_suffix('.old'));p.write_bytes(raw(T+300_000_000,1))
            self.assertTrue(a.poll(T+300_000_000));self.assertNotEqual(a.token,token)
            token=a.token
            with p.open('ab') as f:f.write(raw(T+350_000_000,2,q=1))
            self.assertFalse(a.poll(T+350_000_000));self.assertNotEqual(a.token,token);a.close()

if __name__=='__main__':unittest.main(verbosity=2)
