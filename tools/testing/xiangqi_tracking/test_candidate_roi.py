"""Deterministic same-image detector fixtures; no model, ROS or device access."""
import copy
import unittest
from unittest.mock import patch

import numpy as np

from candidate_roi import MAX_CROPS, POLICY, plan_crops, recognize
from measurement_contract import ContractError, camera_measurement, validate
from recognize_tracked import Experiment
from shared_pipeline import FrameSlots, detect_slot, geometry_slot
from scene_state import SceneState
from stable_observation import FeedbackWindow
from test_node_split import FixtureBase, feed, packet


IMAGE = np.zeros((720, 1280, 3), np.uint8)
BOXES = [[500, 300, 550, 350], [610, 320, 660, 370]]
CLASS_NAMES={0:'red_pawn',1:'red_rook',2:'black_pawn',3:'black_elephant'}


def detector_packet(boxes, labels, roi=None):
    roi = roi or [0, 0, 1280, 720]
    x, y, x2, y2 = roi
    predictions = []
    masks = []
    names = dict(CLASS_NAMES)
    for i, (box, label) in enumerate(zip(boxes, labels)):
        class_id=next(k for k,v in names.items() if v==label)
        predictions.append([box[0]-x, box[1]-y, box[2]-x, box[3]-y, .9, class_id])
        mask = np.zeros((y2-y, x2-x), np.uint8)
        a, b, c, d = map(int, box)
        mask[max(0,b-y):max(0,d-y), max(0,a-x):max(0,c-x)] = 1
        masks.append(mask)
    return dict(predictions=np.array(predictions).reshape(-1,6), names=names,
                masks=np.array(masks) if masks else None, compute={}, identity={},
                roi_xyxy=roi, image_size=[1280,720], detector_s=.001,
                inference_speed_ms={'inference':1.})


def fake_detector(full_boxes=BOXES, roi_boxes=BOXES, full_labels=None, roi_labels=None):
    calls = []
    def detect(color, roi=None):
        assert color is IMAGE, 'must use this exact frame for every pass'
        calls.append(roi)
        if roi is None:
            return detector_packet(full_boxes, full_labels or ['red_rook']*len(full_boxes))
        return detector_packet(roi_boxes, roi_labels or ['red_pawn']*len(roi_boxes), roi)
    return detect, calls


def scene_process(state, feedback, p):
    now = p['stamp_ns']+30_000_000
    feed(feedback, now)
    return state.process(p, now, feedback, True, 'controller-one'), now


class CandidateRoiTests(unittest.TestCase):
    def test_first_frame_uses_full_candidates_and_only_roi_class_names(self):
        detect, calls = fake_detector()
        result = recognize(IMAGE, detect)
        self.assertIsNone(calls[0])
        self.assertEqual(len(calls), 2)
        self.assertTrue(result['full_scene_coverage'] and result['classification_complete'])
        self.assertEqual({result['names'][int(r[-1])] for r in result['predictions']}, {'red_pawn'})
        self.assertTrue(all(d['classification_source']=='roi' for d in result['classification_evidence']))

    def test_roi_failure_preserves_unclassified_full_candidates(self):
        detect, _ = fake_detector(roi_boxes=[])
        result = recognize(IMAGE, detect)
        self.assertEqual(len(result['predictions']), 2)
        self.assertFalse(result['classification_complete'])
        self.assertEqual({result['names'][int(r[-1])] for r in result['predictions']}, {'unknown'})
        self.assertTrue(all(not d['classification_valid'] for d in result['classification_evidence']))

    def test_extra_roi_target_is_not_dropped(self):
        boxes = BOXES+[[700,350,750,400]]
        detect, _ = fake_detector(roi_boxes=boxes, roi_labels=['red_pawn','black_pawn','black_elephant'])
        result = recognize(IMAGE, detect)
        self.assertEqual(result['full_candidate_count'],2)
        self.assertEqual(len(result['predictions']),3)
        self.assertIn('black_elephant',result['names'].values())

    def test_duplicate_conflicting_roi_labels_cannot_qualify(self):
        detect, _ = fake_detector(full_boxes=BOXES[:1], roi_boxes=[BOXES[0],BOXES[0]],
                                  roi_labels=['red_pawn','red_rook'])
        result = recognize(IMAGE, detect)
        self.assertFalse(result['classification_complete'])
        self.assertEqual(result['names'][int(result['predictions'][0,-1])],'unknown')

    def test_hint_cannot_hide_new_full_image_candidate(self):
        boxes = [BOXES[0], [1100,400,1150,450]]
        planned = plan_crops(boxes,1280,720,[300,100,780,580])
        self.assertTrue(any(c[0]<=1100 and c[2]>=1150 for c in planned))

    def test_full_image_or_invalid_hint_uses_independent_roi_fallback(self):
        for hint in ([0,0,1280,720],[-1,0,720,720],[0,0,100,100],['bad'],None):
            detect,calls=fake_detector()
            result=recognize(IMAGE,detect,hint)
            self.assertEqual(len(calls),2)
            self.assertNotEqual(calls[1],[0,0,1280,720])
            self.assertTrue(result['classification_complete'])

    def test_cropped_mask_and_box_return_to_original_image(self):
        detect, _ = fake_detector()
        result = recognize(IMAGE, detect)
        np.testing.assert_array_equal(result['predictions'][0,:4],BOXES[0])
        self.assertEqual(result['masks'].shape,(2,720,1280))
        self.assertEqual(int(result['masks'][0].sum()),2500)
        self.assertEqual(result['masks'][0,320,520],1)
        self.assertEqual(result['masks'][0,20,20],0)

    def test_model_class_id_does_not_become_frame_instance_index(self):
        for order, full_label in [(BOXES, 'red_rook'),(list(reversed(BOXES)), 'black_pawn')]:
            labels=['red_pawn','black_elephant']
            detect,_=fake_detector(full_boxes=order,roi_boxes=order,
                                   full_labels=[full_label]*2,roi_labels=labels)
            result=recognize(IMAGE,detect)
            self.assertEqual(result['predictions'][:,-1].tolist(),[0.,3.])
            self.assertEqual([result['names'][int(r[-1])] for r in result['predictions']],labels)

    def test_crop_edge_target_remains_visible_but_unresolved(self):
        detect, calls = fake_detector(full_boxes=BOXES[:1])
        def edge(color,roi=None):
            if roi is None:return detect(color,roi)
            return detector_packet([[roi[0],roi[1]+20,roi[0]+50,roi[1]+70]],['red_pawn'],roi)
        result = recognize(IMAGE,edge)
        self.assertEqual(len(result['predictions']),2)
        self.assertFalse(result['classification_complete'])
        self.assertTrue(all(not d['classification_valid'] for d in result['classification_evidence']))

    def test_excess_crop_budget_revokes_coverage_without_dropping_candidates(self):
        detect, _ = fake_detector()
        crops = [[0,0,480,480]]*(MAX_CROPS+1)
        with patch('candidate_roi.plan_crops',return_value=crops):
            result = recognize(IMAGE,detect)
        self.assertFalse(result['full_scene_coverage'] or result['classification_complete'])
        self.assertGreaterEqual(len(result['predictions']),2)

    def test_instance_overflow_is_an_incomplete_event_not_a_partial_scene(self):
        detect,_=fake_detector()
        with patch('candidate_roi.MAX_INSTANCES',1):
            result=recognize(IMAGE,detect)
        self.assertEqual(len(result['predictions']),0)
        self.assertFalse(result['full_scene_coverage'] or result['classification_complete'])

    def test_shared_slots_preserve_both_passes_and_mask_frame_ownership(self):
        e=Experiment.__new__(Experiment);e.camera_only=True;e.imgsz=640
        calls=[]
        def detect(color,roi=None):
            calls.append(roi)
            return detector_packet(BOXES,['red_rook']*2 if roi is None else ['red_pawn']*2,roi)
        e.detect=detect
        slots=FrameSlots(1280,720,count=1,max_masks=4)
        metadata=dict(K=[600.,0,640.,0,600.,360.,0,0,1.],stamp_ns=100,
                      camera_epoch='fixture',rgbd_frame_key=['fixture',1,1])
        job=dict(slot=0,job_id=7,metadata=metadata,roi=[300,100,780,580],draw=False)
        try:
            slots.write(0,7,100,IMAGE,np.full((720,1280),.2,np.float32))
            with patch('shared_pipeline.worker_slots',slots),patch('shared_pipeline.worker_experiment',e):
                result=detect_slot(job)
                self.assertNotIn('masks',result['packet'])
                self.assertEqual(result['packet']['mask_shape'],[2,720,1280])
                record,_=geometry_slot(job,result)
                self.assertEqual(record['stamp_ns'],100)
                self.assertEqual(record['pipeline_job_id'],7)
                self.assertEqual([d['class_id'] for d in record['detections']],[0,0])
                self.assertTrue(all(d['camera_geometry_valid'] for d in record['detections']))
                slots.headers[0][1]=101
                with self.assertRaises(AssertionError):geometry_slot(job,result)
            self.assertEqual(calls,[None,[300,100,780,580]])
        finally:slots.close()

    def test_wrong_crop_mask_shape_is_rejected(self):
        detect,_=fake_detector()
        def broken(color,roi=None):
            result=detect(color,roi)
            if roi is not None:result['masks']=result['masks'][:,:-1,:]
            return result
        with self.assertRaises(ValueError):recognize(IMAGE,broken)

    def test_original_stamp_and_geometry_survive_new_packet(self):
        detect, _ = fake_detector()
        detection = recognize(IMAGE, detect)
        e = Experiment.__new__(Experiment);e.camera_only=True;e.imgsz=640
        metadata = dict(K=[600.,0,640.,0,600.,360.,0,0,1.], stamp_ns=100_000_000_000,
                        camera_epoch='fixture',rgbd_frame_key=['fixture',1,1])
        record, _ = e.run(IMAGE,np.full((720,1280),.2,np.float32),metadata,
                          roi=[300,100,780,580],draw=False,detection=detection)
        p = camera_measurement(record,metadata,'fixture',0,metadata['stamp_ns']+1)
        validate(p,metadata['stamp_ns']+30_000_000)
        self.assertEqual(p['candidate_scan_stamp_ns'],metadata['stamp_ns'])
        self.assertTrue(all(d['classification_stamp_ns']==metadata['stamp_ns'] for d in p['detections']))
        self.assertTrue(all(d['camera_geometry_valid'] for d in p['detections']))
        self.assertTrue(all(d['class_name']=='red_pawn' for d in p['detections']))
        self.assertFalse(p['motion_ready'] or p['grasp_ready'])

    def test_legacy_or_forged_provenance_rejected_before_guard_mutation(self):
        changes = [lambda p:p.update(schema='xiangqi_camera_measurements/v1'),
                   lambda p:p.pop('recognition_policy'),
                   lambda p:p.update(candidate_scan_stamp_ns=p['stamp_ns']-1),
                   lambda p:p['detections'][0].update(classification_stamp_ns=p['stamp_ns']-1),
                   lambda p:p['detections'][0].update(classification_source='full_image'),
                   lambda p:p.update(classification_rois=[]),
                   lambda p:p.update(full_candidate_count=5)]
        for change in changes:
            p = packet();change(p)
            state = SceneState(base=FixtureBase()); f = FeedbackWindow()
            with self.assertRaises(ContractError):scene_process(state,f,p)
            self.assertIsNone(state.guard.epoch)

    def test_unknown_candidate_cannot_pollute_votes_or_seed_snapshot(self):
        state = SceneState(base=FixtureBase()); f = FeedbackWindow()
        for i in range(65):
            p = packet(i)
            p['detections'][0].update(class_name='unknown',classification_valid=False,
                                      class_id=-1,classification_source='unresolved',classification_roi_xyxy=None)
            p['classification_complete']=False
            result, now = scene_process(state,f,p)
        self.assertIsNone(state.observation.snapshot)
        self.assertIsNone(result['detections'][0]['track_id'])
        self.assertTrue(all('unknown' not in t['class_votes'] for t in state.tracker.tracks.values()))

    def locked(self):
        state = SceneState(base=FixtureBase());f = FeedbackWindow()
        for i in range(65):result,now=scene_process(state,f,packet(i))
        self.assertTrue(state.observation.status(now)['observation_eligible'])
        return state,f

    def test_low_confidence_roi_class_conflict_pauses_existing_snapshot(self):
        state,f = self.locked()
        p = packet(65);p['detections'][0].update(class_name='red_rook',confidence=.3)
        _,now = scene_process(state,f,p)
        self.assertFalse(state.observation.status(now)['observation_eligible'])
        self.assertEqual(state.observation.snapshot['targets'][0]['class_name'],'red_pawn')

    def test_incomplete_geometry_or_missing_candidate_pauses_snapshot(self):
        for change in ('plane','missing','coverage'):
            state,f=self.locked();p=packet(65)
            if change=='plane':p['detections'][0]['camera_geometry_valid']=False
            if change=='missing':p['detections'].pop();p['full_candidate_count']=3
            if change=='coverage':p.update(full_scene_coverage=False,classification_complete=False)
            _,now=scene_process(state,f,p)
            self.assertFalse(state.observation.status(now)['observation_eligible'])

    def test_class_change_during_collection_restarts_sample(self):
        state=SceneState(base=FixtureBase());f=FeedbackWindow()
        for i in range(40):scene_process(state,f,packet(i))
        p=packet(40);p['detections'][0]['class_name']='red_rook'
        scene_process(state,f,p)
        self.assertEqual(len(state.observation.frames),1)
        self.assertIsNone(state.observation.snapshot)

    def test_single_geometry_rejection_skips_frame_without_resetting_window(self):
        state=SceneState(base=FixtureBase());f=FeedbackWindow()
        for i in range(30):scene_process(state,f,packet(i))
        before=len(state.observation.frames)
        p=packet(30);p['detections'][0]['camera_geometry_valid']=False
        _,now=scene_process(state,f,p)
        self.assertEqual(len(state.observation.frames),before)
        self.assertFalse(state.observation.status(now)['observation_eligible'])
        for i in range(31,65):_,now=scene_process(state,f,packet(i))
        self.assertIsNotNone(state.observation.snapshot)
        self.assertTrue(state.observation.status(now)['observation_eligible'])
        self.assertLess(state.observation.snapshot['targets'][0]['valid_fraction'],1.)

    def test_last_sample_movement_cannot_hide_inside_p95_and_grant_eligibility(self):
        state=SceneState(base=FixtureBase());f=FeedbackWindow()
        # Tracker confirmation starts sampling at frame 2; frame 62 closes 2s.
        for i in range(62):scene_process(state,f,packet(i))
        self.assertIsNone(state.observation.snapshot)
        p=packet(62)
        p['detections'][0]['camera_surface_center_m'][0]+=.008
        _,now=scene_process(state,f,p)
        self.assertIsNone(state.observation.snapshot)
        self.assertFalse(state.observation.status(now)['observation_eligible'])
        self.assertEqual(state.observation.counts['lock_scene_mismatch'],1)

    def test_full_candidate_movement_survives_overlapping_old_roi_geometry(self):
        state=SceneState(base=FixtureBase());feedback=FeedbackWindow()
        e=Experiment.__new__(Experiment);e.camera_only=True;e.imgsz=640
        old=[500,300,550,350];moved=[522,300,572,350]
        for i in range(66):
            fullbox=old if i<65 else moved
            def detect(color,roi=None):
                return detector_packet([fullbox if roi is None else old],['red_pawn'],roi)
            detection=recognize(IMAGE,detect)
            t=100_000_000_000+round(i*1e9/30)
            meta=dict(K=[600.,0,640.,0,600.,360.,0,0,1.],stamp_ns=t,
                      camera_epoch='movement',rgbd_frame_key=['movement',i+1,i+1])
            r,_=e.run(IMAGE,np.full((720,1280),.2,np.float32),meta,draw=False,detection=detection)
            p=camera_measurement(r,meta,'movement',i,t)
            last,now=scene_process(state,feedback,p)
            if i==64:
                self.assertTrue(last['observation']['observation_eligible'])
                old_center=last['detections'][0]['camera_surface_center_m']
        self.assertEqual(last['detections'][0]['box_xyxy'],moved)
        self.assertGreater(last['detections'][0]['camera_surface_center_m'][0]-old_center[0],state.policy.scene_distance_m)
        self.assertFalse(last['observation']['observation_eligible'])
        self.assertEqual(last['detections'][0]['class_name'],'red_pawn')

    def multicrop(self, reverse=False):
        boxes=[[100,300,150,350],[620,300,670,350]]
        extra=[440,300,490,350]
        crops=plan_crops(boxes,1280,720)
        if reverse:crops=list(reversed(crops))
        def detect(color,roi=None):
            if roi is None:return detector_packet(boxes,['red_pawn']*2)
            visible=[];labels=[]
            for b in boxes+[extra]:
                a=max(roi[0],b[0]);top=max(roi[1],b[1]);c=min(roi[2],b[2]);bottom=min(roi[3],b[3])
                if c>a and bottom>top:
                    visible.append([a,top,c,bottom]);labels.append('red_pawn')
            return detector_packet(visible,labels,roi)
        with patch('candidate_roi.plan_crops',return_value=crops):
            return recognize(IMAGE,detect)

    def test_complete_roi_repairs_edge_fragment_in_both_crop_orders(self):
        for reverse in (False,True):
            result=self.multicrop(reverse=reverse)
            self.assertTrue(result['classification_complete'])
            self.assertEqual(len(result['predictions']),3)
            self.assertIn([440.,300.,490.,350.],result['predictions'][:,:4].tolist())

    def test_two_complete_roi_conflicts_remain_unknown_in_both_orders(self):
        # Both complete boxes must geometrically describe the same target.
        # Explicit duplicate crops isolate class evidence from crop planning.
        for reverse in (False,True):
            crops=[[300,100,780,580],[320,100,800,580]]
            if reverse:crops.reverse()
            def detect(color,roi=None):
                label='red_rook' if roi is not None and roi[0]==300 else 'red_pawn'
                return detector_packet(BOXES[:1],[label],roi)
            with patch('candidate_roi.plan_crops',return_value=crops):
                result=recognize(IMAGE,detect)
            self.assertFalse(result['classification_complete'])
            self.assertEqual(result['predictions'][0,-1],-1)

    def test_same_crop_complete_conflict_is_not_erased_by_later_clean_crop(self):
        for reverse in (False,True):
            crops=[[300,100,780,580],[320,100,800,580]]
            if reverse:crops.reverse()
            def detect(color,roi=None):
                if roi is not None and roi[0]==300:
                    return detector_packet([BOXES[0],BOXES[0]],['red_pawn','red_rook'],roi)
                return detector_packet(BOXES[:1],['red_pawn'],roi)
            with patch('candidate_roi.plan_crops',return_value=crops):
                result=recognize(IMAGE,detect)
            self.assertFalse(result['classification_complete'])
            self.assertEqual(result['predictions'][0,-1],-1)

    def wire_from_result(self,result):
        e=Experiment.__new__(Experiment);e.camera_only=True;e.imgsz=640
        t=100_000_000_000
        meta=dict(K=[600.,0,640.,0,600.,360.,0,0,1.],stamp_ns=t,
                  camera_epoch='provenance',rgbd_frame_key=['provenance',1,1])
        record,_=e.run(IMAGE,np.full((720,1280),.2,np.float32),meta,draw=False,detection=result)
        p=camera_measurement(record,meta,'provenance',0,t)
        validate(p,t+30_000_000)
        return p

    def provenance_result(self,reverse=False,conflict=False):
        fullboxes=[[100,300,150,350],[620,300,670,350],[380,300,450,350]]
        crops=plan_crops(fullboxes,1280,720)
        if reverse:crops.reverse()
        def detect(color,roi=None):
            frame=roi or [0,0,1280,720]
            source=fullboxes if roi is None else fullboxes[:2]+[[405,300,455,350]]
            boxes=[];labels=[]
            for i,b in enumerate(source):
                clipped=[max(frame[0],b[0]),max(frame[1],b[1]),min(frame[2],b[2]),min(frame[3],b[3])]
                if clipped[2]>clipped[0] and clipped[3]>clipped[1]:
                    boxes.append(clipped)
                    labels.append('red_rook' if conflict and roi is not None and roi[0]==400 and i==2 else 'red_pawn')
            result=detector_packet(boxes,labels,roi)
            if roi is not None and roi[0]==400:result['predictions'][:,-2]=.99
            return result
        with patch('candidate_roi.plan_crops',return_value=crops):
            return recognize(IMAGE,detect)

    def test_class_source_must_cover_retained_full_geometry_in_both_crop_orders(self):
        evidence=[]
        for reverse in (False,True):
            p=self.wire_from_result(self.provenance_result(reverse=reverse))
            self.assertTrue(p['classification_complete'])
            d=next(d for d in p['detections'] if d['box_xyxy']==[380,300,450,350])
            self.assertEqual(d['classification_roi_xyxy'],[0,80,480,560])
            self.assertEqual(d['confidence'],.9)
            evidence.append((d['box_xyxy'],d['classification_roi_xyxy'],d['confidence']))
        self.assertEqual(evidence[0],evidence[1])

    def test_no_class_source_covering_full_geometry_is_unresolved_valid_wire(self):
        fullbox=[380,300,450,350];roi=[400,80,880,560]
        def detect(color,crop=None):
            return detector_packet([fullbox if crop is None else [405,300,455,350]],['red_pawn'],crop)
        with patch('candidate_roi.plan_crops',return_value=[roi]):
            p=self.wire_from_result(recognize(IMAGE,detect))
        self.assertFalse(p['classification_complete'])
        self.assertEqual(len(p['detections']),1)
        d=p['detections'][0]
        self.assertEqual(d['box_xyxy'],fullbox)
        self.assertTrue(d['camera_geometry_valid'])
        self.assertEqual(d['class_name'],'unknown')
        self.assertFalse(d['classification_valid'])
        self.assertIsNone(d['classification_roi_xyxy'])

    def test_selecting_compatible_source_does_not_erase_complete_class_conflict(self):
        for reverse in (False,True):
            p=self.wire_from_result(self.provenance_result(reverse=reverse,conflict=True))
            self.assertFalse(p['classification_complete'])
            d=next(d for d in p['detections'] if d['box_xyxy']==[380,300,450,350])
            self.assertFalse(d['classification_valid'])
            self.assertEqual(d['class_name'],'unknown')


if __name__=='__main__':unittest.main(verbosity=2)
