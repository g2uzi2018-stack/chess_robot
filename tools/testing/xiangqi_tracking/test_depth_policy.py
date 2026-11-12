"""Depth/ROI integration regressions: no model/GPU, ROS or actuator access."""
import copy
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from candidate_roi import DEPTH_POLICY, load_depth_configuration, recognize_depth
from measurement_contract import ContractError, camera_measurement, validate
from recognize_tracked import Experiment
from shared_pipeline import FrameSlots, detect_slot, geometry_slot
from scene_state import SceneState
from stable_observation import FeedbackWindow
from test_candidate_roi import IMAGE, BOXES, CLASS_NAMES, detector_packet, scene_process
from test_node_split import FixtureBase, packet


K=[600.,0.,640.,0.,600.,360.,0.,0.,1.]
SHA='a'*64
IDENTITY=dict(candidate_mode='depth_shadow',depth_config_sha256=SHA,depth_qualification_mode='shadow')


def metadata(i=0):
    return dict(K=K,stamp_ns=100_000_000_000+i*33_333_333,
                camera_epoch='fixture',rgbd_frame_key=['fixture',i+1,i+1])


def configuration():
    return dict(config=dict(view_binding=dict(view_id='current-fixture-view',
                camera_epoch='fixture',image_size=[1280,720],K=K)),
                detector_config=object(),config_sha256=SHA)


def scan_result(boxes=BOXES, valid=True, coverage=True):
    candidates=[]
    for box in boxes:
        mask=np.zeros((720,1280),np.uint8)
        x,y,x2,y2=box
        # Deliberately different from the ROI segmentation to verify geometry
        # ownership rather than merely matching output shapes.
        mask[y+3:y2-3,x+3:x2-3]=1
        candidates.append(dict(box_xyxy=list(box),mask=mask,candidate_complete=True,roi_eligible=True))
    return dict(valid=valid,coverage_valid=coverage,candidates=candidates,board_regions=[],
                reasons=[] if valid else ['table reference rejected'],diagnostics={})


def recognize(scan=None,roi_boxes=BOXES,labels=None,meta=None,hint=None):
    calls=[]
    def detect(color,roi=None):
        if roi is None:
            raise AssertionError('depth policy must never run the full-image network')
        assert color is IMAGE
        calls.append(roi)
        return detector_packet(roi_boxes,labels or ['red_pawn']*len(roi_boxes),roi)
    with patch('depth_candidates.detect_depth_candidates',return_value=scan or scan_result()):
        result=recognize_depth(IMAGE,np.full((720,1280),.2,np.float32),meta or metadata(),
                               detect,CLASS_NAMES,IDENTITY,{},configuration(),hint)
    return result,calls


def wire_from_detection(detection,meta=None):
    meta=meta or metadata()
    e=Experiment.__new__(Experiment);e.camera_only=True;e.imgsz=640
    record,_=e.run(IMAGE,np.full((720,1280),.2,np.float32),meta,draw=False,detection=detection)
    return camera_measurement(record,meta,'fixture',0,meta['stamp_ns']+1)


class DepthPolicyTests(unittest.TestCase):
    def test_depth_geometry_and_one_roi_call_without_full_model(self):
        result,calls=recognize()
        self.assertEqual(len(calls),1)
        self.assertEqual(result['recognition_policy'],DEPTH_POLICY)
        self.assertEqual(result['depth_scan']['candidate_network_calls'],0)
        self.assertEqual(result['depth_scan']['roi_network_calls'],1)
        self.assertTrue(result['depth_scan']['coverage_valid'])
        self.assertFalse(result['full_scene_coverage'] or result['classification_complete'])
        self.assertEqual(result['predictions'][:,:4].tolist(),BOXES)
        self.assertEqual(int(result['masks'][0].sum()),44*44)
        self.assertTrue(all(d['classification_valid'] for d in result['classification_evidence']))
        p=wire_from_detection(result)
        validate(p,p['stamp_ns']+30_000_000)
        self.assertTrue(all(d['camera_geometry_valid'] for d in p['detections']))

    def test_roi_miss_retains_depth_candidate_unknown(self):
        result,_=recognize(roi_boxes=[])
        self.assertEqual(len(result['predictions']),2)
        self.assertEqual(result['predictions'][:,-1].tolist(),[-1.,-1.])
        self.assertTrue(np.any(result['masks'][0]))

    def test_roi_only_extra_is_unknown_without_workspace_admission(self):
        result,_=recognize(roi_boxes=BOXES+[[700,350,750,400]])
        self.assertEqual(result['depth_scan']['roi_extra_count'],1)
        self.assertEqual(result['predictions'][-1,-1],-1)
        self.assertFalse(result['classification_evidence'][-1]['classification_valid'])

    def test_incomplete_geometry_still_gets_roi_class_but_cannot_qualify_geometry(self):
        scan=scan_result();scan['candidates'][0]['candidate_complete']=False
        result,calls=recognize(scan)
        self.assertEqual(len(calls),1)
        self.assertEqual(result['predictions'][0,-1],0)
        p=wire_from_detection(result)
        self.assertTrue(p['detections'][0]['classification_valid'])
        self.assertFalse(p['detections'][0]['camera_geometry_valid'])
        validate(p,p['stamp_ns']+30_000_000)

    def test_only_roi_ineligible_candidates_do_not_spend_roi_network_calls(self):
        scan=scan_result()
        for candidate in scan['candidates']:
            candidate.update(candidate_complete=False,roi_eligible=False)
        result,calls=recognize(scan,hint=[300,100,780,580])
        self.assertEqual(calls,[])
        self.assertEqual(result['predictions'][:,-1].tolist(),[-1.,-1.])
        self.assertEqual(result['depth_scan']['candidate_count'],2)

    def test_class_conflict_remains_unknown(self):
        result,_=recognize(roi_boxes=[BOXES[0],BOXES[0],BOXES[1]],
                            labels=['red_pawn','red_rook','red_pawn'])
        self.assertEqual(result['predictions'][0,-1],-1)

    def test_invalid_depth_reference_does_not_run_hint_classification(self):
        result,calls=recognize(scan_result([],False,False),hint=[300,100,780,580])
        self.assertEqual(calls,[])
        self.assertFalse(result['depth_scan']['valid'])
        self.assertFalse(result['full_scene_coverage'])
        self.assertEqual(result['predictions'].shape,(0,6))
        p=wire_from_detection(result)
        validate(p,p['stamp_ns']+30_000_000)

    def test_changed_camera_or_intrinsics_prevents_scan_and_network(self):
        for change in ('epoch','intrinsics'):
            meta=metadata()
            if change=='epoch':meta['camera_epoch']='another-session'
            else:meta['K']=list(K);meta['K'][0]+=1
            with patch('depth_candidates.detect_depth_candidates') as scan:
                calls=[]
                def detect(*args):calls.append(args);raise AssertionError('unexpected network')
                result=recognize_depth(IMAGE,np.full((720,1280),.2,np.float32),meta,
                    detect,CLASS_NAMES,IDENTITY,{},configuration(),[300,100,780,580])
                scan.assert_not_called()
            self.assertEqual(calls,[])
            self.assertFalse(result['depth_scan']['view_binding_valid'])

    def test_shadow_wire_forgery_or_stale_depth_provenance_rejected(self):
        original=wire_from_detection(recognize()[0])
        changes=[lambda p:p.update(full_scene_coverage=True,classification_complete=True),
                 lambda p:p['depth_scan'].update(stamp_ns=p['stamp_ns']-1),
                 lambda p:p['depth_scan'].update(pose_binding_verified=True),
                 lambda p:p['depth_scan'].update(candidate_network_calls=1),
                 lambda p:p['depth_scan'].update(roi_network_calls=0),
                 lambda p:p['depth_scan'].update(config_sha256='b'*64),
                 lambda p:p['detections'][0].update(candidate_geometry_valid=False),
                 lambda p:p['detections'][0].update(roi_eligible=False),
                 lambda p:p.pop('depth_scan')]
        for change in changes:
            p=copy.deepcopy(original);change(p)
            with self.assertRaises(ContractError):validate(p,p['stamp_ns']+30_000_000)

    def test_shadow_never_seeds_snapshot_despite_correct_labels_and_geometry(self):
        state=SceneState(base=FixtureBase());feedback=FeedbackWindow()
        shadow=recognize()[0]['depth_scan']
        for i in range(65):
            p=packet(i)
            p.update(recognition_policy=DEPTH_POLICY,full_scene_coverage=False,
                     classification_complete=False,identity=dict(p['identity'],**IDENTITY))
            p['depth_scan']=dict(shadow,stamp_ns=p['stamp_ns'],candidate_count=4)
            for d in p['detections']:
                d.update(candidate_geometry_valid=True,candidate_geometry_source='aligned_depth',
                         candidate_geometry_reasons=[],roi_eligible=True)
            result,now=scene_process(state,feedback,p)
        self.assertIsNone(state.observation.snapshot)
        self.assertFalse(result['observation']['observation_eligible'])
        self.assertFalse(result['motion_ready'] or result['grasp_ready'])

    def test_shared_slot_passes_same_frame_depth_metadata_and_stamp(self):
        e=Experiment.__new__(Experiment);e.camera_only=True;e.imgsz=640
        e.candidate_mode='depth_shadow';e.depth_configuration=configuration()
        e.model=SimpleNamespace(names=CLASS_NAMES);e.identity=IDENTITY
        e.inference_backend='fixture';e.artifact_sha256='fixture'
        calls=[]
        def detect(color,roi=None):
            self.assertIsNotNone(roi);calls.append(roi)
            return detector_packet(BOXES,['red_pawn']*2,roi)
        e.detect=detect
        slots=FrameSlots(1280,720,count=1,max_masks=4)
        meta=metadata();job=dict(slot=0,job_id=7,metadata=meta,roi=None,draw=False)
        try:
            slots.write(0,7,meta['stamp_ns'],IMAGE,np.full((720,1280),.2,np.float32))
            def scan(depth,k,config):
                self.assertTrue(np.shares_memory(depth,slots.depths[0]))
                np.testing.assert_array_equal(k,np.array(K).reshape(3,3))
                self.assertIs(config,e.depth_configuration['detector_config'])
                return scan_result()
            with patch('depth_candidates.detect_depth_candidates',side_effect=scan),\
                 patch('shared_pipeline.worker_slots',slots),patch('shared_pipeline.worker_experiment',e):
                detected=detect_slot(job);record,_=geometry_slot(job,detected)
                self.assertEqual(record['depth_scan']['stamp_ns'],meta['stamp_ns'])
                self.assertEqual(record['candidate_scan_stamp_ns'],meta['stamp_ns'])
                self.assertEqual(len(calls),1)
                slots.headers[0][1]+=1
                with self.assertRaises(AssertionError):detect_slot(job)
        finally:slots.close()

    def test_missing_or_nonshadow_configuration_is_startup_error(self):
        with self.assertRaises(ValueError):load_depth_configuration(None)
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'config.json'
            for config in ({},dict(qualification_mode='production'),dict(qualification_mode='shadow')):
                path.write_text(json.dumps(config))
                with self.assertRaises(ValueError):load_depth_configuration(path)

    def test_real_depth_plane_and_two_objects_feed_roi_without_full_model(self):
        config=dict(qualification_mode='shadow',view_binding=dict(view_id='synthetic-view',
            camera_epoch='fixture',image_size=[1280,720],K=K),detector=dict(image_size=[1280,720],
            workspace_polygon_xy=[[100,50],[1100,50],[1100,650],[100,650]],
            table_seed_polygon_xy=[[200,100],[1000,100],[1000,200],[200,200]],
            min_object_height_m=.02,single_piece_diameter_m=.03))
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'config.json';path.write_text(json.dumps(config))
            cfg=load_depth_configuration(path)
            depth=np.full((720,1280),.3,np.float32)
            for x,y,x2,y2 in BOXES:depth[y:y2,x:x2]=.27
            calls=[]
            def detect(color,roi=None):
                self.assertIsNotNone(roi);calls.append(roi)
                return detector_packet(BOXES,['red_pawn']*2,roi)
            identity=dict(IDENTITY,depth_config_sha256=cfg['config_sha256'])
            result=recognize_depth(IMAGE,depth,metadata(),detect,CLASS_NAMES,identity,{},cfg)
            self.assertEqual(len(calls),1)
            self.assertTrue(result['depth_scan']['valid'])
            self.assertTrue(result['depth_scan']['coverage_valid'])
            self.assertEqual(result['depth_scan']['candidate_count'],2)
            self.assertEqual(result['predictions'][:,-1].tolist(),[0.,0.])
            self.assertAlmostEqual(result['depth_scan']['table_plane']['d_m'],.3,places=6)
            self.assertFalse(result['full_scene_coverage'])
            p=wire_from_detection(result)
            validate(p,p['stamp_ns']+30_000_000)
            # An incomplete configuration must fail at load, before inference.
            del config['detector']['table_seed_polygon_xy']
            path.write_text(json.dumps(config))
            with self.assertRaises(ValueError):load_depth_configuration(path)


if __name__=='__main__':unittest.main()
