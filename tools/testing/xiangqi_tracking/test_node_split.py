"""CPU-only source/scene boundary regressions, with no controller or ROS calls."""
import ast
import copy
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from measurement_contract import (ContractError, MAX_MESSAGE_BYTES, RoiHint, SequenceGuard,
                                  camera_measurement, decode, encode, validate)
from piece_tracker import PieceTracker
from scene_state import SceneState
from stable_observation import FeedbackWindow, StableObservation
from test_stable_observation import T, arm, frame, joint
from candidate_roi import POLICY


class FixtureBase:
    identity = dict(calibration='fixture', calibration_sha256='fixture-calibration', urdf_sha256='fixture-urdf')

    def base_transform(self, metadata):
        return (np.eye(4), 0., 'fixture FK') if metadata.get('base_eligible') else (None, None, metadata.get('base_reason'))


def packet(i=0, epoch='source-one', timestamp=None, roi=None):
    r = frame(i)
    t = r['stamp_ns'] if timestamp is None else timestamp
    r.update(stamp_ns=t, image_size=[1280, 720], inference_roi_xyxy=roi or [0, 0, 1280, 720],
             compute=dict(inference_backend='fixture', fp16=False), detector_ms=0., geometry_ms=0., processing_s=0.,
             recognition_policy=POLICY, full_scene_coverage=True, classification_complete=True,
             candidate_scan_stamp_ns=t, full_candidate_count=4, classification_rois=[[0,0,720,720]])
    for n, d in enumerate(r['detections']):
        d.update(id=n, class_id=0, detection_valid=True, camera_geometry_valid=True, center_pixel=[280+n*120, 330],
                 box_xyxy=[250+n*120, 300, 310+n*120, 360], touches_image_edge=False,
                 touches_processing_edge=False, mask_outline_xy=[[250+n*120, 300], [310+n*120, 360]])
        d.update(classification_valid=True, classification_source='roi', classification_stamp_ns=t,
                 classification_roi_xyxy=[0,0,720,720])
    meta = dict(K=r['source_K'], camera_epoch=epoch, rgbd_frame_key=[epoch, i+1, i+1])
    return camera_measurement(r, meta, epoch, i, t)


def feed(feedback, now):
    start = T-1_000_000_000
    last = max(feedback.last+50_000_000, start) if feedback.last else start
    while last <= now:
        feedback.ingest(last, arm(), joint(), last, 'controller-one')
        last += 50_000_000


class SplitTests(unittest.TestCase):
    def scene(self):
        return SceneState(base=FixtureBase(), session='scene-one')

    def test_measurement_has_no_robot_or_stable_identity(self):
        p = packet()
        validate(p, T+30_000_000)
        self.assertNotIn('base_frame_id', p)
        self.assertNotIn('track_namespace', p)
        for d in p['detections']:
            self.assertNotIn('track_id', d)
            self.assertNotIn('base_surface_center_m', d)
            self.assertIn('instance_index', d)
        self.assertEqual(p['stamp_ns'], T)

    def test_source_module_has_no_arm_subscription_or_audit_import(self):
        text = Path(__file__).with_name('perception_node.py').read_text()
        tree = ast.parse(text)
        imports = [n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)]
        self.assertNotIn('chess_robot_interfaces.msg', imports)
        self.assertNotIn('stable_observation', imports)
        self.assertNotIn('/arm/', text)
        self.assertNotIn('AuditEpoch', text)
        self.assertNotIn('base_geometry', text)

    def test_no_arm_keeps_camera_geometry_and_revokes_base(self):
        state = self.scene()
        result = state.process(packet(), T+30_000_000, FeedbackWindow(), False, 'controller-one')
        self.assertTrue(result['detection_valid'] and result['camera_geometry_valid'])
        self.assertFalse(result['base_transform_valid'])
        self.assertTrue(all(d['camera_geometry_valid'] and d['track_id'] is None for d in result['detections']))
        self.assertFalse(state.observation.status(T+30_000_000)['observation_eligible'])

    def test_original_camera_and_base_geometry_and_stable_ids(self):
        state = self.scene()
        f = FeedbackWindow()
        for i in range(5):
            p = packet(i)
            now = p['stamp_ns']+30_000_000
            feed(f, now)
            r = state.process(p, now, f, True, 'controller-one')
            for d, original in zip(r['detections'], frame(i)['detections']):
                np.testing.assert_array_equal(d['camera_surface_center_m'], original['camera_surface_center_m'])
                np.testing.assert_array_equal(d['base_surface_center_m'], original['base_surface_center_m'])
            self.assertTrue(r['base_transform_valid'])
        self.assertEqual(len(state.tracker.tracks), 4)
        self.assertTrue(all(d['track_confirmed'] for d in r['detections']))

    def test_two_second_snapshot_and_single_consumption(self):
        state = self.scene()
        f = FeedbackWindow()
        for i in range(65):
            p = packet(i)
            now = p['stamp_ns']+30_000_000
            feed(f, now)
            state.process(p, now, f, True, 'controller-one')
        s = state.observation.snapshot
        self.assertIsNotNone(s)
        self.assertGreaterEqual(s['unique_frames'], 60)
        self.assertEqual(len(s['targets']), 4)
        self.assertGreaterEqual(s['sample_end_ns']-s['sample_start_ns'], 2_000_000_000)
        cmd = dict(action='consume', operation_id='one', snapshot_id=s['snapshot_id'])
        self.assertTrue(state.observation.request(cmd, now)['ok'])
        self.assertTrue(state.observation.request(cmd, now)['idempotent'])
        self.assertFalse(state.observation.request(dict(cmd, operation_id='two'), now)['ok'])
        self.assertFalse(s['motion_ready'] or s['grasp_ready'])

    def test_full_frame_required_after_collect(self):
        state = self.scene(); f = FeedbackWindow()
        for i in range(65):
            p = packet(i); p.update(full_scene_coverage=False,classification_complete=False)
            now = p['stamp_ns']+30_000_000
            feed(f, now); state.process(p, now, f, True, 'controller-one')
        self.assertIsNone(state.observation.snapshot)
        self.assertEqual(state.observation.frames, [])
        self.assertTrue(state.hint(now)['full_image'])

    def test_duplicates_reorder_old_epoch_and_duplicate_rgbd(self):
        g = SequenceGuard(); g.accept(packet(1), packet(1)['stamp_ns'])
        for p in [packet(1), packet(0)]:
            with self.assertRaises(ContractError): g.accept(p, packet(1)['stamp_ns'])
        p = packet(2); p['rgbd_frame_key'] = packet(1)['rgbd_frame_key']
        with self.assertRaises(ContractError):g.accept(p, p['stamp_ns'])
        g.accept(packet(3, epoch='source-two'), packet(3)['stamp_ns'])
        with self.assertRaises(ContractError):g.accept(packet(4), packet(4)['stamp_ns'])

    def test_perception_restart_retires_tracks_and_snapshots(self):
        state = self.scene(); f = FeedbackWindow()
        for i in range(65):
            p = packet(i); now = p['stamp_ns']+30_000_000; feed(f, now)
            state.process(p, now, f, True, 'controller-one')
        old_ids = set(state.tracker.tracks)
        old_sid = state.observation.snapshot['snapshot_id']
        p = packet(0, epoch='restarted', timestamp=now+33_000_000)
        now = p['stamp_ns']+20_000_000; feed(f, now)
        state.process(p, now, f, True, 'controller-one')
        self.assertFalse(old_ids & set(state.tracker.tracks))
        self.assertFalse(state.observation.status(now)['observation_eligible'])
        self.assertEqual(old_sid, state.observation.snapshot['snapshot_id'])

    def test_source_loss_does_not_renew_stamp_or_leave_xyz(self):
        state = self.scene(); f = FeedbackWindow(); now = T+30_000_000; feed(f, now)
        state.process(packet(), now, f, True, 'controller-one')
        now += 600_000_000; feed(f, now); state.tick(now, f, True, 'controller-one')
        r = state.expired_record(now)
        self.assertEqual(r['stamp_ns'], T)
        self.assertFalse(r['detection_valid'] or r['base_transform_valid'])
        self.assertFalse(state.observation.status(now)['observation_eligible'])
        self.assertNotIn('camera_surface_center_m', r['detections'][0])
        self.assertNotIn('base_surface_center_m', r['detections'][0])

    def test_configuration_change_latches_until_new_source(self):
        state = self.scene(); f = FeedbackWindow()
        p = packet(); now = T+30_000_000; feed(f, now); state.process(p, now, f, True, 'controller-one')
        p = packet(1); p['identity']['model'] = 'different'
        now = p['stamp_ns']+30_000_000; feed(f, now)
        r = state.process(p, now, f, True, 'controller-one')
        self.assertFalse(r['base_transform_valid'])
        self.assertTrue(state.source_configuration_fault)
        p = packet(2); now = p['stamp_ns']+30_000_000; feed(f, now)
        self.assertFalse(state.process(p, now, f, True, 'controller-one')['base_transform_valid'])
        p = packet(0, epoch='new', timestamp=now+30_000_000); now = p['stamp_ns']+10_000_000; feed(f, now)
        self.assertTrue(state.process(p, now, f, True, 'controller-one')['base_transform_valid'])

    def test_scene_configuration_change_cannot_grant_base(self):
        state = self.scene(); state.configuration_fault = True
        f = FeedbackWindow(); now = T+30_000_000; feed(f, now)
        r = state.process(packet(), now, f, True, 'controller-one')
        self.assertTrue(r['camera_geometry_valid']); self.assertFalse(r['base_transform_valid'])

    def test_expired_future_invalid_and_unbounded_messages_reject(self):
        p = packet()
        for now in [T-1, T+500_000_001]:
            with self.assertRaises(ContractError):validate(p, now)
        for mutate in [lambda p:p.update(base_transform_valid=True),
                       lambda p:p.update(masks=[]),
                       lambda p:p['detections'][0].update(confidence=None),
                       lambda p:p['detections'][0].update(camera_surface_normal=[0,0,0]),
                       lambda p:p.update(source_K=[0]*9)]:
            q = copy.deepcopy(p); mutate(q)
            with self.assertRaises(ContractError):validate(q, T)
        with self.assertRaises(ContractError):decode('x'*(MAX_MESSAGE_BYTES+1))
        with self.assertRaises(ContractError):decode('{"x":NaN}')

    def test_outline_bound_does_not_change_geometry(self):
        r = frame(0); d = r['detections'][0]
        d['id'] = 0; d['mask_outline_xy'] = [[i, i] for i in range(1000)]
        r['detections'] = [d]
        p = camera_measurement(r, dict(K=r['source_K'], camera_epoch='one', rgbd_frame_key=['one',1,1]), 'one', 0, T)
        self.assertEqual(len(p['detections'][0]['mask_outline_xy']), 128)
        self.assertEqual(p['detections'][0]['camera_surface_center_m'], d['camera_surface_center_m'])

    def test_roi_hint_fallback_expiry_source_reset_and_reorder(self):
        state = self.scene(); f = FeedbackWindow(); now = T+30_000_000; feed(f, now)
        state.process(packet(), now, f, True, 'controller-one')
        hint = state.hint(now); hint.update(full_image=False, roi_xyxy=[0, 0, 720, 720])
        control = RoiHint()
        self.assertIsNone(control.roi(now, 'source-one'))
        self.assertTrue(control.receive(hint, now, 'source-one', [(0,T)]))
        self.assertEqual(control.roi(now, 'source-one'), [0,0,720,720])
        self.assertIsNone(control.roi(now+301_000_000, 'source-one'))
        self.assertIsNone(control.roi(now, 'restarted'))
        old = dict(hint, issued_ns=hint['issued_ns']-1)
        self.assertFalse(control.receive(old, now, 'source-one', [(0,T)]))
        self.assertFalse(control.receive(hint, now, 'source-one', [(99,T)]))
        hint['full_image'] = True
        self.assertTrue(control.receive(hint, now, 'source-one', [(0,T)]))
        self.assertIsNone(control.roi(now, 'source-one'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
