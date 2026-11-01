"""Targeted independent-review reproductions; CPU and isolated mocked ROS only."""
import argparse
import ast
import collections
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import types
import unittest

import measurement_contract as wire
from scene_state import SceneState
from stable_observation import FeedbackWindow
from test_node_split import FixtureBase, T, arm, feed, joint, packet

HERE = Path(__file__).resolve().parent


class SwitchBase(FixtureBase):
    good = True
    def base_transform(self, metadata):
        if not self.good:
            return None, None, 'fixture FK/TCP mismatch'
        return super().base_transform(metadata)


def locked():
    base = SwitchBase()
    state = SceneState(base=base)
    feedback = FeedbackWindow()
    for i in range(65):
        p = packet(i); now = p['stamp_ns']+30_000_000
        feed(feedback, now); state.process(p, now, feedback, True, 'controller-one')
    assert state.observation.snapshot is not None
    return state, feedback, base, now


def callback(filename, name):
    tree = ast.parse((HERE/filename).read_text())
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    namespace = dict(json=json, lock=threading.RLock(), hints=collections.deque(maxlen=1),
                     requests=collections.deque(maxlen=32), rejects=collections.Counter(),
                     input_rejections=collections.Counter(), ContractError=wire.ContractError,
                     decode_json=getattr(wire, 'decode_json', wire.decode))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn], type_ignores=[])), filename, 'exec'), namespace)
    return namespace[name], namespace


class ReviewFixTests(unittest.TestCase):
    def test_fk_failure_pauses_existing_snapshot_and_good_base_restores(self):
        state, feedback, base, now = locked()
        snapshot = copy.deepcopy(state.observation.snapshot)
        base.good = False
        for i in (65, 66, 67):
            p = packet(i); now = p['stamp_ns']+30_000_000; feed(feedback, now)
            r = state.process(p, now, feedback, True, 'controller-one')
            self.assertTrue(r['camera_geometry_valid']); self.assertFalse(r['base_transform_valid'])
            self.assertFalse(state.observation.status(now)['observation_eligible'])
            self.assertFalse(state.observation.request(dict(action='consume', snapshot_id=snapshot['snapshot_id'], operation_id='blocked'), now)['ok'])
            self.assertEqual(state.observation.request(dict(action='query'), now)['snapshot'], snapshot)
        base.good = True
        p = packet(68); now = p['stamp_ns']+30_000_000; feed(feedback, now)
        state.process(p, now, feedback, True, 'controller-one')
        self.assertTrue(state.observation.status(now)['observation_eligible'])
        self.assertEqual(state.observation.snapshot, snapshot)

    def test_invalid_frame_wrong_identity_rejects_before_sequence_commit(self):
        s = SceneState(base=FixtureBase()); f = FeedbackWindow(); now = T+30_000_000; feed(f, now)
        p = packet(); p.update(detection_valid=False, camera_geometry_valid=False, detections=[], identity=[['a','b','c']])
        with self.assertRaises(wire.ContractError):s.process(p, now, f, True, 'controller-one')
        self.assertIsNone(s.guard.epoch)
        self.assertTrue(s.process(packet(), now, f, True, 'controller-one')['base_transform_valid'])

    def test_invalid_frame_nonfinite_metadata_rejects(self):
        p = packet(); p.update(detection_valid=False, camera_geometry_valid=False, detections=[], identity={'bad':float('inf')})
        with self.assertRaises(wire.ContractError):wire.validate(p, T+30_000_000)

    def test_overflow_float_json_rejected(self):
        text = json.dumps(packet()).replace('"identity": {"model": "fixture"}', '"identity": {"bad": 1e999}')
        self.assertIn('1e999', text)
        with self.assertRaises(wire.ContractError):wire.decode(text)

    def test_live_stage_types_reject_before_state_mutation(self):
        for value in (['abc'], {'geometry_stage':'bad'}, {'analysis':None}, {'tracking':float('inf')}):
            s = SceneState(base=FixtureBase()); f = FeedbackWindow(); now = T+30_000_000; feed(f, now)
            p = packet(); p['live_stage_ms'] = value
            with self.assertRaises(wire.ContractError):s.process(p, now, f, True, 'controller-one')
            self.assertIsNone(s.guard.epoch)
            self.assertTrue(s.process(packet(), now, f, True, 'controller-one')['base_transform_valid'])

    def test_arithmetic_fields_are_finite_numbers(self):
        for key, value in [('geometry_ms','bad'),('processing_s',None),('perception_publish_ns',float('inf'))]:
            p = packet(); p[key] = value
            with self.assertRaises(wire.ContractError):wire.validate(p, T+30_000_000)

    def test_zero_confidence_rejected_without_fabricated_votes(self):
        for confidence in (0.,10**1000):
            p = packet(); p['detections'][0]['confidence'] = confidence
            p = json.loads(json.dumps(p))
            with self.assertRaises(wire.ContractError):wire.validate(p, T+30_000_000)

    def test_invalid_nan_camera_intrinsics_publish_safe_metadata(self):
        p = packet(); p.update(detection_valid=False, camera_geometry_valid=False, detections=[])
        metadata = dict(K=[float('nan')]*9, camera_epoch='safe', rgbd_frame_key=['safe',1,1])
        result = wire.camera_measurement(p, metadata, 'safe', 0, T+30_000_000)
        self.assertIsNone(result['source_K'])
        wire.validate(result, T+40_000_000)

    def test_invalid_image_does_not_poison_model_latch_or_snapshot(self):
        s, f, base, now = locked(); old = copy.deepcopy(s.observation.snapshot)
        p = packet(65); p.update(detection_valid=False, camera_geometry_valid=False, detections=[], source_K=None)
        p.pop('identity', None); p.pop('compute', None)
        now = p['stamp_ns']+30_000_000; feed(f, now); s.process(p, now, f, True, 'controller-one')
        self.assertFalse(s.source_configuration_fault)
        self.assertFalse(s.observation.status(now)['observation_eligible'])
        p = packet(66); now = p['stamp_ns']+30_000_000; feed(f, now); s.process(p, now, f, True, 'controller-one')
        self.assertTrue(s.observation.status(now)['observation_eligible'])
        self.assertEqual(s.observation.snapshot, old)

    def test_image_must_itself_follow_settle_interval(self):
        f = FeedbackWindow()
        for i in range(13):
            t = T+i*50_000_000; f.ingest(t, arm(), joint(), t, 'controller-one')
        self.assertFalse(f.at(T+100_000_000, T+600_000_000)['ok'])
        self.assertTrue(f.at(T+600_000_000, T+600_000_000)['ok'])

    def test_latest_feedback_must_also_have_matured(self):
        f = FeedbackWindow()
        for i in range(11):
            t = T+i*50_000_000; f.ingest(t, arm(), joint(), t, 'controller-one')
        self.assertFalse(f.at(T+600_000_000, T+600_000_000)['ok'])

    def test_empty_measurements_do_not_renew_old_roi_evidence(self):
        s, f, base, now = locked(); old = s.last_measurement
        for i, delay in [(65, 100_000_000),(66, 600_000_000)]:
            t = old['stamp_ns']+delay
            p = packet(i, timestamp=t); p.update(detections=[], camera_geometry_valid=False)
            now = t+30_000_000; feed(f, now); s.process(p, now, f, True, 'controller-one')
            hint = s.hint(now)
            if delay < 500_000_000:
                self.assertFalse(hint['full_image'])
                self.assertEqual(hint['based_on_sequence'], old['sequence'])
                self.assertEqual(hint['based_on_stamp_ns'], old['stamp_ns'])
            else:
                self.assertTrue(hint['full_image'])

    def test_deep_roi_does_not_escape_callback(self):
        fn, ns = callback('perception_node.py', 'receive_hint')
        fn(types.SimpleNamespace(data='['*1100+'0'+']'*1100))
        self.assertEqual(len(ns['hints']), 0)

    def test_bad_requests_reject_and_valid_query_recovers(self):
        fn, ns = callback('scene_node.py', 'receive_request')
        for text in ['['*1100+'0'+']'*1100, '{"action":"query","request_id":NaN}',
                     '{"action":"query","request_id":1e999}']:
            fn(types.SimpleNamespace(data=text))
        fn(types.SimpleNamespace(data='{"action":"query","request_id":"good"}'))
        self.assertEqual(list(ns['requests']), [dict(action='query', request_id='good')])

    def test_common_decoder_depth_and_escaped_strings(self):
        decoder = getattr(wire, 'decode_json', wire.decode)
        text = '{"text":"brackets [ { and escaped quote \\\" are text"}'
        self.assertEqual(decoder(text)['text'], 'brackets [ { and escaped quote " are text')
        with self.assertRaises(wire.ContractError):decoder('{"x":'+'['*64+'0'+']'*64+'}')

    def test_spin_thread_failure_exits_both_node_parents(self):
        for module in ('perception_node','scene_node'):
            with tempfile.TemporaryDirectory() as tmp:
                code = "import sys;sys.path.insert(0,"+repr(str(HERE))+ ");import "+module+" as n\n"
                if module == 'perception_node':
                    code += "class P:\n def __init__(self,**kw):pass\n def poll(self):return []\n def busy(self):return False\n def can_submit(self):return True\n def close(self):pass\nn.Pipeline=P\n"
                else:
                    code += "from test_node_split import FixtureBase;n.BaseGeometry=FixtureBase\n"
                code += "n.rclpy.spin=lambda node:(_ for _ in ()).throw(RuntimeError('injected spin failure'));n.main()"
                env = dict(os.environ, ROS_DOMAIN_ID='121', ROS_LOCALHOST_ONLY='1', XIANGQI_RUN_DIR=tmp,
                           XIANGQI_FEEDBACK_AUDIT=str(Path(tmp)/'no-real-audit'))
                completed = subprocess.run([sys.executable,'-c',code], env=env, capture_output=True, text=True, timeout=4)
                self.assertNotEqual(completed.returncode, 0)
                self.assertIn('spin', completed.stdout+completed.stderr)


if __name__ == '__main__':
    p = argparse.ArgumentParser();p.add_argument('--output', type=Path, required=True);a = p.parse_args()
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(ReviewFixTests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    report = dict(passed=result.wasSuccessful(), tests_run=result.testsRun,
                  failures=[dict(test=str(test),trace=trace) for test,trace in result.failures],
                  errors=[dict(test=str(test),trace=trace) for test,trace in result.errors],
                  physical_commands=0)
    a.output.write_text(json.dumps(report, indent=2));sys.exit(0 if result.wasSuccessful() else 1)
