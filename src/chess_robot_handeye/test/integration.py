#!/usr/bin/env python3
"""Synthetic peers only; separate ROS domain and no controller IO."""
import argparse
import csv
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import yaml

CASES = ['success', 'reject', 'no_motion', 'stale', 'no_vision',
         'camera_unhealthy', 'mismatch', 'cancel', 'plan',
         'cancel_moving_zero_velocity', 'cancel_duplicate_stamp',
         'cancel_backward_stamp', 'cancel_invalid_feedback',
         'cancel_unpaired_feedback']


def read_mapping(path):
    value = yaml.safe_load(path.read_text())
    if not isinstance(value, dict):
        raise RuntimeError(f'fixture emitted incomplete YAML mapping: {path}')
    return value


def stop_peer(peer):
    if peer.poll() is None:
        peer.send_signal(signal.SIGINT)
        try:
            peer.wait(timeout=5)
        except subprocess.TimeoutExpired:
            peer.terminate()
            peer.wait(timeout=5)
            raise RuntimeError('fixture required forced termination')
    return peer.returncode


def main():
    parser = argparse.ArgumentParser()
    for key in ['template', 'fixture', 'node']:
        parser.add_argument('--' + key, required=True)
    outputs = parser.add_mutually_exclusive_group(required=True)
    outputs.add_argument('--output')
    outputs.add_argument('--output-parent')
    parser.add_argument('--case', choices=CASES, default='success')
    args = parser.parse_args()
    if args.output_parent:
        parent = Path(args.output_parent).resolve()
        parent.mkdir(parents=True, exist_ok=True)
        root = parent / (args.case + '_' + uuid.uuid4().hex)
    else:
        root = Path(args.output).resolve()
    root.mkdir(exist_ok=False)
    n = read_mapping(Path(args.template))
    n['mode'] = 'plan' if args.case == 'plan' else 'execute'
    n['output_directory'] = str(root / 'session')
    n['execution'] = {'enabled': True, 'fixture': True, 'poses_confirmed': True}
    n['arm'].update(action='/simulation/arm/move',
                    state_topic='/simulation/arm/state',
                    joint_topic='/simulation/arm/joint_states',
                    parameter_node='/simulation/handeye/arm')
    n['vision'].update(image_topic='/simulation/vision/color/image_rect',
                       camera_info_topic='/simulation/vision/color/camera_info',
                       frame_info_topic='/simulation/vision/frame_info')
    n['board'].update(confirmed=True, squares_x=11, squares_y=8,
                      square_length_m=.015, marker_length_m=.01125,
                      dictionary_id=5, legacy_pattern=True)
    n['poses'] = [{'id': 'bootstrap', 'role': 'train',
                   'position_m': [0., 0., 0.],
                   'quaternion_xyzw': [0., 0., 0., 1.]}]
    n['quality'].update(stationary_dwell_s=.4, feedback_freshness_s=.2,
                         maximum_feedback_gap_s=.15, pair_tolerance_s=.05,
                         minimum_rotation_rad=.03, validation_translation_m=.02,
                         validation_rotation_rad=.1, settle_timeout_s=3.,
                         capture_timeout_s=2.,
                         action_timeout_s=1. if args.case.startswith('cancel') else 5.)
    if args.case == 'cancel_unpaired_feedback':
        n['quality']['pair_tolerance_s'] = .002
    config = root / 'fixture.yaml'
    config.write_text(yaml.safe_dump(n))
    env = dict(os.environ, ROS_DOMAIN_ID='119', ROS_LOCALHOST_ONLY='1')
    summary = None
    with (root / 'fixture.log').open('w') as log:
        peer = subprocess.Popen([args.fixture, str(config), args.case],
                                stdout=log, stderr=subprocess.STDOUT, env=env)
        try:
            ready_path = Path(str(config) + '.ready.yaml')
            deadline = time.monotonic() + 5
            while True:
                if peer.poll() is not None:
                    raise RuntimeError('fixture initialization exited; inspect fixture.log')
                if ready_path.exists():
                    ready = read_mapping(ready_path)
                    generated = read_mapping(config)
                    if ready.get('ready') is True and ready.get('poses') == 12 and len(generated.get('poses', [])) == 12:
                        break
                    raise RuntimeError('fixture readiness/config contract invalid')
                if time.monotonic() >= deadline:
                    raise RuntimeError('fixture initialization timed out before ready handshake')
                time.sleep(.01)
            with (root / 'node.log').open('w') as out:
                run = subprocess.run([args.node, '--ros-args', '-p',
                                      f'config_file:={config}'],
                                     stdout=out, stderr=subprocess.STDOUT,
                                     env=env, timeout=50)
            report = read_mapping(root / 'session/session.yaml')
            original = read_mapping(root / 'session/config.input.yaml')
            resolved = read_mapping(root / 'session/config.yaml')
            assert original == read_mapping(config)
            assert Path(resolved['model']['urdf']).is_absolute()
            assert Path(resolved['model']['urdf']).is_file()
            assert resolved['output_directory'] == str(root / 'session')
            assert report['resolved_urdf'] == resolved['model']['urdf']
            stats = read_mapping(Path(str(config) + '.stats.yaml'))
            assert stats['invalid_goals'] == 0, stats
            assert report['fixture'] and not report['production_tf_published']
            if args.case == 'success':
                assert run.returncode == 0 and report['accepted'], report
                assert report['training_count'] == 8 and report['validation_count'] == 4
                assert stats['goals_received'] == 12
                assert len(list((root / 'session').glob('fixture_*/color.png'))) == 12
            elif args.case == 'plan':
                # The fixture intentionally publishes NO vision in plan mode.
                assert not report['accepted'] and stats['goals_received'] == 12
                assert not list((root / 'session').glob('fixture_*/color.png'))
            else:
                assert run.returncode != 0 and report['status'] == 'failed' and not report['accepted'], report
                if args.case in ['stale', 'no_vision', 'camera_unhealthy', 'mismatch']:
                    assert stats['goals_received'] == 0, stats
                    if args.case != 'stale':
                        assert report['failure'].startswith('vision preflight failed'), report
                else:
                    assert stats['goals_received'] == 1, stats
                if args.case.startswith('cancel'):
                    assert stats['cancels_received'] == 1, stats
                    assert report['physical_stop_confirmed'] is (args.case == 'cancel'), report
                    proof = root / 'session/stop_proof.csv'
                    assert proof.exists() and proof.stat().st_size > 100
                    if args.case != 'cancel':
                        assert report['stop_confirmation_failure']
                        if args.case != 'cancel_unpaired_feedback':
                            assert report['stop_rejection_counts']['callback_feedback_anomaly'] > 0
                    if args.case == 'cancel_moving_zero_velocity':
                        drift = [line.split(',') for line in Path(str(config) + '.drift.csv').read_text().splitlines()]
                        assert len(drift) >= 10
                        speed = (float(drift[-1][1]) - float(drift[0][1])) / ((int(drift[-1][0]) - int(drift[0][0])) * 1e-9)
                        assert speed > .1, speed
                        with proof.open() as evidence:
                            zero_velocity = [r for r in csv.DictReader(evidence) if float(r['v1']) == 0.]
                        assert max(float(r['q1']) for r in zero_velocity) - min(float(r['q1']) for r in zero_velocity) > .1
            summary = {'case': args.case, 'status': 'PASS',
                       'returncode': run.returncode,
                       'goals': stats['goals_received'],
                       'session_status': report['status'],
                       'failure': report.get('failure'),
                       'physical_stop_confirmed': report.get('physical_stop_confirmed')}
        finally:
            peer_code = stop_peer(peer)
        assert peer_code == 0, f'fixture teardown failed: {peer_code}'
    (root / 'result.yaml').write_text(yaml.safe_dump(summary))
    print(yaml.safe_dump(summary))


if __name__ == '__main__':
    main()
