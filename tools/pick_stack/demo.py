#!/usr/bin/env python3
"""Finite scan -> fresh pick -> bare-table stack orchestration.

No ROS imports here. Historical scan coordinates are search hints only.
"""
import argparse
import json
import math
import copy
import hashlib
from pathlib import Path


def distance(a, b):
    return math.dist(a, b)


def top_z(config, layers):
    return config['table_z_m'] + layers * config['piece_thickness_m']


def validate_config(config):
    if config['schema'] != 'xiangqi_pick_stack_config/v1' or not 1 <= config['max_pieces'] <= 8:
        raise ValueError('expected v1 config and finite batch of 1..8 pieces')
    if config['table_z_m'] != -.010 or config['piece_thickness_m'] != .013:
        raise ValueError('this demo is bound to the confirmed -10mm table and 13mm piece')
    if not 20 <= config['force_percent'] <= 30:
        raise ValueError('force must remain within current site 20..30 percent limit')


def calibration_fingerprint(config):
    bound={k:v for k,v in config.items() if k!='commissioning_review_file'}
    return hashlib.sha256(json.dumps(bound,sort_keys=True,allow_nan=False).encode()).hexdigest()


def execution_blockers(config):
    validate_config(config)
    missing=['unmeasured: '+k for k,v in config['measured'].items() if v is None]
    if not config.get('commissioning_review_file'):
        missing.append('calibration-bound contact/payload/stack review artifact unavailable')
    m=config['measured']
    for key in ('surface_to_nominal_tcp_base_m','surface_to_observation_tcp_base_m','stack_stability_observation_pose'):
        if m[key] is not None and (len(m[key])!=3 or not all(math.isfinite(x) for x in m[key])):
            missing.append('expected finite base-frame xyz in metres: '+key)
    for key in ('open_raw','close_raw'):
        if m[key] is not None and (type(m[key]) is not int or not 0<=m[key]<=1000):
            missing.append('expected calibrated integer raw 0..1000: '+key)
    if m['piece_radius_m'] is not None and not 0 < m['piece_radius_m'] <= .020:
        missing.append('piece radius exceeds current scan design envelope (0..20mm)')
    return missing


def select_target(config, candidates, retired):
    """Select a search seed, never merge scan IDs into physical identities."""
    stack = config['stack_xy_m']
    for seed in sorted(candidates, key=lambda c: distance(c['base_surface_center_m'][:2], stack)):
        p = seed['base_surface_center_m']
        if seed['class_name'] == 'unknown' or seed['association_state'] != 'within_view_unique':
            continue
        if distance(p[:2], stack) <= config['stack_exclusion_radius_m']:
            continue
        if any(distance(p[:2], q[:2]) <= config['retired_radius_m'] for q in retired):
            continue
        # Nearby hypotheses (including unknown) remain unresolved. Exact nearby
        # duplicates may also block a seed; fresh observation must resolve them.
        if any(c is not seed and c['scan_target_id'] != seed['scan_target_id'] and
               distance(p[:2], c['base_surface_center_m'][:2]) < config['neighbor_clearance_m']
               for c in candidates):
            continue
        return seed
    raise RuntimeError('no isolated search seed; retain ambiguous/unknown hypotheses for review')


def qualified_target(config, seed, frames):
    """Check a contiguous fresh stationary window; do not reuse track_id."""
    points = []
    frame_keys = set()
    identity = None
    stamps = []
    for r in frames:
        if not (r['detection_valid'] and r['base_transform_valid'] and r['full_scene_coverage']):
            raise RuntimeError('unqualified scene window')
        key = tuple(r['rgbd_frame_key'])
        if key in frame_keys:
            raise RuntimeError('duplicate RGBD frame')
        frame_keys.add(key)
        signature = (r['stationary_generation'], r['controller_epoch'], r['track_namespace'],
                     json.dumps(dict(identity=r['identity'], K=r['source_K']), sort_keys=True), r['source_epoch'])
        if identity is None:
            identity = signature
        if signature != identity:
            raise RuntimeError('scene epoch/configuration changed')
        stamps.append(r['stamp_ns'])
        detections = r['detections']
        # An unlocated observation could be a neighbor hidden by missing depth.
        if any(not d.get('base_geometry_valid') or not d.get('complete_center_target')
               for d in detections):
            raise RuntimeError('unlocated/incomplete observation in target view')
        matches = [d for d in detections if distance(d['base_surface_center_m'],
                   seed['base_surface_center_m']) <= config['reassociate_radius_m']]
        if len(matches) != 1:
            raise RuntimeError('target absent, displaced or association ambiguous')
        d = matches[0]
        if (not d['classification_valid'] or d['class_name'] != seed['class_name'] or
                d.get('identity_ambiguous')):
            raise RuntimeError('target label unresolved/changed')
        if any(other is not d and distance(other['base_surface_center_m'][:2],
               d['base_surface_center_m'][:2]) < config['neighbor_clearance_m']
               for other in detections):
            raise RuntimeError('neighbor/unknown too close for reviewed finger envelope')
        points.append(d['base_surface_center_m'])
    if len(points) < 20 or (stamps[-1]-stamps[0])*1e-9 < 2:
        raise RuntimeError('need at least 20 independent frames spanning two seconds')
    if any(not 0 < (b-a)*1e-9 <= .3 for a, b in zip(stamps, stamps[1:])):
        raise RuntimeError('scene frame gap')
    center = [sum(p[i] for p in points)/len(points) for i in range(3)]
    if max(distance(p, center) for p in points) > .003:
        raise RuntimeError('target moved / repeatability exceeds 3mm')
    return dict(base_surface_center_m=center, class_name=seed['class_name'],
                first_stamp_ns=stamps[0], last_stamp_ns=stamps[-1],
                samples=len(points), scene_identity=identity)


def target_poses(config, measured, layers):
    """Offsets are measured at the one fixed reviewed orientation, in base axes."""
    m = config['measured']
    p = measured['base_surface_center_m']
    grasp = [a+b for a, b in zip(p, m['surface_to_nominal_tcp_base_m'])]
    place_top = [*config['stack_xy_m'], top_z(config, layers+1)]
    place = [a+b for a, b in zip(place_top, m['surface_to_nominal_tcp_base_m'])]
    lift_z = max(grasp[2], place[2]) + config['vertical_clearance_m']
    return dict(observe=[a+b for a, b in zip(p, m['surface_to_observation_tcp_base_m'])],
                grasp=grasp, pregrasp=[grasp[0], grasp[1], lift_z],
                place=place, preplace=[place[0], place[1], lift_z])


class Demo:
    """One finite batch, no automatic resume/retry of grip or motion."""
    def __init__(self, config, emit):
        validate_config(config)
        self.c = config
        self.emit = emit
        self.layers = 0
        self.retired = []
        self.object_state = 'empty'

    def phase(self, name, **data):
        self.emit(dict(phase=name, layers=self.layers, object_state=self.object_state, **data))

    def run(self, io):
        try:
            self.phase('PRECHECK')
            io.precheck(self.c)  # Must fail before scan motion if commissioning is incomplete.
            self.phase('HOME_START')
            io.home()
            self.phase('SCAN')
            scan = io.scan()
            if not (scan['returned_home'] and scan['physical_stop_confirmed']):
                raise RuntimeError('scan did not return Home and prove measured stop')
            # Scan's normal endpoint is formal Home. Never redefine stack XY.
            candidates = scan['target_hypotheses']
            for _ in range(self.c['max_pieces']):
                seed = select_target(self.c, candidates, self.retired)
                self.phase('SELECT', search_seed=seed['scan_target_id'])
                # Coarse point used only for the calibrated observation pose.
                observe = target_poses(self.c, seed, self.layers)['observe']
                io.move('PREGRASP', observe, 'empty')
                self.phase('REOBSERVE')
                measured = None
                for attempt in range(2):
                    try:
                        measured = qualified_target(self.c, seed, io.observe())
                        break
                    except RuntimeError as e:
                        self.phase('OBSERVATION_REJECTED', attempt=attempt+1, reason=str(e))
                if measured is None:
                    raise RuntimeError('bounded fresh reobservation failed')
                p = measured['base_surface_center_m']
                if distance(p[:2], self.c['stack_xy_m']) <= self.c['stack_exclusion_radius_m'] or any(
                        distance(p[:2], q[:2]) <= self.c['retired_radius_m'] for q in self.retired):
                    raise RuntimeError('fresh target lies in stack/retired exclusion zone')
                poses = target_poses(self.c, measured, self.layers)
                self.phase('PREGRASP', fresh_measurement=measured)
                io.open_empty()
                io.move('PREGRASP', poses['pregrasp'], 'empty')
                # Reassociation after this last alignment is mandatory too;
                # otherwise a second move/occlusion could stale the descent XY.
                check = qualified_target(self.c, measured | dict(class_name=seed['class_name']), io.observe())
                if check['scene_identity'][1:] != measured['scene_identity'][1:]:
                    raise RuntimeError('camera/scene/reference identity changed after alignment')
                if distance(check['base_surface_center_m'], p) > self.c['descent_tolerance_m']:
                    raise RuntimeError('target changed after pregrasp alignment')
                io.check_descent_age(check['last_stamp_ns'])
                self.phase('DESCEND_PICK')
                io.move('DESCEND', poses['grasp'], 'empty')
                self.object_state = 'unknown'  # Any attempted write can change physical state.
                self.phase('GRIP')
                io.grasp()
                self.object_state = 'held'
                self.phase('LIFT')
                io.move('RETREAT', poses['pregrasp'], 'held')
                self.phase('TRANSFER_HOME_XY')
                io.move('PREGRASP', poses['preplace'], 'held')
                self.phase('DESCEND_PLACE', support_top_z_m=top_z(self.c, self.layers))
                io.move('DESCEND', poses['place'], 'held')
                # Existing gripper has no independent support sensor. Explicit
                # operator evidence is required while still holding the piece.
                io.confirm_support(self.layers+1)
                self.object_state = 'unknown'
                self.phase('RELEASE')
                io.release()
                self.object_state = 'released_unverified'
                self.phase('WITHDRAW')
                io.move('RETREAT', poses['preplace'], 'released_unverified')
                io.confirm_placement(self.layers+1, top_z(self.c, self.layers+1))
                # Only support + release feedback + post-withdraw stability
                # evidence can commit a layer and retire the original location.
                self.layers += 1
                self.retired.append(p)
                self.object_state = 'empty'
                self.phase('PLACED', expected_top_z_m=top_z(self.c, self.layers))
            self.phase('RETURN_HOME')
            io.home()
            self.phase('COMPLETE')
        except BaseException as e:
            self.phase('FAULT', reason=str(e))
            io.halt(self.object_state)
            raise


def audit(config, scan):
    validate_config(config)
    missing = [key for key, value in config['measured'].items() if value is None]
    selected = None
    try:
        selected = select_target(config, scan['target_hypotheses'], [])['scan_target_id'] if scan else None
    except RuntimeError:
        pass
    return dict(schema='xiangqi_pick_stack_audit/v1', physical_actions_sent=0,
                scan_id=scan['scan_id'] if scan else None, scan_hypotheses=len(scan['target_hypotheses']) if scan else None,
                unique_physical_target_count=None, first_search_seed=selected,
                batch_limit=config['max_pieces'], table_z_m=config['table_z_m'],
                stack_xy_m=config['stack_xy_m'],
                theoretical_stack_top_z_m=[top_z(config, n) for n in range(1, 9)],
                missing_measurements=missing, physical_io_implemented=True,
                execution_configuration_complete=not execution_blockers(config),
                blockers=execution_blockers(config),
                historical_coordinates_authorize_descent=False)


def rehearse(config, scan):
    """Demonstrate orchestration with explicitly synthetic IO, never ROS."""
    c = copy.deepcopy(config)
    # These illustrative offsets exist only in memory and are never a hardware
    # calibration or executable config. Real scan points remain search seeds.
    c['measured']['surface_to_nominal_tcp_base_m'] = [0., 0., 0.]
    c['measured']['surface_to_observation_tcp_base_m'] = [0., 0., .15]
    events = []

    class SyntheticIO:
        def precheck(self, _):
            pass

        def scan(self):
            return scan

        def move(self, operation, xyz, object_state):
            events.append(dict(kind='synthetic_arm', operation=operation, nominal_tcp_m=xyz,
                               object_state=object_state, physical_command_sent=False))

        def observe(self):
            seed = select_target(c, scan['target_hypotheses'], demo.retired)
            events.append(dict(kind='synthetic_observation', search_seed=seed['scan_target_id']))
            return [dict(detection_valid=True, base_transform_valid=True, full_scene_coverage=True,
                         rgbd_frame_key=['synthetic', i], source_epoch='synthetic', stationary_generation=1,
                         controller_epoch='synthetic', track_namespace='synthetic',
                         identity={'synthetic': True}, source_K=[], stamp_ns=10**9+i*10**8,
                         detections=[dict(base_geometry_valid=True, complete_center_target=True,
                                          classification_valid=True, class_name=seed['class_name'],
                                          base_surface_center_m=seed['base_surface_center_m'])])
                    for i in range(21)]

        def check_descent_age(self, _):
            pass  # Synthetic clock; never a measurement freshness claim.

        def open_empty(self):
            events.append(dict(kind='synthetic_open_empty'))

        def grasp(self):
            events.append(dict(kind='synthetic_new_grasp'))

        def confirm_support(self, layer):
            events.append(dict(kind='synthetic_support', layer=layer))

        def release(self):
            events.append(dict(kind='synthetic_release'))

        def confirm_placement(self, layer, expected_z):
            events.append(dict(kind='synthetic_placement', layer=layer, expected_top_z_m=expected_z))

        def home(self):
            events.append(dict(kind='synthetic_formal_home'))

        def halt(self, object_state):
            events.append(dict(kind='synthetic_halt', object_state=object_state, automatic_open=False,
                               automatic_home=False))

    demo = Demo(c, lambda row: events.append(dict(kind='state', **row)))
    result = dict(schema='xiangqi_pick_stack_rehearsal/v1', synthetic=True,
                  physical_actions_sent=0, calibration_validated=False,
                  motion_plans_validated=False, events=events)
    try:
        demo.run(SyntheticIO())
        result['status'] = 'SYNTHETIC_COMPLETE'
    except RuntimeError as e:
        result.update(status='SYNTHETIC_FAULT', reason=str(e))
    result['synthetic_layers'] = demo.layers
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(Path(__file__).with_name('demo.current.json')))
    parser.add_argument('--scan-result', help='history for offline audit only; execute always rescans')
    parser.add_argument('--output', required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--plan-current', action='store_true', help='real arm plan_only, from current feedback only')
    mode.add_argument('--execute', action='store_true')
    mode.add_argument('--preflight',action='store_true',help='read-only live feedback/parameter check')
    mode.add_argument('--observe-alignment',action='store_true',help='physical 100mm nominal hover for operator observation, no grip')
    mode.add_argument('--rehearse', action='store_true', help='synthetic IO trace, no ROS and no calibration claims')
    parser.add_argument('--authorization',help='record of explicit current-run physical authorization; not inferred from history')
    args = parser.parse_args()
    config = json.loads(Path(args.config).read_text())
    if not args.scan_result and not (args.execute or args.preflight):
        parser.error('--scan-result is required for offline audit/rehearsal/current-start probes')
    if args.execute and (not args.authorization or args.scan_result):
        parser.error('--execute requires --authorization and a fresh scan, omit --scan-result')
    if args.observe_alignment and not args.authorization:
        parser.error('--observe-alignment requires explicit current-run --authorization')
    scan = json.loads(Path(args.scan_result).read_text()) if args.scan_result else None
    report = audit(config, scan)
    out = Path(args.output)
    out.mkdir(parents=False, exist_ok=False)
    (out/'audit.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    if args.execute:
        (out/'workflow.json').write_text(json.dumps(dict(
            schema='xiangqi_pick_stack_workflow/v1',max_pieces=config['max_pieces'],
            authorization=args.authorization,
            sequence=['PRECHECK','HOME_START','SCAN','REOBSERVE','PREGRASP','DESCEND_PICK',
                      'GRIP','LIFT','TRANSFER_HOME_XY','DESCEND_PLACE','RELEASE','WITHDRAW',
                      'VERIFY_PLACEMENT','RETURN_HOME'],
            dispatcher='Demo.run(RealIO)',manual_action_dispatch=False),ensure_ascii=False,indent=2))
    if args.rehearse:
        (out/'rehearsal.json').write_text(json.dumps(rehearse(config, scan), ensure_ascii=False, indent=2))
    if args.execute:
        blockers=execution_blockers(config)
        if blockers:
            (out/'run-result.json').write_text(json.dumps(dict(status='BLOCKED_PRECHECK',
                blockers=blockers,physical_arm_actions=0,gripper_commands_sent=0,
                last_phase='PRECHECK',authorization=args.authorization),ensure_ascii=False,indent=2))
            raise SystemExit('PRECHECK BLOCKED before ROS commands: '+'; '.join(blockers))
        from ros_io import execute
        result=execute(config,out,args.authorization)
        print(json.dumps(dict(status=result['status'],layers=result['layers'],
                              result_file=str(out/'run-result.json')),ensure_ascii=False))
        return
    if args.preflight:
        from ros_io import preflight
        preflight(config,out)
    if args.observe_alignment:
        from ros_io import observe_alignment
        result=observe_alignment(config,scan,out,args.authorization)
        print(json.dumps(dict(status=result['status'],physical_moves_completed=result['physical_moves_completed'],
                              gripper_commands_sent=0,result_file=str(out/'alignment-result.json')),ensure_ascii=False))
        return
    if args.plan_current:
        from ros_io import plan_current
        plan_current(config, scan, out)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
