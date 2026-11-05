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
    limit=32 if config.get('business_mode')=='row' else 8
    if config['schema'] != 'xiangqi_pick_stack_config/v1' or not 1 <= config['max_pieces'] <= limit:
        raise ValueError('expected v1 config and finite batch of 1..'+str(limit)+' pieces')
    if config.get('business_mode')=='row':
        if config.get('calibration_mode')!='supervised_flat_table':
            raise ValueError('row mode uses the empirically validated flat-table calibration')
        slots=config['row_slots_m']
        if len(slots)<config['max_pieces'] or any(len(p)!=2 or not all(math.isfinite(x) for x in p) for p in slots):
            raise ValueError('row requires one finite base XY slot for every batch position')
        clearance=config['row_slot_clearance_xy_m']
        radius=config['measured']['piece_radius_m']
        if len(clearance)!=2 or any(not math.isfinite(x) or x<radius+.003 for x in clearance):
            raise ValueError('row XY clearances must cover the piece radius and 3mm allowance')
        if any(distance(a,b)<=2*radius+.003 or row_slot_occupied(config,a,b)
               for i,a in enumerate(slots) for b in slots[i+1:]):
            raise ValueError('row slots overlap the configured low-tool/piece clearance')
        if any(config.get(k) for k in ('resume_confirmed_stack_file','resume_held_placement','resume_pending_placement')):
            raise ValueError('row mode starts a fresh inventory; stack checkpoints cannot be resumed')
    if config['table_z_m'] != -.010 or config['piece_thickness_m'] != .013:
        raise ValueError('this demo is bound to the confirmed -10mm table and 13mm piece')
    if not 20 <= config['force_percent'] <= 30:
        raise ValueError('force must remain within current site 20..30 percent limit')


def calibration_fingerprint(config):
    bound={k:v for k,v in config.items() if k!='commissioning_review_file'}
    return hashlib.sha256(json.dumps(bound,sort_keys=True,allow_nan=False).encode()).hexdigest()


def execution_blockers(config):
    validate_config(config)
    if config.get('calibration_mode') == 'supervised_flat_table':
        if config.get('business_mode')!='row' and not 1 <= config['max_pieces'] <= 3:
            return ['supervised flat-table batch is limited to three pieces']
        m=config['measured']
        required=('open_raw','close_raw','piece_radius_m')
        missing=['unmeasured: '+k for k in required if m[k] is None]
        if not math.isfinite(config['nominal_pick_tcp_base_z_m']):
            missing.append('finite empirically validated pick height required')
        if not config.get('workspace_observation'):
            missing.append('current-run empty bare-table workspace confirmation required')
        return missing
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


def row_slot_occupied(config, point, slot):
    return all(abs(a-b)<=r for a,b,r in zip(point[:2],slot,config['row_slot_clearance_xy_m']))


def excluded_target(config, point, retired):
    if config.get('business_mode')=='row':
        if any(row_slot_occupied(config,point,p) for p in config['row_slots_m']):
            return True
    elif distance(point[:2],config['stack_xy_m'])<=config['stack_exclusion_radius_m']:
        return True
    return any(distance(point[:2],p[:2])<=config['retired_radius_m'] for p in retired)


def select_target(config, candidates, retired):
    """Select a search seed, never merge scan IDs into physical identities."""
    stack = config['stack_xy_m']
    if config.get('business_mode')=='row':
        # Cross-view labels/positions are search hints, including unknown labels.
        # Fresh stationary observations still decide classification and neighbors.
        hints=[c for c in candidates if len(c.get('base_surface_center_m') or [])==3 and
               c.get('association_state')=='within_view_unique']
        for seed in sorted(hints,key=lambda c:distance(c['base_surface_center_m'][:2],stack)):
            if not excluded_target(config,seed['base_surface_center_m'],retired):
                return seed
        raise RuntimeError('no eligible row search hint; retain unresolved hypotheses')
    for seed in sorted(candidates, key=lambda c: distance(c['base_surface_center_m'][:2], stack)):
        p = seed['base_surface_center_m']
        if seed['class_name'] == 'unknown' or seed['association_state'] != 'within_view_unique':
            continue
        if excluded_target(config,p,retired):
            continue
        # Nearby hypotheses (including unknown) remain unresolved. Exact nearby
        # duplicates may also block a seed; fresh observation must resolve them.
        if any(c is not seed and c['scan_target_id'] != seed['scan_target_id'] and
               distance(p[:2], c['base_surface_center_m'][:2]) < config['neighbor_clearance_m']
               for c in candidates):
            continue
        return seed
    raise RuntimeError('no isolated search seed; retain ambiguous/unknown hypotheses for review')


def qualified_target(config, seed, frames, allow_reclassify=False, observation_only=False,
                     geometry_only=False, unlocated_overlap=None):
    """Check a contiguous fresh stationary window; do not reuse track_id."""
    points = []
    confirmed_class = None if allow_reclassify else seed['class_name']
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
        if not observation_only and any((not d.get('base_geometry_valid') or not d.get('complete_center_target')) and
               (unlocated_overlap is None or unlocated_overlap(r,d)) for d in detections):
            raise RuntimeError('unlocated/incomplete observation in target view')
        matches = [d for d in detections if d.get('base_geometry_valid') and d.get('complete_center_target') and
                   distance(d['base_surface_center_m'],seed['base_surface_center_m']) <= config['reassociate_radius_m']]
        if len(matches) != 1:
            raise RuntimeError('target absent, displaced or association ambiguous')
        d = matches[0]
        if not geometry_only and confirmed_class is None and d['classification_valid'] and d['class_name']!='unknown':
            confirmed_class=d['class_name']
        if not observation_only and not geometry_only and (not d['classification_valid'] or d['class_name'] != confirmed_class or
                d.get('identity_ambiguous')):
            raise RuntimeError('target label unresolved/changed')
        if not observation_only and any(other is not d and other.get('base_geometry_valid') and
               other.get('complete_center_target') and distance(other['base_surface_center_m'][:2],
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
    return dict(base_surface_center_m=center, class_name=None if geometry_only else confirmed_class,
                first_stamp_ns=stamps[0], last_stamp_ns=stamps[-1],
                samples=len(points), scene_identity=identity, classification_required=not (observation_only or geometry_only))


def qualified_scene_window(frames):
    """Independent stationary full-image window, including zero detections."""
    seen=set();stamps=[];identity=None
    for frame in frames:
        if not all(frame.get(k) for k in ('detection_valid','base_transform_valid','full_scene_coverage')):
            raise RuntimeError('unqualified scene window')
        key=tuple(frame['rgbd_frame_key'])
        if key in seen:raise RuntimeError('duplicate RGBD frame')
        seen.add(key)
        signature=(frame['stationary_generation'],frame['controller_epoch'],frame['track_namespace'],
                   json.dumps(dict(identity=frame['identity'],K=frame['source_K']),sort_keys=True),frame['source_epoch'])
        if identity is None:identity=signature
        if signature!=identity:raise RuntimeError('scene epoch/configuration changed')
        stamps.append(frame['stamp_ns'])
    if len(stamps)<20 or (stamps[-1]-stamps[0])*1e-9<2:
        raise RuntimeError('need 20 independent full-scene frames over two seconds')
    if any(not 0<(b-a)*1e-9<=.3 for a,b in zip(stamps,stamps[1:])):
        raise RuntimeError('scene frame gap')
    return dict(first_stamp_ns=stamps[0],last_stamp_ns=stamps[-1],samples=len(stamps),scene_identity=identity)


def target_poses(config, measured, layers):
    """Offsets are measured at the one fixed reviewed orientation, in base axes."""
    m = config['measured']
    p = measured['base_surface_center_m']
    if config.get('calibration_mode') == 'supervised_flat_table':
        grasp=[*p[:2],config['nominal_pick_tcp_base_z_m']]
        place=([*config['row_slots_m'][layers],grasp[2]] if config.get('business_mode')=='row' else
               [*config['stack_xy_m'],grasp[2]+layers*config['piece_thickness_m']])
        height=config['transfer_tcp_base_z_m']
        return dict(observe=[*p[:2],config['hover_tcp_base_z_m']],
                    grasp=grasp,pregrasp=[*p[:2],config['hover_tcp_base_z_m']],
                    place=place,preplace=[*place[:2],height])
    grasp = [a+b for a, b in zip(p, m['surface_to_nominal_tcp_base_m'])]
    place_top = [*config['stack_xy_m'], top_z(config, layers+1)]
    place = [a+b for a, b in zip(place_top, m['surface_to_nominal_tcp_base_m'])]
    lift_z = max(grasp[2], place[2]) + config['vertical_clearance_m']
    return dict(observe=[a+b for a, b in zip(p, m['surface_to_observation_tcp_base_m'])],
                grasp=grasp, pregrasp=[grasp[0], grasp[1], lift_z],
                place=place, preplace=[place[0], place[1], lift_z])


class TargetUnavailable(RuntimeError):
    def __init__(self, seed):
        super().__init__('bounded fresh reobservation failed: '+seed['scan_target_id'])
        self.seed=seed


class Demo:
    """One finite batch, no automatic resume/retry of grip or motion."""
    def __init__(self, config, emit):
        validate_config(config)
        self.c = config
        self.emit = emit
        self.layers = 0
        self.retired = []
        self.unavailable = list(config.get("excluded_pick_positions_m",[]))
        self.object_state = 'unknown'

    def phase(self, name, **data):
        self.emit(dict(phase=name, layers=self.layers, object_state=self.object_state, **data))

    def return_home(self, io, phase='RETURN_HOME'):
        self.phase(phase)
        io.home()

    def prepare_startup(self, io):
        self.phase('STARTUP_PRECHECK')
        io.startup_precheck(self.c)
        self.phase('STARTUP_OPEN')
        # Startup deliberately releases a previous grip before any Home motion.
        io.open_startup()
        self.object_state = 'empty'
        self.return_home(io, 'HOME_START')

    def scan_workspace(self, io):
        self.phase('SCAN')
        scan = io.scan()
        if not (scan['returned_home'] and scan['physical_stop_confirmed']):
            raise RuntimeError('scan did not return Home and prove measured stop')
        return scan['target_hypotheses']

    def grasp_piece(self, io, candidates):
        seed = select_target(self.c, candidates, self.retired+self.unavailable)
        qualify=(io.qualify_pick_target if self.c.get('business_mode')=='row' else
                 lambda seed,frames,**options:qualified_target(self.c,seed,frames,**options))
        self.phase('SELECT', search_seed=seed['scan_target_id'])
        # Coarse point used only for the calibrated observation pose.
        observe = target_poses(self.c, seed, self.layers)['observe']
        io.move('PREGRASP', observe, 'empty')
        self.phase('REOBSERVE')
        measured = None
        for attempt in range(2):
            try:
                measured = qualify(seed, io.observe(),allow_reclassify=self.c.get('calibration_mode')=='supervised_flat_table')
                break
            except RuntimeError as e:
                self.phase('OBSERVATION_REJECTED', attempt=attempt+1, reason=str(e))
        if measured is None:
            raise TargetUnavailable(seed)
        measured['scan_target_id']=seed['scan_target_id']
        p = measured['base_surface_center_m']
        if excluded_target(self.c,p,self.retired+self.unavailable):
            raise RuntimeError('fresh target lies in stack/retired exclusion zone')
        poses = target_poses(self.c, measured, self.layers)
        self.source_measurement=measured
        self.phase('PREGRASP', fresh_measurement=measured)
        io.open_empty()
        io.move('PREGRASP', poses['pregrasp'], 'empty')
        # Reassociation after this last alignment is mandatory too;
        # otherwise a second move/occlusion could stale the descent XY.
        check=None
        for attempt in range(2):
            try:
                check=qualify(measured,io.observe())
                break
            except RuntimeError as e:
                self.phase('OBSERVATION_REJECTED',attempt=attempt+1,reason=str(e))
        if check is None:raise TargetUnavailable(seed)
        if check['scene_identity'][2:] != measured['scene_identity'][2:]:
            raise RuntimeError('camera/scene/reference identity changed after alignment')
        if distance(check['base_surface_center_m'], p) > self.c['descent_tolerance_m']:
            raise RuntimeError('target changed after pregrasp alignment')
        check['scan_target_id']=seed['scan_target_id']
        self.phase('PICK_ALIGNED',fresh_measurement=check)
        io.check_descent_age(check['last_stamp_ns'])
        self.phase('DESCEND_PICK')
        io.move('DESCEND', poses['grasp'], 'empty')
        self.object_state = 'unknown'  # Any attempted write can change physical state.
        self.phase('GRIP')
        io.grasp()
        self.object_state = 'held'
        self.phase('LIFT')
        io.move('RETREAT', poses['pregrasp'], 'held')
        return p, poses

    def place_piece(self, io, source_position, poses):
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
        self.retired.append(source_position)
        self.object_state = 'empty'
        self.phase('PLACED', expected_top_z_m=top_z(self.c, self.layers))

    def stack_pieces(self, io, candidates):
        available=list(candidates)
        while self.layers<self.c['max_pieces']:
            try:
                source_position,poses=self.grasp_piece(io,available)
            except TargetUnavailable as e:
                self.phase('TARGET_SKIPPED',search_seed=e.seed['scan_target_id'],reason=str(e))
                self.unavailable.append(e.seed['base_surface_center_m'])
                continue
            self.place_piece(io,source_position,poses)

    def run(self, io):
        try:
            confirmed_file=self.c.get('resume_confirmed_stack_file')
            held_pending=self.c.get('resume_held_placement')
            pending=self.c.get('resume_pending_placement')
            if confirmed_file:
                checkpoint=json.loads(Path(confirmed_file).read_text())
                self.layers=checkpoint['layers']
                self.retired=list(checkpoint['retired'])
                self.phase('RESUME_PRECHECK')
                io.startup_precheck(self.c)
                io.restore_confirmed_stack(checkpoint)
                self.object_state='empty'
                self.phase('STACK_RESTORED')
                self.return_home(io,'HOME_START')
            elif held_pending:
                self.layers=held_pending['layers']
                self.retired=held_pending['retired']
                self.phase('RESUME_PRECHECK')
                io.startup_precheck(self.c)
                io.restore_held_placement(held_pending)
                self.object_state='held'
                self.phase('RESUME_SUPPORT')
                io.confirm_support(self.layers+1)
                self.object_state='unknown'
                self.phase('RELEASE')
                io.release()
                self.object_state='released_unverified'
                self.phase('WITHDRAW')
                io.move('RETREAT',held_pending['preplace'],'released_unverified')
                io.confirm_placement(self.layers+1,top_z(self.c,self.layers+1))
                self.layers+=1
                self.retired.append(held_pending['source_position_m'])
                self.object_state='empty'
                self.phase('PLACED',expected_top_z_m=top_z(self.c,self.layers),recovered=True)
                self.return_home(io,'HOME_START')
            elif pending:
                self.layers=pending.get('layers',0)
                self.retired=list(pending.get('retired',[]))
                # Recover a released-but-uncommitted layer without repeating grip/release.
                self.phase('RESUME_PRECHECK')
                io.startup_precheck(self.c)
                io.restore_pending_placement(pending)
                self.object_state='released_unverified'
                self.phase('RESUME_VERIFY_PLACEMENT')
                io.confirm_placement(self.layers+1,top_z(self.c,self.layers+1))
                self.layers+=1
                self.retired.append(pending['source_position_m'])
                self.object_state='empty'
                self.phase('PLACED',expected_top_z_m=top_z(self.c,self.layers),recovered=True)
                self.return_home(io,'HOME_START')
            else:
                self.prepare_startup(io)
            self.phase('PRECHECK')
            io.precheck(self.c)
            candidates = self.scan_workspace(io)
            self.stack_pieces(io, candidates)
            self.return_home(io)
            self.phase('COMPLETE')
        except BaseException as e:
            self.phase('FAULT', reason=str(e))
            io.halt(self.object_state)
            raise


def row_held_resume_record(config, previous, audit_record, events, site_config):
    """Bind a held-only continuation to recorded motion, grip and source evidence."""
    if not (previous['business_mode']=='row' and previous['object_state']=='held' and
            previous['physical_stop_confirmed'] and previous['status']=='FAULT'):
        raise RuntimeError('row continuation requires a stopped held fault result')
    layers=previous['layers']
    if layers!=len(previous['placements']) or layers!=len(previous['retired']) or layers>=config['max_pieces']:
        raise RuntimeError('row continuation committed-slot records disagree')
    if not (audit_record['business_mode']=='row' and audit_record['row_slots_m']==config['row_slots_m'] and
            audit_record['table_z_m']==config['table_z_m'] and audit_record['batch_limit']==config['max_pieces']):
        raise RuntimeError('row continuation slots/table/batch changed')
    old_config=previous.get('demo_config')
    if old_config:
        for key in ('schema','business_mode','calibration_mode','row_slots_m','row_slot_clearance_xy_m',
                    'max_pieces','table_z_m','piece_thickness_m','nominal_pick_tcp_base_z_m','force_percent','orientation_xyzw'):
            if old_config[key]!=config[key]:raise RuntimeError('row continuation setting changed: '+key)
        for key in ('open_raw','close_raw','piece_radius_m'):
            if old_config['measured'][key]!=config['measured'][key]:
                raise RuntimeError('row continuation gripper setting changed: '+key)
    else:
        # The historical adapter recorded grip outcomes, not force/close goals.
        # Bind those missing request settings to the existing site configuration;
        # do not present the old result as proof of unrecorded command parameters.
        for key in ('schema','table_z_m','piece_thickness_m','nominal_pick_tcp_base_z_m','force_percent','orientation_xyzw'):
            if config[key]!=site_config[key]:raise RuntimeError('row continuation differs from site setting: '+key)
        for key in ('open_raw','close_raw','piece_radius_m'):
            if config['measured'][key]!=site_config['measured'][key]:
                raise RuntimeError('row continuation differs from site gripper setting: '+key)
    own=[r for r in events if r.get('task_id')==previous['scan_id']]
    aligned=next(r for r in reversed(own) if r['phase']=='PICK_ALIGNED')
    tail=[r for r in own if r['stamp_ns']>=aligned['stamp_ns']]
    if any(r['phase'] in ('RELEASE_ROW','WITHDRAW_ROW','VERIFY_ROW_PLACEMENT','ROW_PLACED') for r in tail):
        raise RuntimeError('row continuation cannot repeat a release or an already placed piece')
    phases=[r['phase'] for r in tail]
    if not all(p in phases for p in ('GRIP','LIFT','TRANSFER_ROW')):
        raise RuntimeError('row continuation lacks successful grip/transfer sequence')
    transfer=next(r for r in reversed(tail) if r['phase']=='TRANSFER_ROW')
    if transfer['slot_index']!=layers or transfer['slot_xy_m']!=config['row_slots_m'][layers]:
        raise RuntimeError('row continuation held target slot changed')
    actual=[a for a in previous['actions'] if not a['plan_only']]
    last=actual[-1]
    if not (last['operation']=='PREGRASP' and last['status']==4 and last['result']['executed'] and
            last['result']['code']==1 and last['result']['final_tcp_valid'] and
            last['requested_tcp_m'][:2]==transfer['slot_xy_m']):
        raise RuntimeError('row continuation has no successful measured transfer endpoint')
    descent=next(a for a in reversed(actual) if a['operation']=='DESCEND')
    source=dict(aligned['fresh_measurement'])
    source['scene_identity']=tuple(source['scene_identity'])
    if not (descent['status']==4 and descent['result']['executed'] and descent['result']['code']==1 and
            descent['requested_tcp_m'][2]==config['nominal_pick_tcp_base_z_m'] and
            distance(descent['requested_tcp_m'][:2],source['base_surface_center_m'][:2])<=config['descent_tolerance_m']):
        raise RuntimeError('row continuation source/pick height differs from recorded grasp')
    grip=previous['gripper_latest'];last_grip=previous['gripper_actions'][-1]
    if not (grip['object_detected'] and not grip['object_dropped'] and grip['grip_status']==2 and
            last_grip['status']==4 and last_grip['result']['code']==2 and
            last_grip['result']['final_state']['object_detected']):
        raise RuntimeError('row continuation prior grip is not confirmed held')
    opens=[a['result']['final_state']['position_raw'] for a in previous['gripper_actions']
           if a['result']['final_state']['grip_status']==1 and not a['result']['final_state']['object_detected']]
    if not opens or opens[-1]!=config['measured']['open_raw']:
        raise RuntimeError('row continuation open setting differs from recorded open result')
    return dict(previous=previous,source=source,transfer_endpoint=last['result']['final_tcp'],
                recorded_gripper_raw=grip['position_raw'],
                prior_gripper_request_settings_recorded=bool(old_config))


class RowDemo(Demo):
    """Finite bare-table row placement; commit only independently observed slots."""
    def __init__(self, config, emit):
        super().__init__(config,emit)
        self.unresolved=[]
        self.scan_unresolved=[]
        self.placements=[]

    def resume_held_row(self, io):
        result_path=Path(self.c['resume_row_result_file'])
        previous=json.loads(result_path.read_text())
        audit_record=json.loads(result_path.with_name('audit.json').read_text())
        events=[json.loads(line) for line in result_path.with_name('events.jsonl').read_text().splitlines() if line.strip()]
        site=json.loads(Path(__file__).with_name('demo.current.json').read_text())
        record=row_held_resume_record(self.c,previous,audit_record,events,site)
        self.layers=previous['layers'];self.retired=list(previous['retired'])
        self.placements=list(previous['placements']);self.unresolved=list(previous['unresolved'])
        self.scan_unresolved=list(previous['scan_unresolved'])
        self.object_state='held';self.source_measurement=record['source']
        self.phase('ROW_RESUME_PRECHECK',resume_result_file=str(result_path))
        io.startup_precheck(self.c)
        io.restore_row_held(record)
        self.phase('ROW_HELD_RESTORED',source_measurement=self.source_measurement,slot_index=self.layers)
        poses=target_poses(self.c,self.source_measurement,self.layers)
        self.place_in_row(io,self.source_measurement['base_surface_center_m'],poses)
        self.return_home(io,'HOME_AFTER_ROW_RESUME')

    def scan_workspace(self, io):
        self.phase('SCAN')
        scan=io.scan()
        if not (scan['returned_home'] and scan['physical_stop_confirmed']):
            raise RuntimeError('scan did not return Home and prove measured stop')
        views=scan.get('views',[])
        if not views:
            self.scan_unresolved.append(dict(reason='scan contains no view records'))
        for view in views:
            if view.get('sampling_requirement_met') is not True:
                self.scan_unresolved.append(dict(view_id=view.get('view_id'),attempt=view.get('attempt'),
                    reason='fresh view sampling requirement failed or was not recorded',
                    independent_frames=view.get('independent_frames'),
                    coverage_state=view.get('coverage_state')))
            if view.get('unlocated_observations'):
                self.scan_unresolved.append(dict(view_id=view.get('view_id'),attempt=view.get('attempt'),
                    reason='scan contains unlocated/incomplete observations',
                    observations=view['unlocated_observations']))
        expected={v['id'] for v in scan.get('config',{}).get('views',[])}
        observed={v.get('view_id') for v in views}
        for view_id in sorted(expected-observed):
            self.scan_unresolved.append(dict(view_id=view_id,reason='configured view has no scan record'))
        self.phase('ROW_SCAN_RECORDED',scan_unresolved=self.scan_unresolved,
                   source_hypotheses=len(scan['target_hypotheses']))
        if not scan['target_hypotheses']:
            raise RuntimeError('fresh scan contains no independently located sources')
        return scan['target_hypotheses']

    def update_unresolved(self,candidates):
        completed=self.retired+[p['measurement']['base_surface_center_m'] for p in self.placements]
        self.unresolved=[dict(search_seed=p['scan_target_id'],class_name=p['class_name'],
                              position_m=p['base_surface_center_m'],
                              reason='not independently picked and verified in row')
                         for p in candidates if not any(distance(p['base_surface_center_m'][:2],q[:2])<=
                             self.c['retired_radius_m'] for q in completed)]

    def place_in_row(self, io, source_position, poses):
        slot=poses['place'][:2]
        self.phase('TRANSFER_ROW',slot_index=self.layers,slot_xy_m=slot)
        io.move('PREGRASP',poses['preplace'],'held')
        self.phase('VERIFY_EMPTY_SLOT',slot_index=self.layers)
        io.check_row_slot_empty(slot)
        self.phase('DESCEND_ROW',slot_index=self.layers)
        io.move('DESCEND',poses['place'],'held')
        io.verify_row_release_pose(poses['place'])
        self.object_state='unknown'
        self.phase('RELEASE_ROW',slot_index=self.layers)
        io.release()
        self.object_state='released_unverified'
        self.phase('WITHDRAW_ROW',slot_index=self.layers)
        io.move('RETREAT',poses['preplace'],'released_unverified')
        self.phase('VERIFY_ROW_PLACEMENT',slot_index=self.layers)
        measured=io.verify_row_placement(slot,self.source_measurement)
        self.placements.append(dict(slot_index=self.layers,slot_xy_m=slot,
                                    source_position_m=source_position,measurement=measured))
        self.retired.append(source_position)
        self.layers+=1
        self.object_state='empty'
        self.phase('ROW_PLACED',slot_index=self.layers-1,measurement=measured)

    def run(self, io):
        try:
            if self.c.get('resume_row_result_file'):
                self.resume_held_row(io)
            else:
                self.prepare_startup(io)
                self.phase('PRECHECK')
                io.precheck(self.c)
            candidates=self.scan_workspace(io)
            self.update_unresolved(candidates)
            while self.layers<self.c['max_pieces']:
                available=[p for p in candidates if not any(distance(p['base_surface_center_m'][:2],q[:2])<=
                           self.c['retired_radius_m'] for q in self.retired)]
                try:
                    select_target(self.c,available,self.retired+self.unavailable)
                except RuntimeError:
                    break
                try:
                    self.return_home(io,'HOME_BEFORE_ROW_PICK')
                    source,poses=self.grasp_piece(io,available)
                except TargetUnavailable as e:
                    self.unavailable.append(e.seed['base_surface_center_m'])
                    self.phase('TARGET_SKIPPED',search_seed=e.seed['scan_target_id'],reason=str(e))
                    continue
                self.place_in_row(io,source,poses)
                self.update_unresolved(candidates)
            self.return_home(io)
            self.phase('ROW_PARTIAL',search_inventory_processed=not self.unresolved and not self.scan_unresolved,
                       placed_count=self.layers,unresolved=self.unresolved,scan_unresolved=self.scan_unresolved,
                       all_physical_pieces_confirmed=False)
        except BaseException as e:
            self.phase('FAULT',reason=str(e))
            io.halt(self.object_state)
            raise


def audit(config, scan):
    validate_config(config)
    required=('open_raw','close_raw','piece_radius_m') if config.get('calibration_mode')=='supervised_flat_table' else config['measured'].keys()
    missing = [key for key in required if config['measured'][key] is None]
    selected = None
    try:
        selected = select_target(config, scan['target_hypotheses'], [])['scan_target_id'] if scan else None
    except RuntimeError:
        pass
    return dict(schema='xiangqi_pick_stack_audit/v1', physical_actions_sent=0,
                scan_id=scan['scan_id'] if scan else None, scan_hypotheses=len(scan['target_hypotheses']) if scan else None,
                unique_physical_target_count=None, first_search_seed=selected,
                batch_limit=config['max_pieces'], table_z_m=config['table_z_m'],
                business_mode=config.get('business_mode','stack'),
                row_slots_m=config.get('row_slots_m'),stack_xy_m=config['stack_xy_m'],
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
        def startup_precheck(self, _):
            pass

        def open_startup(self):
            events.append(dict(kind='synthetic_startup_open'))

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
    mode.add_argument('--observe-centers',action='store_true',help='laser alignment at current height; no gripper writes or Home')
    parser.add_argument('--center-limit',type=int,default=32)
    mode.add_argument('--rehearse', action='store_true', help='synthetic IO trace, no ROS and no calibration claims')
    parser.add_argument('--workspace-clear',action='store_true',help='confirm intended bare-table destinations and hands/ruler clear for this run')
    parser.add_argument('--authorization',help='record of explicit current-run physical authorization; not inferred from history')
    args = parser.parse_args()
    config = json.loads(Path(args.config).read_text())
    if args.rehearse and config.get('business_mode')=='row':
        parser.error('row mode has no synthetic rehearsal dispatcher')
    if args.workspace_clear:
        config['workspace_observation']='Operator explicitly passed --workspace-clear for this execution'
    if not args.scan_result and not (args.execute or args.preflight or args.observe_centers):
        parser.error('--scan-result is required for offline audit/rehearsal/current-start probes')
    if args.execute and (not args.authorization or args.scan_result):
        parser.error('--execute requires --authorization and a fresh scan, omit --scan-result')
    if (args.observe_alignment or args.observe_centers) and not args.authorization:
        parser.error('physical observation requires explicit current-run --authorization')
    scan = json.loads(Path(args.scan_result).read_text()) if args.scan_result else None
    report = audit(config, scan)
    out = Path(args.output)
    out.mkdir(parents=False, exist_ok=False)
    (out/'audit.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    if args.execute:
        (out/'workflow.json').write_text(json.dumps(dict(
            schema='xiangqi_pick_stack_workflow/v1',max_pieces=config['max_pieces'],
            authorization=args.authorization,
            sequence=(['STARTUP_PRECHECK','STARTUP_OPEN','HOME_START','PRECHECK','SCAN','REOBSERVE','PREGRASP','DESCEND_PICK',
                       'GRIP','LIFT','TRANSFER_ROW','VERIFY_EMPTY_SLOT','DESCEND_ROW','RELEASE_ROW','WITHDRAW_ROW',
                       'VERIFY_ROW_PLACEMENT','RETURN_HOME'] if config.get('business_mode')=='row' else
                      ['STARTUP_PRECHECK','STARTUP_OPEN','HOME_START','PRECHECK','SCAN','REOBSERVE','PREGRASP','DESCEND_PICK',
                       'GRIP','LIFT','TRANSFER_HOME_XY','DESCEND_PLACE','RELEASE','WITHDRAW','VERIFY_PLACEMENT','RETURN_HOME']),
            business_mode=config.get('business_mode','stack'),
            dispatcher=('RowDemo.run(RealIO)' if config.get('business_mode')=='row' else 'Demo.run(RealIO)'),manual_action_dispatch=False),ensure_ascii=False,indent=2))
    if args.rehearse:
        (out/'rehearsal.json').write_text(json.dumps(rehearse(config, scan), ensure_ascii=False, indent=2))
    if args.execute:
        from ros_io import execute
        result=execute(config,out,args.authorization)
        print(json.dumps(dict(status=result['status'],layers=result['layers'],
                              result_file=str(out/'run-result.json')),ensure_ascii=False))
        return
    if args.preflight:
        from ros_io import preflight
        preflight(config,out)
    if args.observe_centers:
        from ros_io import observe_centers
        result=observe_centers(config,scan,out,args.authorization,args.center_limit)
        print(json.dumps(dict(status=result['status'],centers=len(result['observations']),
                              gripper_commands_sent=0,result_file=str(out/'center-alignment-result.json')),ensure_ascii=False))
        return
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
