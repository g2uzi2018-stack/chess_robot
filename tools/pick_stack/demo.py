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



def complete_detection(detection):
    """A full-image packet must still respect the actual mask's ROI boundary."""
    if not (detection.get('base_geometry_valid') and detection.get('complete_center_target')):
        return False
    crop=detection.get('geometry_roi_xyxy')
    if crop is None:
        return True
    box=detection['box_xyxy']
    return (crop[0]+2<=box[0] and crop[1]+2<=box[1]
            and box[2]<=crop[2]-2 and box[3]<=crop[3]-2)


def top_z(config, layers):
    return config['table_z_m'] + layers * config['piece_thickness_m']


def validate_config(config):
    limit=32 if config.get('business_mode')=='row' else 8
    if config['schema'] != 'xiangqi_pick_stack_config/v1' or not 1 <= config['max_pieces'] <= limit:
        raise ValueError('expected v1 config and finite batch of 1..'+str(limit)+' pieces')
    if config.get('business_mode')=='row':
        if config.get('calibration_mode')!='supervised_flat_table':
            raise ValueError('row mode uses the empirically validated flat-table calibration')
        policy=config.get('pick_neighbor_policy','radial')
        if policy not in ('radial','finger_model'):
            raise ValueError('unknown pick neighbor policy')
        yaws=config.get('pick_yaw_rechecks_deg',[])
        if len(yaws)>2 or len(set(yaws))!=len(yaws) or any(yaw not in (-90,90) for yaw in yaws):
            raise ValueError('pick yaw rechecks are limited to distinct +/-90 degree world-Z rotations')
        if yaws and policy!='finger_model':
            raise ValueError('pick yaw rechecks require the model-based neighbor envelope')
        slots=config['row_slots_m']
        order=config.get('row_slot_order',list(range(len(slots))))
        if (len(order)!=len(slots) or any(type(i) is not int for i in order) or
                sorted(order)!=list(range(len(slots))) or
                sorted(order[:config['max_pieces']])!=list(range(config['max_pieces']))):
            raise ValueError('row slot order must permute the configured goal slots')
        if len(slots)<config['max_pieces'] or any(len(p)!=2 or not all(math.isfinite(x) for x in p) for p in slots):
            raise ValueError('row requires one finite base XY slot for every batch position')
        if config.get('row_occupied_policy','occupant_first') not in ('skip','occupant_first'):
            raise ValueError('unknown row occupied-slot policy')
        clearance=config['row_slot_clearance_xy_m']
        radius=config['measured']['piece_radius_m']
        if len(clearance)!=2 or any(not math.isfinite(x) or x<radius+.003 for x in clearance):
            raise ValueError('row XY clearances must cover the piece radius and 3mm allowance')
        if any(distance(a,b)<=2*radius+2*config.get('row_placement_tolerance_m',config['descent_tolerance_m']) or row_slot_occupied(config,a,b)
               for i,a in enumerate(slots) for b in slots[i+1:]):
            raise ValueError('row slots overlap the configured low-tool/piece clearance')
        if type(config.get('target_recheck_attempts',2)) is not int or not 0<=config.get('target_recheck_attempts',2)<=2:
            raise ValueError('target realignment rechecks must be an integer within 0..2')
        approach=config.get('active_observation',{})
        if approach.get('enabled'):
            drops=approach['drop_offsets_m'];floor=approach['min_tcp_base_z_m']
            if (not 1<=len(drops)<=2 or any(not math.isfinite(x) or x<=0 for x in drops)
                    or any(b<=a for a,b in zip(drops,drops[1:]))
                    or not math.isfinite(floor) or floor<config['table_z_m']+config['piece_thickness_m']+.045):
                raise ValueError('active observation requires at most two increasing drops and an above-payload clearance floor')
            steps=approach.get('lateral_steps_m',[])
            radius=approach.get('max_xy_offset_m',.04)
            if (len(steps)>2 or any(not math.isfinite(step) or not 0<step<=.03 for step in steps)
                    or any(b<=a for a,b in zip(steps,steps[1:])) or not 0<radius<=.04
                    or any(step>radius for step in steps)):
                raise ValueError('directional observation requires at most two bounded XY steps')
            side=approach.get('side_step_m',0.)
            if (not math.isfinite(side) or not 0<=side<=.015
                    or side and (not steps or math.hypot(steps[0],side)>radius)):
                raise ValueError('side observation requires a bounded 15mm step inside the XY corridor')
            if not 0<approach.get('search_hint_radius_m',.04)<=.04:
                raise ValueError('unlocated image hints require at most 40mm association radius')
            if not 0<=approach.get('scan_rechecks',0)<=4:
                raise ValueError('scan rechecks must remain within the finite four-view budget')
        verification=config.get('row_verification',{})
        if (not .8<=verification.get('min_valid_fraction',.8)<=1
                or not 0<verification.get('max_missing_s',.3)<=.3
                or not 2<=verification.get('window_s',3.)<config['observation_timeout_s']):
            raise ValueError('row verification requires a bounded independent observation window')
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



def ordered_row_slots(config):
    slots=config['row_slots_m']
    return [slots[i] for i in config.get('row_slot_order',range(len(slots)))]


def row_destination_conflicts(config, candidates, first_slot=0):
    # 粗扫坐标只用于保守拦截疑似占位，不能据此批准任何下降。
    return [dict(slot_index=index,slot_xy_m=slot,
                 scan_target_id=candidate['scan_target_id'],
                 observed_position_m=candidate['base_surface_center_m'])
            for index,slot in enumerate(ordered_row_slots(config)) if index>=first_slot
            for candidate in candidates
            if row_slot_occupied(config,candidate['base_surface_center_m'],slot)]

def excluded_target(config, point, retired, filled_slots=0):
    if config.get('business_mode')=='row':
        slots=ordered_row_slots(config)
        protected=slots if config.get('row_occupied_policy')=='skip' else slots[:filled_slots]
        if any(row_slot_occupied(config,point,p) for p in protected):
            return True
    elif distance(point[:2],config['stack_xy_m'])<=config['stack_exclusion_radius_m']:
        return True
    return any(distance(point[:2],p[:2])<=config['retired_radius_m'] for p in retired)


def select_target(config, candidates, retired, slot_index=0, unavailable=()):
    """Rank search hints only; source geometry and empty-slot checks remain mandatory."""
    stack = config['stack_xy_m']
    if config.get('business_mode')=='row':
        destination=ordered_row_slots(config)[slot_index]
        excluded=retired+list(config.get('excluded_pick_positions_m',[]))
        hints=[c for c in candidates if len(c.get('base_surface_center_m') or [])==3 and
               c.get('association_state')=='within_view_unique' and
               not excluded_target(config,c['base_surface_center_m'],excluded,slot_index)]
        # 当前槽位的占位者优先；它不可抓时不能另抓一颗再搬到被占的位置。
        occupants=[c for c in hints if row_slot_occupied(config,c['base_surface_center_m'],destination)]
        pool=occupants or hints
        for seed in sorted(pool,key=lambda c:(distance(c['base_surface_center_m'][:2],destination),c['scan_target_id'])):
            if not any(distance(seed['base_surface_center_m'][:2],q[:2])<=config['retired_radius_m'] for q in unavailable):
                return seed
        if occupants:
            raise RuntimeError('current row destination has an unavailable occupant; no alternative pick permitted')
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


def pick_target_observation(config):
    if config.get('business_mode')!='row':
        return 20, 2.
    spec=config.get('pick_target_observation') or {}
    count=spec.get('min_frames',20);span=spec.get('window_s',2.)
    if type(count) is not int or count<6 or not math.isfinite(span) or span<.25:
        raise ValueError('pick observation requires at least six independent frames spanning 0.25 seconds')
    return count,span


def qualified_target(config, seed, frames, allow_reclassify=False, observation_only=False,
                     geometry_only=False, unlocated_overlap=None, neighbor_conflict=None,
                     min_frames=20, min_span_s=2):
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
        if not observation_only and any(not complete_detection(d) and
               (unlocated_overlap is None or unlocated_overlap(r,d)) for d in detections):
            raise RuntimeError('unlocated/incomplete observation in target view')
        matches = [d for d in detections if complete_detection(d) and
                   distance(d['base_surface_center_m'],seed['base_surface_center_m']) <= config['reassociate_radius_m']]
        if len(matches) != 1:
            raise RuntimeError('target absent, displaced or association ambiguous')
        d = matches[0]
        if not geometry_only and confirmed_class is None and d['classification_valid'] and d['class_name']!='unknown':
            confirmed_class=d['class_name']
        if not observation_only and not geometry_only and (not d['classification_valid'] or d['class_name'] != confirmed_class or
                d.get('identity_ambiguous')):
            raise RuntimeError('target label unresolved/changed')
        if not observation_only and any(other is not d and complete_detection(other) and
                (neighbor_conflict(d['base_surface_center_m'],other['base_surface_center_m'])
                 if neighbor_conflict is not None else distance(other['base_surface_center_m'][:2],
                    d['base_surface_center_m'][:2])<config['neighbor_clearance_m']) for other in detections):
            raise RuntimeError('neighbor/unknown too close for reviewed finger envelope')
        points.append(d['base_surface_center_m'])
    if len(points) < min_frames or (stamps[-1]-stamps[0])*1e-9 < min_span_s:
        raise RuntimeError('need at least '+str(min_frames)+' independent frames spanning '+str(min_span_s)+' seconds')
    if any(not 0 < (b-a)*1e-9 <= .3 for a, b in zip(stamps, stamps[1:])):
        raise RuntimeError('scene frame gap')
    center = [sum(p[i] for p in points)/len(points) for i in range(3)]
    if max(distance(p, center) for p in points) > .003:
        raise RuntimeError('target moved / repeatability exceeds 3mm')
    return dict(base_surface_center_m=center, class_name=None if geometry_only else confirmed_class,
                first_stamp_ns=stamps[0], last_stamp_ns=stamps[-1],
                samples=len(points), scene_identity=identity, classification_required=not (observation_only or geometry_only))


def qualified_scene_window(frames, min_frames=20, min_span_s=2):
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
    if len(stamps)<min_frames or (stamps[-1]-stamps[0])*1e-9<min_span_s:
        raise RuntimeError('need '+str(min_frames)+' independent full-scene frames over '+str(min_span_s)+' seconds')
    if any(not 0<(b-a)*1e-9<=.3 for a,b in zip(stamps,stamps[1:])):
        raise RuntimeError('scene frame gap')
    return dict(first_stamp_ns=stamps[0],last_stamp_ns=stamps[-1],samples=len(stamps),scene_identity=identity)


def qualified_row_observation(config, slot, frames, search_radius_m=.015):
    """Post-release geometry tolerates brief mask loss, with explicit limits."""
    evidence=qualified_scene_window(frames)
    policy=config.get('row_verification',{})
    samples=[]
    missing=[]
    for frame in frames:
        matches=[d for d in frame['detections'] if complete_detection(d) and
                 distance(d['base_surface_center_m'][:2],slot)<=search_radius_m]
        if len(matches)>1:
            raise ObservationUnavailable('row observation has multiple complete targets near slot')
        if matches:
            samples.append((frame['stamp_ns'],matches[0]['base_surface_center_m']))
        else:
            missing.append(frame['stamp_ns'])
    if len(samples)<20 or len(samples)/len(frames)<policy.get('min_valid_fraction',.8):
        raise ObservationUnavailable('row observation has insufficient valid target samples')
    stamps=[sample[0] for sample in samples]
    gaps=[(b-a)*1e-9 for a,b in zip(stamps,stamps[1:])]
    gaps.extend([(stamps[0]-frames[0]['stamp_ns'])*1e-9,
                 (frames[-1]['stamp_ns']-stamps[-1])*1e-9])
    if stamps[-1]-stamps[0]<2*10**9 or max(gaps)>policy.get('max_missing_s',.3):
        raise ObservationUnavailable('row observation target missing for too long')
    center=[sum(point[i] for _,point in samples)/len(samples) for i in range(3)]
    if max(distance(point,center) for _,point in samples)>.003:
        raise ObservationUnavailable('row observation target moved beyond 3mm repeatability')
    return dict(evidence,base_surface_center_m=center,class_name=None,
                samples=len(samples),total_frames=len(frames),missing_stamp_ns=missing,
                max_missing_gap_s=max(gaps),classification_required=False)


def target_poses(config, measured, layers):
    """Offsets are measured at the one fixed reviewed orientation, in base axes."""
    m = config['measured']
    p = measured['base_surface_center_m']
    if config.get('calibration_mode') == 'supervised_flat_table':
        grasp=[*p[:2],config['nominal_pick_tcp_base_z_m']]
        place=([*ordered_row_slots(config)[layers],grasp[2]] if config.get('business_mode')=='row' else
               [*config['stack_xy_m'],grasp[2]+layers*config['piece_thickness_m']])
        height=config['transfer_tcp_base_z_m']
        source_height=(height if config.get('business_mode')=='row' and
                       config.get('pick_neighbor_policy')=='finger_model' else config['hover_tcp_base_z_m'])
        return dict(observe=[*p[:2],source_height],
                    grasp=grasp,pregrasp=[*p[:2],source_height],
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


class TargetRecheckRequired(RuntimeError):
    """A new alignment is needed before submitting any descent goal."""
    def __init__(self, measurement):
        super().__init__('fresh target position requires alignment recheck')
        self.measurement=measurement


class PickPlanRejected(RuntimeError):
    """A completed empty-gripper descent request was rejected before execution."""


class ObservationUnavailable(RuntimeError):
    """Bounded visual recovery ended without enough measurement evidence."""


class RowSlotOccupied(RuntimeError):
    def __init__(self, evidence):
        super().__init__(evidence.get('reason','row destination clearance is occupied'))
        self.evidence=evidence


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
        row=self.c.get('business_mode')=='row'
        row_index=getattr(self,'slot_index',self.layers)
        destination=ordered_row_slots(self.c)[row_index] if row else None
        seed = select_target(self.c,candidates,self.retired if row else self.retired+self.unavailable,
                             row_index if row else 0,self.unavailable if row else ())
        qualify=(io.qualify_pick_target if self.c.get('business_mode')=='row' else
                 lambda seed,frames,**options:qualified_target(self.c,seed,frames,**options))
        if self.c.get('business_mode')=='row':io.begin_pick()
        selection=dict(search_seed=seed['scan_target_id'])
        if destination is not None:
            selection.update(selection_policy='nearest_source_to_empty_slot' if self.c.get('row_occupied_policy')=='skip' else 'current_occupant_then_nearest_slot',destination_slot_index=row_index,
                             current_slot_occupant=row_slot_occupied(self.c,seed['base_surface_center_m'],destination),
                             destination_xy_m=destination,
                             coarse_transfer_distance_m=distance(seed['base_surface_center_m'][:2],destination))
        self.phase('SELECT',**selection)
        # Coarse point used only for the calibrated observation pose.
        observe = target_poses(self.c, seed, row_index if row else self.layers)['observe']
        io.move('PREGRASP', observe, 'empty')
        self.phase('REOBSERVE')
        measured = None
        observation_errors=('unlocated/incomplete observation in target view',
            'target absent, displaced or association ambiguous',
            'neighbor/unknown too close for reviewed finger envelope',
            'target label unresolved/changed','target moved / repeatability exceeds 3mm')
        observation_attempts=2+(len(self.c.get('pick_yaw_rechecks_deg',[])) if row else 0)
        n,s=pick_target_observation(self.c)
        pick_observe=dict(duration_s=s,min_frames=n) if row else {}
        for attempt in range(observation_attempts):
            try:
                measured = qualify(seed, io.observe(**pick_observe),allow_reclassify=self.c.get('calibration_mode')=='supervised_flat_table')
                break
            except RuntimeError as e:
                if not isinstance(e,ObservationUnavailable) and str(e) not in observation_errors:
                    raise
                self.phase('OBSERVATION_REJECTED', attempt=attempt+1, reason=str(e))
                if row and self.c.get('pick_yaw_rechecks_deg') and (isinstance(e,ObservationUnavailable) or str(e) in (
                        'neighbor/unknown too close for reviewed finger envelope',
                        'target absent, displaced or association ambiguous',
                        'unlocated/incomplete observation in target view')):
                    io.try_next_pick_orientation(seed)
        if measured is None:
            raise TargetUnavailable(seed)
        measured['scan_target_id']=seed['scan_target_id']
        if excluded_target(self.c,measured['base_surface_center_m'],self.retired+self.unavailable,row_index if row else 0):
            raise TargetUnavailable(seed)
        io.open_empty()
        rechecks=self.c.get('target_recheck_attempts',2) if row else 0
        for realignment in range(rechecks+1):
            p=measured['base_surface_center_m']
            if excluded_target(self.c,p,self.retired+self.unavailable,row_index if row else 0):
                raise TargetUnavailable(seed)
            poses=target_poses(self.c,measured,row_index if row else self.layers)
            self.source_measurement=measured
            self.phase('PREGRASP',fresh_measurement=measured)
            try:
                io.move('PREGRASP',poses['pregrasp'],'empty')
                check=None
                for attempt in range(observation_attempts):
                    try:
                        check=qualify(measured,io.observe(**pick_observe))
                        break
                    except RuntimeError as error:
                        if not isinstance(error,ObservationUnavailable) and str(error) not in observation_errors:
                            raise
                        self.phase('OBSERVATION_REJECTED',attempt=attempt+1,reason=str(error))
                        if row and self.c.get('pick_yaw_rechecks_deg') and (isinstance(error,ObservationUnavailable) or str(error) in (
                            'neighbor/unknown too close for reviewed finger envelope',
                            'target absent, displaced or association ambiguous',
                            'unlocated/incomplete observation in target view')):
                            io.try_next_pick_orientation(measured)
                if check is None:
                    raise TargetUnavailable(seed)
                if tuple(check['scene_identity'][2:])!=tuple(measured['scene_identity'][2:]):
                    raise RuntimeError('camera/scene/reference identity changed after alignment')
                if distance(check['base_surface_center_m'],p)>self.c['descent_tolerance_m']:
                    raise TargetRecheckRequired(check)
                check['scan_target_id']=seed['scan_target_id']
                self.phase('PICK_ALIGNED',fresh_measurement=check)
                io.check_descent_age(check['last_stamp_ns'])
                self.phase('DESCEND_PICK')
                io.move('DESCEND',poses['grasp'],'empty')
                self.source_measurement=check
                p=check['base_surface_center_m']
                break
            except TargetRecheckRequired as changed:
                if realignment==rechecks:
                    self.phase('TARGET_RECHECK_EXHAUSTED',attempts=rechecks,
                        prior_position_m=p,latest_position_m=changed.measurement['base_surface_center_m'])
                    raise TargetUnavailable(seed) from changed
                self.phase('TARGET_ALIGNMENT_RECHECK',attempt=realignment+1,
                    prior_position_m=p,latest_position_m=changed.measurement['base_surface_center_m'])
                measured=dict(changed.measurement,scan_target_id=seed['scan_target_id'])
            except PickPlanRejected as error:
                self.phase('PICK_PLAN_RECHECK',attempt=realignment+1,reason=str(error))
                if realignment==rechecks:
                    raise TargetUnavailable(seed) from error
                io.try_next_pick_orientation(measured)
        self.object_state = 'unknown'  # Any attempted write can change physical state.
        self.phase('GRIP')
        io.grasp()
        self.object_state = 'held'
        self.phase('LIFT')
        io.move('RETREAT', poses['pregrasp'], 'held')
        if self.c.get('business_mode')=='row':io.end_pick()
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


def inherit_held_history(report, events, previous, previous_events):
    """Keep the original grasp evidence across a failed held continuation."""
    new_grasp=any(event.get('task_id')==report['scan_id'] and event['phase']=='PICK_ALIGNED'
                  for event in events)
    report['held_source_task_id']=(report['scan_id'] if new_grasp else
        previous.get('held_source_task_id',previous['scan_id']))
    report['actions']=previous['actions']+report['actions']
    report['gripper_actions']=previous['gripper_actions']+report['gripper_actions']
    if not report.get('scan_result_file') and previous.get('scan_result_file'):
        report['scan_result_file']=previous['scan_result_file']
    return previous_events+events


def row_held_resume_record(config, previous, audit_record, events, site_config):
    """Bind a held-only continuation to recorded motion, grip and source evidence."""
    if not (previous['business_mode']=='row' and previous['object_state']=='held' and
            previous['physical_stop_confirmed'] and previous['status']=='FAULT'):
        raise RuntimeError('row continuation requires a stopped held fault result')
    layers=previous['layers']
    slot_index=previous.get('next_slot_index',layers)
    skipped=previous.get('skipped_slots',[])
    pending=previous.get('pending_slots',[])
    if layers!=len(previous['placements']) or len(previous['retired'])!=layers+len(pending) or slot_index>=config['max_pieces']:
        raise RuntimeError('row continuation committed-slot records disagree')
    if sorted([p['slot_index'] for p in previous['placements']+pending]+[p['slot_index'] for p in skipped])!=list(range(slot_index)):
        raise RuntimeError('row continuation processed-slot records disagree')
    if not (audit_record['business_mode']=='row' and audit_record['row_slots_m']==config['row_slots_m'] and
            audit_record['table_z_m']==config['table_z_m'] and audit_record['batch_limit']==config['max_pieces']):
        raise RuntimeError('row continuation slots/table/batch changed')
    old_config=previous.get('demo_config')
    if old_config:
        if old_config.get('row_occupied_policy','occupant_first')!=config.get('row_occupied_policy','occupant_first'):
            raise RuntimeError('row continuation occupied-slot policy changed')
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
    default_order=list(range(len(config['row_slots_m'])))
    old_order=previous.get('row_slot_order',(old_config or {}).get('row_slot_order',default_order))
    ordered_config=dict(config,row_slot_order=old_order)
    validate_config(ordered_config)
    if ('row_slot_order' in config and config['row_slot_order']!=old_order or
            old_config and old_config.get('row_slot_order',old_order)!=old_order):
        raise RuntimeError('row continuation slot order changed')
    for processed in previous['placements']+pending+skipped:
        index=processed['slot_index']
        if (processed['slot_xy_m']!=ordered_row_slots(ordered_config)[index] or
                processed.get('physical_slot_index',old_order[index])!=old_order[index]):
            raise RuntimeError('row continuation physical slot records disagree')
    source_task=previous.get('held_source_task_id',previous['scan_id'])
    own=[r for r in events if r.get('task_id')==source_task]
    aligned=next(r for r in reversed(own) if r['phase']=='PICK_ALIGNED')
    tail=[r for r in events if r['stamp_ns']>=aligned['stamp_ns']]
    if any(r['phase'] in ('RELEASE_ROW','WITHDRAW_ROW','VERIFY_ROW_PLACEMENT','ROW_PLACED') for r in tail):
        raise RuntimeError('row continuation cannot repeat a release or an already placed piece')
    phases=[r['phase'] for r in tail]
    if not all(p in phases for p in ('GRIP','LIFT','TRANSFER_ROW')):
        raise RuntimeError('row continuation lacks successful grip/transfer sequence')
    transfer=next(r for r in reversed(tail) if r['phase']=='TRANSFER_ROW')
    intent=next(r for r in reversed(tail) if r['phase'] in ('TRANSFER_ROW','ROW_SLOT_DEFERRED'))
    intended_xy=intent['buffer_slot_xy_m'] if intent['phase']=='ROW_SLOT_DEFERRED' else intent['slot_xy_m']
    if intent['slot_index']!=slot_index or intended_xy!=ordered_row_slots(ordered_config)[slot_index]:
        raise RuntimeError('row continuation held target slot changed')
    actual=[a for a in previous['actions'] if not a['plan_only']]
    last=actual[-1]
    target=last['requested_tcp_m']
    endpoint_reviewed=(last['operation']=='PREGRASP' and
        target==[*transfer['slot_xy_m'],config['transfer_tcp_base_z_m']])
    if not endpoint_reviewed:
        # 持物观察可能停在侧向或下探视角，只接受历史中已规划并执行成功的观察端点。
        policy=previous['demo_config'].get('active_observation',{})
        endpoint_reviewed=(last['operation'] in ('PREGRASP','DESCEND') and policy.get('enabled') and
            policy['min_tcp_base_z_m']<=target[2]<=config['transfer_tcp_base_z_m'] and
            distance(target[:2],transfer['slot_xy_m'])<=policy.get('max_xy_offset_m',.04) and
            any(attempt['purpose']=='empty_row_slot' and attempt['status']=='MOVED_FOR_FRESH_OBSERVATION' and
                attempt['target_tcp_m']==target for attempt in previous.get('active_observation_attempts',[])))
    if any(event['phase']=='ROW_SLOT_DEFERRED' for event in tail):
        floor=config.get('active_observation',{}).get('min_tcp_base_z_m',.11)
        endpoint_reviewed=(last['operation'] in ('PREGRASP','RETREAT','DESCEND') and
            last['result']['final_tcp_valid'] and
            last['result']['final_tcp']['pose']['position']['z']>=floor)
    if not (endpoint_reviewed and last['status']==4 and last['result']['executed'] and
            last['result']['code']==1 and last['result']['final_tcp_valid']):
        raise RuntimeError('row continuation has no successful measured transfer/observation endpoint')
    descent=next(a for a in reversed(actual) if a['operation']=='DESCEND' and a['requested_tcp_m'][2]==config['nominal_pick_tcp_base_z_m'])
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
    return dict(previous=previous,source=source,row_slot_order=old_order,transfer_endpoint=last['result']['final_tcp'],
                recorded_gripper_raw=grip['position_raw'],
                prior_gripper_request_settings_recorded=bool(old_config))


class RowDemo(Demo):
    """Finite bare-table row placement; commit only independently observed slots."""
    def __init__(self, config, emit):
        super().__init__(config,emit)
        self.unresolved=[]
        self.scan_unresolved=[]
        self.placements=[]
        self.slot_index=0
        self.skipped_slots=[]
        self.pending_slots=[]
        self.search_candidates=[]
        if not config.get('resume_row_result_file'):
            self.c['row_slot_order']=list(config.get('row_slot_order',range(len(config['row_slots_m']))))

    def phase(self, name, **data):
        index=data.get('slot_index',self.slot_index)
        if index<self.c['max_pieces']:
            order=self.c.get('row_slot_order',list(range(len(self.c['row_slots_m']))))
            data.setdefault('physical_slot_index',order[index])
        data.setdefault('next_slot_index',self.slot_index)
        data.setdefault('placed_count',len(self.placements))
        data.setdefault('skipped_slot_count',len(self.skipped_slots))
        data.setdefault('pending_slot_count',len(self.pending_slots))
        super().phase(name,**data)

    def goal_complete(self):
        return self.layers==self.c['max_pieces'] and self.slot_index>=self.c['max_pieces'] and not self.pending_slots

    def skip_occupied_slot(self, evidence):
        row=dict(slot_index=self.slot_index,physical_slot_index=self.c['row_slot_order'][self.slot_index],
                 slot_xy_m=ordered_row_slots(self.c)[self.slot_index],evidence=evidence)
        self.skipped_slots.append(row)
        self.slot_index+=1
        self.phase('ROW_SLOT_SKIPPED_OCCUPIED',**row)

    def resume_held_row(self, io):
        result_path=Path(self.c['resume_row_result_file'])
        previous=json.loads(result_path.read_text())
        audit_record=json.loads(result_path.with_name('audit.json').read_text())
        events=[json.loads(line) for line in result_path.with_name('events.jsonl').read_text().splitlines() if line.strip()]
        site=json.loads(Path(__file__).with_name('demo.current.json').read_text())
        record=row_held_resume_record(self.c,previous,audit_record,events,site)
        self.c['row_slot_order']=list(record['row_slot_order'])
        if previous.get('scan_result_file'):
            self.search_candidates=json.loads(Path(previous['scan_result_file']).read_text())['target_hypotheses']
        self.layers=previous['layers'];self.retired=list(previous['retired'])
        self.slot_index=previous.get('next_slot_index',self.layers)
        self.skipped_slots=list(previous.get('skipped_slots',[]))
        self.pending_slots=list(previous.get('pending_slots',[]))
        self.placements=list(previous['placements']);self.unresolved=list(previous['unresolved'])
        self.scan_unresolved=list(previous['scan_unresolved'])
        self.object_state='held';self.source_measurement=record['source']
        self.phase('ROW_RESUME_PRECHECK',resume_result_file=str(result_path))
        io.startup_precheck(self.c)
        io.restore_row_held(record)
        self.phase('ROW_HELD_RESTORED',source_measurement=self.source_measurement,slot_index=self.slot_index)
        poses=target_poses(self.c,self.source_measurement,self.slot_index)
        self.place_in_row(io,self.source_measurement['base_surface_center_m'],poses)
        self.return_home(io,'HOME_AFTER_ROW_RESUME')

    def scan_workspace(self, io, rescan=False):
        self.phase('SCAN')
        scan=(io.rescan([p['measurement']['scene_identity'][2:] for p in self.placements])
              if rescan else io.scan())
        if rescan:
            self.scan_unresolved=[]
        if not (scan['returned_home'] and scan['physical_stop_confirmed']):
            raise RuntimeError('scan did not return Home and prove measured stop')
        if scan.get('search_mode')=='known_source_continuation':
            self.scan_unresolved.append(dict(reason='known remaining-source continuation; no new whole-table scan'))
            self.phase('ROW_SEARCH_HINTS',source_hypotheses=len(scan['target_hypotheses']))
            return scan['target_hypotheses']
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
        completed=self.retired+[p['measurement']['base_surface_center_m'] for p in self.placements]+[p['evidence']['occupant_position_m'] for p in self.skipped_slots]
        self.unresolved=[dict(search_seed=p['scan_target_id'],class_name=p['class_name'],
                              position_m=p['base_surface_center_m'],
                              reason='not independently picked and verified in row')
                         for p in candidates if not any(distance(p['base_surface_center_m'][:2],q[:2])<=
                             self.c['retired_radius_m'] for q in completed)]

    def verify_existing_piece(self, io, seed):
        slot=ordered_row_slots(self.c)[self.slot_index]
        if distance(seed['base_surface_center_m'][:2],slot)>self.c['reassociate_radius_m']:
            return False
        self.phase('VERIFY_EXISTING_ROW',slot_index=self.slot_index,search_seed=seed['scan_target_id'])
        io.end_pick()
        view_xy=seed['base_surface_center_m'][:2]
        io.move('PREGRASP',[*view_xy,self.c['hover_tcp_base_z_m']],'empty')
        try:
            source=io.reobserve_row_occupant(slot,None,view_xy)
        except ObservationUnavailable as error:
            self.phase('EXISTING_ROW_UNCONFIRMED',slot_index=self.slot_index,reason=str(error))
            raise TargetUnavailable(seed) from error
        if distance(source['base_surface_center_m'][:2],slot)>self.c.get('row_placement_tolerance_m',self.c['descent_tolerance_m']):
            return False
        # 再采一个独立窗口确认原位已摆正；不闭爪、不下降、不搬动邻近棋子。
        try:
            measured=io.verify_row_placement(slot,source,keep_view=True)
        except ObservationUnavailable as error:
            self.phase('EXISTING_ROW_UNCONFIRMED',slot_index=self.slot_index,reason=str(error))
            raise TargetUnavailable(seed) from error
        self.placements.append(dict(slot_index=self.slot_index,physical_slot_index=self.c['row_slot_order'][self.slot_index],slot_xy_m=slot,
            source_position_m=source['base_surface_center_m'],measurement=measured,verified_in_place=True))
        self.retired.append(source['base_surface_center_m'])
        self.layers+=1
        self.slot_index+=1
        self.object_state='empty'
        self.phase('ROW_PLACED',slot_index=self.slot_index-1,measurement=measured,verified_in_place=True)
        return True

    def defer_blocked_slot(self, source_position, occupied_checks):
        # 只在既定槽位里调整执行顺序，给所有尚未处理的扫描提示保留抓取空间。
        if not self.search_candidates:
            raise RowSlotOccupied(dict(occupied_checks[-1],reason='no scan inventory available for occupied-slot rescheduling'))
        points=[candidate['base_surface_center_m'] for candidate in self.search_candidates]
        points.extend(check['occupant_position_m'] for check in occupied_checks)
        points=[point for point in points if
            distance(point[:2],source_position[:2])>self.c['retired_radius_m'] and
            not excluded_target(self.c,point,self.retired,self.slot_index)]
        clearance=self.c['neighbor_clearance_m']+self.c['reassociate_radius_m']
        order=self.c['row_slot_order']
        blocked={check['physical_slot_index'] for check in occupied_checks}
        alternate=next((index for index in range(self.slot_index+1,self.c['max_pieces'])
            if order[index] not in blocked and all(distance(self.c['row_slots_m'][order[index]],point[:2])>=clearance
                                                  for point in points)),None)
        if alternate is None:
            raise RowSlotOccupied(dict(occupied_checks[-1],reason='occupied row slot has no alternative with preserved source clearance'))
        blocked_physical=order[self.slot_index]
        chosen=order.pop(alternate)
        order.insert(self.slot_index,chosen)
        self.phase('ROW_SLOT_DEFERRED',slot_index=self.slot_index,
            physical_slot_index=blocked_physical,buffer_physical_slot_index=chosen,
            slot_xy_m=self.c['row_slots_m'][blocked_physical],buffer_slot_xy_m=self.c['row_slots_m'][chosen],
            row_slot_order=list(order),source_clearance_m=clearance,occupant_evidence=occupied_checks[-1])

    def place_in_row(self, io, source_position, poses):
        occupied_checks=[]
        while self.slot_index<self.c['max_pieces']:
            poses=target_poses(self.c,self.source_measurement,self.slot_index)
            slot=poses['place'][:2]
            self.phase('TRANSFER_ROW',slot_index=self.slot_index,slot_xy_m=slot)
            io.move('PREGRASP',poses['preplace'],'held')
            try:
                self.phase('VERIFY_EMPTY_SLOT',slot_index=self.slot_index)
                io.check_row_slot_empty(slot)
                self.phase('DESCEND_ROW',slot_index=self.slot_index)
                io.move('DESCEND',poses['place'],'held')
                break
            except RowSlotOccupied as error:
                if self.c.get('row_occupied_policy')=='skip':
                    self.skip_occupied_slot(error.evidence)
                    if self.slot_index>=self.c['max_pieces']:
                        raise RuntimeError('all remaining row slots are occupied; retaining held piece') from error
                    io.raise_row_transfer()
                else:
                    io.move('PREGRASP',[*slot,self.c['transfer_tcp_base_z_m']],'held')
                    occupied_checks.append(dict(error.evidence,
                        physical_slot_index=self.c['row_slot_order'][self.slot_index]))
                    self.defer_blocked_slot(source_position,occupied_checks)
        io.verify_row_release_pose(poses['place'])
        self.object_state='unknown'
        self.phase('RELEASE_ROW',slot_index=self.slot_index)
        io.release()
        self.object_state='released_unverified'
        self.phase('WITHDRAW_ROW',slot_index=self.slot_index)
        io.move('RETREAT',poses['preplace'],'released_unverified')
        self.phase('VERIFY_ROW_PLACEMENT',slot_index=self.slot_index)
        try:
            measured=io.verify_row_placement(slot,self.source_measurement)
        except ObservationUnavailable as error:
            if io.gripper_state(idle=True).object_detected:
                raise RuntimeError('gripper still detects object after row release')
            self.pending_slots.append(dict(slot_index=self.slot_index,physical_slot_index=self.c['row_slot_order'][self.slot_index],slot_xy_m=slot,
                source_position_m=source_position,source_measurement=self.source_measurement,
                reason=str(error)))
            self.retired.append(source_position)
            self.slot_index+=1
            self.object_state='empty'
            self.phase('ROW_PLACEMENT_PENDING',reason=str(error))
            return
        placed_slot=self.slot_index
        self.placements.append(dict(slot_index=placed_slot,physical_slot_index=self.c['row_slot_order'][placed_slot],slot_xy_m=slot,
                                    source_position_m=source_position,measurement=measured))
        self.retired.append(source_position)
        self.layers+=1
        self.slot_index+=1
        self.object_state='empty'
        self.phase('ROW_PLACED',slot_index=placed_slot,measurement=measured)

    def recheck_pending_slots(self, io):
        for pending in list(self.pending_slots):
            self.phase('RECHECK_PENDING_ROW',slot_index=pending['slot_index'])
            try:
                measured=io.verify_row_placement(pending['slot_xy_m'],pending['source_measurement'])
            except ObservationUnavailable as error:
                pending['reason']=str(error)
                continue
            self.placements.append(dict(slot_index=pending['slot_index'],
                physical_slot_index=self.c['row_slot_order'][pending['slot_index']],slot_xy_m=pending['slot_xy_m'],
                source_position_m=pending['source_position_m'],measurement=measured))
            self.placements.sort(key=lambda placement:placement['slot_index'])
            self.pending_slots.remove(pending)
            self.layers+=1
            self.phase('ROW_PLACED',slot_index=pending['slot_index'],measurement=measured,rechecked=True)

    def run(self, io):
        try:
            if self.c.get('resume_row_result_file'):
                self.resume_held_row(io)
            else:
                self.prepare_startup(io)
                self.phase('PRECHECK')
                io.precheck(self.c)
            candidates=self.scan_workspace(io)
            self.search_candidates=candidates
            conflicts=row_destination_conflicts(self.c,candidates,self.slot_index)
            if conflicts:
                self.phase('ROW_DESTINATION_OCCUPANTS',search_hints=conflicts,
                           handling='confirm occupied slots and skip; preserve all row occupants' if self.c.get('row_occupied_policy')=='skip' else 'current slot occupants first')
            self.update_unresolved(candidates)
            initial_unavailable=list(self.unavailable)
            initial_layers=self.layers
            rechecked_skipped=False
            rescanned=False
            while self.slot_index<self.c['max_pieces']:
                if self.c.get('row_occupied_policy')=='skip' and any(
                        row_slot_occupied(self.c,p['base_surface_center_m'],ordered_row_slots(self.c)[self.slot_index]) for p in candidates):
                    slot=ordered_row_slots(self.c)[self.slot_index]
                    self.phase('CHECK_ROW_OCCUPANT',slot_index=self.slot_index,slot_xy_m=slot)
                    io.move('PREGRASP',[*slot,self.c['transfer_tcp_base_z_m']],'empty')
                    evidence=io.inspect_row_slot(slot)
                    if evidence['occupied']:
                        self.skip_occupied_slot(evidence)
                        self.update_unresolved(candidates)
                        continue
                available=[p for p in candidates if not any(distance(p['base_surface_center_m'][:2],q[:2])<=
                           self.c['retired_radius_m'] for q in self.retired)]
                try:
                    seed=select_target(self.c,available,self.retired,self.slot_index,self.unavailable)
                except RuntimeError:
                    if (not rechecked_skipped and self.layers>initial_layers and
                            len(self.unavailable)>len(initial_unavailable)):
                        # 其他棋子已搬走，只重查尚未抓取的候选一轮；不重抓失败持物。
                        rechecked_skipped=True
                        self.phase('ROW_RECHECK_SKIPPED',skipped_count=len(self.unavailable)-len(initial_unavailable))
                        self.unavailable=list(initial_unavailable)
                        continue
                    if not rescanned and self.unavailable and self.object_state=='empty':
                        rescanned=True
                        self.phase('ROW_RESCAN',unavailable_source_positions=self.unavailable,
                                   completed_placements=len(self.placements),prior_scan_unresolved=self.scan_unresolved)
                        self.return_home(io,'HOME_BEFORE_ROW_RESCAN')
                        candidates=self.scan_workspace(io,rescan=True)
                        self.search_candidates=candidates
                        self.unavailable=list(initial_unavailable)
                        self.update_unresolved(candidates)
                        continue
                    break
                try:
                    self.return_home(io,'HOME_BEFORE_ROW_PICK')
                    if self.c.get('row_occupied_policy')!='skip' and self.verify_existing_piece(io,seed):
                        self.update_unresolved(candidates)
                        continue
                    source,poses=self.grasp_piece(io,available)
                except TargetUnavailable as e:
                    self.unavailable.append(e.seed['base_surface_center_m'])
                    self.phase('TARGET_SKIPPED',search_seed=e.seed['scan_target_id'],reason=str(e))
                    continue
                self.place_in_row(io,source,poses)
                self.update_unresolved(candidates)
            self.recheck_pending_slots(io)
            self.return_home(io)
            self.phase('ROW_COMPLETE' if self.goal_complete() else 'ROW_PARTIAL',row_goal_complete=self.goal_complete(),search_inventory_processed=not self.unresolved and not self.scan_unresolved and not self.pending_slots,
                       placed_count=len(self.placements),processed_slot_count=self.slot_index,skipped_slots=self.skipped_slots,unresolved=self.unresolved,scan_unresolved=self.scan_unresolved,
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
    workspace=parser.add_mutually_exclusive_group()
    workspace.add_argument('--workspace-clear',action='store_true',help='confirm intended bare-table destinations and hands/ruler clear for this run')
    workspace.add_argument('--workspace-ready',action='store_true',help='row tabletop and hands clear; occupied destinations follow the configured skip policy')
    parser.add_argument('--authorization',help='record of explicit current-run physical authorization; not inferred from history')
    args = parser.parse_args()
    config = json.loads(Path(args.config).read_text())
    if args.rehearse and config.get('business_mode')=='row':
        parser.error('row mode has no synthetic rehearsal dispatcher')
    if args.workspace_clear:
        config['workspace_observation']='Operator explicitly passed --workspace-clear for this execution'
    if args.workspace_ready:
        if config.get('business_mode')!='row':
            parser.error('--workspace-ready is only supported by row mode')
        config['workspace_observation']='Operator confirmed tabletop ready and hands clear; row occupants follow configured policy; live source/slot guards retained'
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
