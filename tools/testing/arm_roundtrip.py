#!/usr/bin/env python3
"""Finite Home -> exact TCP -> formal Home business-node acceptance.

No motion is sent except MoveArm actions. The tool never directly calls enable,
reset or initialize-reference APIs. Explicit, human-evidenced power-cycle recovery
may request enable through the business node after history restore. Import opens no sockets.
"""
import argparse
import csv
import gzip
import hashlib
import json
import math
import pathlib
from pathlib import Path
import re
import signal
import subprocess
import time


def sha(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    path = pathlib.Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    temporary.replace(path)


def load_home(root):
    """Existing formal YAML plus the matching nominal model, never a new Home."""
    import sys
    import numpy as np
    import yaml
    from scipy.spatial.transform import Rotation
    sys.path.insert(0, str(root / 'tools/testing'))
    from arm_controller_fixture import Model  # read-only URDF FK, no fixture instance
    config = root / 'src/chess_robot_arm/config'
    arm = yaml.safe_load((config / 'arm.yaml').read_text())['chess_robot_arm']['ros__parameters']
    data = yaml.safe_load((config / 'home.yaml').read_text())['chess_robot_arm']['ros__parameters']['home']
    other = root / 'config/arm/home.yaml'
    if other.exists() and sha(other) != sha(config / 'home.yaml'):
        raise ValueError('formal Home YAML copies differ; do not redefine Home')
    q = data['joint_positions_rad']
    tcp = np.eye(4)
    tcp[:3, 3] = arm['tcp']['translation_m']
    tcp[:3, :3] = Rotation.from_euler('xyz', arm['tcp']['rpy_rad']).as_matrix()
    pose = Model(root).transform(q) @ tcp
    return {'joints_rad': q, 'tcp_m': pose[:3, 3].tolist(),
            'quaternion_xyzw': Rotation.from_matrix(pose[:3, :3]).as_quat().tolist(),
            'home_yaml_sha256': sha(config / 'home.yaml'), 'arm_parameters': arm}


def manifest(home):
    """26 sparse points, at most 12 bounded bisections, no automatic expansion."""
    origin = home['tcp_m']
    rays = {'left': [-1, 0, 0], 'right': [1, 0, 0],
            'front': [0, 1, 0], 'back': [0, -1, 0],
            'up': [0, 0, 1], 'down': [0, 0, -1]}
    points = []
    for name, vector in rays.items():
        for distance in [.15, .30, .60]:
            points.append({'id': f'{name}_{round(distance * 1000):03d}', 'ray': name,
                           'distance_m': distance, 'kind': 'coarse',
                           'tcp_m': [a + distance*b for a, b in zip(origin, vector)]})
    for height in [-.15, .15]:
        for name in ['left', 'right', 'front', 'back']:
            vector = rays[name]
            xyz = [a + .30*b for a, b in zip(origin, vector)]
            xyz[2] += height
            points.append({'id': f'{"lower" if height < 0 else "upper"}_{name}',
                           'kind': 'height_slice', 'tcp_m': xyz})
    return {'schema': 'chess_robot.home_roundtrip/v1', 'home': home | {'arm_parameters': None},
            'quaternion_xyzw': home['quaternion_xyzw'], 'rays': rays,
            'coarse_points': points, 'max_refinements_per_ray': 2,
            'refinement_min_interval_m': .025, 'maximum_targets': 38,
            'maximum_roundtrip_attempts_including_qualification': 40,
            'maximum_motion_legs_including_optional_recovery': 81,
            'workspace_bounds': home.get('arm_parameters',{}).get('workspace',
                {'min_m':[-.8,-.8,0.], 'max_m':[.8,.8,1.2]}),
            'estimated_duration_minutes': [10, 25],
            'estimate_note': 'Replace with checked outbound + return durations before actuation; '
                             'includes planning, feedback dwell and finite boundary refinement.',
            'range_note': 'Coordinates are exact TCP endpoints in base_link. Directions are '
                          'base-frame axes, not camera left/right. All-pass rays are explicitly '
                          'unbracketed; no global workspace or continuous-volume claim.'}


def validate_manifest(data, home):
    if data.get('schema') != 'chess_robot.home_roundtrip/v1':
        raise ValueError('unsupported manifest schema')
    if data['home']['home_yaml_sha256'] != home['home_yaml_sha256']:
        raise ValueError('manifest formal Home changed')
    if data['home']['joints_rad'] != home['joints_rad']:
        raise ValueError('manifest may not redefine Home')
    n = len(data['coarse_points']) + len(data['rays'])*int(data['max_refinements_per_ray'])
    if n > int(data['maximum_targets']) or n > 128:
        raise ValueError('manifest exceeds finite target budget')
    if not 0 <= int(data['max_refinements_per_ray']) <= 8:
        raise ValueError('invalid finite refinement count')
    ids = set()
    for p in data['coarse_points']:
        if p['id'] in ids or not re.fullmatch('[a-zA-Z0-9_-]+', p['id']):
            raise ValueError('duplicate or unsafe point ID')
        ids.add(p['id'])
        if len(p['tcp_m']) != 3 or not all(math.isfinite(x) for x in p['tcp_m']):
            raise ValueError('invalid exact TCP target')


def return_precheck(detail):
    if 'home_return_checked=true' not in detail:
        raise ValueError('node did not verify predicted target -> formal Home path')
    found = re.search(r'home_return_duration_s=([0-9.eE+-]+)', detail)
    if not found or not math.isfinite(float(found[1])) or float(found[1]) <= 0:
        raise ValueError('node return precheck duration missing')
    return float(found[1])


def apply_controller_limits(params, limits):
    """Bind source-backed absolute limits to actual planner parameters, no percent conversion."""
    if limits.get('model') != 'Nova5' or limits.get('firmware_version') != params['controller']['firmware_version']:
        raise ValueError('current controller limit model/firmware mismatch')
    source=limits.get('source')
    if not isinstance(source,dict) or not source.get('kind') or not source.get('reference'):
        raise ValueError('current absolute controller limits need explicit source kind/reference')
    if limits.get('units') != {'velocity':'rad/s','acceleration':'rad/s2'}:
        raise ValueError('current absolute controller limits need explicit rad/s and rad/s2 units')
    requested={name:dict(params['profiles']['roundtrip_'+name]) for name in ['low','high']}
    for key in ['velocity_rad_s','acceleration_rad_s2']:
        values=limits.get(key,[])
        if len(values)!=6 or not all(isinstance(v,(int,float)) and not isinstance(v,bool)
                                    and math.isfinite(v) and v>0 for v in values):
            raise ValueError('invalid 6-axis absolute controller limits: '+key)
        params['limits'][key]=[min(float(a),float(b)) for a,b in zip(params['limits'][key],values)]
    effective={}
    for name in ['low','high']:
        profile=params['profiles']['roundtrip_'+name]
        profile['speed_rad_s']=min(profile['speed_rad_s'],max(params['limits']['velocity_rad_s']))
        profile['acceleration_rad_s2']=min(profile['acceleration_rad_s2'],max(params['limits']['acceleration_rad_s2']))
        effective[name]={'requested':requested[name],'node_profile':dict(profile),
            'per_axis_velocity_rad_s':[min(profile['speed_rad_s'],v) for v in params['limits']['velocity_rad_s']],
            'per_axis_acceleration_rad_s2':[min(profile['acceleration_rad_s2'],v) for v in params['limits']['acceleration_rad_s2']]}
    return effective



def apply_command_policy(params, policy):
    """Apply an explicit user planning request, without asserting hardware ratings."""
    if policy.get('schema') != 'chess_robot.command_policy/v1' or not policy.get('user_statement'):
        raise ValueError('explicit user command policy and statement required')
    if policy.get('hardware_limits_verified') is not False:
        raise ValueError('command policy must not masquerade as verified hardware limits')
    v,a=policy.get('velocity_cap_rad_s'),policy.get('acceleration_cap_rad_s2')
    if not all(isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x) and x>0 for x in [v,a]):
        raise ValueError('invalid commanded planning caps')
    params['limits']['velocity_rad_s']=[min(x,v) for x in params['limits']['velocity_rad_s']]
    params['limits']['acceleration_rad_s2']=[min(x,a) for x in params['limits']['acceleration_rad_s2']]
    effective={}
    for name in ['low','high']:
        requested=policy['profiles'][name]
        speed,acc=requested['speed_rad_s'],requested['acceleration_rad_s2']
        if not all(isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x) and x>0 for x in [speed,acc]) or speed>v or acc>a:
            raise ValueError('profile exceeds explicit command policy')
        profile=params['profiles']['roundtrip_'+name]
        profile['speed_rad_s']=min(profile['speed_rad_s'],speed,max(params['limits']['velocity_rad_s']))
        profile['acceleration_rad_s2']=min(profile['acceleration_rad_s2'],acc,max(params['limits']['acceleration_rad_s2']))
        effective[name]={'requested':requested,'node_profile':dict(profile),
            'per_axis_velocity_rad_s':[min(profile['speed_rad_s'],x) for x in params['limits']['velocity_rad_s']],
            'per_axis_acceleration_rad_s2':[min(profile['acceleration_rad_s2'],x) for x in params['limits']['acceleration_rad_s2']],
            'basis':'explicit user planning policy; hardware limits remain unverified'}
    return effective


def boundary_bracket(records, ray):
    """Only measured round-trip successes/reused successes bracket physical range."""
    points = [p for p in records if p.get('ray') == ray]
    passed = [p['distance_m'] for p in points if p['status'] in ['roundtrip_success', 'reused_success']]
    rejected = [p['distance_m'] for p in points if p['status'] == 'planning_rejected']
    low = max(passed, default=0.0)  # already verified formal Home is distance zero
    high = min((d for d in rejected if d > low), default=None)
    return low, high


class ExecutionFault(RuntimeError):
    pass


class PreSendFeedbackFault(TimeoutError):
    def __init__(self,message,observation=None):
        super().__init__(message)
        self.observation=observation


def load_power_cycle_evidence(path):
    data=json.loads(Path(path).read_text())
    if not data.get('human_statement') or data.get('only_power_cycle_no_manual_joint_or_cable_motion') is not True:
        raise ValueError('power-cycle handoff needs actual human confirmation of power only, no manual joint/cable motion')
    prior=data.get('prior_fault_result',{})
    if not prior.get('path') or not prior.get('sha256') or sha(prior['path'])!=prior['sha256']:
        raise ValueError('power-cycle evidence prior fault result missing/changed')
    original=json.loads(Path(prior['path']).read_text())
    if not original.get('status','').startswith(('EXECUTION_FAULT','PRE_SEND_FEEDBACK_FAULT')):
        raise ValueError('power-cycle evidence must bind the unchanged prior fault result')
    return {'path':str(Path(path).resolve()),'sha256':sha(path),'data':data,
            'prior_status':original['status'],'epoch_handoff_explicit':True}


def configure_power_cycle_handoff(params,evidence,execute):
    explicit=bool(execute and evidence)
    params['reference']['power_cycle_handoff']=explicit
    params['controller']['enable_after_history_restore']=explicit
    return explicit


def continuation_safety(context):
    # Source/binary and explicit recovery controls may change in a diagnosed new
    # batch; physical/model/motion policy invariants may not silently change.
    keys=['home_sha256','geometry_sha256','urdf_sha256','sdk_manifest_sha256','sdk_patch_sha256',
          'tcp','profiles','limits','exploration','firmware_version','controller_ip',
          'controller_limit_sha256','command_policy_sha256','motion_parameters']
    missing=[key for key in keys if key not in context]
    if missing:raise ValueError('continuation lacks safety context: '+','.join(missing))
    selected=json.loads(json.dumps({key:context[key] for key in keys}))
    selected['motion_parameters'].get('reference',{}).pop('power_cycle_handoff',None)
    return selected


def load_continuation(path,statement_path,context,data,fixture=False):
    if not statement_path:raise ValueError('explicit continuation needs current human statement JSON')
    statement=json.loads(Path(statement_path).read_text())
    if not statement.get('human_statement'):raise ValueError('continuation statement must retain actual human words')
    previous=json.loads(Path(path).read_text())
    if previous.get('status') in ['COMPLETE','COMPLETE_WITH_UNBRACKETED_RAYS','PLANNING_ONLY']:
        raise ValueError('continue-results is for an explicitly authorized partial/faulted batch; use reuse-results for complete batches')
    if previous.get('simulation') is not fixture:
        raise ValueError('continuation real/simulation source differs')
    old=previous.get('qualification_context',{})
    if continuation_safety(old)!=continuation_safety(context):
        raise ValueError('continuation Home/TCP/URDF/geometry/SDK/profile/limits/firmware/IP/command-policy safety invariant changed')
    if previous['home']['home_yaml_sha256']!=data['home']['home_yaml_sha256']:
        raise ValueError('continuation formal Home differs')
    expected={p['id']:p for p in data['coarse_points']}
    successes={}
    for point in previous.get('targets',[]):
        if point.get('status') not in ['roundtrip_success','reused_success']:continue
        if point['id'] not in expected or point['tcp_m']!=expected[point['id']]['tcp_m']:
            raise ValueError('continuation target identity/coordinates differ')
        for leg in ['outbound','return_home']:
            measured=point.get(leg) or {}
            if measured.get('code')!=1 or measured.get('executed') is not True or not measured.get('final_tcp_valid'):
                raise ValueError('historical success lacks both measured legs')
            if (measured.get('after') or {}).get('simulated') is not fixture:
                raise ValueError('historical measured feedback source differs')
        successes[point['id']]=point
    if not successes:raise ValueError('partial continuation has no valid measured successes to retain')
    differences={key:{'previous':old.get(key),'current':context.get(key)}
                 for key in sorted(old.keys()|context.keys()) if old.get(key)!=context.get(key)}
    return {'source_results_path':str(Path(path).resolve()),'source_results_sha256':sha(path),
       'previous_status_preserved':previous['status'],'previous_qualification_context':old,
       'current_context_delta':differences,'safety_invariants_equal':True,
       'statement':{'path':str(Path(statement_path).resolve()),'sha256':sha(statement_path),'data':statement},
       'successes':successes,'historical_success_count':len(successes),
       'note':'Only old measured successes retained with their original timestamps/context; no new arrivals claimed.'}


class Runner:
    """Orchestration independent of ROS; tested with deterministic session fakes."""
    def __init__(self, data, session, out, execute=False, reuse=None, continuation=None):
        self.data, self.session, self.out, self.execute = data, session, pathlib.Path(out), execute
        self.report = {'schema': data['schema'], 'status': 'STARTING', 'execute': execute,
                       'home': data['home'], 'fixed_quaternion_xyzw': data['quaternion_xyzw'],
                       'manifest': data, 'targets': [], 'boundaries': {},
                       'reference_initializations': 0, 'reference_resets': 0,
                       'gripper_operations': 0, 'started_utc_ns': time.time_ns()}
        self.reuse = {}
        self.historical_ids=set()
        if reuse and continuation:raise ValueError('reuse-results and continue-results are mutually exclusive')
        if continuation:
            self.reuse=dict(continuation['successes'])
            self.historical_ids=set(self.reuse)
            self.report['continuation']={key:value for key,value in continuation.items() if key!='successes'}
        if reuse:
            if reuse.get('status') not in ['COMPLETE', 'COMPLETE_WITH_UNBRACKETED_RAYS']:
                raise ValueError('a faulted/partial batch cannot be automatically resumed')
            if reuse['home']['home_yaml_sha256'] != data['home']['home_yaml_sha256']:
                raise ValueError('reuse Home provenance differs')
            context=getattr(session,'qualification_context',None)
            if context is not None and reuse.get('qualification_context')!=context:
                raise ValueError('reuse binary/source/Home/TCP/geometry/profile/limit context differs')
            self.reuse = {p['id']: p for p in reuse['targets'] if p['status'] in ['roundtrip_success','reused_success']}

    def save(self):
        write_json(self.out / 'results.json', self.report)

    def checked_plan(self, point, profile):
        result = self.session.action(point, profile, plan_only=True)
        if result['code'] != 0:
            text = result['detail'].lower()
            if any(x in text for x in ['socket deadline','connection closed','ambiguous session',
                                       'echo mismatch','partial reply','nonfinite sdk','invalid sdk']):
                raise ExecutionFault('protocol/reference fault is not a geometric rejection: '+result['detail'])
            if result['code'] == -1 and not any(x in text for x in ['tcp outside configured workspace','grasp axis is not downward']):
                raise ExecutionFault('unexpected action admission rejection: '+result['detail'])
            return result
        result['home_return_duration_s'] = return_precheck(result['detail'])
        result['pair_planned_duration_s'] = result['planned_duration_s'] + result['home_return_duration_s']
        return result

    def plan(self, point):
        high = self.checked_plan(point, 'roundtrip_high')
        plans = {'initial': high}
        if high['code'] != 0:
            return high, plans
        profile, why, margins = self.session.choose_profile(point, high)
        plans['profile_reason'], plans['margins'] = why, margins
        selected = high if profile == 'roundtrip_high' else self.checked_plan(point, profile)
        plans['selected'] = selected
        return selected, plans

    def target(self, point, prepared=None):
        record = dict(point, status='pending', outbound=None, return_home=None)
        self.report['targets'].append(record)
        self.save()
        old = self.reuse.get(point['id'])
        if old and old['tcp_m'] == point['tcp_m'] and old.get('return_home', {}).get('code') == 1:
            record.update(status='reused_success', reused_record=old,
                          outbound=old['outbound'],return_home=old['return_home'],profile=old.get('profile'),
                          reuse_note='Prior measured legs retained with original timestamps; no new execution claimed',
                          reuse_origin='historical_continuation' if point['id'] in self.historical_ids else 'prior_measured_success')
            self.save()
            return
        self.session.require_home()
        selected, plans = prepared if prepared else self.plan(point)
        if prepared:
            record['preflight_planning']=plans
        if self.execute:
            # The node replans execution from live feedback/history. Cached
            # whole-batch preflight paths may not select that motion's speed.
            if plans.get('qualification'):
                selected=self.checked_plan(point,selected['profile'])
                fresh_profile,why,margins=self.session.choose_profile(point,selected) if selected['code']==0 else (None,[],None)
                if selected['profile']=='roundtrip_high' and fresh_profile!='roundtrip_high':
                    raise ValueError('fresh high qualification path is now near a constraint')
                plans={'selected':selected,'qualification':True,'profile_reason':why,'margins':margins}
            else:
                selected,plans=self.plan(point)
        record['planning'] = plans
        if selected['code'] != 0:
            if selected.get('executed') or selected['code'] in [12, 13, 19]:
                raise ExecutionFault('non-planning fault during precheck: ' + selected['detail'])
            record.update(status='planning_rejected', failure_reason=selected['detail'])
            self.save()
            return
        record['profile'] = selected['profile']
        record['estimated_pair_motion_s'] = selected['pair_planned_duration_s']
        if not self.execute:
            record['status'] = 'planning_passed'
            self.save()
            return
        # No return is sent unless the outbound terminal result AND fresh measured
        # state independently confirm the exact TCP endpoint and fixed attitude.
        started = time.monotonic()
        try:
            record['outbound'] = self.session.action(point, selected['profile'], plan_only=False)
            self.save()
            self.session.require_arrival(record['outbound'], point['tcp_m'], self.data['quaternion_xyzw'])
            record['return_home'] = self.session.action(None, selected['profile'], plan_only=False)
            self.save()
            self.session.require_arrival(record['return_home'], self.data['home']['tcp_m'], self.data['quaternion_xyzw'])
            self.session.require_home()
            record['status'] = 'roundtrip_success'
        except Exception as error:
            record.update(status='execution_failed', failure_reason=str(error))
            record['stop_confirmation'] = self.session.confirm_stopped()
            if isinstance(error,PreSendFeedbackFault):
                record['status']='pre_send_feedback_failed'
                record['fault_stage']='before_action_submission'
                record['feedback_diagnostic']=error.observation
                self.report['status']='PRE_SEND_FEEDBACK_FAULT_STOPPED' if record['stop_confirmation']['confirmed'] else 'PRE_SEND_FEEDBACK_FAULT_STOP_UNCONFIRMED'
            else:
                record['fault_stage']='after_action_submission_or_terminal_check'
                self.report['status'] = 'EXECUTION_FAULT_STOPPED' if record['stop_confirmation']['confirmed'] else 'EXECUTION_FAULT_STOP_UNCONFIRMED'
            raise ExecutionFault(str(error)) from error
        finally:
            record['pair_elapsed_s'] = time.monotonic() - started
            self.save()

    def run(self):
        try:
            self.session.require_home()
        except ValueError:
            if not self.execute or not getattr(self.session.args,'recover_home',False):
                raise
            self.session.require_reference_idle()
            recovery={'status':'PLANNING','profile':'roundtrip_low','plan':None,'execution':None}
            self.report['recovery_home']=recovery
            recovery['plan']=self.session.action(None,'roundtrip_low',plan_only=True)
            if recovery['plan']['code']!=0:
                raise ValueError('explicit recovery Home planning rejected: '+recovery['plan']['detail'])
            self.report['status']='RECOVERING_FORMAL_HOME'
            self.save()
            print(json.dumps({'explicit_recovery_home_plan':recovery['plan']},ensure_ascii=False),flush=True)
            try:
                recovery['execution']=self.session.action(None,'roundtrip_low',plan_only=False)
                self.session.require_arrival(recovery['execution'],self.data['home']['tcp_m'],self.data['quaternion_xyzw'])
                self.session.require_home()
                recovery['status']='MEASURED_HOME_REACHED'
                self.save()
            except Exception as error:
                recovery.update(status='EXECUTION_FAILED',failure_reason=str(error),stop_confirmation=self.session.confirm_stopped())
                self.report['status']='EXECUTION_FAULT_STOPPED' if recovery['stop_confirmation']['confirmed'] else 'EXECUTION_FAULT_STOP_UNCONFIRMED'
                self.save()
                raise ExecutionFault(str(error)) from error
        self.report['status'] = 'PREFLIGHT'
        # Before first actuation, check every coarse outbound + predicted return
        # from the actual formal Home. This also creates a finite timing estimate.
        prepared = {}
        self.report['coarse_preflight']={}
        for point in self.data['coarse_points']:
            if point['id'] in self.historical_ids:
                self.report['coarse_preflight'][point['id']]={'status':'historical_measured_reuse',
                    'point':point,'source_results_sha256':self.report['continuation']['source_results_sha256']}
                self.save()
                continue
            item={'status':'planning','point':point}
            self.report['coarse_preflight'][point['id']]=item
            self.save()
            try:
                prepared[point['id']] = self.plan(point)
                selected,plans=prepared[point['id']]
                item.update(status='planning_passed' if selected['code']==0 else 'planning_rejected',selected=selected,plans=plans)
            except Exception as error:
                item.update(status='preflight_fault',error=repr(error),
                    feedback_diagnostic=getattr(error,'observation',None))
                self.save()
                raise
            self.save()
        self.report['preflight_estimate_s'] = {
            'accepted_pair_motion': sum(x[0].get('pair_planned_duration_s', 0) for x in prepared.values()),
            'coarse_count': len(self.data['coarse_points']), 'new_preflight_count':len(prepared),
            'historical_reused_count':len(self.historical_ids), 'maximum_targets': self.data['maximum_targets'],
            'note': 'motion times exclude repeated SDK planning, feedback dwell and optional bisection'}
        self.save()
        print(json.dumps({'preflight': self.report['preflight_estimate_s']}, ensure_ascii=False), flush=True)
        if getattr(self.session,'needs_qualification',False):
            candidates=[p for p in self.data['coarse_points'] if p['id'] in prepared and prepared[p['id']][0]['code']==0
                        and prepared[p['id']][0]['profile']=='roundtrip_high']
            if self.historical_ids:
                # Requalify the new binary/epoch at one old safe high-mode point,
                # not by re-scanning or re-moving all historical successes.
                for point in self.data['coarse_points']:
                    old=self.reuse.get(point['id'])
                    if not old or old.get('profile')!='roundtrip_high':continue
                    current=self.plan(point)
                    self.report['coarse_preflight'][point['id']]['qualification_precheck']={'selected':current[0],'plans':current[1]}
                    self.save()
                    if current[0]['code']==0 and current[0]['profile']=='roundtrip_high':
                        candidates=[point]
                        break
            if not candidates:
                raise ValueError('no checked ordinary-margin qualification point; high mode remains unqualified')
            point=candidates[0]
            self.report['status']='QUALIFYING_SPEED_MODES'
            self.report['qualification_trials']=[]
            for profile in ['roundtrip_low','roundtrip_high']:
                trial=dict(point,id='qualification_'+profile)
                plan=self.checked_plan(trial,profile)
                if plan['code']!=0:
                    raise ValueError('speed qualification precheck rejected: '+plan['detail'])
                prior_count=len(self.report['targets'])
                try:
                    self.target(trial,(plan,{'selected':plan,'qualification':True}))
                finally:
                    # Preserve a failed qualification in its own ledger too.
                    # An exception must not leave it looking like an ordinary
                    # target while the qualification ledger remains empty.
                    if len(self.report['targets'])>prior_count:
                        record=self.report['targets'].pop()
                        self.report['qualification_trials'].append(record)
                    self.save()
            evidence=self.session.make_speed_evidence(self.report['qualification_trials'])
            write_json(self.out/'speed_qualification.json',evidence)
            self.report['speed_qualification']='speed_qualification.json'
            if point['id'] not in self.historical_ids:
                self.reuse[point['id']]=self.report['qualification_trials'][-1]
            self.session.needs_qualification=False
        self.report['status'] = 'RUNNING'
        self.save()
        for point in self.data['coarse_points']:
            # A preflight plan is evidence, never a replayable motion input. Node
            # always replans from fresh feedback when the execute action is sent.
            self.target(point, prepared.get(point['id']))
        if self.execute:
            for ray, vector in self.data['rays'].items():
                for step in range(self.data['max_refinements_per_ray']):
                    low, high = boundary_bracket(self.report['targets'], ray)
                    edge=False
                    if high is None:
                        axis=next(i for i,v in enumerate(vector) if v)
                        bound=self.data['workspace_bounds']['max_m' if vector[axis]>0 else 'min_m'][axis]
                        distance=abs(bound-self.data['home']['tcp_m'][axis])
                        if distance <= low+self.data['refinement_min_interval_m']:
                            break
                        edge=True
                    elif high-low <= self.data['refinement_min_interval_m']:
                        break
                    else:
                        distance = (low+high)/2
                    point = {'id': f'{ray}_{"edge" if edge else "bisect"}_{step+1}', 'ray': ray, 'kind': 'refinement',
                             'distance_m': distance,
                             'tcp_m': [a+distance*b for a, b in zip(self.data['home']['tcp_m'], vector)]}
                    if len(self.report['targets']) >= self.data['maximum_targets']:
                        raise ValueError('finite target budget exhausted')
                    self.target(point)
        for ray in self.data['rays']:
            low, high = boundary_bracket(self.report['targets'], ray)
            vector=self.data['rays'][ray]
            axis=next(i for i,v in enumerate(vector) if v)
            edge=self.data['workspace_bounds']['max_m' if vector[axis]>0 else 'min_m'][axis]
            edge_distance=abs(edge-self.data['home']['tcp_m'][axis])
            boundary_status=('bracketed_sampled_boundary' if high is not None else
                             'configured_workspace_edge_reached' if self.execute and abs(edge_distance-low)<=self.data['refinement_min_interval_m'] else
                             'unbracketed_not_global_limit')
            self.report['boundaries'][ray] = {'farthest_measured_roundtrip_m': low,
                'nearest_farther_planning_rejection_m': high,
                'remaining_bracket_width_m': high-low if high is not None else None,
                'configured_workspace_edge_distance_m':edge_distance,'status':boundary_status}
        self.report['status'] = ('PLANNING_ONLY' if not self.execute else
             'COMPLETE_WITH_UNBRACKETED_RAYS' if any(x['status']=='unbracketed_not_global_limit'
                                                  for x in self.report['boundaries'].values()) else 'COMPLETE')
        self.report['finished_utc_ns'] = time.time_ns()
        self.save()
        return self.report


class RosSession:
    def __init__(self, root, out, data, args):
        import os
        import numpy as np
        import yaml
        import rclpy
        from rclpy.action import ActionClient
        from chess_robot_interfaces.action import MoveArm
        from chess_robot_interfaces.msg import ArmState
        from chess_robot_interfaces.srv import RestoreReference
        from sensor_msgs.msg import JointState
        from scipy.spatial.transform import Rotation
        self.np, self.Rotation, self.rclpy, self.MoveArm = np, Rotation, rclpy, MoveArm
        self.root, self.out, self.data, self.args = root, out, data, args
        self.states, self.joints, self.active = {}, {}, None
        self.frames = []
        self.shutdown_requested=False
        self.shutdown_confirmation=None
        self.audit_files = [str(p.resolve()) for p in args.history_audit]
        self.restore_count = 0
        self.needs_qualification=False
        self.controller_limit_evidence=None
        self.command_policy_evidence=None
        self.effective_speed_modes=None
        self.power_cycle_evidence=None
        self.continuation=None
        self.last_state_observation=None
        evidence_path=getattr(args,'power_cycle_evidence',None)
        if evidence_path:
            self.power_cycle_evidence=load_power_cycle_evidence(evidence_path)
            write_json(out/'power_cycle_evidence.json',self.power_cycle_evidence)
        self.params = yaml.safe_load((root/'src/chess_robot_arm/config/arm.yaml').read_text())
        p = self.params['chess_robot_arm']['ros__parameters']
        p.update(backend='exploration' if not args.fixture else 'hardware',
                 test_fixture=args.fixture, approach_clearance_m=0.0,
                 urdf_file=str(root/'src/chess_robot_description/urdf/nova5_with_ag95.urdf'))
        p['testing'] = {'home_roundtrip_precheck': True}
        p['motion'] = {'paused': not args.execute}
        p['hardware']['enable'] = args.execute
        explicit_power_handoff=configure_power_cycle_handoff(p,self.power_cycle_evidence,args.execute)
        if args.fixture:
            p['hardware'] = {key: args.execute for key in p['hardware']}
        p['controller'].update(ip=args.controller_ip)
        if args.fixture_endpoints:
            endpoints = json.loads(args.fixture_endpoints.read_text())
            if endpoints['host'] != '127.0.0.1' or not endpoints['simulation']:
                raise ValueError('fixture endpoints must be local simulated data')
            p['controller'].update(dashboard_port=endpoints['dashboard_port'], feedback_port=endpoints['feedback_port'])
        p['profiles'] = {'default': 'roundtrip_low', 'names': ['roundtrip_low','roundtrip_high'],
                         'roundtrip_low': dict(p['profiles']['low']),
                         'roundtrip_high': dict(p['profiles']['high'])}
        exploration_config = root/'src/chess_robot_arm/config/exploration.yaml'
        exploration_parameters = yaml.safe_load(exploration_config.read_text())['chess_robot_arm']['ros__parameters']
        full = exploration_parameters['exploration']['full']
        # Retain the existing exploration endpoint/stop tolerances. Changing to
        # point transfers must not silently relax stationary speed or dwell.
        p['tolerances'].update(exploration_parameters['tolerances'])
        p['exploration'] = {'stages': ['roundtrip_low','roundtrip_high'],
                            'roundtrip_low': dict(full), 'roundtrip_high': dict(full)}
        p['safety'] = {'model_file': str(root/'src/chess_robot_arm/config/nova5_ag95_geometry.yaml')}
        p['audit'] = {'directory': str(out/'audit')}
        p['feedback']['freshness_s'] = .1
        p['servo'].update(command_time_s=0.0, max_pending=6)
        if args.fixture:
            p['arrival_timeout_s'] = 1.0
        policy_path=getattr(args,'command_policy',None)
        if args.execute and not args.fixture:
            if not getattr(args,'controller_limits',None) and not policy_path:
                raise ValueError('provide controller limit evidence or an explicit user-commanded planning policy; no motion')
            if getattr(args,'controller_limits',None):
                limits=json.loads(args.controller_limits.read_text())
                self.effective_speed_modes=apply_controller_limits(p,limits)
                self.controller_limit_evidence={'path':str(args.controller_limits.resolve()),
                    'sha256':sha(args.controller_limits),'data':limits}
            self.needs_qualification=not bool(args.speed_evidence)
        if policy_path:
            policy=json.loads(policy_path.read_text())
            self.effective_speed_modes=apply_command_policy(p,policy)
            self.command_policy_evidence={'path':str(policy_path.resolve()),'sha256':sha(policy_path),'data':policy}
        sources=list((root/'src/chess_robot_arm/src').glob('*.cpp'))+list((root/'src/chess_robot_arm/include/chess_robot_arm').glob('*.hpp'))
        sources.append(root/'tools/testing/arm_roundtrip.py')
        self.qualification_context={
            'binary_sha256':sha(root/'install/chess_robot_arm/lib/chess_robot_arm/arm_node'),
            'source_sha256':{str(x.relative_to(root)):sha(x) for x in sorted(sources)},
            'home_sha256':data['home']['home_yaml_sha256'],
            'geometry_sha256':sha(root/'src/chess_robot_arm/config/nova5_ag95_geometry.yaml'),
            'urdf_sha256':{name:sha(root/'src/chess_robot_description/urdf'/name)
                           for name in ['nova5.urdf','nova5_with_ag95.urdf']},
            'sdk_manifest_sha256':sha(root/'src/chess_robot_arm/third_party/dobot_v4/UPSTREAM.json'),
            'sdk_patch_sha256':sha(root/'src/chess_robot_arm/third_party/dobot_v4/LOCAL.patch'),
            'arm_config_sha256':sha(root/'src/chess_robot_arm/config/arm.yaml'),
            'exploration_config_sha256':sha(exploration_config),
            'motion_parameters':{key:p[key] for key in ['sampling','servo','feedback','reference','tolerances','twist',
                'arrival_timeout_s','stop_timeout_s','planning_timeout_s']},
            'tcp':p['tcp'],'profiles':p['profiles'],'limits':p['limits'],
            'exploration':p['exploration'],'firmware_version':p['controller']['firmware_version'],
            'controller_ip':args.controller_ip,
            'controller_limit_sha256':self.controller_limit_evidence['sha256'] if self.controller_limit_evidence else None,
            'command_policy_sha256':self.command_policy_evidence['sha256'] if self.command_policy_evidence else None,
            'restore_parameters':{'power_cycle_handoff':explicit_power_handoff,
                                  'enable_after_history_restore':explicit_power_handoff}}
        continued=getattr(args,'continue_results',None)
        if continued:
            if getattr(args,'reuse_results',None):raise ValueError('reuse-results and continue-results are mutually exclusive')
            self.continuation=load_continuation(continued,getattr(args,'continuation_statement',None),
                self.qualification_context,data,args.fixture)
            if self.power_cycle_evidence and self.power_cycle_evidence['data']['prior_fault_result']['sha256']!=self.continuation['source_results_sha256']:
                raise ValueError('power-cycle and continuation must bind the same unchanged prior fault result')
            # An explicitly changed binary/epoch must demonstrate its own modes.
            self.needs_qualification=bool(args.execute)
        if args.execute and not args.fixture and args.speed_evidence:
            evidence = json.loads(args.speed_evidence.read_text())
            if evidence.get('status') != 'PASS' or evidence.get('simulation') is not False:
                raise ValueError('speed evidence is not a real measured PASS')
            if not all(x in evidence.get('validated_profiles',[]) for x in ['low','high']):
                raise ValueError('both speed modes must be measured before the range batch')
            if evidence.get('firmware_version') != p['controller']['firmware_version']:
                raise ValueError('speed evidence firmware differs')
            if evidence.get('profiles') != {'low': p['profiles']['roundtrip_low'], 'high': p['profiles']['roundtrip_high']}:
                raise ValueError('speed evidence parameters differ')
            if evidence.get('qualification_context')!=self.qualification_context:
                raise ValueError('speed qualification binary/source/Home/TCP/geometry/profile/limit context differs')
            if not evidence.get('source_results') or not (evidence.get('controller_limit_evidence') or evidence.get('command_policy_evidence')):
                raise ValueError('measured results/controller limit evidence missing')
            if self.controller_limit_evidence and (evidence.get('controller_limit_evidence') or {}).get('sha256')!=self.controller_limit_evidence['sha256']:
                raise ValueError('speed evidence controller limits changed; requalification required')
            source=evidence['source_results']
            if sha(source['path'])!=source['sha256']:
                raise ValueError('speed qualification result evidence changed/missing')
            trials=json.loads(pathlib.Path(source['path']).read_text())
            if len(trials)!=2 or not all(x['status']=='roundtrip_success' and x['outbound']['code']==1
                   and x['return_home']['code']==1 for x in trials):
                raise ValueError('speed qualification lacks both measured successful legs in both modes')
        self.p = p
        write_json(out/'parameters.json', p)
        write_json(out/'effective_speed_modes.json',self.effective_speed_modes)
        (out/'parameters.yaml').write_text(yaml.safe_dump(self.params))
        rclpy.init()
        self.node = rclpy.create_node('arm_home_roundtrip_acceptance')
        prefix = '/simulation/arm' if args.fixture else '/arm'
        self.node.create_subscription(ArmState, prefix+'/state', self.receive_state, 100)
        self.node.create_subscription(JointState, prefix+'/joint_states', self.receive_joint, 100)
        self.client = ActionClient(self.node, MoveArm, prefix+'/move')
        self.restore = self.node.create_client(RestoreReference, prefix+'/restore_reference')
        command = [str(root/'install/chess_robot_arm/lib/chess_robot_arm/arm_node'),'--ros-args',
                   '--params-file',str(root/'src/chess_robot_arm/config/home.yaml'),
                   '--params-file',str(out/'parameters.yaml')]
        write_json(out/'command.json', command)
        self.log = (out/'node.log').open('w')
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT)
        self.wait(lambda: self.client.server_is_ready() and self.snapshot(False) is not None, 20)
        snapshot = self.snapshot()
        if snapshot['simulated'] != args.fixture:
            raise ValueError('real/simulation endpoint mismatch')
        if not args.fixture:
            if not self.audit_files:raise ValueError('explicit complete history audits required')
            if self.power_cycle_evidence:
                attestation={'no_unobserved_motion':True,
                    'user_statement':self.power_cycle_evidence['data']['human_statement'],
                    'observed_power_cycle':True,'power_cycle_evidence_sha256':self.power_cycle_evidence['sha256'],
                    'scope':'Known power-cycle handoff; no assertion that controller time or epoch stayed continuous.'}
            else:
                if not args.history_attestation:raise ValueError('explicit human no-motion attestation required')
                attestation = json.loads(args.history_attestation.read_text())
                if attestation.get('no_unobserved_motion') is not True or not attestation.get('user_statement'):
                    raise ValueError('human no-unobserved-motion statement missing')
            write_json(out/'history_attestation.json', attestation)
        self.wait(lambda: self.restore.service_is_ready(), 10)
        if self.audit_files:
            if explicit_power_handoff:
                stationary_since=None
                def before_restore_dwell():
                    nonlocal stationary_since
                    current=self.snapshot(False)
                    if current and current['mode'] in [4,5] and max(map(abs,current['speed_rad_s']))<=self.p['tolerances']['stationary_rad_s']:
                        stationary_since=stationary_since or time.monotonic()
                        return time.monotonic()-stationary_since>=self.p['tolerances']['dwell_s']
                    stationary_since=None
                    return False
                self.wait(before_restore_dwell,max(5,self.p['tolerances']['dwell_s']+3))
            request = RestoreReference.Request()
            request.audit_files = self.audit_files
            request.no_unobserved_motion = True  # explicit copied human attestation checked above
            response = self.future(self.restore.call_async(request), 60)
            if not response.success:
                raise ValueError('continuous history restore rejected: '+response.detail)
            self.restore_count += 1
            write_json(out/'reference_restore.json',{'success':response.success,'detail':response.detail,
                'power_cycle_handoff':explicit_power_handoff,'enable_requested_via_node':explicit_power_handoff,
                'controller_epoch_handoff_explicit':bool(self.power_cycle_evidence),
                'controller_time_converted_or_rewritten':False})
            if explicit_power_handoff:
                # The node performs GetErrorID + EnableRobot only after actual
                # history restore. This client does not issue either command.
                self.wait(lambda: self.snapshot(False) and self.snapshot()['reference_valid'] and
                    self.snapshot()['mode']==5 and max(map(abs,self.snapshot()['speed_rad_s']))<=self.p['tolerances']['stationary_rad_s'],15)
                ready=self.confirm_stopped() # existing fresh mode5/stationary/dwell observer; sends no Stop
                live=self.snapshot(False)
                ready_record={'observation':ready,'reference_valid':bool(live and live['reference_valid']),
                    'basis':'After node enable, reuse original fresh mode5 + stationary + dwell observation',
                    'stationary_rad_s':self.p['tolerances']['stationary_rad_s'],'dwell_s':self.p['tolerances']['dwell_s'],
                    'stop_command_sent_by_this_observation':False}
                write_json(out/'enabled_stationary_ready.json',ready_record)
                if not ready['confirmed'] or not ready_record['reference_valid']:
                    raise ValueError('enabled controller not ready with continuous reference and original stationary dwell')
            else:
                self.wait(lambda: self.snapshot(False) and self.snapshot()['reference_valid'], 3)
        elif not args.fixture:
            raise ValueError('never initialize real Home reference automatically')
        # Fixtures initialize explicitly in their harness, through the normal service.

    def receive_state(self, message):
        self.last_state_observation={'receipt_host_ns':time.time_ns(),
            'stamp_ns':message.header.stamp.sec*10**9+message.header.stamp.nanosec,
            'controller_mode':message.controller_mode,'feedback_fresh':message.feedback_fresh,
            'reference_valid':message.reference_valid,'diagnostic':message.diagnostic,
            'phase':message.phase,'simulated':message.simulated}
        key = (message.header.stamp.sec, message.header.stamp.nanosec)
        self.states[key] = message
        if len(self.states) > 100:
            self.states.pop(next(iter(self.states)))

    def receive_joint(self, message):
        key = (message.header.stamp.sec, message.header.stamp.nanosec)
        self.joints[key] = message
        if len(self.joints) > 100:
            self.joints.pop(next(iter(self.joints)))

    def snapshot(self, required=True):
        keys = self.states.keys() & self.joints.keys()
        if not keys:
            if required:
                raise ValueError('no synchronized joint/state feedback')
            return None
        key = max(keys)
        s, j = self.states[key], self.joints[key]
        age = self.node.get_clock().now().nanoseconds/1e9 - (key[0]+key[1]/1e9)
        if not s.feedback_fresh or age > self.p['feedback']['freshness_s'] or age < -.05:
            if required:
                raise ValueError('stale or future feedback snapshot')
            return None
        if (s.tcp.header.stamp.sec, s.tcp.header.stamp.nanosec) != key:
            raise ValueError('TCP and joint/state receipt stamps differ')
        p, q = s.tcp.pose.position, s.tcp.pose.orientation
        return {'stamp_ns': key[0]*10**9+key[1], 'tcp_m':[p.x,p.y,p.z],
                'quaternion_xyzw':[q.x,q.y,q.z,q.w], 'joints_rad':list(j.position),
                'speed_rad_s':list(j.velocity), 'mode':s.controller_mode,
                'reference_valid':s.reference_valid,'simulated':s.simulated,'phase':s.phase,
                'net_twist_rad':s.signed_twist_rad,'total_twist_rad':s.total_twist_rad,'diagnostic':s.diagnostic}

    def wait(self, predicate, timeout):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=.005)
            if predicate():
                return
            if self.process.poll() is not None:
                raise RuntimeError('business arm_node exited')
        raise TimeoutError('bounded ROS wait expired')

    def future(self, future, timeout):
        self.wait(future.done, timeout)
        return future.result()

    def require_home(self):
        self.wait(lambda: self.snapshot(False) is not None, 3)
        current = self.snapshot()
        if not current['reference_valid']:
            raise ValueError('continuous reference required, no automatic reset')
        if current['mode'] != 5 or max(map(abs,current['speed_rad_s'])) > self.p['tolerances']['stationary_rad_s']:
            raise ValueError('formal Home start must be idle/stationary')
        if max(abs(a-b) for a,b in zip(current['joints_rad'],self.data['home']['joints_rad'])) > self.p['tolerances']['joint_rad']:
            raise ValueError('current feedback is not formal Home; no implicit recovery movement')

    def require_reference_idle(self):
        self.wait(lambda:self.snapshot(False) is not None,3)
        current=self.snapshot()
        if not current['reference_valid'] or current['mode']!=5 or max(map(abs,current['speed_rad_s']))>self.p['tolerances']['stationary_rad_s']:
            raise ValueError('explicit recovery requires restored continuous reference and fresh idle stationary feedback')

    def action(self, point, profile, plan_only):
        g = self.MoveArm.Goal()
        g.operation = self.MoveArm.Goal.HOME if point is None else self.MoveArm.Goal.PREGRASP
        g.profile, g.plan_only = profile, plan_only
        if point is not None:
            g.target.header.frame_id = 'base_link'
            g.target.header.stamp = self.node.get_clock().now().to_msg()
            g.target.pose.position.x,g.target.pose.position.y,g.target.pose.position.z = map(float,point['tcp_m'])
            g.target.pose.orientation.x,g.target.pose.orientation.y,g.target.pose.orientation.z,g.target.pose.orientation.w = map(float,self.data['quaternion_xyzw'])
        # Margin evaluation and disk output can outlast the receipt freshness
        # budget while this single-threaded ROS client is not spinning. Drain
        # subscriptions and obtain an actual new matched snapshot before send;
        # never restamp an older measurement as current.
        try:
            self.wait(lambda:self.snapshot(False) is not None,3)
        except Exception as error:
            observation=getattr(self,'last_state_observation',None)
            event={'action_result_received':False,'code':None,'executed':False,'plan_only':plan_only,
                'profile':profile,'point_id':point['id'] if point else None,
                'fault_stage':'before_action_submission','motion_submitted':False,
                'error':repr(error),'last_state':observation}
            if not hasattr(self,'frames'):self.frames=[]
            self.frames.append(event)
            if getattr(self,'out',None):
                write_json(self.out/'actions.json',self.frames)
                write_json(self.out/'pre_send_feedback_fault.json',event)
            raise PreSendFeedbackFault('fresh feedback unavailable before action submission: '+str(error),observation) from error
        if point is not None:g.target.header.stamp=self.node.get_clock().now().to_msg()
        before, started = self.snapshot(), time.monotonic()
        command_ns = self.node.get_clock().now().nanoseconds
        offset = (self.out/'node.log').stat().st_size
        self.active = self.goal_handle(g,plan_only,command_ns)
        if not self.active.accepted:
            self.active = None
            with (self.out/'node.log').open('rb') as log:
                log.seek(offset)
                reason = log.read().decode(errors='replace')
            if not plan_only:
                raise ExecutionFault('execute action rejected: '+reason)
            return {'code':-1,'detail':'ADMISSION_REJECTED: '+reason,'executed':False,
                    'profile':profile,'elapsed_s':time.monotonic()-started}
        pending = self.active.get_result_async()
        try:
            wrapped = self.future(pending, self.args.goal_timeout)
        except Exception:
            self.future(self.active.cancel_goal_async(), 5)
            try:
                self.future(pending, self.p['stop_timeout_s']+5)
            finally:
                self.active = None
            raise
        self.active = None
        result = wrapped.result
        after_error = None
        try:
            self.wait(lambda:self.snapshot(False) is not None,3)
            after = self.snapshot()
        except Exception as error:
            # Keep the original action terminal failure even if its feedback
            # disappeared; absence of a snapshot must not erase that evidence.
            after = None
            after_error = repr(error)
        record = {'code':result.code,'detail':result.detail,'executed':result.executed,
                  'profile':profile,'plan_only':plan_only,'action_status':wrapped.status,
                  'elapsed_s':time.monotonic()-started,'before':before,
                  'command_ros_ns':command_ns,'after':after,'after_feedback_error':after_error,
                  'final_tcp_valid':result.final_tcp_valid,
                  'after_feedback_diagnostic':getattr(self,'last_state_observation',None) if after_error else None}
        if result.final_tcp_valid:
            p,q = result.final_tcp.pose.position,result.final_tcp.pose.orientation
            record['final_tcp_m'],record['final_quaternion_xyzw'] = [p.x,p.y,p.z],[q.x,q.y,q.z,q.w]
            record['final_stamp_ns'] = result.final_tcp.header.stamp.sec*10**9+result.final_tcp.header.stamp.nanosec
        if result.trajectory.points:
            t = [x.time_from_start.sec+x.time_from_start.nanosec/1e9 for x in result.trajectory.points]
            q = [list(x.positions) for x in result.trajectory.points]
            record['planned_duration_s'],record['samples'] = t[-1],len(q)
            label = ('home' if point is None else point['id'])+f'_{profile}_{command_ns}'
            with gzip.open(self.out/(label+'.json.gz'),'wt') as file:
                json.dump({'joints_rad':q,'time_s':t},file)
            record['trajectory_file'] = label+'.json.gz'
        self.frames.append(record)
        write_json(self.out/'actions.json',self.frames)
        return record

    def goal_handle(self,goal,plan_only,command_ns):
        try:
            return self.future(self.client.send_goal_async(goal),10)
        except Exception as error:
            # Submission may already have reached the business node although
            # no handle reached this client. No resend and no natural-completion
            # wait: request the node's owned-motion shutdown/Stop path now.
            self.controlled_shutdown_on_ambiguous_goal(error,plan_only,command_ns)
            raise ExecutionFault('ambiguous action acceptance; controlled node shutdown requested, no retry') from error

    def controlled_shutdown_on_ambiguous_goal(self,error,plan_only,command_ns):
        event={'action_result_received':False,'code':None,'executed':None,
               'motion_acceptance_unknown':not plan_only,'plan_only':plan_only,
               'command_ros_ns':command_ns,'error':repr(error),
               'response':'controlled business-node SIGINT shutdown; no action resend'}
        self.shutdown_requested=True
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            event['shutdown_requested_utc_ns']=time.time_ns()
            try:
                event['node_exit_code']=self.process.wait(timeout=self.p['stop_timeout_s']+8)
            except subprocess.TimeoutExpired:
                event['node_exit_code']=None
                event['shutdown_timeout']=True
        self.shutdown_confirmation={'confirmed':False,
             'reason':'action handle unknown; node shutdown requested; fresh post-Stop dwell unavailable to client'}
        event['stop_confirmation']=self.shutdown_confirmation
        self.frames.append(event)
        write_json(self.out/'actions.json',self.frames)
        write_json(self.out/'ambiguous_goal_shutdown.json',event)

    def choose_profile(self, point, result):
        reasons = []
        if point.get('kind') == 'refinement':
            reasons.append('boundary refinement')
        if point['tcp_m'][2] <= .15:
            reasons.append('near table height')
        margins = None
        if self.args.margin_evaluator and result.get('trajectory_file'):
            with gzip.open(self.out/result['trajectory_file'],'rt') as file:
                q = self.np.asarray(json.load(file)['joints_rad'])
            # Parent Planner's HOME PTP is a joint-space line with monotonic
            # timing. Sample the predicted endpoint -> original Home at least
            # as densely as the configured geometric knot step. Its winding and
            # hard path validity remain checked by the business-node precheck.
            home=self.np.asarray(self.data['home']['joints_rad'])
            count=max(1,int(math.ceil(float(self.np.max(abs(q[-1]-home)))/self.p['sampling']['joint_step_rad'])))
            back=self.np.linspace(q[-1],home,count+1)
            both=self.np.vstack([q,back])
            both.astype('<f8').tofile(self.out/'margin_q.bin')
            subprocess.run([str(self.args.margin_evaluator),str(self.root/'src/chess_robot_arm/config/nova5_ag95_geometry.yaml'),
                            str(self.out/'margin_q.bin'),str(self.out/'margin_metrics.bin')],check=True,stdout=subprocess.DEVNULL)
            m = self.np.fromfile(self.out/'margin_metrics.bin',dtype='<f8').reshape(-1,12)
            margins = {'table_clearance_lower_bound_m':float(m[:,4].min()+.01),
                       'joint_margin_rad':float(m[:,5].min()),'sigma_min':float(m[:,6].min()),
                       'condition_max':float(m[:,7].max()),'self_clearance_lower_bound_m':float(m[:,11].min())}
            margins.update(scope='outbound plus predicted joint-linear target-to-formal-Home return',
                           outbound_samples=len(q),predicted_return_samples=len(back),
                           sampled_envelope_only=True)
            # These select a validated speed mode; they do not alter hard safety
            # limits or create a second set of tiny commanded speed ceilings.
            if margins['table_clearance_lower_bound_m'] < .10 or margins['self_clearance_lower_bound_m'] < .015:
                reasons.append('near geometry envelope')
            if margins['joint_margin_rad'] < .25 or margins['sigma_min'] < .12 or margins['condition_max'] > 40:
                reasons.append('near joint/singularity constraint')
        elif not self.args.fixture:
            reasons.append('no independent path clearance evidence')
        return ('roundtrip_low' if reasons else 'roundtrip_high',reasons or ['ordinary checked transfer'],margins)

    def require_arrival(self, result, xyz, quaternion):
        if result['code'] != 1 or not result['executed'] or not result.get('final_tcp_valid'):
            raise ExecutionFault('business-node measured completion failed: '+result['detail'])
        self.wait(lambda: self.snapshot(False) is not None and self.snapshot()['stamp_ns'] >= result['final_stamp_ns'],3)
        current = self.snapshot()
        position_error = math.dist(result['final_tcp_m'],xyz)
        orientation_error = float((self.Rotation.from_quat(result['final_quaternion_xyzw']).inv()*self.Rotation.from_quat(quaternion)).magnitude())
        result['position_error_m'],result['orientation_error_rad'] = position_error,orientation_error
        result['published_endpoint_position_error_m'] = math.dist(current['tcp_m'],xyz)
        result['published_endpoint_orientation_error_rad'] = float((self.Rotation.from_quat(current['quaternion_xyzw']).inv()*self.Rotation.from_quat(quaternion)).magnitude())
        if position_error > self.p['tolerances']['position_m'] or orientation_error > self.p['tolerances']['rotation_rad']:
            raise ExecutionFault('measured final TCP differs from exact requested endpoint')
        if result['published_endpoint_position_error_m'] > self.p['tolerances']['position_m'] or result['published_endpoint_orientation_error_rad'] > self.p['tolerances']['rotation_rad']:
            raise ExecutionFault('fresh published endpoint differs from exact requested endpoint')
        if result['final_stamp_ns'] < result['command_ros_ns']:
            raise ExecutionFault('completion reused pre-command feedback')
        if current['mode'] != 5 or not current['reference_valid'] or max(map(abs,current['speed_rad_s'])) > self.p['tolerances']['stationary_rad_s']:
            raise ExecutionFault('fresh endpoint is not idle/stationary with continuous reference')
        if current['simulated']!=self.args.fixture:
            raise ExecutionFault('real/simulation feedback mismatch at measured endpoint')
        result['after'] = current  # confirmed fresh endpoint, with original ROS receipt stamp


    def make_speed_evidence(self,trials):
        if len(trials)!=2 or not all(x['status']=='roundtrip_success' and
            all(x[leg]['after']['simulated']==self.args.fixture for leg in ['outbound','return_home']) for x in trials):
            raise ValueError('qualification requires measured endpoints from correct real/simulation source')
        frozen=self.out/'speed_qualification_results.json'
        write_json(frozen,trials)
        return {'status':'PASS','simulation':self.args.fixture,
            'validated_profiles':['low','high'],
            'firmware_version':self.p['controller']['firmware_version'],
            'profiles':{'low':self.p['profiles']['roundtrip_low'],'high':self.p['profiles']['roundtrip_high']},
            'controller_limit_evidence':self.controller_limit_evidence,
            'command_policy_evidence':self.command_policy_evidence,
            'hardware_limits_verified':self.controller_limit_evidence is not None,
            'qualification_context':self.qualification_context,
            'effective_speed_modes':self.effective_speed_modes,
            'source_results':{'path':str(frozen),'sha256':sha(frozen)},
            'scope':'Two measured round trips at one checked manifest point; not a physical maximum-speed certification',
            'home_sha256':self.data['home']['home_yaml_sha256'],
            'binary_sha256':sha(self.root/'install/chess_robot_arm/lib/chess_robot_arm/arm_node')}

    def confirm_stopped(self):
        if self.shutdown_requested:
            return self.shutdown_confirmation
        started, since, stamps = time.monotonic(), None, set()
        last = None
        while time.monotonic()-started < self.p['stop_timeout_s']:
            self.rclpy.spin_once(self.node,timeout_sec=.005)
            last = self.snapshot(False)
            if last and last['mode'] == 5 and max(map(abs,last['speed_rad_s'])) <= self.p['tolerances']['stationary_rad_s']:
                stamps.add(last['stamp_ns'])
                since = since or time.monotonic()
                if len(stamps) > 1 and time.monotonic()-since >= self.p['tolerances']['dwell_s']:
                    return {'confirmed':True,'elapsed_s':time.monotonic()-started,'last_feedback':last}
            else:
                since, stamps = None,set()
        return {'confirmed':False,'elapsed_s':time.monotonic()-started,'last_feedback':last}

    def close(self):
        if getattr(self,'active',None) is not None:
            try:
                self.future(self.active.cancel_goal_async(),5)
                self.confirm_stopped()
            except Exception:
                pass
        if getattr(self,'process',None) is not None and self.process.poll() is None:
            if not getattr(self,'shutdown_requested',False):
                self.process.send_signal(signal.SIGINT)
            self.process.wait(timeout=self.p['stop_timeout_s']+8)
        if getattr(self,'log',None) is not None:
            self.log.close()
        if getattr(self,'node',None) is not None:
            self.node.destroy_node()
        if getattr(self,'rclpy',None) is not None and self.rclpy.ok():
            self.rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root',type=pathlib.Path,default=pathlib.Path(__file__).resolve().parents[2])
    parser.add_argument('--make-manifest',type=pathlib.Path)
    parser.add_argument('--manifest',type=pathlib.Path)
    parser.add_argument('--out',type=pathlib.Path)
    parser.add_argument('--controller-ip',default='127.0.0.1')
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--history-audit',type=pathlib.Path,action='append',default=[])
    parser.add_argument('--history-attestation',type=pathlib.Path)
    parser.add_argument('--speed-evidence',type=pathlib.Path)
    parser.add_argument('--controller-limits',type=pathlib.Path,
        help='Source-backed current-model/current-firmware absolute 6-axis limits; unknown blocks real motion')
    parser.add_argument('--command-policy',type=pathlib.Path,
        help='Explicit user-commanded planning caps; never reported as hardware ratings')
    parser.add_argument('--recover-home',action='store_true',
        help='Explicit one-time low HOME after history restore/limit gates; plan + measure, never reset reference')
    parser.add_argument('--margin-evaluator',type=pathlib.Path)
    parser.add_argument('--reuse-results',type=pathlib.Path)
    parser.add_argument('--power-cycle-evidence',type=pathlib.Path,
        help='Human-confirmed power only/no manual joint or cable motion; binds unchanged prior fault result')
    parser.add_argument('--continue-results',type=pathlib.Path,
        help='Explicit partial-batch continuation, measured successes only; fresh requalification required')
    parser.add_argument('--continuation-statement',type=pathlib.Path,
        help='Current actual human continuation statement JSON')
    parser.add_argument('--goal-timeout',type=float,default=120)
    parser.add_argument('--fixture',action='store_true')
    parser.add_argument('--fixture-endpoints',type=pathlib.Path)
    args = parser.parse_args()
    import ipaddress,os
    args.controller_ip = str(ipaddress.IPv4Address(args.controller_ip))
    if args.fixture and args.controller_ip != '127.0.0.1':
        raise ValueError('simulation must remain on IPv4 loopback')
    root = args.project_root.resolve()
    home = load_home(root)
    if args.make_manifest:
        write_json(args.make_manifest,manifest(home))
        print(json.dumps({'coarse_points':26,'maximum_targets':38,'estimate_minutes':[10,25]}))
        return
    if not args.manifest or not args.out:
        parser.error('--manifest and --out are required for a run')
    if os.environ.get('ROS_DOMAIN_ID') != '98' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise ValueError('use isolated ROS_DOMAIN_ID=98 ROS_LOCALHOST_ONLY=1')
    data = json.loads(args.manifest.read_text())
    validate_manifest(data,home)
    out = args.out.resolve()
    out.mkdir(parents=True,exist_ok=False)
    session = None
    runner = None
    try:
        session = RosSession.__new__(RosSession)
        session.__init__(root,out,data,args)
        reuse = json.loads(args.reuse_results.read_text()) if args.reuse_results else None
        runner = Runner(data,session,out,args.execute,reuse,session.continuation)
        runner.report['history_audits'] = [{'path':str(p.resolve()),'sha256':sha(p)} for p in args.history_audit]
        runner.report['reference_restores'] = session.restore_count
        runner.report['parameters_sha256'] = sha(out/'parameters.yaml')
        runner.report['binary_sha256'] = sha(root/'install/chess_robot_arm/lib/chess_robot_arm/arm_node')
        runner.report['qualification_context']=session.qualification_context
        runner.report['effective_speed_modes']=session.effective_speed_modes
        runner.report['simulation']=args.fixture
        runner.run()
    except Exception as error:
        if runner:
            if not runner.report['status'].startswith(('EXECUTION_FAULT','PRE_SEND_FEEDBACK_FAULT')):
                runner.report['status'] = 'BLOCKED'
            runner.report['error'] = repr(error)
            runner.save()
        else:
            write_json(out/'results.json',{'status':'PRECONDITION_BLOCKED','error':repr(error),
                'home':data['home'],'manifest':data,
                'targets':[dict(p,status='pending',outbound=None,return_home=None) for p in data['coarse_points']],
                'simulation':args.fixture,
                'physical_motion_attempted':False,'physical_roundtrip_successes':0})
        raise
    finally:
        if session:
            session.close()
        try:
            from arm_roundtrip_report import render
            render(out)
        except Exception as error:
            write_json(out/'report_render_error.json',{'error':repr(error)})


if __name__ == '__main__':
    main()
