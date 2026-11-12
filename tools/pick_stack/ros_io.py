"""ROS Action adapter for the finite demo; hardware is used only by --execute."""
import hashlib
import json
import math
import signal
import sys
import time
import uuid
from pathlib import Path

import numpy as np
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from rcl_interfaces.srv import GetParameters
from chess_robot_interfaces.action import MoveArm, MoveGripper
from chess_robot_interfaces.msg import GripperState
from rosidl_runtime_py.convert import message_to_ordereddict as convert

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scan_table'))
from scan_controller import Controller, write, stamp, position


def detection_overlaps_roi(detection, roi):
    """Unknown geometry may be ignored only with image bounds outside the slot."""
    box=detection.get('box_xyxy')
    if box is None:
        outline=detection.get('mask_outline_xy')
        if not outline:return True
        box=[min(p[0] for p in outline),min(p[1] for p in outline),
             max(p[0] for p in outline),max(p[1] for p in outline)]
    if len(box)!=4 or not all(math.isfinite(x) for x in box) or box[0]>=box[2] or box[1]>=box[3]:
        return True
    return not (box[2]<roi[0] or box[0]>roi[2] or box[3]<roi[1] or box[1]>roi[3])


def finger_neighbor_envelope(guard, orientation, piece_radius, clearance, position_tolerance, orientation_tolerance=0.):
    """Project both complete open/close union hulls, retaining the radial margin."""
    expected={'left_finger_assembly_21_state_union','right_finger_assembly_21_state_union'}
    fingers=[h for h in guard['hulls'] if h.get('role')=='finger_assembly']
    if {h['name'] for h in fingers}!=expected or any(h['link']!=6 for h in fingers):
        raise ValueError('finger neighbor policy requires both link-6 21-state union hulls')
    vertices=np.concatenate([np.array(h['vertices']) for h in fingers])
    tcp_T_flange=np.linalg.inv(np.array(guard['tcp']).reshape(4,4))
    local=(tcp_T_flange@np.column_stack([vertices,np.ones(len(vertices))]).T).T[:,:3]
    x,y,z,w=orientation
    rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                       [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                       [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    world=(rotation@local.T).T
    radial=float(np.linalg.norm(world[:,:2],axis=1).max())
    angular=2*math.sin(orientation_tolerance/2)*float(np.linalg.norm(local,axis=1).max())
    margin=clearance-piece_radius-radial-angular
    if margin<2*position_tolerance:
        raise ValueError('configured radial neighbor clearance lacks two position-tolerance margins')
    padding=piece_radius+margin+angular
    return dict(lower_xy_m=(world[:,:2].min(0)-padding).tolist(),
                upper_xy_m=(world[:,:2].max(0)+padding).tolist(),
                finger_radial_bound_m=radial,position_margin_m=margin,rotation_margin_m=angular,
                outer_radius_m=clearance,piece_radius_m=piece_radius)


def recent_clear_neighbor(detection, stamp_ns, recent, slot, clearance):
    """Bridge at most 0.3s of mask loss using a fresh stable full-image box."""
    if (detection.get('classification_reason')!='no_roi_match'
            or detection.get('touches_image_edge') or detection.get('touches_processing_edge')):
        return None
    box=detection.get('box_xyxy')
    if box is None:
        return None
    matches=[]
    for prior in recent:
        old=prior['detection']
        point=old['base_surface_center_m']
        if (0<=(stamp_ns-prior['stamp_ns'])*1e-9<=.3
                and any(abs(a-b)>limit+.008 for a,b,limit in zip(point[:2],slot,clearance))
                and max(abs(a-b) for a,b in zip(box,old['box_xyxy']))<=2):
            matches.append(prior)
    return matches[0] if len(matches)==1 else None


class PlanInspector(Controller):
    def __init__(self, config, out):
        scan_config = json.loads(Path(config['scan_config']).read_text())
        super().__init__(scan_config, out, False, node_name='xiangqi_pick_stack_controller')
        self.demo = config
        self.latest_scene = None
        self.gripper = None
        self.subs.append(self.n.create_subscription(GripperState, '/gripper/state', self.receive_gripper, 10))

    def receive_scene(self, msg):
        super().receive_scene(msg)
        self.latest_scene=self.scene[-1]

    def receive_gripper(self, msg):
        if self.gripper is None or stamp(msg) >= stamp(self.gripper):
            self.gripper = msg

    def close(self):
        self.client.destroy()
        self.events.close()
        self.n.destroy_node()

    def read_gripper_policy(self):
        parameters=self.n.create_client(GetParameters,'/gripper_node/get_parameters')
        try:
            if not parameters.wait_for_service(timeout_sec=5):
                raise RuntimeError('gripper freshness parameter unavailable')
            request=GetParameters.Request();request.names=['feedback_max_age_sec']
            values=self.wait(parameters.call_async(request),5).values
            if len(values)!=1 or values[0].type!=3 or not 0 < values[0].double_value <= 1.:
                raise RuntimeError('site gripper freshness policy not supported')
            self.gripper_freshness=values[0].double_value
            self.report['gripper_feedback_max_age_sec']=self.gripper_freshness
        finally:
            self.n.destroy_client(parameters)

    def probe(self, name, operation, xyz):
        self.stopped()
        self.check_parameters()
        if not self.client.wait_for_server(timeout_sec=5):
            raise RuntimeError('existing arm action unavailable')
        g = MoveArm.Goal()
        g.operation = getattr(g, operation)
        g.profile = self.c['profile']
        g.plan_only = True
        g.target.header.frame_id = 'base_link'
        g.target.header.stamp = self.n.get_clock().now().to_msg()
        if operation != 'HOME':
            p = list(xyz)
            p[2] -= self.c['expected_approach_clearance_m']
            g.target.pose.position.x, g.target.pose.position.y, g.target.pose.position.z = p
            q = self.demo['orientation_xyzw']
            g.target.pose.orientation.x, g.target.pose.orientation.y, g.target.pose.orientation.z, g.target.pose.orientation.w = q
        self.pending = self.client.send_goal_async(g)
        self.goal = self.wait(self.pending, 8)
        self.pending = None
        if not self.goal.accepted:
            raise RuntimeError('arm plan goal rejected')
        self.terminal = self.goal.get_result_async()
        reply = self.wait(self.terminal, 60, monitor=True)
        row = dict(probe=name, operation=operation, requested_tcp_m=xyz,
                   plan_only=True, status=reply.status, result=convert(reply.result))
        self.report['actions'].append(row)
        self.goal = self.terminal = None
        if reply.result.executed:
            raise RuntimeError('plan-only contract violated')
        return row


class RealIO(PlanInspector):
    """One arm owner and one gripper client with bounded observation recovery."""
    def __init__(self, config, out, authorization):
        super().__init__(config, out)
        self.execute = True  # Enables cancellation of this client's physical arm goal.
        self.authorization = authorization
        self.report.update(schema='xiangqi_pick_stack_run/v2', execute=True,
                           authorization=authorization, gripper_actions=[], confirmations=[],demo_config=config)
        self.grip_client = ActionClient(self.n, MoveGripper, '/gripper/command')
        self.grip_pending = self.grip_goal = self.grip_terminal = None
        self.held = False
        self.prechecked = False
        self.commissioned = False
        self.transfer_guard=yaml.load(Path(self.c['guard_file']).read_text(),Loader=getattr(yaml,'CSafeLoader',yaml.SafeLoader)) if config.get('calibration_mode')=='supervised_flat_table' else None
        self.at_home = False
        self.active_orientation_xyzw=list(config['orientation_xyzw'])
        self.layers = 0
        self.last_placement_stamp = 0
        self.home_joints = yaml.safe_load(Path(config['home_config']).read_text())['chess_robot_arm']['ros__parameters']['home']['joint_positions_rad']
        self.replies = []
        self.stationary_history_after_ns = self.now()
        self.observation_window = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.n.create_publisher(String, '/pick_stack/status', qos)
        self.request_pub = self.n.create_publisher(String, '/pick_stack/confirmation_request', qos)
        self.subs.append(self.n.create_subscription(String, '/pick_stack/confirmation', self.receive_confirmation, 10))

    def stopped(self, timeout=12, check_abort=True):
        history_start_ns=(self.stationary_history_after_ns
                          if self.demo.get('business_mode')=='row' else None)
        return super().stopped(timeout,check_abort,history_start_ns=history_start_ns)

    def emit(self, row):
        if row['phase'] in ('PREGRASP','PICK_ALIGNED') and 'fresh_measurement' in row:
            self.target_measurement=row['fresh_measurement']
            self.expected_stack_class=self.target_measurement['class_name']
        self.layers = row['layers']
        self.report['layers'] = self.layers
        self.report['object_state'] = row['object_state']
        if 'next_slot_index' in row:
            self.report['next_slot_index']=row['next_slot_index']
        data = dict(task_id=self.report['scan_id'], stamp_ns=self.now(), **row)
        wire = json.dumps(data, ensure_ascii=False)
        self.events.write(wire+'\n'); self.events.flush()
        self.status_pub.publish(String(data=wire))
        print(wire, flush=True)

    def receive_confirmation(self, msg):
        try:
            self.replies.append((self.now(), json.loads(msg.data)))
            self.replies = self.replies[-20:]
        except (ValueError, TypeError):
            self.report.setdefault('rejected_confirmations', []).append('malformed JSON')

    def gripper_state(self, held=False, idle=False):
        s = self.gripper
        if not (s is not None and not s.simulated and s.connected and s.feedback_valid and
                0 <= s.feedback_age_sec and 0 <= (self.now()-stamp(s))*1e-9 <= .5 and
                s.feedback_age_sec+(self.now()-stamp(s))*1e-9 <= getattr(self,'gripper_freshness',.5) and
                s.commands_enabled and s.initialization == 1 and not s.object_dropped):
            raise RuntimeError('fresh real initialized gripper feedback unavailable / dropped: '+str(None if s is None else dict(topic_age_s=(self.now()-stamp(s))*1e-9,device_age_s=s.feedback_age_sec,connected=s.connected,valid=s.feedback_valid,dropped=s.object_dropped,initialized=s.initialization)))
        if held and not (s.object_detected and s.grip_status == 2):
            raise RuntimeError('carried piece no longer held')
        if idle and (s.busy or s.grip_status not in (1,2)):
            raise RuntimeError('gripper still busy')
        return s

    def spin(self, check_abort=True):
        super().spin(check_abort)
        # Drain ready paired-feedback callbacks before checking their age.
        for _ in range(8):
            rclpy.spin_once(self.n,timeout_sec=0.)
        observation_floor=getattr(self,'active_observation_floor',None)
        if check_abort and observation_floor is not None and position(self.pair()[0])[2]<observation_floor:
            raise RuntimeError('observation approach crossed its high-air clearance floor')
        if check_abort and self.held:
            self.gripper_state(held=True)
            floor=getattr(self,'active_transfer_floor',None)
            if floor is not None and position(self.pair()[0])[2]<floor:
                raise RuntimeError('held transfer left monitored payload clearance')

    def confirm(self, kind, layer=0, **details):
        self.stopped()
        request = dict(task_id=self.report['scan_id'], request_id=uuid.uuid4().hex,
                       issued_ns=self.now(), kind=kind, layer=layer, **details)
        self.replies.clear()
        self.request_pub.publish(String(data=json.dumps(request)))
        self.events.write(json.dumps(dict(confirmation_request=request))+'\n'); self.events.flush()
        deadline = time.monotonic()+self.demo['confirmation_timeout_s']
        while time.monotonic() < deadline:
            self.spin()
            self.gripper_state(held=self.held, idle=True)
            s, j = self.pair()
            if not (s.feedback_fresh and s.reference_valid and not s.busy and
                    0 <= (self.now()-stamp(s))*1e-9 <= .2 and max(map(abs,j.velocity)) <= .002):
                raise RuntimeError('stationary feedback lost while awaiting operator evidence')
            for received, reply in self.replies:
                if not (received >= request['issued_ns'] and
                        reply.get('task_id') == request['task_id'] and
                        reply.get('request_id') == request['request_id'] and
                        reply.get('kind') == kind and reply.get('layer') == layer):
                    continue
                if reply.get('decision') != 'confirmed' or not reply.get('operator') or not reply.get('observation'):
                    raise RuntimeError('operator did not confirm: '+kind)
                self.report['confirmations'].append(dict(request=request, reply=reply, received_ns=received))
                return reply
        raise TimeoutError('operator confirmation timed out: '+kind)

    def startup_precheck(self, config):
        # Device readiness is independent of the later grasp/stack calibration.
        if not __debug__:
            raise RuntimeError('do not run physical controller with Python assertions disabled')
        self.stopped()
        self.check_parameters()
        self.read_gripper_policy()
        if not self.client.wait_for_server(timeout_sec=5) or not self.grip_client.wait_for_server(timeout_sec=5):
            raise RuntimeError('existing arm/gripper Action unavailable')
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            self.spin()
            try:
                initial=self.gripper_state(idle=True)
                break
            except RuntimeError:
                continue
        else:raise RuntimeError('fresh initialized idle gripper feedback unavailable at startup')
        self.report['startup_gripper_before'] = convert(initial)
        self.prechecked = True

    def open_startup(self):
        state=self.gripper_state(idle=True)
        reference=self.demo.get('startup_held_source_xy_m')
        if state.object_detected and reference is not None:
            current=position(self.pair()[0])
            if np.linalg.norm(current[:2]-reference)>.003 or not 0 <= current[2]-self.demo['nominal_pick_tcp_base_z_m'] <= .030:
                raise RuntimeError('held startup pose differs from recorded source; cannot set it down automatically')
            self.held=True
            target=[*current[:2],self.demo['nominal_pick_tcp_base_z_m']]
            self.arm_request('DESCEND',target,True)
            self.arm_request('DESCEND',target,False)
            self.report['startup_held_piece_set_down']=True
        self.held = False
        self.gripper_move(self.demo['startup_open_raw'], False)
        self.report['startup_open_completed'] = True
        # A low empty start must first leave the contact region vertically.
        if self.demo.get('calibration_mode')=='supervised_flat_table':
            arm=self.pair()[0]
            current=position(arm)
            if current[2]<self.demo['hover_tcp_base_z_m']:
                target=[*current[:2],self.demo['hover_tcp_base_z_m']]
                previous=self.active_orientation_xyzw
                q=arm.tcp.pose.orientation
                self.active_orientation_xyzw=[q.x,q.y,q.z,q.w]
                try:
                    self.arm_request('RETREAT',target,True)
                    self.arm_request('RETREAT',target,False)
                finally:
                    self.active_orientation_xyzw=previous

    def precheck(self, config):
        if not self.prechecked or not self.at_home:
            raise RuntimeError('startup open and formal Home must complete before pick precheck')
        from demo import execution_blockers, calibration_fingerprint
        blockers = execution_blockers(config)
        if blockers:
            raise RuntimeError('; '.join(blockers))
        if config.get('calibration_mode') != 'supervised_flat_table':
            # Review is bound to the exact measured parameters and unchanged models.
            review = json.loads(Path(config['commissioning_review_file']).read_text())
            if not (review['calibration_sha256'] == calibration_fingerprint(config) and
                    review['source_sha256'] == self.c['source_sha256'] and
                    review['max_pieces'] >= config['max_pieces'] and review['reviewer']):
                raise RuntimeError('commissioning review does not cover this calibration, model or batch')
            for kind in ('contact_policy_evidence','payload_stack_evidence'):
                artifact=review[kind]
                if hashlib.sha256(Path(artifact['path']).read_bytes()).hexdigest()!=artifact['sha256']:
                    raise RuntimeError('review evidence changed: '+kind)
        self.stopped(); self.check_parameters()
        if not self.client.wait_for_server(timeout_sec=5) or not self.grip_client.wait_for_server(timeout_sec=5):
            raise RuntimeError('existing arm/gripper Action unavailable')
        s = self.gripper_state(idle=True)
        if s.object_detected or s.grip_status != 1:
            raise RuntimeError('initial gripper object state is not empty')
        if config.get('calibration_mode')=='supervised_flat_table':
            self.report['workspace_observation']=config['workspace_observation']
        else:
            self.confirm('WORKSPACE', stack_xy_m=config['stack_xy_m'],
                         requirements='exclusive control; empty bare-table support; source and stack envelopes/exit clear')
        self.commissioned=True

    def scan(self, force_fresh=False):
        if not self.commissioned:
            raise RuntimeError('physical scan requires completed pick precheck')
        if not self.at_home:
            raise RuntimeError('workflow must reach formal Home before scanning')
        known=None if force_fresh else self.demo.get('remaining_search_targets')
        if known:
            self.stopped();self.check_parameters()
            self.report['provided_search_targets']=known
            self.report['inventory_coordinates_are_search_hints_only']=True
            result=dict(schema='known_source_continuation/v1',search_mode='known_source_continuation',
                target_hypotheses=known,returned_home=True,physical_stop_confirmed=True,
                home_basis='startup Home completed and measured stop in this execution',
                whole_table_coverage_confirmed=False)
            write(self.out/'provided-search-hints.json',result)
            return result
        inventory=None if force_fresh else self.demo.get('completed_scan_file')
        if inventory:
            report=json.loads(Path(inventory).read_text())
            if not (report['returned_home'] and report['physical_stop_confirmed'] and report['status']!='FAULT'):
                raise RuntimeError('continuation requires a completed scan')
            if 0 <= (self.now()-report['ended_ns'])*1e-9 <= self.demo.get('search_hint_max_age_s',900):
                self.report['search_inventory_file']=inventory
                self.report['inventory_coordinates_are_search_hints_only']=True
                self.report['search_hint_age_s']=(self.now()-report['ended_ns'])*1e-9
                self.report['search_hint_max_age_s']=self.demo.get('search_hint_max_age_s',900)
                return report
            self.report['expired_search_inventory_file']=inventory
        directory=self.out/('scan-recheck' if force_fresh and (self.out/'scan').exists() else 'scan')
        directory.mkdir()
        # Demo owns HOME_START; retain the scanner's final Home and all guards.
        scanner = Controller(dict(self.c, initial_home=False), directory, True)
        # Give the existing scanner the same interrupt and real gripper monitor.
        original_spin = scanner.spin
        def scan_spin(check_abort=True):
            original_spin(check_abort)
            # Both nodes subscribe to 100Hz paired arm feedback. Service the
            # ready queues of both, instead of accumulating gripper callbacks
            # behind hundreds of arm/scene callbacks in the second node.
            for _ in range(8):
                rclpy.spin_once(scanner.n,timeout_sec=0.)
                rclpy.spin_once(self.n,timeout_sec=0.)
            if check_abort and self.abort:
                raise RuntimeError('operator interrupted')
            if check_abort:
                s = self.gripper_state(idle=True)
                if s.object_detected:
                    raise RuntimeError('unexpected object during scan')
        scanner.spin = scan_spin
        okay = scanner.run()
        self.report['scan_result_file'] = str(directory/'scan-result.json')
        if not okay:
            raise RuntimeError('scan fault: '+scanner.report['error']+
                '; no automatic Home recovery or pick')
        self.stopped()
        if max(abs(a-b) for a,b in zip(self.pair()[1].position,self.home_joints))>self.demo['home_joint_tolerance_rad']:
            raise RuntimeError('scan did not finish at measured formal Home')
        self.at_home=True
        if self.demo.get('business_mode')=='row':
            self.recheck_unlocated_scan_targets(scanner.report,directory)
        self.report.setdefault('scan_history',[]).append(dict(
            result_file=self.report['scan_result_file'],fresh_recheck=force_fresh))
        return scanner.report

    def rescan(self, prior_identities):
        from demo import qualified_scene_window
        self.gripper_state(idle=True)
        if self.held or self.gripper.object_detected:
            raise RuntimeError('workspace rescan requires an empty gripper')
        before=qualified_scene_window(self.observe())['scene_identity'][2:]
        if any(list(identity)!=list(before) for identity in prior_identities):
            raise RuntimeError('completed row measurements belong to another camera/scene identity')
        result=self.scan(force_fresh=True)
        after=qualified_scene_window(self.observe())['scene_identity'][2:]
        if before!=after:
            raise RuntimeError('camera/scene/reference identity changed during workspace rescan')
        self.report.setdefault('workspace_rescan_identity_checks',[]).append(dict(
            scene_identity=after,completed_measurement_count=len(prior_identities)))
        return result

    def recheck_unlocated_scan_targets(self, scan, directory):
        from demo import complete_detection,qualified_scene_window,qualified_row_observation,ObservationUnavailable,distance,excluded_target
        limit=self.demo.get('active_observation',{}).get('scan_rechecks',0)
        if not limit:
            return
        hints=[]
        for view in scan.get('views',[]):
            if not view.get('unlocated_observations'):
                continue
            stem=view['view_id']+('-retry'+str(view['attempt']) if view.get('attempt') else '')
            for line in (directory/(stem+'-scene.jsonl')).read_text().splitlines():
                frame=json.loads(line)
                for detection in frame['detections']:
                    box=detection.get('box_xyxy')
                    if complete_detection(detection) or box is None:
                        continue
                    pixel=[(box[0]+box[2])/2,(box[1]+box[3])/2]
                    xy=self.image_point_on_table(frame,pixel).tolist()
                    existing=[p['base_surface_center_m'][:2] for p in scan['target_hypotheses']]+hints
                    if not any(distance(xy,point)<.025 for point in existing):
                        hints.append(xy)
                    if len(hints)>=limit:
                        break
                if len(hints)>=limit:
                    break
            if len(hints)>=limit:
                break
        for xy in hints:
            row=dict(search_xy_m=xy,coordinate_basis='image ray/table plane; search hint only')
            self.report.setdefault('scan_directional_rechecks',[]).append(row)
            target=[*xy,self.demo['hover_tcp_base_z_m']]
            plan=self.arm_request('PREGRASP',target,True,allow_plan_rejection=True)
            floor=self.demo['active_observation']['min_tcp_base_z_m']
            poses=list(sampled_tcp_poses(self.transfer_guard,[dict(positions=list(p.positions)) for p in plan.trajectory.points])) if plan.code==0 else []
            if not poses or any(pose[2,3]<floor for pose in poses):
                row.update(status='PLAN_REJECTED',detail=plan.detail)
                continue
            scan['returned_home']=False
            self.active_observation_floor=floor
            try:
                self.arm_request('PREGRASP',target,False)
            finally:
                self.active_observation_floor=None
            self.at_home=False
            frame=self.latest_scene
            source=dict(scene_identity=(frame['stationary_generation'],frame['controller_epoch'],frame['track_namespace'],
                json.dumps(dict(identity=frame['identity'],K=frame['source_K']),sort_keys=True),frame['source_epoch']))
            def observe_target():
                return qualified_row_observation(self.demo,xy,self.observe(duration_s=3.),
                    search_radius_m=self.demo['active_observation'].get('search_hint_radius_m',.04))
            try:
                measured=self.observe_with_approach(observe_target,source,'unlocated_scan_target',
                    self.demo['hover_tcp_base_z_m'],focus_xy=xy)
            except RuntimeError as error:
                if not isinstance(error,ObservationUnavailable) and str(error) not in (
                        'row observation has multiple complete targets near slot',
                        'row observation target moved beyond 3mm repeatability'):
                    raise
                row.update(status='UNRESOLVED',reason=str(error))
                continue
            if tuple(measured['scene_identity'][2:])!=tuple(source['scene_identity'][2:]):
                raise RuntimeError('camera/scene/reference identity changed during scan recheck')
            if excluded_target(self.demo,measured['base_surface_center_m'],[],self.report.get('next_slot_index',0)):
                row.update(status='PROCESSED_ROW_TARGET',position_m=measured['base_surface_center_m'])
                continue
            hint=dict(measured,scan_target_id='directional-'+str(len(scan['target_hypotheses'])+1),
                class_name='unknown',association_state='within_view_unique',
                position_state='stable_repeatability_only',search_hint_only=True)
            scan['target_hypotheses'].append(hint)
            row.update(status='LOCATED',position_m=measured['base_surface_center_m'])
        if not scan['returned_home']:
            self.home()
            self.stopped()
            scan['returned_home']=True
            scan['physical_stop_confirmed']=True
        write(self.out/(directory.name+'-rechecked.json'),scan)
        self.report['scan_result_file']=str(self.out/(directory.name+'-rechecked.json'))

    def set_pick_orientation(self, orientation):
        self.active_orientation_xyzw=list(orientation)
        if self.demo.get('pick_neighbor_policy','radial')=='finger_model':
            self.pick_envelope=finger_neighbor_envelope(self.transfer_guard,self.active_orientation_xyzw,
                self.demo['measured']['piece_radius_m'],self.demo['neighbor_clearance_m'],self.demo['descent_tolerance_m'],
                self.c['rotation_tolerance_rad'])
            self.report.setdefault('source_neighbor_envelopes',[]).append(dict(
                orientation_xyzw=self.active_orientation_xyzw,**self.pick_envelope))

    def begin_pick(self):
        self.pick_yaw_index=0
        self.set_pick_orientation(self.demo.get('pick_orientation_xyzw',self.demo['orientation_xyzw']))

    def try_next_pick_orientation(self, seed):
        from demo import TargetUnavailable
        options=self.demo.get('pick_yaw_rechecks_deg',[])
        base=self.demo.get('pick_orientation_xyzw',self.demo['orientation_xyzw'])
        while self.pick_yaw_index<len(options):
            yaw=options[self.pick_yaw_index]
            self.pick_yaw_index+=1
            angle=math.radians(yaw)/2
            ca,sa=math.cos(angle),math.sin(angle)
            x,y,z,w=base
            orientation=[ca*x-sa*y,ca*y+sa*x,ca*z+sa*w,ca*w-sa*z]
            previous=list(self.active_orientation_xyzw)
            state,joint,_=self.stopped()
            if self.gripper_state(idle=True).object_detected:
                raise RuntimeError('pick reorientation requires an empty gripper')
            start=position(state).copy()
            start_joints=list(joint.position)
            self.set_pick_orientation(orientation)
            target=[*seed['base_surface_center_m'][:2],self.demo['transfer_tcp_base_z_m']]
            row=dict(yaw_deg=yaw,target_tcp_m=target,orientation_xyzw=orientation,status='PLANNING')
            self.report.setdefault('pick_orientation_rechecks',[]).append(row)
            plan=self.arm_request('PREGRASP',target,True,allow_plan_rejection=True)
            if plan.code!=0:
                row.update(status='PLAN_REJECTED',detail=plan.detail)
                self.set_pick_orientation(previous)
                continue
            poses=list(sampled_tcp_poses(self.transfer_guard,[dict(positions=list(p.positions)) for p in plan.trajectory.points]))
            floor=self.demo['active_observation']['min_tcp_base_z_m']
            radius=self.demo['active_observation'].get('max_xy_offset_m',.04)
            if not poses or any(p[2,3]<floor or p[2,3]>target[2]+self.c['position_tolerance_m'] or
                                np.linalg.norm(p[:2,3]-target[:2])>radius for p in poses):
                row.update(status='PLAN_REJECTED',detail='pick reorientation leaves the high-air viewing corridor')
                self.set_pick_orientation(previous)
                continue
            def unchanged_start():
                current,current_joint=self.pair()
                if (np.linalg.norm(position(current)-start)>.003 or
                        max(abs(a-b) for a,b in zip(current_joint.position,start_joints))>.001):
                    raise RuntimeError('pick reorientation start changed after planning')
            self.active_observation_floor=floor
            try:
                self.arm_request('PREGRASP',target,False,before_send=unchanged_start)
            finally:
                self.active_observation_floor=None
            self.at_home=False
            contact=[*target[:2],self.demo['nominal_pick_tcp_base_z_m']]
            proof=self.arm_request('DESCEND',contact,True,allow_plan_rejection=True)
            if proof.code!=0:
                row.update(status='CONTACT_PLAN_REJECTED',detail=proof.detail)
                continue
            row['status']='MOVED_FOR_FRESH_OBSERVATION'
            print(json.dumps(dict(phase='PICK_ORIENTATION_RECHECK',**row)),flush=True)
            return True
        raise TargetUnavailable(seed)

    def end_pick(self):
        self.active_orientation_xyzw=self.demo['orientation_xyzw']

    def arm_goal(self, operation, xyz, plan):
        g = MoveArm.Goal()
        g.operation = getattr(g, operation)
        g.profile = ('table_contact' if self.demo.get('calibration_mode')=='supervised_flat_table' and operation in ('DESCEND','RETREAT') else self.c['profile']); g.plan_only = plan
        g.target.header.frame_id = 'base_link'
        g.target.header.stamp = self.n.get_clock().now().to_msg()
        if operation != 'HOME':
            p = list(xyz)
            if operation in ('PREGRASP','RETREAT'):
                p[2] -= self.c['expected_approach_clearance_m']
            g.target.pose.position.x,g.target.pose.position.y,g.target.pose.position.z = p
            q = getattr(self,'active_orientation_xyzw',self.demo['orientation_xyzw'])
            g.target.pose.orientation.x,g.target.pose.orientation.y,g.target.pose.orientation.z,g.target.pose.orientation.w = q
        return g

    def arm_request(self, operation, xyz, plan, before_send=None, allow_plan_rejection=False):
        if not self.prechecked:
            raise RuntimeError('physical arm adapter requires completed precheck')
        self.stopped(); self.check_parameters()
        self.gripper_state(held=self.held, idle=True)
        if before_send is not None:
            before_send()
        request=self.arm_goal(operation,xyz,plan)
        self.observation_window=None
        self.stationary_history_after_ns=self.now()
        self.pending = self.client.send_goal_async(request)
        self.goal = self.wait(self.pending, 8); self.pending = None
        if not self.goal.accepted:
            raise RuntimeError('arm rejected '+operation)
        self.terminal = self.goal.get_result_async()
        if not plan:
            self.report['physical_stop_confirmed'] = False
        reply = self.wait(self.terminal, 60, monitor=True)
        self.report['actions'].append(dict(operation=operation, requested_tcp_m=xyz,
                                          plan_only=plan, status=reply.status, result=convert(reply.result)))
        if (plan and allow_plan_rejection and reply.status==6 and
                reply.result.code==14 and not reply.result.executed):
            self.goal=self.terminal=None
            return reply.result
        if (not plan and operation=='DESCEND' and not self.held
                and self.demo.get('business_mode')=='row' and reply.status==6
                and reply.result.code==14 and not reply.result.executed
                and reply.result.detail=='guard joint limit margin'
                and reply.result.final_tcp_valid and not reply.result.trajectory.points):
            # 已完成的拒绝结果明确没有执行轨迹。重新证明停稳后，交回抓取层有限换向。
            self.goal=self.terminal=None
            self.stopped()
            from demo import PickPlanRejected
            raise PickPlanRejected(reply.result.detail)
        # Leave the owned handle intact on failure for halt()/cancel_owned().
        if not (reply.status == 4 and reply.result.code == (0 if plan else 1) and reply.result.executed == (not plan)):
            raise RuntimeError(reply.result.detail)
        self.goal = self.terminal = None
        if not plan:
            self.stationary_history_after_ns=self.now()
            if not reply.result.final_tcp_valid:
                raise RuntimeError('measured final TCP unavailable')
            s,j,t = self.stopped()
            if operation=='HOME':
                if max(abs(a-b) for a,b in zip(j.position,self.home_joints))>self.demo['home_joint_tolerance_rad']:
                    raise RuntimeError('measured formal Home joints mismatch')
            elif xyz is not None:
                if np.linalg.norm(position(s)-xyz) > self.c['position_tolerance_m']:
                    raise RuntimeError('measured endpoint position mismatch')
                q=s.tcp.pose.orientation; expected=request.target.pose.orientation
                target=np.array([expected.x,expected.y,expected.z,expected.w])
                angle=2*math.acos(float(np.clip(abs(np.array([q.x,q.y,q.z,q.w])@target),0,1)))
                if angle > self.c['rotation_tolerance_rad']:
                    raise RuntimeError('measured endpoint orientation mismatch')
        return reply.result

    def check_held_transfer(self, plan):
        # Check the added piece, which is absent from the arm's fixed-tool hulls.
        guard=self.transfer_guard
        axes=[np.asarray(x) for x in guard['axes']]
        origins=[np.asarray(x).reshape(4,4) for x in guard['joint_origins']]
        tcp=np.asarray(guard['tcp']).reshape(4,4)
        # A conservative sphere encloses the 40mm x 13mm piece around the TCP.
        radius=.025
        top=self.demo['table_z_m']+(1 if self.demo.get('business_mode')=='row' else self.layers)*self.demo['piece_thickness_m']
        minimum=top+radius+.020
        checked=0
        for sample in plan.trajectory.points:
            pose=np.eye(4)
            for origin,axis,angle in zip(origins,axes,sample.positions):
                cross=np.array([[0,-axis[2],axis[1]],[axis[2],0,-axis[0]],[-axis[1],axis[0],0]])
                rotate=np.eye(4);rotate[:3,:3]=np.eye(3)+math.sin(angle)*cross+(1-math.cos(angle))*(cross@cross)
                pose=pose@origin@rotate
            pose=pose@tcp
            if pose[2,3]<minimum:
                raise RuntimeError('held transfer payload envelope below stack clearance')
            checked+=1
        if not checked:raise RuntimeError('held transfer preview has no trajectory samples')
        self.report.setdefault('payload_transfer_checks',[]).append(dict(samples=checked,min_tcp_z_m=minimum,enclosing_radius_m=radius,layers=self.layers))
        return minimum

    def move(self, operation, xyz, object_state):
        if self.demo.get('business_mode')=='row' and operation=='PREGRASP' and xyz[2]>=.11:
            state,_,_=self.stopped()
            q=state.tcp.pose.orientation
            expected=np.array(getattr(self,'active_orientation_xyzw',self.demo['orientation_xyzw']))
            actual=np.array([q.x,q.y,q.z,q.w])
            angle=2*math.acos(float(np.clip(abs(actual@expected),0,1)))
            if np.linalg.norm(position(state)-np.array(xyz))<=.0005 and angle<=.001:
                self.held=object_state=='held'
                self.gripper_state(held=self.held,idle=True)
                self.report.setdefault('already_at_observation_pose',[]).append(dict(
                    requested_tcp_m=xyz,measured_tcp_m=position(state).tolist(),stamp_ns=self.now()))
                return
        if self.demo.get('business_mode')=='row' and operation=='DESCEND':
            current=position(self.pair()[0])
            model_pick=object_state=='empty' and self.demo.get('pick_neighbor_policy')=='finger_model'
            height=(self.demo['transfer_tcp_base_z_m'] if object_state=='held' or model_pick
                    else self.demo['hover_tcp_base_z_m'])
            if (np.linalg.norm(current[:2]-np.array(xyz[:2]))>.003 or
                    model_pick and abs(current[2]-height)>self.c['position_tolerance_m']):
                if current[2]<height-.001:
                    self.move('RETREAT',[*current[:2],height],object_state)
                self.move('PREGRASP',[*xyz[:2],height],object_state)
        if self.demo.get('calibration_mode')=='supervised_flat_table' and operation in ('DESCEND','RETREAT'):
            current=position(self.pair()[0])
            if np.linalg.norm(current[:2]-np.array(xyz[:2]))>.003:
                raise RuntimeError('vertical segment is not aligned within 3mm')
            xyz=[*current[:2],xyz[2]]
        self.at_home=False
        self.held = object_state == 'held'
        # 实际请求由机械臂服务完成规划和整条路径验证。持物横移另作包络预检查，
        # 观察与换向入口另作走廊预检查，常规空爪和竖直动作直接进入服务端规划。
        direct_guarded_row_motion=(self.demo.get('business_mode')=='row' and
            self.demo.get('calibration_mode')=='supervised_flat_table' and
            (operation in ('HOME','DESCEND','RETREAT') or operation=='PREGRASP' and not self.held))
        plan=None if direct_guarded_row_motion else self.arm_request(operation, xyz, True)
        transfer_floor=None
        if self.demo.get('calibration_mode')=='supervised_flat_table' and self.held and operation=='PREGRASP':
            transfer_floor=self.check_held_transfer(plan)
        # The fixed arm guard does not contain payload/stack geometry. A reviewed
        # commissioning artifact and a concrete operator segment review are
        # mandatory for these stages; never describe PLAN_READY as payload proof.
        if self.demo.get('calibration_mode')!='supervised_flat_table' and (self.held or operation == 'DESCEND'):
            path = self.out/('segment-'+str(len(self.report['actions']))+'-plan.json')
            write(path,convert(plan))
            self.confirm('SEGMENT', self.layers, operation=operation, nominal_tcp_m=xyz,
                         plan_file=str(path), plan_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                         held=self.held, requirements='review calibrated contact/payload/stack swept envelope')
        refresh=None
        if operation=='DESCEND' and object_state=='empty':
            # Planning and human review take time. Reobserve after both, inside
            # arm_request after its final stopped/parameter checks, immediately
            # before sending the descent goal. Never reuse the pre-plan stamp.
            def refresh():
                from demo import qualified_target, distance
                try:
                    latest=self.qualify_pick_target(self.target_measurement,self.observe(reuse_recent=True),allow_motion=False)
                except RuntimeError as e:
                    from demo import TargetUnavailable,ObservationUnavailable
                    if not isinstance(e,ObservationUnavailable) and str(e) not in (
                            'unlocated/incomplete observation in target view',
                            'target absent, displaced or association ambiguous',
                            'neighbor/unknown too close for reviewed finger envelope',
                            'target label unresolved/changed','target moved / repeatability exceeds 3mm'):
                        raise
                    raise TargetUnavailable(self.target_measurement) from e
                identity_offset=2 if self.demo.get('business_mode')=='row' else 1
                if tuple(latest['scene_identity'][identity_offset:])!=tuple(self.target_measurement['scene_identity'][identity_offset:]):
                    raise RuntimeError('camera/scene/reference identity changed during descent qualification')
                delta=distance(latest['base_surface_center_m'],self.target_measurement['base_surface_center_m'])
                xy_delta=distance(latest['base_surface_center_m'][:2],xyz[:2])
                if max(delta,xy_delta)>self.demo['descent_tolerance_m']:
                    self.report.setdefault('descent_target_rejections',[]).append(dict(
                        prior_position_m=self.target_measurement['base_surface_center_m'],
                        latest_position_m=latest['base_surface_center_m'],
                        requested_tcp_m=xyz,measurement_delta_m=delta,target_xy_delta_m=xy_delta))
                    from demo import TargetRecheckRequired
                    raise TargetRecheckRequired(latest)
                if latest['scene_identity'][1]!=self.target_measurement['scene_identity'][1]:
                    self.report.setdefault('descent_stationary_requalifications',[]).append(dict(
                        previous_epoch=self.target_measurement['scene_identity'][1],
                        current_epoch=latest['scene_identity'][1],samples=latest['samples'],
                        measurement_delta_m=delta))
                self.check_descent_age(latest['last_stamp_ns'])
        if operation=='DESCEND' and object_state=='held' and self.demo.get('business_mode')=='row':
            def refresh():
                self.check_row_slot_empty(xyz[:2],allow_motion=False,reuse_recent=True)
        self.active_transfer_floor=transfer_floor
        try:
            self.arm_request(operation, xyz, False,before_send=refresh)
        finally:
            self.active_transfer_floor=None
        self.at_home=operation=='HOME'

    def observe(self, frame_filter=None, timeout_s=None, duration_s=2., window_validator=None,
                reuse_recent=False):
        saved=(self.observation_window if reuse_recent and self.demo.get('business_mode')=='row' else None)
        self.observation_window=None
        self.stopped()
        start=self.now()
        deadline = time.monotonic()+(self.demo['observation_timeout_s'] if timeout_s is None else timeout_s)
        initial,joint=self.pair()
        frames=[]; seen=set(); anchor=position(initial); anchor_joints=np.array(joint.position)
        window_signature=None; scene_source=None; window_error=None
        reused_stamps=set()
        if (saved is not None and 0<=(start-saved['frames'][-1]['stamp_ns'])*1e-9<=.3
                and np.linalg.norm(anchor-saved['anchor'])<=.001
                and np.max(np.abs(anchor_joints-saved['anchor_joints']))<=.001):
            anchor=saved['anchor']
            anchor_joints=saved['anchor_joints']
            # 对相同静止姿态的旧窗口重新应用本次净空过滤，并等待调用后的新帧。
            for frame in saved['frames']:
                if frame_filter is not None and not frame_filter(frame):
                    frames.clear()
                else:
                    frames.append(frame)
            if frames:
                latest=frames[-1]
                window_signature=(latest.get('stationary_generation'),latest.get('controller_epoch'))
                scene_source=(latest.get('track_namespace'),latest.get('identity'),
                              tuple(latest.get('source_K',[])),latest.get('source_epoch'))
                seen={tuple(frame['rgbd_frame_key']) for frame in frames}
                reused_stamps={frame['stamp_ns'] for frame in frames}
        minimum_stamp=frames[-1]['stamp_ns'] if frames else start
        if not frames:
            self.scene.clear()
        while time.monotonic() < deadline:
            self.spin()
            s,j=self.pair()
            detail=dict(stamp_ns=stamp(s),age_s=(self.now()-stamp(s))*1e-9,
                reference_valid=s.reference_valid,feedback_fresh=s.feedback_fresh,
                busy=s.busy,controller_mode=s.controller_mode,
                vmax_rad_s=max(map(abs,j.velocity)),
                tcp_drift_m=float(np.linalg.norm(position(s)-anchor)),
                joint_drift_rad=float(np.max(np.abs(np.array(j.position)-anchor_joints))))
            if not (s.feedback_fresh and s.reference_valid and not s.busy and not s.simulated and
                    s.controller_mode==5 and 0<=detail['age_s']<=.2 and
                    math.isfinite(detail['vmax_rad_s']) and detail['tcp_drift_m']<=.001
                    and detail['joint_drift_rad']<=.001):
                self.report.setdefault('observation_feedback_faults',[]).append(detail)
                raise RuntimeError('observation feedback/reference or pose changed: '+json.dumps(detail))
            if detail['vmax_rad_s']>.002:
                # 微小持位修正使本窗口作废。重新证明连续停稳，再采独立窗口。
                # 保留最初的位置锚点和截止时间，避免连续小偏移逐次累积。
                reset=dict(detail,discarded_frames=len(frames))
                self.report.setdefault('observation_stationary_resets',[]).append(reset)
                print(json.dumps(dict(phase='OBSERVATION_WINDOW_RESET',**reset)),flush=True)
                frames.clear();self.scene.clear();window_signature=None
                try:
                    self.stopped(timeout=max(0.,deadline-time.monotonic()))
                except RuntimeError as error:
                    if not str(error).startswith('fresh measured stationary interval unavailable:'):
                        raise
                    current,current_joint=self.pair()
                    healthy=(current.feedback_fresh and current.reference_valid and not current.busy and
                        not current.simulated and current.controller_mode==5 and
                        0<=(self.now()-stamp(current))*1e-9<=.2 and
                        all(math.isfinite(v) for v in current_joint.velocity) and
                        np.linalg.norm(position(current)-anchor)<=.001)
                    if not healthy:
                        raise
                    from demo import ObservationUnavailable
                    self.report.setdefault('observation_stationary_timeouts',[]).append(dict(
                        stamp_ns=self.now(),reason=str(error),discarded_frames=len(frames)))
                    raise ObservationUnavailable('observation window exhausted while requalifying stationary feedback') from error
                start=self.now();minimum_stamp=start;self.scene.clear()
                continue
            while self.scene:
                r=self.scene.popleft()
                if r['stamp_ns']<=minimum_stamp or not 0 <= (self.now()-r['stamp_ns'])*1e-9<=.5:
                    continue
                if not r.get('base_transform_valid'):
                    if frames:
                        restart=dict(stamp_ns=r['stamp_ns'],reason='base transform qualification unavailable',
                            detail=r.get('base_transform_reason'),discarded_frames=len(frames))
                        self.report.setdefault('observation_window_restarts',[]).append(restart)
                        print(json.dumps(dict(phase='OBSERVATION_WINDOW_RECHECK',**restart)),flush=True)
                    frames.clear();window_signature=None
                    continue
                current_source=(r.get('track_namespace'),r.get('identity'),
                                tuple(r.get('source_K',[])),r.get('source_epoch'))
                if scene_source is None:
                    scene_source=current_source
                elif current_source!=scene_source:
                    raise RuntimeError('scene epoch/configuration changed')
                signature=(r.get('stationary_generation'),r.get('controller_epoch'))
                reason=None
                if frames and signature!=window_signature:
                    reason='stationary qualification changed'
                elif frames and (r['stamp_ns']-frames[-1]['stamp_ns'])*1e-9>.3:
                    reason='scene frame gap'
                if reason:
                    restart=dict(stamp_ns=r['stamp_ns'],reason=reason,discarded_frames=len(frames))
                    self.report.setdefault('observation_window_restarts',[]).append(restart)
                    print(json.dumps(dict(phase='OBSERVATION_WINDOW_RECHECK',**restart)),flush=True)
                    frames.clear()
                window_signature=signature
                key=tuple(r.get('rgbd_frame_key') or [r['source_epoch'],r['sequence'],r['stamp_ns']])
                if key in seen:
                    continue
                seen.add(key);r['rgbd_frame_key']=key
                if frame_filter is not None and not frame_filter(r):
                    # 不拼接未知框出现前后的帧；重新开始完整连续观察窗口。
                    frames.clear()
                    continue
                frames.append(r)
            if (len(frames)>=20 and frames[-1]['stamp_ns']>start
                    and (frames[-1]['stamp_ns']-frames[0]['stamp_ns'])*1e-9>=duration_s):
                if reused_stamps:
                    while len(frames)>20 and (frames[-1]['stamp_ns']-frames[1]['stamp_ns'])*1e-9>=duration_s:
                        frames.pop(0)
                if window_validator is not None:
                    from demo import ObservationUnavailable
                    # 在原截止时间内滚动寻找完整合格窗口，不能拼接缺失前后的测量。
                    while len(frames)>20 and (frames[-1]['stamp_ns']-frames[1]['stamp_ns'])*1e-9>=duration_s:
                        frames.pop(0)
                    try:
                        window_validator(frames)
                    except ObservationUnavailable as error:
                        window_error=error
                        continue
                self.observation_window=dict(frames=list(frames),anchor=anchor.copy(),
                                             anchor_joints=anchor_joints.copy())
                if reused_stamps:
                    self.report.setdefault('observation_window_reuses',[]).append(dict(
                        entered_ns=start,first_stamp_ns=frames[0]['stamp_ns'],
                        last_stamp_ns=frames[-1]['stamp_ns'],samples=len(frames),
                        reused_samples=sum(frame['stamp_ns'] in reused_stamps for frame in frames)))
                write(self.out/('observation-'+str(self.now())+'.json'),frames)
                return frames
        if window_error is not None:
            self.report.setdefault('observation_window_timeouts',[]).append(dict(
                reason=str(window_error),samples=len(frames),duration_s=duration_s))
            raise window_error
        from demo import ObservationUnavailable
        raise ObservationUnavailable('two seconds of fresh independent scene frames unavailable')

    def check_descent_age(self, t):
        self.descent_stamp = t
        if not 0 <= (self.now()-t)*1e-9<=.5:
            raise RuntimeError('target measurement expired before descent')

    def gripper_move(self, raw, contact):
        if not self.prechecked:
            raise RuntimeError('gripper command requires completed precheck')
        self.stopped()
        before=self.gripper_state(held=self.held, idle=True)
        if contact and (before.object_detected or before.grip_status!=1):
            raise RuntimeError('new grasp requires confirmed empty baseline')
        g=MoveGripper.Goal();g.operation=g.MOVE;g.position_raw=raw
        g.force_percent=self.demo['force_percent'];g.require_contact=contact;g.timeout_sec=5.
        started=self.now()
        self.observation_window=None
        self.grip_pending=self.grip_client.send_goal_async(g)
        self.grip_goal=self.wait(self.grip_pending,8);self.grip_pending=None
        if not self.grip_goal.accepted:
            raise RuntimeError('gripper goal rejected')
        self.grip_terminal=self.grip_goal.get_result_async()
        reply=self.wait(self.grip_terminal,8,monitor=True)
        self.report['gripper_actions'].append(dict(status=reply.status,result=convert(reply.result)))
        r=reply.result;s=r.final_state
        if not (reply.status==4 and r.code==(2 if contact else 1) and r.command_attempted and
                not r.simulated and not s.simulated and s.connected and s.feedback_valid and
                s.initialization==1 and not s.busy and not s.object_dropped and
                stamp(s)>started and 0 <= s.feedback_age_sec<=.5 and 0 <= (self.now()-stamp(s))*1e-9<=.2):
            raise RuntimeError('new real gripper completion not established: '+r.detail)
        if contact:
            if not (s.object_detected and s.grip_status==2 and
                    (s.grip_status!=before.grip_status or s.position_raw!=before.position_raw)):
                raise RuntimeError('no new grasp transition')
        elif s.object_detected or s.grip_status!=1 or abs(s.position_raw-raw)>5:
            raise RuntimeError('release/open feedback mismatch')
        # The Action result can arrive before its corresponding topic sample.
        # Adopt this validated final sample and ignore older queued topic data;
        # do not interpret a pre-command empty sample as a post-grasp drop.
        self.receive_gripper(s)
        self.grip_goal=self.grip_terminal=None

    def open_empty(self):
        if self.gripper_state(idle=True).object_detected:
            raise RuntimeError('unexpected object; refusing to open')
        self.gripper_move(self.demo['measured']['open_raw'],False)

    def grasp(self):
        self.gripper_move(self.demo['measured']['close_raw'],True)
        self.held=True
        deadline=time.monotonic()+2.
        while time.monotonic()<deadline:
            self.spin()
            self.gripper_state(held=True,idle=True)
        self.report['grip_retention_verified_at_ns']=self.now()

    def confirm_support(self, layer):
        self.gripper_state(held=True,idle=True)
        self.confirm('SUPPORT',layer,requirements='piece supported at the intended layer, still held; not suspended')

    def release(self):
        self.held=False  # A deliberate release is not interpreted as a drop.
        self.gripper_move(self.demo['measured']['open_raw'],False)
        self.last_placement_stamp=self.now()

    def restore_row_held(self, record):
        from demo import execution_blockers,qualified_scene_window
        blockers=execution_blockers(self.demo)
        if blockers:raise RuntimeError('; '.join(blockers))
        previous=record['previous']
        if (previous['config']['source_sha256']!=self.c['source_sha256'] or
                previous['config']['rotation_tolerance_rad']!=self.c['rotation_tolerance_rad']):
            raise RuntimeError('row continuation robot/camera model or orientation tolerance changed')
        current,_,_=self.stopped();old=record['transfer_endpoint']['pose']
        old_xyz=np.array([old['position'][k] for k in ('x','y','z')])
        old_q=np.array([old['orientation'][k] for k in ('x','y','z','w')])
        q=current.tcp.pose.orientation;now_q=np.array([q.x,q.y,q.z,q.w])
        angle=lambda a,b:2*math.acos(float(np.clip(abs(a@b),0,1)))
        if (np.linalg.norm(position(current)-old_xyz)>.003 or
                angle(now_q,old_q)>self.c['rotation_tolerance_rad'] or
                angle(old_q,np.array(self.demo['orientation_xyzw']))>self.c['rotation_tolerance_rad']):
            raise RuntimeError('row continuation arm no longer matches recorded stopped transfer pose')
        grip=self.gripper_state(held=True,idle=True)
        if abs(grip.position_raw-record['recorded_gripper_raw'])>5:
            raise RuntimeError('row continuation grip width differs from recorded held state')
        self.held=True
        evidence=qualified_scene_window(self.observe())
        if evidence['scene_identity'][2:]!=record['source']['scene_identity'][2:]:
            raise RuntimeError('row continuation camera/source identity changed')
        self.target_measurement=record['source'];self.expected_stack_class=record['source']['class_name']
        self.layers=previous['layers'];self.commissioned=True;self.at_home=False
        self.report.update(workspace_observation=self.demo['workspace_observation'],
            row_held_resume=dict(result_file=self.demo['resume_row_result_file'],
                old_measured_transfer_tcp=record['transfer_endpoint'],measured_tcp=convert(current.tcp),
                recorded_gripper_raw=record['recorded_gripper_raw'],current_gripper_raw=grip.position_raw,
                source_identity_window=evidence,
                prior_gripper_request_settings_recorded=record['prior_gripper_request_settings_recorded'],
                missing_prior_request_settings_bound_to_site_config=not record['prior_gripper_request_settings_recorded']))

    def qualify_pick_target(self, seed, frames, allow_reclassify=False, allow_motion=True):
        if self.demo.get('business_mode')!='row':
            from demo import qualified_target
            return qualified_target(self.demo,seed,frames,allow_reclassify=allow_reclassify)
        first=True
        def window():
            nonlocal first
            current_frames=frames if first else self.observe()
            first=False
            return self.qualify_pick_target_window(seed,current_frames,allow_reclassify)
        from demo import qualified_scene_window
        source=dict(scene_identity=qualified_scene_window(frames)['scene_identity'])
        ceiling=(self.demo['transfer_tcp_base_z_m'] if self.demo.get('pick_neighbor_policy')=='finger_model'
                 else self.demo['hover_tcp_base_z_m'])
        return self.observe_with_approach(window,source,'pick_target',ceiling,
            focus_xy=seed['base_surface_center_m'][:2],allow_motion=allow_motion)

    def qualify_pick_target_window(self, seed, frames, allow_reclassify=False):
        from demo import qualified_target
        if self.demo.get('business_mode')!='row':
            return qualified_target(self.demo,seed,frames,allow_reclassify=allow_reclassify)
        attempts=1 if self.demo.get('active_observation',{}).get('enabled') else 6
        for attempt in range(attempts):
            envelope=self.pick_envelope if self.demo.get('pick_neighbor_policy')=='finger_model' else None
            half_extent,heights=self.pick_view_bounds() if envelope else (None,None)
            rois={id(frame):self.row_slot_visible(frame,seed['base_surface_center_m'][:2],half_extent,heights)
                  for frame in frames}
            def neighbor_conflict(target,other):
                delta=np.array(other[:2])-target[:2]
                return (np.linalg.norm(delta)<envelope['outer_radius_m'] and
                        np.all(delta>=envelope['lower_xy_m']) and np.all(delta<=envelope['upper_xy_m']))
            try:
                return qualified_target(self.demo,seed,frames,geometry_only=True,
                    unlocated_overlap=lambda frame,detection:detection_overlaps_roi(detection,rois[id(frame)]),
                    neighbor_conflict=neighbor_conflict if envelope else None)
            except RuntimeError as error:
                if str(error)!='unlocated/incomplete observation in target view' or attempt==attempts-1:
                    raise
                self.report.setdefault('row_mask_window_retries',[]).append(dict(
                    attempt=attempt+1,first_stamp_ns=frames[0]['stamp_ns'],
                    last_stamp_ns=frames[-1]['stamp_ns'],reason=str(error)))
                # Restart a complete stationary window; never remove bad frames.
                frames=self.observe()

    def pick_view_bounds(self):
        # 三维近邻用棋子中心判断，二维外框已经包含棋子半径，只扩展完整指体和姿态误差。
        envelope=self.pick_envelope
        half=np.maximum(np.abs(envelope['lower_xy_m']),np.abs(envelope['upper_xy_m']))-envelope['piece_radius_m']
        vertical_margin=self.demo['descent_tolerance_m']+envelope['rotation_margin_m']
        heights=[self.demo['table_z_m']-vertical_margin,
                 self.demo['table_z_m']+self.demo['piece_thickness_m']+vertical_margin]
        return half.tolist(),heights

    def base_camera_transform(self, frame):
        calibration=frame['identity']['calibration']
        content=Path(calibration).read_bytes()
        if hashlib.sha256(content).hexdigest()!=frame['identity']['calibration_sha256']:
            raise RuntimeError('scene hand-eye calibration file changed')
        flange_camera=np.array(yaml.load(content,Loader=getattr(yaml,'CSafeLoader',yaml.SafeLoader))['flange_T_camera'])
        tcp_pose=next(sampled_tcp_poses(self.transfer_guard,[{'positions':frame['source_joint_position_rad']}]))
        return tcp_pose@np.linalg.inv(np.array(self.transfer_guard['tcp']).reshape(4,4))@flange_camera

    def row_slot_visible(self, frame, slot, clearance_xy=None, height_range=None):
        base_camera=self.base_camera_transform(frame)
        camera_base=np.linalg.inv(base_camera)
        K=np.array(frame['source_K']).reshape(3,3)
        width,height=frame['image_size']
        rx,ry=self.demo['row_slot_clearance_xy_m'] if clearance_xy is None else clearance_xy
        projected=[]
        heights=height_range if height_range is not None else [self.demo['table_z_m']+self.demo['piece_thickness_m']]
        for plane_z in heights:
            for dx,dy in ((-rx,-ry),(-rx,0),(-rx,ry),(0,-ry),(0,ry),(rx,-ry),(rx,0),(rx,ry)):
                camera=camera_base@np.array([slot[0]+dx,slot[1]+dy,plane_z,1.])
                if camera[2]<=0:raise RuntimeError('row slot is behind camera')
                uv=K@camera[:3];uv=uv[:2]/uv[2]
                if not (10<=uv[0]<width-10 and 10<=uv[1]<height-10):
                    from demo import ObservationUnavailable
                    raise ObservationUnavailable('entire row-slot clearance is not inside camera view')
                projected.append(uv)
        projected=np.array(projected)
        return [*projected.min(axis=0).tolist(),*projected.max(axis=0).tolist()]

    def wait_observation_view(self, source, start, purpose):
        # 规划也会暂时使视觉静止资格重建；等待新鲜合格帧，不降低原门限。
        began=time.monotonic();deadline=began+self.demo['observation_timeout_s']
        _,joint=self.pair();anchor=list(joint.position)
        last=dict(reason='no scene received');minimum_scene_stamp=0
        while time.monotonic()<deadline:
            self.spin()
            state,joint=self.pair()
            self.gripper_state(held=self.held,idle=True)
            speed=max(map(abs,joint.velocity))
            if not (state.reference_valid and state.feedback_fresh and not state.simulated and not state.busy
                    and state.controller_mode==5 and 0<=(self.now()-stamp(state))*1e-9<=.2
                    and math.isfinite(speed)
                    and max(abs(a-b) for a,b in zip(anchor,joint.position))<=.001
                    and np.linalg.norm(position(state)-start)<=.003):
                raise RuntimeError('stationary feedback/reference lost while waiting for observation scene')
            if speed>.002:
                row=dict(purpose=purpose,status='REQUALIFY_STOP',vmax_rad_s=speed,stamp_ns=self.now())
                self.report.setdefault('observation_scene_stationary_resets',[]).append(row)
                print(json.dumps(dict(phase='OBSERVATION_VIEW_RESET',**row)),flush=True)
                self.stopped(timeout=max(0.,deadline-time.monotonic()))
                minimum_scene_stamp=self.now()
                continue
            frame=self.latest_scene
            if frame is None or frame['stamp_ns']<=minimum_scene_stamp:continue
            q=frame.get('source_joint_position_rad',[])
            last=dict(age_s=(self.now()-frame['stamp_ns'])*1e-9,
                      base_valid=frame.get('base_transform_valid'),coverage=frame.get('full_scene_coverage'),
                      detection_valid=frame.get('detection_valid'),reason=frame.get('base_transform_reason'),
                      joint_delta_rad=max(abs(a-b) for a,b in zip(q,joint.position)) if len(q)==6 else None)
            if not (last['detection_valid'] and last['base_valid'] and last['coverage']
                    and 0<=last['age_s']<=.5 and last['joint_delta_rad'] is not None and last['joint_delta_rad']<=.001):
                continue
            identity=(frame['track_namespace'],json.dumps(dict(identity=frame['identity'],K=frame['source_K']),sort_keys=True),frame['source_epoch'])
            if identity!=tuple(source['scene_identity'][2:]):
                raise RuntimeError('camera/scene/reference identity changed before observation approach')
            row=dict(purpose=purpose,status='READY',waited_s=time.monotonic()-began,**last)
            self.report.setdefault('observation_scene_waits',[]).append(row)
            return frame
        row=dict(purpose=purpose,status='TIMEOUT',waited_s=time.monotonic()-began,**last)
        self.report.setdefault('observation_scene_waits',[]).append(row)
        raise RuntimeError('observation scene readiness timeout: '+json.dumps(last,ensure_ascii=False))

    def image_point_on_table(self, frame, pixel):
        camera=self.base_camera_transform(frame)
        ray=camera[:3,:3]@np.linalg.solve(np.array(frame['source_K']).reshape(3,3),[*pixel,1.])
        plane=self.demo['table_z_m']+self.demo['piece_thickness_m']
        scale=(plane-camera[2,3])/ray[2]
        if scale<=0 or not np.isfinite(scale):
            raise RuntimeError('observation image ray does not intersect the tabletop ahead')
        return (camera[:3,3]+scale*ray)[:2]

    def observation_direction(self, frame, focus_xy, clearance_xy=None, height_range=None):
        camera=self.base_camera_transform(frame)
        point=np.linalg.inv(camera)@np.array([*focus_xy,
            self.demo['table_z_m']+self.demo['piece_thickness_m'],1.])
        if point[2]<=0:
            raise RuntimeError('observation focus is behind camera')
        projection=np.array(frame['source_K']).reshape(3,3)@point[:3]
        pixel=projection[:2]/projection[2]
        from demo import complete_detection,ObservationUnavailable
        try:
            roi=self.row_slot_visible(frame,focus_xy,clearance_xy,height_range)
        except ObservationUnavailable:
            blocking=[]
        else:
            blocking=[d['box_xyxy'] for d in frame['detections'] if not complete_detection(d)
                      and d.get('box_xyxy') is not None and detection_overlaps_roi(d,roi)]
        if blocking:
            centers=[np.array([(box[0]+box[2])/2,(box[1]+box[3])/2]) for box in blocking]
            closest=min(centers,key=lambda center:np.linalg.norm(center-pixel))
            pixel=closest
        center_pixel=np.array(frame['source_K']).reshape(3,3)[:2,2]
        direction=self.image_point_on_table(frame,pixel)-self.image_point_on_table(frame,center_pixel)
        length=np.linalg.norm(direction)
        return direction/length if length>.001 else np.zeros(2)

    def observe_with_approach(self, observe_window, source, purpose, ceiling, focus_xy=None, allow_motion=True):
        """Bound visual recovery by calibrated XY viewpoints and two lower views."""
        from demo import ObservationUnavailable
        policy=self.demo.get('active_observation',{})
        moves=[]
        if policy.get('enabled') and allow_motion:
            moves=[('direction',step) for step in policy.get('lateral_steps_m',[])]
            side=policy.get('side_step_m',0.)
            if side:
                moves.extend([('side',side),('side',-side)])
            drops=[('descend',drop) for drop in policy.get('drop_offsets_m',[])]
            # 放置净空先沿下降轴换高度，成功后可原位复查，避免重新对准又失去有效视角。
            moves=drops+moves if purpose=='empty_row_slot' else moves+drops
        if focus_xy is None:
            focus_xy=position(self.pair()[0])[:2].tolist()
        attempt=0;qualification_rechecks=0
        while True:
            try:
                return observe_window()
            except RuntimeError as error:
                if not isinstance(error,ObservationUnavailable) and str(error) not in (
                        'two seconds of fresh independent scene frames unavailable',
                        'unlocated/incomplete observation in target view',
                        'released row piece not independently visible/unique',
                        'target absent, displaced or association ambiguous'):
                    raise
                if str(error) in ('observation window exhausted while requalifying stationary feedback',
                                  'two seconds of fresh independent scene frames unavailable') and qualification_rechecks<2:
                    qualification_rechecks+=1
                    row=dict(purpose=purpose,attempt=qualification_rechecks,reason=str(error),stamp_ns=self.now())
                    self.report.setdefault('observation_qualification_rechecks',[]).append(row)
                    print(json.dumps(dict(phase='OBSERVATION_QUALIFICATION_RECHECK',**row)),flush=True)
                    continue
                moved=False
                while attempt<len(moves):
                    kind,offset=moves[attempt];attempt+=1
                    state,joint,_=self.stopped()
                    self.gripper_state(held=self.held,idle=True)
                    current=position(state)
                    tolerance=self.c['position_tolerance_m']
                    lowest=ceiling-max(policy['drop_offsets_m'])
                    if current[2]>ceiling+tolerance or current[2]<lowest-tolerance:
                        raise RuntimeError('observation start is outside the standard high-air viewing band')
                    floor=max(policy['min_tcp_base_z_m'],self.demo['table_z_m']+self.demo['piece_thickness_m']+.045)
                    frame=self.wait_observation_view(source,current,purpose+':before_plan')
                    if kind in ('direction','side'):
                        if purpose=='pick_target' and self.demo.get('pick_neighbor_policy')=='finger_model':
                            clearance,heights=self.pick_view_bounds()
                            direction=self.observation_direction(frame,focus_xy,clearance,heights)
                        else:
                            direction=self.observation_direction(frame,focus_xy)
                        if kind=='side':
                            # 两侧观察打破邻棋遮挡和分割重叠，仍限制在同一高空观察范围内。
                            perpendicular=np.array([-direction[1],direction[0]])
                            xy=np.array(focus_xy)+policy['lateral_steps_m'][0]*direction+offset*perpendicular
                        else:
                            xy=np.array(focus_xy)+offset*direction
                        if np.linalg.norm(xy-current[:2])<.002:
                            continue
                        # 持物放前的低位补查失败后，侧移回到标准高位观察。
                        height=ceiling if purpose=='empty_row_slot' and self.held else max(current[2],lowest)
                        target=[*xy,height]
                        operation='PREGRASP'
                    else:
                        height=ceiling-offset
                        if height<floor+tolerance or height>=current[2]-.001:
                            continue
                        target=[*current[:2],height]
                        operation='DESCEND'
                    row=dict(purpose=purpose,attempt=attempt,stamp_ns=self.now(),start_tcp_m=current.tolist(),
                             target_tcp_m=target,min_tcp_z_m=floor,held=self.held,kind=kind,
                             reason=str(error),status='PLANNING')
                    self.report.setdefault('active_observation_attempts',[]).append(row)
                    print(json.dumps(dict(phase='OBSERVATION_APPROACH_PLAN',**row),ensure_ascii=False),flush=True)
                    plan=self.arm_request(operation,target,True,allow_plan_rejection=True)
                    if plan.code!=0:
                        row.update(status='PLAN_REJECTED',detail=plan.detail)
                        continue
                    points=[dict(positions=list(point.positions)) for point in plan.trajectory.points]
                    poses=list(sampled_tcp_poses(self.transfer_guard,points))
                    radius=policy.get('max_xy_offset_m',.04)
                    if not poses or any(pose[2,3]<floor or pose[2,3]>ceiling+tolerance or
                                        np.linalg.norm(pose[:2,3]-np.array(focus_xy))>radius for pose in poses):
                        row.update(status='PLAN_REJECTED',detail='preview leaves bounded observation corridor')
                        continue
                    if self.held:self.check_held_transfer(plan)
                    def unchanged_start():
                        if np.linalg.norm(position(self.pair()[0])-current)>.003:
                            raise RuntimeError('observation start changed after planning')
                        self.wait_observation_view(source,current,purpose+':before_execute')
                    self.active_observation_floor=floor
                    try:
                        self.arm_request(operation,target,False,before_send=unchanged_start)
                    except RuntimeError as failure:
                        row.update(status='APPROACH_ABORTED',detail=str(failure))
                        raise
                    finally:
                        self.active_observation_floor=None
                    self.at_home=False
                    row['status']='MOVED_FOR_FRESH_OBSERVATION'
                    print(json.dumps(dict(phase='OBSERVATION_APPROACH_RECHECK',**row),ensure_ascii=False),flush=True)
                    moved=True
                    break
                if not moved:
                    raise ObservationUnavailable(str(error)) from error

    def inspect_row_slot(self, slot):
        from demo import RowSlotOccupied
        self.stopped()
        if self.gripper_state(idle=True).object_detected:
            raise RuntimeError('pre-pick slot inspection requires empty gripper')
        frame=self.latest_scene
        if frame is None:raise RuntimeError('scene unavailable before row-slot inspection')
        source=dict(scene_identity=(frame['stationary_generation'],frame['controller_epoch'],frame['track_namespace'],
                    json.dumps(dict(identity=frame['identity'],K=frame['source_K']),sort_keys=True),frame['source_epoch']))
        try:
            evidence=self.observe_with_approach(lambda:self.check_row_slot_empty_window(slot,source),
                source,'inspect_row_occupant',self.demo['transfer_tcp_base_z_m'],focus_xy=slot)
            return dict(evidence,occupied=False)
        except RowSlotOccupied as error:
            return dict(error.evidence,occupied=True)

    def raise_row_transfer(self):
        state,_,_=self.stopped()
        self.gripper_state(held=True,idle=True)
        current=position(state)
        if current[2]<self.demo['transfer_tcp_base_z_m']-.001:
            self.move('RETREAT',[*current[:2],self.demo['transfer_tcp_base_z_m']],'held')

    def confirm_row_slot_occupied(self, slot, evidence):
        from demo import qualified_row_observation,row_slot_occupied,ObservationUnavailable
        focus=evidence['occupant_position_m'][:2]
        qualify=lambda frames:qualified_row_observation(self.demo,focus,frames)
        frames=self.observe(duration_s=self.demo.get('row_verification',{}).get('window_s',3.),
                            window_validator=qualify)
        measured=qualify(frames)
        if tuple(measured['scene_identity'][2:])!=tuple(evidence['scene_identity'][2:]):
            raise RuntimeError('camera/scene/reference identity changed during occupied-slot review')
        if not row_slot_occupied(self.demo,measured['base_surface_center_m'],slot):
            raise ObservationUnavailable('initial row-slot occupancy was not confirmed')
        confirmed=dict(evidence,occupant_position_m=measured['base_surface_center_m'],measurement=measured)
        self.report.setdefault('row_occupied_slot_checks',[]).append(confirmed)
        return confirmed

    def check_row_slot_empty(self, slot, allow_motion=True, reuse_recent=False):
        from demo import RowSlotOccupied
        def inspect_window():
            try:
                return self.check_row_slot_empty_window(slot,reuse_recent=reuse_recent)
            except RowSlotOccupied as error:
                raise RowSlotOccupied(self.confirm_row_slot_occupied(slot,error.evidence)) from error
        return self.observe_with_approach(inspect_window,
            self.target_measurement,'empty_row_slot',self.demo['transfer_tcp_base_z_m'],
            focus_xy=slot,allow_motion=allow_motion)

    def check_row_slot_empty_window(self, slot, source=None, reuse_recent=False):
        from demo import qualified_scene_window,row_slot_occupied,complete_detection,RowSlotOccupied,ObservationUnavailable,distance
        source=self.target_measurement if source is None else source
        self.gripper_state(held=self.held,idle=True)
        ignored_outside_roi=0
        recent=[]
        bridged_stamps=set()
        affected=[]
        def clear_frame(frame):
            nonlocal ignored_outside_roi
            if not all(frame.get(k) for k in ('detection_valid','base_transform_valid','full_scene_coverage')):
                raise RuntimeError('unqualified scene window')
            identity=(frame['track_namespace'],json.dumps(dict(identity=frame['identity'],K=frame['source_K']),sort_keys=True),frame['source_epoch'])
            if identity!=tuple(source['scene_identity'][2:]):
                raise RuntimeError('camera/scene/reference identity changed during row-slot inspection')
            roi=self.row_slot_visible(frame,slot)
            recent[:]=[entry for entry in recent if (frame['stamp_ns']-entry['stamp_ns'])*1e-9<=.3]
            for detection in frame['detections']:
                if complete_detection(detection):
                    point=detection['base_surface_center_m']
                    if row_slot_occupied(self.demo,point,slot):
                        raise RowSlotOccupied(dict(slot_xy_m=list(slot),stamp_ns=frame['stamp_ns'],
                            rgbd_frame_key=frame['rgbd_frame_key'],occupant_position_m=point,
                            scene_identity=source['scene_identity'],box_xyxy=detection.get('box_xyxy')))
                    if detection.get('box_xyxy') is not None:
                        recent[:]=[entry for entry in recent if distance(entry['detection']['base_surface_center_m'][:2],point[:2])>.006]
                        recent.append(dict(stamp_ns=frame['stamp_ns'],detection=detection))
            incomplete=[]
            for detection in frame['detections']:
                if not complete_detection(detection):
                    if detection_overlaps_roi(detection,roi):
                        prior=recent_clear_neighbor(detection,frame['stamp_ns'],recent,slot,
                            self.demo['row_slot_clearance_xy_m'])
                        if prior is not None:
                            bridged_stamps.add(frame['stamp_ns'])
                            point=prior['detection']['base_surface_center_m']
                            if not any(distance(point[:2],old[:2])<.006 for old in affected):
                                affected.append(point)
                            self.report.setdefault('row_slot_temporal_neighbors',[]).append(dict(
                                stamp_ns=frame['stamp_ns'],geometry_stamp_ns=prior['stamp_ns'],
                                slot_xy_m=list(slot),neighbor_position_m=point,box_xyxy=detection['box_xyxy']))
                            continue
                        incomplete.append(dict(box_xyxy=detection.get('box_xyxy'),
                            reason=detection.get('reason'),
                            roi_association_reason=detection.get('roi_association_reason')))
                    else:ignored_outside_roi+=1
            if incomplete:
                self.report.setdefault('row_slot_restarted_frames',[]).append(dict(
                    slot_xy_m=list(slot),stamp_ns=frame['stamp_ns'],
                    rgbd_frame_key=frame['rgbd_frame_key'],roi_xyxy=roi,detections=incomplete))
                return False
            return True
        # 要求至少20帧、连续2秒合格场景；未能与新鲜邻棋匹配的相交未知框重置窗口。
        # 连续取流保留不合格帧之后的新窗口，避免固定窗口反复丢弃合格尾部。
        frames=self.observe(frame_filter=clear_frame,timeout_s=self.demo['observation_timeout_s']*(1 if self.demo.get('active_observation',{}).get('enabled') else 3),reuse_recent=reuse_recent)
        # 邻棋跟踪要求至少5帧直接三维测量，承接帧至多80%；逐帧新鲜2D框保持位置。
        bridged=sum(frame['stamp_ns'] in bridged_stamps for frame in frames)
        direct=[sum(any(complete_detection(d) and distance(d['base_surface_center_m'][:2],point[:2])<.006
                        for d in frame['detections']) for frame in frames) for point in affected]
        if bridged>len(frames)*.8 or any(count<5 for count in direct):
            raise ObservationUnavailable('insufficient fresh neighbor geometry during slot clearance')
        evidence=qualified_scene_window(frames)
        if tuple(evidence['scene_identity'][2:])!=tuple(source['scene_identity'][2:]):
            raise RuntimeError('camera/scene/reference identity changed during held transfer')
        self.check_descent_age(evidence['last_stamp_ns'])
        self.report.setdefault('row_empty_slot_checks',[]).append(dict(slot_xy_m=slot,
            ignored_outside_roi_observations=ignored_outside_roi,
            temporally_bridged_frames=bridged,direct_neighbor_samples=direct,**evidence))
        return evidence

    def verify_row_release_pose(self, place):
        self.stopped();self.gripper_state(held=True,idle=True)
        measured=position(self.pair()[0])
        if np.linalg.norm(measured-np.array(place))>self.demo['descent_tolerance_m']:
            raise RuntimeError('row release TCP differs from calibrated bare-table target')
        self.report.setdefault('row_release_geometry',[]).append(dict(
            nominal_tcp_m=place,measured_tcp_m=measured.tolist(),
            calibrated_bare_table_height=not self.demo.get('experimental_height_adjustment_m',0),
            experimental_height_adjustment_m=self.demo.get('experimental_height_adjustment_m',0),
            support_sensor_confirmed=False,stamp_ns=self.now()))

    def reobserve_row_occupant(self, slot, evidence, focus_xy):
        from demo import qualified_row_observation,ObservationUnavailable
        if evidence is None:
            frame=self.latest_scene
            if frame is None:
                raise ObservationUnavailable('scene unavailable before row-occupant review')
            evidence=dict(scene_identity=(frame['stationary_generation'],frame['controller_epoch'],
                frame['track_namespace'],json.dumps(dict(identity=frame['identity'],K=frame['source_K']),sort_keys=True),
                frame['source_epoch']))
        # 原位确认看不清时先换视角复核，不能直接退回邻棋密集处的抓取流程。
        def observe_occupant():
            frames=self.observe(duration_s=self.demo.get('row_verification',{}).get('window_s',3.),
                window_validator=lambda window:qualified_row_observation(self.demo,slot,window))
            measured=qualified_row_observation(self.demo,slot,frames)
            if tuple(measured['scene_identity'][2:])!=tuple(evidence['scene_identity'][2:]):
                raise RuntimeError('camera/scene/reference identity changed during row occupant review')
            return measured
        return self.observe_with_approach(observe_occupant,evidence,'existing_row_occupant',
            self.demo['hover_tcp_base_z_m'],focus_xy=focus_xy)

    def verify_row_placement(self, slot, source, keep_view=False):
        if not keep_view:
            self.move('PREGRASP',[*slot,self.demo['hover_tcp_base_z_m']],'released_unverified')
        start=position(self.pair()[0]).tolist()
        measured=self.observe_with_approach(lambda:self.verify_row_placement_window(slot,source),
            source,'row_placement_verification',self.demo['hover_tcp_base_z_m'],focus_xy=slot)
        if np.linalg.norm(position(self.pair()[0])-start)>.001:
            self.move('PREGRASP',start,'released_unverified')
        return measured

    def verify_row_placement_window(self, slot, source):
        from demo import qualified_row_observation,distance,ObservationUnavailable
        frames=self.observe(duration_s=self.demo.get('row_verification',{}).get('window_s',3.),
            window_validator=lambda window:qualified_row_observation(self.demo,slot,window))
        measured=qualified_row_observation(self.demo,slot,frames)
        p=measured['base_surface_center_m']
        if tuple(measured['scene_identity'][2:])!=tuple(source['scene_identity'][2:]):
            raise RuntimeError('camera/scene/reference identity changed before row verification')
        if measured['first_stamp_ns']<=self.last_placement_stamp:
            raise RuntimeError('row stability observation predates release')
        if distance(p[:2],slot)>self.demo.get('row_placement_tolerance_m',self.demo['descent_tolerance_m']):
            raise ObservationUnavailable('row piece misplaced beyond configured descent tolerance')
        if abs(p[2]-source['base_surface_center_m'][2])>.003:
            raise ObservationUnavailable('row piece relative top height differs from its bare-table source')
        if self.gripper_state(idle=True).object_detected:
            raise RuntimeError('gripper still detects object after row release')
        self.report.setdefault('row_placement_observations',[]).append(dict(slot_xy_m=slot,
             source_measurement=source,measurement=measured,automatic_geometry_verification=True,
             support_sensor_confirmed=False))
        return measured

    def restore_held_placement(self,pending):
        previous=json.loads(Path(pending['result_file']).read_text())
        if previous['object_state']!='held' or previous['layers']!=pending['layers']:
            raise RuntimeError('continuation does not match the recorded held layer')
        state,joint=self.pair()
        if np.linalg.norm(position(state)-pending['place_tcp_m'])>.001:
            raise RuntimeError('held placement position changed since timeout')
        self.gripper_state(held=True,idle=True)
        if abs(self.gripper.position_raw-previous['gripper_latest']['position_raw'])>5:
            raise RuntimeError('held gripper position changed since timeout')
        self.held=True
        self.expected_stack_class=pending['class_name']
        self.stack_camera_top_z=pending['first_layer_camera_top_z_m']
        self.stack_camera_center_xy=pending['first_layer_camera_center_xy_m']
        self.report['held_placement_continuation']=pending

    def restore_pending_placement(self, pending):
        previous=json.loads(Path(pending['result_file']).read_text())
        if previous['object_state']!='released_unverified' or previous['layers']!=pending.get('layers',0):
            raise RuntimeError('released-layer continuation differs from recorded committed layers')
        if self.gripper_state(idle=True).object_detected:
            raise RuntimeError('released-layer continuation requires empty gripper')
        self.expected_stack_class=pending['class_name']
        if pending.get('layers',0)>0:
            self.stack_camera_top_z=pending['first_layer_camera_top_z_m']
            self.stack_camera_center_xy=pending['first_layer_camera_center_xy_m']
        self.last_placement_stamp=pending['release_stamp_ns']
        self.report['placement_continuation']=pending

    def observe_stack_top(self):
        self.stopped();self.scene.clear()
        end=time.monotonic()+self.demo['observation_timeout_s'];seed=None
        while time.monotonic()<end and seed is None:
            self.spin()
            for row in list(self.scene):
                if not row.get('base_transform_valid'):continue
                matches=[d for d in row['detections'] if d.get('base_geometry_valid') and d.get('complete_center_target') and np.linalg.norm(np.array(d['base_surface_center_m'][:2])-self.demo['stack_xy_m'])<=.015]
                if len(matches)==1:seed=matches[0];break
        if seed is None:raise RuntimeError('placed top piece not independently visible')
        seed=dict(seed,class_name=self.expected_stack_class)
        from demo import qualified_target
        measured=qualified_target(self.demo,seed,self.observe(),observation_only=True)
        return measured

    def restore_confirmed_stack(self, checkpoint):
        if checkpoint.get('invalidated_reason'):
            raise RuntimeError('stack checkpoint invalidated by table rearrangement')
        if self.gripper_state(idle=True).object_detected:
            raise RuntimeError('confirmed-stack continuation requires empty gripper')
        if not 1 <= checkpoint['layers'] < self.demo['max_pieces']:
            raise RuntimeError('confirmed layer count outside this batch')
        self.expected_stack_class=checkpoint['top_observation']['class_name']
        self.move('PREGRASP',[*self.demo['stack_xy_m'],self.demo['hover_tcp_base_z_m']],'empty')
        measured=self.observe_stack_top()
        old=checkpoint['top_observation']['base_surface_center_m']
        if np.linalg.norm(np.array(measured['base_surface_center_m'])-old)>.010:
            raise RuntimeError('current stack differs from the confirmed unchanged stack')
        self.stack_camera_top_z=measured['base_surface_center_m'][2]-(checkpoint['layers']-1)*self.demo['piece_thickness_m']
        self.stack_camera_center_xy=measured['base_surface_center_m'][:2]
        self.demo['stack_xy_m']=list(self.stack_camera_center_xy)
        checkpoint=dict(checkpoint,verified_today=True,current_top_measurement=measured,placement_xy_m=self.demo['stack_xy_m'])
        self.report['restored_stack_checkpoint']=checkpoint
        write(self.out/'restored-stack-checkpoint.json',checkpoint)

    def confirm_placement(self, layer, expected_z):
        if self.demo.get('calibration_mode')=='supervised_flat_table':
            # Absolute camera Z has a known site bias. Verify current XY/class and
            # repeatability; use the first placed layer as the relative Z datum.
            p=[*self.demo['stack_xy_m'],self.demo['hover_tcp_base_z_m']]
            self.move('PREGRASP',p,'released_unverified')
            measured=self.observe_stack_top()
            if measured['first_stamp_ns']<=self.last_placement_stamp:
                raise RuntimeError('stack observation predates release')
            if layer==1:
                self.stack_camera_top_z=measured['base_surface_center_m'][2]
                self.stack_camera_center_xy=measured['base_surface_center_m'][:2]
            elif np.linalg.norm(np.array(measured['base_surface_center_m'][:2])-self.stack_camera_center_xy)>.010:
                raise RuntimeError('stack top displaced over 10mm from first-layer observed center')
            observed=measured['base_surface_center_m'][2]
            relative_expected=self.stack_camera_top_z+(layer-1)*self.demo['piece_thickness_m']
            if abs(observed-relative_expected)>.003:
                raise RuntimeError('stack relative height differs from expected layer')
            self.report.setdefault('stack_observations',[]).append(dict(layer=layer,measurement=measured,relative_expected_z_m=relative_expected))
            self.confirm('STABLE',layer,requirements='piece released, fingers clear; visually confirm supported and stable tower')
            return
        p=self.demo['measured']['stack_stability_observation_pose']
        self.move('PREGRASP',p,'released_unverified')
        from demo import qualified_target
        seed=dict(base_surface_center_m=[*self.demo['stack_xy_m'],expected_z],class_name=self.expected_stack_class)
        measured=qualified_target(self.demo,seed,self.observe())
        if measured['first_stamp_ns']<=self.last_placement_stamp:
            raise RuntimeError('stack observation predates release')
        if abs(measured['base_surface_center_m'][2]-expected_z)>self.demo['descent_tolerance_m']:
            raise RuntimeError('observed stack height mismatch')
        self.gripper_state(idle=True)
        self.confirm('STABLE',layer,expected_top_z_m=expected_z,
                     requirements='piece released; tower supported and stable after withdrawal; fingers clear')

    def home(self):
        if self.gripper_state(idle=True).object_detected:
            raise RuntimeError('cannot finish Home with unexpected object')
        self.move('HOME',None,'empty')

    def halt(self, object_state):
        self.held=False  # Loss-of-grip must not prevent arm cancellation/stop check.
        self.cancel_owned()
        # Gripper cancellation has no physical-stop guarantee. Never send it or
        # automatically open. Resolve only this client's existing result handle.
        if self.grip_pending is not None:
            try:
                self.grip_goal=self.wait(self.grip_pending,8,check_abort=False)
                self.grip_pending=None
            except Exception as e:
                self.report['gripper_acceptance_unknown']=str(e)
        if self.grip_goal is not None and self.grip_goal.accepted:
            try:
                if self.grip_terminal is None:
                    self.grip_terminal=self.grip_goal.get_result_async()
                r=self.wait(self.grip_terminal,8,check_abort=False)
                self.report['gripper_terminal_at_fault']=dict(status=r.status,result=convert(r.result))
            except Exception as e:
                self.report['gripper_completion_unknown']=str(e)
        self.report.update(object_state=object_state,gripper_cancel_guarantees_stop=False)

    def close(self):
        self.grip_client.destroy()
        super().close()


def execute(config, out, authorization):
    from demo import Demo,RowDemo
    rclpy.init();io=RealIO(config,out,authorization)
    def interrupt(*_): io.abort=True
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    demo=(RowDemo if config.get('business_mode')=='row' else Demo)(config,io.emit)
    try:
        demo.run(io)
        if isinstance(demo,RowDemo):
            io.report['status']='COMPLETE' if demo.goal_complete() else 'PARTIAL'
        else:
            io.report['status']='COMPLETE'
    except BaseException as e:
        io.report.update(status='FAULT',error=str(e))
        raise
    finally:
        if isinstance(demo,RowDemo):
            io.report.update(business_mode='row',row_goal_complete=demo.goal_complete(),placements=demo.placements,next_slot_index=demo.slot_index,
                              row_slot_order=demo.c.get('row_slot_order',list(range(len(config['row_slots_m'])))),
                              skipped_slots=demo.skipped_slots,pending_slots=demo.pending_slots,placed_count=len(demo.placements),
                             row_slots_processed=demo.slot_index>=config['max_pieces'],unresolved=demo.unresolved,
                             scan_unresolved=demo.scan_unresolved,
                              search_inventory_processed=bool(demo.layers) and io.report.get('status')!='FAULT' and not demo.unresolved and not demo.scan_unresolved and not demo.pending_slots,
                             all_physical_pieces_confirmed=False)
        io.report.update(ended_ns=io.now(),layers=demo.layers,retired=demo.retired,
                         object_state=demo.object_state,gripper_latest=convert(io.gripper) if io.gripper else None,
                         arm_latest=convert(io.arm[max(io.arm)]) if io.arm else None)
        if isinstance(demo,RowDemo) and demo.object_state=='held' and config.get('resume_row_result_file'):
            from demo import inherit_held_history
            parent=Path(config['resume_row_result_file'])
            previous=json.loads(parent.read_text())
            io.events.flush()
            events=[json.loads(line) for line in (out/'events.jsonl').read_text().splitlines() if line]
            previous_events=[json.loads(line) for line in parent.with_name('events.jsonl').read_text().splitlines() if line]
            combined=inherit_held_history(io.report,events,previous,previous_events)
            (out/'events.jsonl').write_text(''.join(json.dumps(event,ensure_ascii=False)+'\n' for event in combined))
            io.report['held_history_parent']=str(parent)
        write(out/'run-result.json',io.report)
        io.close();rclpy.shutdown()

    return io.report


def preflight(config,out):
    """Read-only endpoint discovery, parameters and fresh stopped feedback."""
    from demo import execution_blockers
    rclpy.init();io=PlanInspector(config,out)
    report=dict(schema='xiangqi_pick_stack_preflight/v1',physical_actions_sent=0,
                configuration_blockers=execution_blockers(config))
    try:
        io.stopped();io.check_parameters()
        report['arm_ready']=True
    except Exception as e:
        report.update(arm_ready=False,arm_error=str(e))
    finally:
        report['arm_latest']=convert(io.arm[max(io.arm)]) if io.arm else None
        report['gripper_latest']=convert(io.gripper) if io.gripper else None
        write(out/'preflight.json',report)
        io.close();rclpy.shutdown()
    return report


def observe_alignment(config,scan,out,authorization):
    """Two checked PTPs: known scan view -> XY-aligned nominal hover. No grip."""
    from demo import select_target,qualified_target
    rclpy.init();io=PlanInspector(config,out);io.execute=True
    io.report.update(schema='xiangqi_alignment_observation/v1',execute=True,
                     authorization=authorization,gripper_commands_sent=0,
                     actual_finger_clearance_m=None,calibration_written=False)
    signal.signal(signal.SIGINT,lambda *_:setattr(io,'abort',True))
    signal.signal(signal.SIGTERM,lambda *_:setattr(io,'abort',True))
    try:
        io.stopped();io.check_parameters()
        s=io.gripper
        if not (s and not s.simulated and s.connected and s.feedback_valid and
                0<=s.feedback_age_sec<=.5 and 0<=(io.now()-stamp(s))*1e-9<=.2 and
                not s.busy and s.grip_status==1 and not s.object_detected and
                not s.object_dropped and 995<=s.position_raw<=1000):
            raise RuntimeError('fresh real open/empty gripper feedback required for alignment observation')
        seed=select_target(config,scan['target_hypotheses'],[])
        view=next(v for v in io.c['views'] if v['id']==seed['view_id'])
        io.report.update(search_seed=seed,mode='observation_only_no_contact')
        io.action(view,True);io.action(view,False)
        # Reuse original guarded collection, with the two-second qualification
        # window rather than the scanner's coarse three-frame inventory.
        io.c=dict(io.c,sampling_mode='stable_window',collection_s=3.)
        io.collect(view)
        frames=[json.loads(line) for line in (out/(view['id']+'-scene.jsonl')).read_text().splitlines()]
        measured=qualified_target(config,seed,frames)
        p=measured['base_surface_center_m']
        hover=[p[0],p[1],max(view['tcp_position_m'][2],p[2]+.100)]
        target=dict(id='alignment-hover',tcp_position_m=hover,
                    tcp_quaternion_xyzw=config['orientation_xyzw'])
        io.report.update(fresh_target=measured,nominal_hover_tcp_m=hover,
                         nominal_tcp_to_piece_top_m=hover[2]-p[2])
        io.action(target,True);io.action(target,False)
        io.report.update(status='HOLD_FOR_OPERATOR_OBSERVATION',returned_home=False,
                         instructions='Observe actual lowest finger to piece-top gap; no automatic descent or gripper action')
        io.phase('HOLD_FOR_OPERATOR_OBSERVATION',class_name=seed['class_name'],nominal_tcp_m=hover,
                 nominal_tcp_to_piece_top_m=hover[2]-p[2],gripper_commands_sent=0)
    except BaseException as e:
        io.report.update(status='FAULT',error=str(e));io.cancel_owned();raise
    finally:
        if io.arm:io.report['arm_latest']=convert(io.arm[max(io.arm)])
        if io.joint:io.report['joint_latest']=convert(io.joint[max(io.joint)])
        io.report['gripper_latest']=convert(io.gripper) if io.gripper else None
        io.report['physical_moves_completed']=sum(not a['plan_only'] and a['status']==4 and
            a['result']['code']==1 and a['result']['executed'] for a in io.report['actions'])
        write(out/'alignment-result.json',io.report)
        io.close();rclpy.shutdown()

    return io.report


def plan_current(config, scan, out):
    """Independent current-start endpoint probes; never a simulated full path."""
    rclpy.init()
    io = PlanInspector(config, out)
    signal.signal(signal.SIGINT, lambda *_: setattr(io, 'abort', True))
    signal.signal(signal.SIGTERM, lambda *_: setattr(io, 'abort', True))
    rows = []
    start = None
    status = 'INCOMPLETE'
    error = None
    try:
        s, j, t = io.stopped()
        start = dict(arm=convert(s), joint=convert(j))
        # Contact probes are geometry sensitivity only, with nominal TCP set at
        # theoretical piece top: no claim of calibrated gripping geometry.
        probes = [('formal_home', 'HOME', None),
                  ('existing_p1_observation', 'PREGRASP', io.c['views'][0]['tcp_position_m'])]
        for k in (1, 2, 8):
            probes.append((f'uncalibrated_nominal_at_layer_{k}_top', 'PREGRASP',
                           [*config['stack_xy_m'], config['table_z_m']+k*config['piece_thickness_m']]))
        for name, op, p in probes:
            rows.append(io.probe(name, op, p))
        io.stopped()
        status = 'CURRENT_START_ENDPOINT_PROBES_FINISHED'
    except BaseException as e:
        status = 'BLOCKED'
        error = str(e)
        # execute=False: only check measured stop; no physical cancel, Home,
        # gripper command, reconnect or fault clearing is requested.
        io.cancel_owned()
        raise
    finally:
        if io.arm:
            io.report['arm_latest'] = convert(io.arm[max(io.arm)])
        if io.joint:
            io.report['joint_latest'] = convert(io.joint[max(io.joint)])
        io.report['gripper_latest'] = convert(io.gripper) if io.gripper else None
        write(out/'current-start-plans.json', dict(status=status, error=error,
              start=start, probes=rows, physical_actions_sent=0,
              chained_path_validated=False, grasp_calibration_validated=False,
              physical_stop_confirmed=io.report['physical_stop_confirmed']))
        write(out/'ros-report.json', io.report)
        io.close()
        rclpy.shutdown()


def sampled_tcp_poses(guard, points):
    origins=[np.asarray(x).reshape(4,4) for x in guard['joint_origins']]
    axes=[np.asarray(x) for x in guard['axes']]
    tcp=np.asarray(guard['tcp']).reshape(4,4)
    for point in points:
        pose=np.eye(4)
        for origin,axis,angle in zip(origins,axes,point['positions']):
            x,y,z=axis
            skew=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
            turn=np.eye(4)
            turn[:3,:3]=np.eye(3)+math.sin(angle)*skew+(1-math.cos(angle))*(skew@skew)
            pose=pose@origin@turn
        yield pose@tcp


class CenterObserver(PlanInspector):
    """Laser alignment only: zero gripper writes, fixed nominal height and attitude."""
    gripper_state=RealIO.gripper_state
    receive_confirmation=RealIO.receive_confirmation
    confirm=RealIO.confirm

    def __init__(self,config,out,authorization):
        super().__init__(dict(config,confirmation_timeout_s=600.),out)
        self.execute=True
        self.ready=False
        self.moving=False
        self.held=True
        self.layers=0
        self.replies=[]
        self.guard=yaml.load(Path(self.c['guard_file']).read_text(),Loader=getattr(yaml,'CSafeLoader',yaml.SafeLoader))
        self.report.update(schema='xiangqi_laser_center_observation/v1',execute=True,
                           authorization=authorization,gripper_commands_sent=0,
                           observations=[],confirmations=[],returned_home=False)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.request_pub=self.n.create_publisher(String,'/pick_stack/confirmation_request',qos)
        self.subs.append(self.n.create_subscription(String,'/pick_stack/confirmation',self.receive_confirmation,10))

    def spin(self,check_abort=True):
        super().spin(check_abort)
        if check_abort and self.ready:
            self.gripper_state(held=True,idle=True)
            if self.moving:
                state,_=self.pair()
                if position(state)[2]<self.height-self.c['position_tolerance_m']:
                    raise RuntimeError('alignment motion went below the fixed observation height')
                q=state.tcp.pose.orientation
                angle=2*math.acos(float(np.clip(abs(np.array([q.x,q.y,q.z,q.w])@self.orientation),0,1)))
                if angle>self.c['rotation_tolerance_rad']:
                    raise RuntimeError('alignment motion changed the fixed observation attitude')

    def prepare(self):
        if not __debug__:raise RuntimeError('physical observation requires Python assertions')
        state,joint,_=self.stopped()
        self.check_parameters()
        self.read_gripper_policy()
        if not self.client.wait_for_server(timeout_sec=5):
            raise RuntimeError('existing arm action unavailable')
        deadline=time.monotonic()+3.
        while time.monotonic()<deadline:
            self.spin()
            try:
                self.gripper_state(held=True,idle=True)
                break
            except RuntimeError:
                continue
        else:raise RuntimeError('fresh initialized held-laser feedback unavailable; no motion sent')
        self.height=float(position(state)[2])
        q=state.tcp.pose.orientation
        self.orientation=np.array([q.x,q.y,q.z,q.w])
        self.rotation=next(sampled_tcp_poses(self.guard,[dict(positions=list(joint.position))]))[:3,:3]
        self.demo['orientation_xyzw']=self.orientation.tolist()
        self.report.update(fixed_tcp_base_z_m=self.height,
                           fixed_orientation_xyzw=self.orientation.tolist(),
                           initial_gripper=convert(self.gripper))
        self.ready=True

    def move_xy(self,name,xy):
        target=[float(xy[0]),float(xy[1]),self.height]
        preview=self.probe(name,'PREGRASP',target)
        if preview['status']!=4 or preview['result']['code']!=0:
            raise RuntimeError(preview['result']['detail'])
        poses=list(sampled_tcp_poses(self.guard,preview['result']['trajectory']['points']))
        if not poses:raise RuntimeError('alignment preview has no trajectory samples')
        minimum=min(float(t[2,3]) for t in poses)
        angle=max(math.acos(float(np.clip((np.trace(self.rotation.T@t[:3,:3])-1)/2,-1,1))) for t in poses)
        if minimum<self.height-self.c['position_tolerance_m'] or angle>self.c['rotation_tolerance_rad']:
            raise RuntimeError('preview would lower or tilt the laser fixture; no move sent')
        self.report.setdefault('fixed_pose_path_checks',[]).append(dict(name=name,min_tcp_z_m=minimum,max_rotation_rad=angle))
        view=dict(id=name,tcp_position_m=target,tcp_quaternion_xyzw=self.orientation.tolist())
        self.moving=True
        try:self.action(view,False)
        finally:self.moving=False


def observe_centers(config,scan,out,authorization,limit=32):
    """Fresh constant-height inventory, then one manually acknowledged center at a time."""
    from demo import distance
    if not 1<=limit<=32:raise ValueError('center limit must be 1..32')
    rclpy.init();io=CenterObserver(config,out,authorization)
    signal.signal(signal.SIGINT,lambda *_:setattr(io,'abort',True))
    signal.signal(signal.SIGTERM,lambda *_:setattr(io,'abort',True))
    try:
        io.prepare()
        if scan is not None and 'fixed_tcp_base_z_m' in scan:
            if abs(io.height-scan['fixed_tcp_base_z_m'])>io.c['position_tolerance_m']:
                raise RuntimeError('current height differs from this laser scan reference')
            io.height=scan['fixed_tcp_base_z_m']
            io.orientation=np.array(scan['fixed_orientation_xyzw'])
            io.demo['orientation_xyzw']=io.orientation.tolist()
            io.report['fixed_tcp_base_z_m']=io.height
        if scan is None:
            io.phase('FIXED_HEIGHT_SCAN')
            for source in io.c['views']:
                view=dict(source,tcp_position_m=[*source['tcp_position_m'][:2],io.height],
                          tcp_quaternion_xyzw=io.orientation.tolist())
                io.move_xy('scan-'+view['id'],view['tcp_position_m'][:2])
                io.collect(view)
            seeds=io.ledger.targets
        else:
            if scan['status']=='FAULT':raise RuntimeError('failed scan cannot supply search centers')
            seeds=scan['target_hypotheses']
        targets=[]
        for seed in seeds:
            if seed['association_state']!='within_view_unique':continue
            if any(distance(seed['base_surface_center_m'][:2],t['base_surface_center_m'][:2])<.030 for t in targets):continue
            targets.append(seed)
            if len(targets)>=limit:break
        if not targets:raise RuntimeError('no distinct qualified search centers')
        io.report['search_targets']=targets
        write(out/'center-search-inventory.json',dict(schema='laser_center_search_inventory/v1',scan_id=io.report['scan_id'],status='COMPLETE_SEARCH_HINTS',target_hypotheses=targets,ended_ns=io.now(),fixed_tcp_base_z_m=io.height,fixed_orientation_xyzw=io.orientation.tolist()))
        for index,seed in enumerate(targets,1):
            io.phase('CENTER_APPROACH',index=index,total=len(targets),search_seed=seed['scan_target_id'])
            io.move_xy('center-'+str(index),seed['base_surface_center_m'][:2])
            actual=convert(io.pair()[0].tcp)
            reply=io.confirm('ALIGNMENT',index,index=index,total=len(targets),
                             search_seed=seed['scan_target_id'],reference_center_m=seed['base_surface_center_m'],
                             actual_tcp=actual,requirements='observe laser spot offset from piece center; record offset or stop; hands clear before next')
            io.report['observations'].append(dict(index=index,search_seed=seed['scan_target_id'],
                reference_target=seed,reference_kind='frozen_new_scan_center',actual_tcp=actual,operator_observation=reply['observation']))
            write(out/'center-observations.json',io.report['observations'])
        io.stopped()
        io.report['status']='COMPLETE_HOLD'
        io.phase('COMPLETE_HOLD',centers=len(io.report['observations']))
    except BaseException as e:
        io.report.update(status='STOPPED',error=str(e))
        io.cancel_owned()
        raise
    finally:
        if io.arm:io.report['arm_latest']=convert(io.arm[max(io.arm)])
        io.report.update(ended_ns=io.now(),gripper_latest=convert(io.gripper) if io.gripper else None)
        write(out/'center-alignment-result.json',io.report)
        io.close();rclpy.shutdown()
    return io.report
