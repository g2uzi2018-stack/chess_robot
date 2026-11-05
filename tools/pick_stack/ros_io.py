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
    """One arm owner, one gripper Action client, bounded waits and no recovery."""
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
        self.layers = 0
        self.last_placement_stamp = 0
        self.home_joints = yaml.safe_load(Path(config['home_config']).read_text())['chess_robot_arm']['ros__parameters']['home']['joint_positions_rad']
        self.replies = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.n.create_publisher(String, '/pick_stack/status', qos)
        self.request_pub = self.n.create_publisher(String, '/pick_stack/confirmation_request', qos)
        self.subs.append(self.n.create_subscription(String, '/pick_stack/confirmation', self.receive_confirmation, 10))

    def emit(self, row):
        if row['phase'] in ('PREGRASP','PICK_ALIGNED') and 'fresh_measurement' in row:
            self.target_measurement=row['fresh_measurement']
            self.expected_stack_class=self.target_measurement['class_name']
        self.layers = row['layers']
        self.report['layers'] = self.layers
        self.report['object_state'] = row['object_state']
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
            current=position(self.pair()[0])
            if current[2]<self.demo['hover_tcp_base_z_m']:
                target=[*current[:2],self.demo['hover_tcp_base_z_m']]
                self.arm_request('RETREAT',target,True)
                self.arm_request('RETREAT',target,False)

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

    def scan(self):
        if not self.commissioned:
            raise RuntimeError('physical scan requires completed pick precheck')
        if not self.at_home:
            raise RuntimeError('workflow must reach formal Home before scanning')
        known=self.demo.get('remaining_search_targets')
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
        inventory=self.demo.get('completed_scan_file')
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
        directory = self.out/'scan'
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
            raise RuntimeError('scan fault; no automatic Home recovery or pick')
        self.stopped()
        if max(abs(a-b) for a,b in zip(self.pair()[1].position,self.home_joints))>self.demo['home_joint_tolerance_rad']:
            raise RuntimeError('scan did not finish at measured formal Home')
        self.at_home=True
        return scanner.report

    def begin_pick(self):
        self.active_orientation_xyzw=self.demo.get('pick_orientation_xyzw',self.demo['orientation_xyzw'])

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

    def arm_request(self, operation, xyz, plan, before_send=None):
        if not self.prechecked:
            raise RuntimeError('physical arm adapter requires completed precheck')
        self.stopped(); self.check_parameters()
        self.gripper_state(held=self.held, idle=True)
        if before_send is not None:
            before_send()
        request=self.arm_goal(operation,xyz,plan)
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
        # Leave the owned handle intact on failure for halt()/cancel_owned().
        if not (reply.status == 4 and reply.result.code == (0 if plan else 1) and reply.result.executed == (not plan)):
            raise RuntimeError(reply.result.detail)
        self.goal = self.terminal = None
        if not plan:
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
        if self.demo.get('calibration_mode')=='supervised_flat_table' and operation in ('DESCEND','RETREAT'):
            current=position(self.pair()[0])
            if np.linalg.norm(current[:2]-np.array(xyz[:2]))>.003:
                raise RuntimeError('vertical segment is not aligned within 3mm')
            xyz=[*current[:2],xyz[2]]
        self.at_home=False
        self.held = object_state == 'held'
        plan = self.arm_request(operation, xyz, True)
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
                    latest=self.qualify_pick_target(self.target_measurement,self.observe())
                except RuntimeError as e:
                    from demo import TargetUnavailable
                    raise TargetUnavailable(self.target_measurement) from e
                if (latest['scene_identity'][1:]!=self.target_measurement['scene_identity'][1:] or
                        distance(latest['base_surface_center_m'],self.target_measurement['base_surface_center_m'])>
                        self.demo['descent_tolerance_m']):
                    raise RuntimeError('target changed while planning/reviewing descent')
                self.check_descent_age(latest['last_stamp_ns'])
        if operation=='DESCEND' and object_state=='held' and self.demo.get('business_mode')=='row':
            def refresh():
                self.check_row_slot_empty(xyz[:2])
        self.active_transfer_floor=transfer_floor
        try:
            self.arm_request(operation, xyz, False,before_send=refresh)
        finally:
            self.active_transfer_floor=None
        self.at_home=operation=='HOME'

    def observe(self):
        self.stopped()
        start = self.now(); self.scene.clear()
        deadline = time.monotonic()+self.demo['observation_timeout_s']
        frames=[]; seen=set(); anchor=position(self.pair()[0])
        while time.monotonic() < deadline:
            self.spin()
            s,j=self.pair()
            if not (s.feedback_fresh and s.reference_valid and not s.busy and not s.simulated and
                    s.controller_mode==5 and 0 <= (self.now()-stamp(s))*1e-9 <= .2 and
                    max(map(abs,j.velocity))<=.002 and np.linalg.norm(position(s)-anchor)<=.001):
                raise RuntimeError('stationary feedback lost during observation')
            while self.scene:
                r=self.scene.popleft()
                if r['stamp_ns']<=start or not 0 <= (self.now()-r['stamp_ns'])*1e-9<=.5:
                    continue
                if not r.get('base_transform_valid'):
                    continue
                key=tuple(r.get('rgbd_frame_key') or [r['source_epoch'],r['sequence'],r['stamp_ns']])
                if key in seen:
                    continue
                seen.add(key);r['rgbd_frame_key']=key;frames.append(r)
            if len(frames)>=20 and (frames[-1]['stamp_ns']-frames[0]['stamp_ns'])*1e-9>=2:
                write(self.out/('observation-'+str(self.now())+'.json'),frames)
                return frames
        raise RuntimeError('two seconds of fresh independent scene frames unavailable')

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

    def qualify_pick_target(self, seed, frames, allow_reclassify=False):
        from demo import qualified_target
        if self.demo.get('business_mode')!='row':
            return qualified_target(self.demo,seed,frames,allow_reclassify=allow_reclassify)
        for attempt in range(6):
            rois={id(frame):self.row_slot_visible(frame,seed['base_surface_center_m'][:2]) for frame in frames}
            try:
                return qualified_target(self.demo,seed,frames,geometry_only=True,
                    unlocated_overlap=lambda frame,detection:detection_overlaps_roi(detection,rois[id(frame)]))
            except RuntimeError as error:
                if str(error)!='unlocated/incomplete observation in target view' or attempt==5:
                    raise
                self.report.setdefault('row_mask_window_retries',[]).append(dict(
                    attempt=attempt+1,first_stamp_ns=frames[0]['stamp_ns'],
                    last_stamp_ns=frames[-1]['stamp_ns'],reason=str(error)))
                # Restart a complete stationary window; never remove bad frames.
                frames=self.observe()

    def row_slot_visible(self, frame, slot):
        # Full-image detection coverage is insufficient if the destination is
        # outside the camera. Project its complete clearance rectangle using the same
        # measured joints and calibrated hand-eye transform as the scene packet.
        calibration=frame['identity']['calibration']
        content=Path(calibration).read_bytes()
        if hashlib.sha256(content).hexdigest()!=frame['identity']['calibration_sha256']:
            raise RuntimeError('scene hand-eye calibration file changed')
        flange_camera=np.array(yaml.load(content,Loader=getattr(yaml,'CSafeLoader',yaml.SafeLoader))['flange_T_camera'])
        tcp_pose=next(sampled_tcp_poses(self.transfer_guard,[{'positions':frame['source_joint_position_rad']}]))
        base_camera=tcp_pose@np.linalg.inv(np.array(self.transfer_guard['tcp']).reshape(4,4))@flange_camera
        camera_base=np.linalg.inv(base_camera)
        K=np.array(frame['source_K']).reshape(3,3)
        width,height=frame['image_size'];rx,ry=self.demo['row_slot_clearance_xy_m']
        projected=[]
        for dx,dy in ((-rx,-ry),(-rx,0),(-rx,ry),(0,-ry),(0,ry),(rx,-ry),(rx,0),(rx,ry)):
            point=np.array([slot[0]+dx,slot[1]+dy,
                            self.demo['table_z_m']+self.demo['piece_thickness_m'],1.])
            camera=camera_base@point
            if camera[2]<=0:raise RuntimeError('row slot is behind camera')
            uv=K@camera[:3];uv=uv[:2]/uv[2]
            if not (10<=uv[0]<width-10 and 10<=uv[1]<height-10):
                raise RuntimeError('entire row-slot clearance is not inside camera view')
            projected.append(uv)
        projected=np.array(projected)
        return [*projected.min(axis=0).tolist(),*projected.max(axis=0).tolist()]

    def check_row_slot_empty(self, slot):
        from demo import qualified_scene_window,row_slot_occupied
        self.gripper_state(held=True,idle=True)
        frames=self.observe()
        evidence=qualified_scene_window(frames)
        if evidence['scene_identity'][2:]!=self.target_measurement['scene_identity'][2:]:
            raise RuntimeError('camera/scene/reference identity changed during held transfer')
        ignored_outside_roi=0
        for frame in frames:
            roi=self.row_slot_visible(frame,slot)
            for d in frame['detections']:
                if not (d.get('base_geometry_valid') and d.get('complete_center_target')):
                    if detection_overlaps_roi(d,roi):
                        raise RuntimeError('unlocated/incomplete detection intersects row-slot image clearance')
                    ignored_outside_roi+=1
                    continue
                if row_slot_occupied(self.demo,d['base_surface_center_m'],slot):
                    raise RuntimeError('row destination clearance is occupied')
        self.check_descent_age(evidence['last_stamp_ns'])
        self.report.setdefault('row_empty_slot_checks',[]).append(dict(slot_xy_m=slot,
            ignored_outside_roi_observations=ignored_outside_roi,**evidence))
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

    def verify_row_placement(self, slot, source):
        from demo import qualified_target,qualified_scene_window,distance
        self.move('PREGRASP',[*slot,self.demo['hover_tcp_base_z_m']],'released_unverified')
        frames=self.observe();qualified_scene_window(frames)
        matches=[d for d in frames[0]['detections'] if d.get('base_geometry_valid') and
                 d.get('complete_center_target') and distance(d['base_surface_center_m'][:2],slot)<=.015]
        if len(matches)!=1:raise RuntimeError('released row piece not independently visible/unique')
        measured=qualified_target(self.demo,matches[0],frames,observation_only=True)
        p=measured['base_surface_center_m']
        if measured['scene_identity'][2:]!=source['scene_identity'][2:]:
            raise RuntimeError('camera/scene/reference identity changed before row verification')
        if measured['first_stamp_ns']<=self.last_placement_stamp:
            raise RuntimeError('row stability observation predates release')
        if distance(p[:2],slot)>self.demo.get('row_placement_tolerance_m',self.demo['descent_tolerance_m']):
            raise RuntimeError('row piece misplaced beyond configured descent tolerance')
        if abs(p[2]-source['base_surface_center_m'][2])>.003:
            raise RuntimeError('row piece relative top height differs from its bare-table source')
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
        io.report['status']='PARTIAL' if isinstance(demo,RowDemo) else 'COMPLETE'
    except BaseException as e:
        io.report.update(status='FAULT',error=str(e))
        raise
    finally:
        if isinstance(demo,RowDemo):
            io.report.update(business_mode='row',placements=demo.placements,unresolved=demo.unresolved,
                             scan_unresolved=demo.scan_unresolved,
                             search_inventory_processed=bool(demo.layers) and io.report.get('status')!='FAULT' and not demo.unresolved and not demo.scan_unresolved,
                             all_physical_pieces_confirmed=False)
        io.report.update(ended_ns=io.now(),layers=demo.layers,retired=demo.retired,
                         object_state=demo.object_state,gripper_latest=convert(io.gripper) if io.gripper else None,
                         arm_latest=convert(io.arm[max(io.arm)]) if io.arm else None)
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
