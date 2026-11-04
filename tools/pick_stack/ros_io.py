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
from chess_robot_interfaces.action import MoveArm, MoveGripper
from chess_robot_interfaces.msg import GripperState
from rosidl_runtime_py.convert import message_to_ordereddict as convert

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scan_table'))
from scan_controller import Controller, write, stamp, position


class PlanInspector(Controller):
    def __init__(self, config, out):
        scan_config = json.loads(Path(config['scan_config']).read_text())
        super().__init__(scan_config, out, False, node_name='xiangqi_pick_stack_controller')
        self.demo = config
        self.gripper = None
        self.subs.append(self.n.create_subscription(GripperState, '/gripper/state', self.receive_gripper, 10))

    def receive_gripper(self, msg):
        if self.gripper is None or stamp(msg) >= stamp(self.gripper):
            self.gripper = msg

    def close(self):
        self.client.destroy()
        self.events.close()
        self.n.destroy_node()

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
                           authorization=authorization, gripper_actions=[], confirmations=[])
        self.grip_client = ActionClient(self.n, MoveGripper, '/gripper/command')
        self.grip_pending = self.grip_goal = self.grip_terminal = None
        self.held = False
        self.prechecked = False
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
        if row['phase']=='PREGRASP' and 'fresh_measurement' in row:
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
                0 <= s.feedback_age_sec <= .5 and 0 <= (self.now()-stamp(s))*1e-9 <= .2 and
                s.commands_enabled and s.initialization == 1 and not s.object_dropped):
            raise RuntimeError('fresh real initialized gripper feedback unavailable / dropped')
        if held and not (s.object_detected and s.grip_status == 2):
            raise RuntimeError('carried piece no longer held')
        if idle and (s.busy or s.grip_status not in (1,2)):
            raise RuntimeError('gripper still busy')
        return s

    def spin(self, check_abort=True):
        super().spin(check_abort)
        if check_abort and self.held:
            self.gripper_state(held=True)

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

    def precheck(self, config):
        from demo import execution_blockers, calibration_fingerprint
        blockers = execution_blockers(config)
        if blockers:
            raise RuntimeError('; '.join(blockers))
        if not __debug__:
            raise RuntimeError('do not run physical controller with Python assertions disabled')
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
        self.confirm('WORKSPACE', stack_xy_m=config['stack_xy_m'],
                     requirements='exclusive control; empty bare-table support; source and stack envelopes/exit clear')
        self.prechecked=True

    def scan(self):
        if not self.prechecked:
            raise RuntimeError('physical scan requires completed precheck')
        if not self.at_home:
            raise RuntimeError('workflow must reach formal Home before scanning')
        directory = self.out/'scan'
        directory.mkdir()
        # Demo owns HOME_START; retain the scanner's final Home and all guards.
        scanner = Controller(dict(self.c, initial_home=False), directory, True)
        # Give the existing scanner the same interrupt and real gripper monitor.
        original_spin = scanner.spin
        def scan_spin(check_abort=True):
            original_spin(check_abort)
            self.spin(check_abort)
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

    def arm_goal(self, operation, xyz, plan):
        g = MoveArm.Goal()
        g.operation = getattr(g, operation)
        g.profile = self.c['profile']; g.plan_only = plan
        g.target.header.frame_id = 'base_link'
        g.target.header.stamp = self.n.get_clock().now().to_msg()
        if operation != 'HOME':
            p = list(xyz)
            if operation in ('PREGRASP','RETREAT'):
                p[2] -= self.c['expected_approach_clearance_m']
            g.target.pose.position.x,g.target.pose.position.y,g.target.pose.position.z = p
            q = self.demo['orientation_xyzw']
            g.target.pose.orientation.x,g.target.pose.orientation.y,g.target.pose.orientation.z,g.target.pose.orientation.w = q
        return g

    def arm_request(self, operation, xyz, plan, before_send=None):
        if not self.prechecked:
            raise RuntimeError('physical arm adapter requires completed precheck')
        self.stopped(); self.check_parameters()
        self.gripper_state(held=self.held, idle=True)
        if before_send is not None:
            before_send()
        self.pending = self.client.send_goal_async(self.arm_goal(operation, xyz, plan))
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
                q=s.tcp.pose.orientation; target=np.array(self.demo['orientation_xyzw'])
                angle=2*math.acos(float(np.clip(abs(np.array([q.x,q.y,q.z,q.w])@target),0,1)))
                if angle > self.c['rotation_tolerance_rad']:
                    raise RuntimeError('measured endpoint orientation mismatch')
        return reply.result

    def move(self, operation, xyz, object_state):
        self.at_home=False
        self.held = object_state == 'held'
        plan = self.arm_request(operation, xyz, True)
        # The fixed arm guard does not contain payload/stack geometry. A reviewed
        # commissioning artifact and a concrete operator segment review are
        # mandatory for these stages; never describe PLAN_READY as payload proof.
        if self.held or operation == 'DESCEND':
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
                latest=qualified_target(self.demo,self.target_measurement,self.observe())
                if (latest['scene_identity'][1:]!=self.target_measurement['scene_identity'][1:] or
                        distance(latest['base_surface_center_m'],self.target_measurement['base_surface_center_m'])>
                        self.demo['descent_tolerance_m']):
                    raise RuntimeError('target changed while planning/reviewing descent')
                self.check_descent_age(latest['last_stamp_ns'])
        self.arm_request(operation, xyz, False,before_send=refresh)
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
                key=tuple(r.get('rgbd_frame_key') or [r['source_epoch'],r['sequence'],r['stamp_ns']])
                if key in seen:
                    continue
                seen.add(key);r['rgbd_frame_key']=key;frames.append(r)
            if len(frames)>=20 and (frames[-1]['stamp_ns']-frames[0]['stamp_ns'])*1e-9>=2:
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

    def confirm_support(self, layer):
        self.gripper_state(held=True,idle=True)
        self.confirm('SUPPORT',layer,requirements='piece supported at the intended layer, still held; not suspended')

    def release(self):
        self.held=False  # A deliberate release is not interpreted as a drop.
        self.gripper_move(self.demo['measured']['open_raw'],False)
        self.last_placement_stamp=self.now()

    def confirm_placement(self, layer, expected_z):
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
    from demo import Demo
    rclpy.init();io=RealIO(config,out,authorization)
    def interrupt(*_): io.abort=True
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    demo=Demo(config,io.emit)
    try:
        demo.run(io)
        io.report['status']='COMPLETE'
    except BaseException as e:
        io.report.update(status='FAULT',error=str(e))
        raise
    finally:
        io.report.update(ended_ns=io.now(),layers=demo.layers,retired=demo.retired,
                         object_state=demo.object_state,gripper_latest=convert(io.gripper) if io.gripper else None)
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
