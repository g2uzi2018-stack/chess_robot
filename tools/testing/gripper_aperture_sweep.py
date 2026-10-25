#!/usr/bin/env python3
"""Explicit, feedback-confirmed AG95 aperture sweep, 1000 -> 0 at 20% force.

Default is mock. --execute uses the verified Nova5 tool RTU port and checks the
currently running arm's full-aperture envelope; never sends arm commands.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

p = argparse.ArgumentParser()
p.add_argument('--execute', action='store_true')
p.add_argument('--out', type=Path, required=True)
args = p.parse_args()
os.environ.setdefault('ROS_DOMAIN_ID', '96' if args.execute else '89')
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from rclpy.action import ActionClient
from chess_robot_interfaces.action import MoveGripper
from chess_robot_interfaces.msg import GripperState, ArmState

root = Path(__file__).resolve().parents[2]
out = args.out.resolve()
out.mkdir(parents=True, exist_ok=False)
report = {'status':'STARTING', 'execute':args.execute, 'force_percent':20,
          'targets_raw':list(range(1000,-1,-100)), 'goals':[], 'arm_commands_sent':0,
          'started_unix_ns':time.time_ns(), 'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
def save():
    temp = out/'results.tmp'
    temp.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    temp.replace(out/'results.json')

def serialize(s):
    return {name:getattr(s,name) for name in ['simulated','connected','feedback_valid',
            'feedback_age_sec','busy','commands_enabled','initialization','grip_status',
            'position_raw','object_detected','object_dropped','detail']}

save()
if subprocess.run(['pgrep','-x','gripper_node'], stdout=subprocess.DEVNULL).returncode == 0:
    raise RuntimeError('another gripper_node exists; refusing competing control')
if args.execute:
    assert os.environ['ROS_DOMAIN_ID'] == '96'
    import yaml
    envelope = Path('/home/kuang/workspace/chess_robot/src/chess_robot_arm/config/nova5_ag95_geometry.yaml')
    geometry = yaml.safe_load(envelope.read_text())
    assert any(h['name']=='camera_and_gripper_21_state_union' for h in geometry['hulls'])
    baseline = Path('/home/kuang/workspace/chess_robot/runs/workspace_self_20261024/model.yaml')
    assert envelope.read_bytes() == baseline.read_bytes(), 'active envelope differs from audited aperture union'
    report['arm_envelope_sha256'] = hashlib.sha256(envelope.read_bytes()).hexdigest()

rclpy.init()
node = rclpy.create_node('gripper_aperture_sweep')
latest = None
arm = None
arm_received = 0
state_log = (out/'gripper-states.jsonl').open('w')
def receive_gripper(message):
    global latest
    latest = message
    state_log.write(json.dumps({'host_unix_ns':time.time_ns(), **serialize(message)},ensure_ascii=False)+'\n')
    state_log.flush()
def receive_arm(message):
    global arm, arm_received
    arm, arm_received = message, time.monotonic()
node.create_subscription(GripperState, '/gripper/state', receive_gripper, 20)
node.create_subscription(ArmState, '/arm/state', receive_arm, 20)
client = ActionClient(node, MoveGripper, '/gripper/command')
command = [str(root/'install/chess_robot_gripper/lib/chess_robot_gripper/gripper_node'), '--ros-args',
           '-p','backend:='+('tcp_rtu' if args.execute else 'mock'),
           '-p','enable_commands:=true','-p','max_force_percent:=20']
if args.execute:
    command += ['-p','host:=192.168.30.234','-p','port:=60000']
report['node_command'] = command
log = (out/'node.log').open('w')
process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)

def arm_check():
    if args.execute:
        assert arm and time.monotonic()-arm_received < 1., 'arm status missing/stale'
        assert not arm.simulated and arm.feedback_fresh, 'arm feedback invalid/simulated'
        assert arm.controller_mode in (5,7), 'arm is not enabled/running normally'

def wait(predicate, timeout=12, check_arm=False):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.02)
        if check_arm:
            arm_check()
        if predicate():
            return
        assert process.poll() is None, 'gripper node exited'
    raise TimeoutError('ROS deadline')

def future(f, timeout=12, check_arm=False):
    wait(f.done, timeout, check_arm)
    return f.result()

def send(operation, position=1000):
    arm_check()
    assert latest.feedback_valid and latest.simulated != args.execute, 'gripper feedback invalid'
    before = serialize(latest)
    start = time.monotonic()
    goal = MoveGripper.Goal(operation=operation, position_raw=position, force_percent=20,
                            require_contact=False, timeout_sec=10.)
    row = {'operation':operation, 'target_raw':position, 'before':before, 'status':'SENDING'}
    report['goals'].append(row); save()
    handle = future(client.send_goal_async(goal), check_arm=True)
    assert handle.accepted, 'gripper goal rejected'
    result = future(handle.get_result_async(), 12, check_arm=True).result
    row.update(code=result.code, detail=result.detail, command_attempted=result.command_attempted,
               after=serialize(result.final_state), elapsed_sec=time.monotonic()-start,
               status='CONFIRMED' if result.code in (0,1) else 'FAILED')
    print(json.dumps(row,ensure_ascii=False), flush=True); save()
    assert result.simulated != args.execute
    assert result.code == (0 if operation==0 else 1), 'feedback did not confirm requested state'
    if operation==1:
        assert abs(result.final_state.position_raw-position) <= 5
    pause_end=time.monotonic()+1.
    while time.monotonic()<pause_end:
        rclpy.spin_once(node,timeout_sec=.02)
        arm_check()
        assert latest.feedback_valid, 'gripper feedback lost during dwell'

try:
    assert client.wait_for_server(timeout_sec=8), 'gripper action unavailable'
    wait(lambda: latest is not None and latest.feedback_valid and (not args.execute or arm is not None))
    arm_check()
    report['initial_gripper_state']=serialize(latest)
    if args.execute:
        report['initial_arm_state']={'backend':arm.backend,'mode':arm.controller_mode,'busy':arm.busy,
            'reference_valid':arm.reference_valid,'signed_twist_rad':arm.signed_twist_rad,
            'total_twist_rad':arm.total_twist_rad}
    report['status']='RUNNING'; save()
    if latest.initialization != 1:
        send(0)
    for target in report['targets_raw']:
        send(1,target)
    report['status']='COMPLETED'
except Exception as e:
    report['status']='HALTED_NO_MORE_COMMANDS'
    report['reason']=str(e)
    print('HALTED: '+str(e),flush=True)
finally:
    report['finished_unix_ns']=time.time_ns()
    save()
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill(); process.wait()
    state_log.close(); log.close(); client.destroy(); node.destroy_node(); rclpy.shutdown()
if report['status'] != 'COMPLETED':
    raise SystemExit(2)
