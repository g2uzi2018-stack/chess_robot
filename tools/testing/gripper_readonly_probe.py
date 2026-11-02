#!/usr/bin/env python3
"""Run the C++ backend with writes disabled and collect only measured state."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
os.environ.setdefault('ROS_DOMAIN_ID', '88')
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from chess_robot_interfaces.msg import GripperState

rclpy.init()
node = rclpy.create_node('gripper_readonly_probe')
states = []
sub = node.create_subscription(GripperState, '/gripper/state', states.append, 10)
Path('runs/gripper-readonly').mkdir(exist_ok=True)
log = open('runs/gripper-readonly/node.log', 'w')
process = subprocess.Popen([
    'install/chess_robot_gripper/lib/chess_robot_gripper/gripper_node', '--ros-args',
    '-p', 'backend:=tcp_rtu', '-p', 'host:=192.168.30.234', '-p', 'port:=60000',
    '-p', 'enable_commands:=false'], stdout=log, stderr=subprocess.STDOUT)
try:
    deadline = time.monotonic() + 12
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
        valid = [s for s in states if s.feedback_valid]
        if len(valid) >= 10:
            break
    assert len(valid) >= 10, 'fewer than 10 valid hardware samples'
    assert all(not s.simulated and not s.commands_enabled and not s.busy for s in states)
    result = {
        'passed': True, 'host': '192.168.30.234', 'port': 60000,
        'backend': 'tcp_rtu', 'enable_commands': False, 'action_goals_sent': 0,
        'samples': [{'initialization':s.initialization, 'grip_status':s.grip_status,
                     'position_raw':s.position_raw, 'feedback_age_sec':s.feedback_age_sec,
                     'simulated':s.simulated, 'connected':s.connected,
                     'feedback_valid':s.feedback_valid} for s in valid],
    }
    Path('runs/gripper-readonly/result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
finally:
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill(); process.wait()
    log.close()
    node.destroy_node()
    rclpy.shutdown()
