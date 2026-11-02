#!/usr/bin/env python3
"""Loopback-only HTTP -> real C++ panel -> ROS Action fixture tests. No hardware."""
import concurrent.futures
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time
import urllib.error
import urllib.request
import uuid

os.environ['ROS_DOMAIN_ID'] = '89'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from chess_robot_interfaces.action import MoveGripper
from chess_robot_interfaces.msg import GripperState


class Fixture:
    def __init__(self):
        self.node = rclpy.create_node('panel_test_fixture')
        self.state = GripperState()
        self.state.simulated = self.state.connected = self.state.feedback_valid = True
        self.state.commands_enabled = True
        self.state.initialization = 0
        self.state.position_raw = 1000
        self.state.grip_status = 1
        self.publish_enabled = True
        self.goals = []
        self.reject = False
        self.delay = .25
        self.code = 1
        self.stamp_offset = 0
        self.pub = self.node.create_publisher(GripperState, '/gripper/state', 10)
        self.timer = self.node.create_timer(.05, self.publish)
        self.server = ActionServer(self.node, MoveGripper, '/gripper/command', self.execute,
            goal_callback=self.accept, callback_group=ReentrantCallbackGroup())
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin)
        self.thread.start()

    def publish(self):
        if self.publish_enabled:
            self.state.header.stamp = self.node.get_clock().now().to_msg()
            self.state.header.stamp.sec -= self.stamp_offset
            self.pub.publish(self.state)

    def accept(self, _):
        return GoalResponse.REJECT if self.reject else GoalResponse.ACCEPT

    def execute(self, handle):
        goal = handle.request
        self.goals.append(goal)
        self.state.busy = True
        time.sleep(self.delay)
        self.state.busy = False
        if goal.operation == 0:
            self.state.initialization = 1
        else:
            self.state.position_raw = goal.position_raw
        result = MoveGripper.Result()
        result.code = 0 if goal.operation == 0 else self.code
        result.command_attempted = True
        result.simulated = True
        result.final_state = self.state
        result.detail = 'test fixture result'
        if result.code <= 2:
            handle.succeed()
        else:
            handle.abort()
        return result

    def close(self):
        self.executor.shutdown()
        self.thread.join()
        self.node.destroy_node()


def wait_for(predicate, timeout=6):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            value = predicate()
            if value:
                return value
        except (OSError, KeyError):
            pass
        time.sleep(.05)
    raise AssertionError('condition timed out')


def run():
    checks = []
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    base = f'http://127.0.0.1:{port}'
    fixture = Fixture()
    log_path = Path('/tmp/chess-panel-integration.log')
    log = log_path.open('w')
    panel = subprocess.Popen(['ros2', 'run', 'chess_robot_panel', 'panel_node', '--ros-args',
        '-p', f'port:={port}', '-p', 'enable_control:=true',
        '-p', 'feedback_max_age_sec:=0.3', '-p', 'timeout_sec:=0.5'], stdout=log, stderr=log, start_new_session=True)

    def request(path='/api/state', body=None, extra=None):
        headers = {'Content-Type': 'application/json', 'X-Panel-Request': '2'}
        headers.update(extra or {})
        req = urllib.request.Request(base + path,
            data=None if body is None else json.dumps(body).encode(), headers=headers)
        try:
            with urllib.request.urlopen(req, timeout=2) as response:
                raw = response.read().decode()
                return response.status, json.loads(raw) if path.startswith('/api/') else raw
        except urllib.error.HTTPError as err:
            return err.code, json.loads(err.read())

    def state():
        return request()[1]

    def payload(**kw):
        return dict(request_id=uuid.uuid4().hex, server_id=state()['server_id'], **kw)

    def idle():
        return wait_for(lambda: state()['can_move'])

    try:
        wait_for(lambda: state()['can_initialize'])
        assert not state()['can_move'] and not fixture.goals
        request('/'); request('/api/state')
        assert not fixture.goals
        checks.append('startup/GET never moves; uninitialized MOVE disabled')
        p = payload()
        assert request('/api/gripper/initialize', p)[0] == 202
        idle()
        assert len(fixture.goals) == 1 and fixture.goals[0].operation == 0
        checks.append('explicit initialization through Action')

        p = payload(position_raw=640, force_percent=20)
        assert request('/api/gripper/move', p)[0] == 202
        assert request('/api/gripper/move', p)[0] == 200
        assert request('/api/gripper/move', dict(p, position_raw=630))[0] == 409
        idle()
        assert len(fixture.goals) == 2
        g = fixture.goals[-1]
        assert g.position_raw == 640 and g.force_percent == 20 and not g.require_contact
        assert state()['command']['phase'] == 'succeeded'
        checks.append('manual target/force, require_contact=false, duplicate not replayed, ID conflicts rejected')

        for pos, force in [(-1, 20), (1001, 20), (10, 19), (10, 31), (2.3, 20)]:
            assert request('/api/gripper/move', payload(position_raw=pos, force_percent=force))[0] == 400
        p = payload(position_raw=500, force_percent=20)
        assert request('/api/gripper/move', dict(p, server_id='old'))[0] == 409
        assert request('/api/gripper/move', p, {'Origin':'https://other.example'})[0] == 403
        assert request('/api/gripper/move', p, {'Sec-Fetch-Site':'cross-site'})[0] == 403
        assert request('/api/gripper/move', p, {'Host':'other.example'})[0] == 403
        assert request('/api/gripper/move', p, {'X-Panel-Request':'1'})[0] == 403
        checks.append('range/decimal validation, restart token and cross-origin rejection')

        fixture.delay = .5
        count = len(fixture.goals)
        candidates = [payload(position_raw=x, force_percent=20) for x in (500, 520)]
        with concurrent.futures.ThreadPoolExecutor(2) as pool:
            results = list(pool.map(lambda x: request('/api/gripper/move', x)[0], candidates))
        assert sorted(results) == [202, 409], results
        idle()
        assert len(fixture.goals) == count + 1
        checks.append('concurrent clients: only one Action, no command queue')

        fixture.reject = True
        assert request('/api/gripper/move', payload(position_raw=1000, force_percent=20))[0] == 202
        wait_for(lambda: state()['command']['phase'] == 'rejected')
        fixture.reject = False
        idle()
        checks.append('driver goal rejection shown separately from HTTP accepted')

        fixture.code = 12
        assert request('/api/gripper/move', payload(position_raw=0, force_percent=20))[0] == 202
        idle()
        s = state()
        assert s['command']['phase'] == 'failed' and s['command']['code'] == 12
        assert '目视' in s['command']['detail']
        fixture.code = 1
        checks.append('contact result preserved, human observation message, no fake grasp success')

        fixture.stamp_offset = 3
        wait_for(lambda: not state()['state']['fresh'])
        assert not state()['can_move']
        fixture.stamp_offset = 0
        idle()
        fixture.publish_enabled = False
        wait_for(lambda: not state()['state']['fresh'])
        s = state()
        assert not s['can_move'] and s['state']['position_raw'] == 65535
        assert request('/api/gripper/move', payload(position_raw=1000, force_percent=20))[0] == 409
        fixture.publish_enabled = True
        idle()
        fixture.state.simulated = False
        wait_for(lambda: not state()['can_move'])
        assert request('/api/gripper/move', payload(position_raw=1000, force_percent=20))[0] == 409
        fixture.state.simulated = True
        idle()
        checks.append('elapsed receive age invalidates stale state; real device guarded in mock mode')

        fixture.delay = 4.2
        count = len(fixture.goals)
        assert request('/api/gripper/move', payload(position_raw=300, force_percent=20))[0] == 202
        wait_for(lambda: state()['command']['phase'] == 'unknown')
        assert not state()['can_move']
        assert request('/api/gripper/move', payload(position_raw=1000, force_percent=20))[0] == 409
        wait_for(lambda: state()['command']['phase'] == 'succeeded')
        idle()
        assert len(fixture.goals) == count + 1
        checks.append('missing result deadline locks; late definitive result resolves; no retries')
        return checks
    finally:
        os.killpg(panel.pid, signal.SIGINT)
        try:
            panel.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(panel.pid, signal.SIGKILL); panel.wait()
        fixture.close()
        log.close()


if __name__ == '__main__':
    rclpy.init()
    try:
        checks = run()
        print(json.dumps({'passed':True, 'checks':checks}, ensure_ascii=False, indent=2))
    finally:
        rclpy.shutdown()
