#!/usr/bin/env python3
"""Loopback-only AG95 gateway fault tests against the real C++ ROS node."""
import json
import math
import os
from pathlib import Path
import signal
import socketserver
import struct
import subprocess
import threading
import time

os.environ.setdefault('ROS_DOMAIN_ID', '87')
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from rclpy.action import ActionClient
from chess_robot_interfaces.action import MoveGripper
from chess_robot_interfaces.msg import GripperState


def crc(data):
    c = 0xffff
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xa001 if c & 1 else c >> 1
    return struct.pack('<H', c)


class Gateway(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, mode='normal', initialized=0):
        super().__init__(('127.0.0.1', 0), Handler)
        self.mode = mode
        self.initialized = initialized
        self.status = 2 if mode == 'old_contact' else 1
        self.position = 450 if mode == 'old_contact' else 1000
        self.target = self.position
        self.steps = 0
        self.writes = []
        self.reads = 0
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.serve_forever, daemon=True)
        self.thread.start()

    def reply(self, q):
        with self.lock:
            assert len(q) == 8 and crc(q[:-2]) == q[-2:]
            _, fun, address, value = struct.unpack('>BBHH', q[:-2])
            if fun == 6:
                self.writes.append([address, value])
                if address == 0x100:
                    self.initialized = 1
                if address == 0x103:
                    self.target = value
                    self.steps = 3
                    if self.mode != 'old_contact':
                        self.status = 0
                    if self.mode == 'bad_echo':
                        return q[:-2] + bytes([q[-2] ^ 1, q[-1]])
                return q
            assert fun == 3 and value == 1
            self.reads += 1
            if self.steps and address == 0x200 and self.mode not in ('stall', 'old_contact'):
                self.steps -= 1
                if not self.steps:
                    if self.mode in ('contact', 'drop') and self.target < 450:
                        self.position = 450
                        self.status = 2 if self.mode == 'contact' else 3
                    else:
                        self.position = self.target
                        self.status = 1
            answer = {0x200: self.initialized, 0x201: self.status, 0x202: self.position}[address]
            b = struct.pack('>BBBH', q[0], 3, 2, answer)
            return b + crc(b)

    def stop(self):
        self.shutdown()
        self.server_close()
        self.thread.join()


class Handler(socketserver.BaseRequestHandler):
    def handle(self):
        self.request.settimeout(2)
        buf = b''
        try:
            while True:
                data = self.request.recv(8 - len(buf))
                if not data:
                    return
                buf += data
                if len(buf) < 8:
                    continue
                q, buf = buf, b''
                reply = self.server.reply(q)
                if self.server.mode == 'slow_read' and q[1] == 3:
                    time.sleep(.4)
                # Deliberate TCP fragmentation; protocol framing must reassemble.
                for byte in reply:
                    self.request.sendall(bytes([byte]))
                    time.sleep(.001)
        except (OSError, ConnectionError):
            return


class Session:
    counter = 0

    def __init__(self, gateway=None, enabled=True, contact=-1):
        Session.counter += 1
        self.client_node = rclpy.create_node('gripper_test_' + str(Session.counter))
        self.states = []
        self.sub = self.client_node.create_subscription(GripperState, '/gripper/state', self.states.append, 10)
        self.client = ActionClient(self.client_node, MoveGripper, '/gripper/command')
        args = ['install/chess_robot_gripper/lib/chess_robot_gripper/gripper_node', '--ros-args',
                '-p', 'enable_commands:=' + str(enabled).lower(),
                '-p', 'poll_period_ms:=50']
        if gateway:
            args += ['-p', 'backend:=tcp_rtu', '-p', 'host:=127.0.0.1', '-p', f'port:={gateway.server_address[1]}']
        else:
            args += ['-p', f'mock_contact_raw:={contact}']
        Path('runs/gripper-integration').mkdir(exist_ok=True)
        self.log = open(f'runs/gripper-integration/node-{Session.counter}.log', 'w')
        self.process = subprocess.Popen(args, stdout=self.log, stderr=subprocess.STDOUT)
        assert self.client.wait_for_server(timeout_sec=8), 'action server unavailable'
        self.until(lambda: self.states, 5)

    def until(self, predicate, timeout=8):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.client_node, timeout_sec=.02)
            if predicate():
                return
            assert self.process.poll() is None, 'node exited unexpectedly'
        raise AssertionError('condition deadline exceeded')

    def wait(self, future, timeout=8):
        self.until(future.done, timeout)
        return future.result()

    def send(self, operation=1, position=1000, contact=False, timeout=5., force=20):
        return self.wait(self.client.send_goal_async(MoveGripper.Goal(
            operation=operation, position_raw=position, force_percent=force,
            require_contact=contact, timeout_sec=timeout)))

    def result(self, handle, code):
        assert handle.accepted
        result = self.wait(handle.get_result_async()).result
        assert result.code == code, (result.code, code, result.detail)
        return result

    def fresh(self):
        self.until(lambda: self.states and self.states[-1].feedback_valid)

    def close(self):
        self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
            raise AssertionError('unbounded node shutdown')
        self.client.destroy()
        self.client_node.destroy_node()
        self.log.close()


checks = []
def record(name):
    checks.append(name)
    print('PASS', name, flush=True)


def run():
    s = Session()
    try:
        s.fresh()
        assert s.states[-1].simulated and not s.states[-1].object_detected
        assert not s.send().accepted
        record('uninitialized MOVE rejected')
        assert s.result(s.send(operation=0), 0).command_attempted
        assert not s.result(s.send(operation=0), 0).command_attempted
        record('initialize feedback and already-initialized no-op')
        for args in ({'position':1001}, {'force':31}, {'timeout':math.nan}, {'operation':2}):
            assert not s.send(**args).accepted
        record('invalid position, force, NaN deadline and operation rejected')
        move = s.send(position=0)
        assert move.accepted
        assert not s.send(position=500).accepted
        cancel = s.wait(move.cancel_goal_async())
        assert not cancel.goals_canceling
        result = s.result(move, 1)
        assert result.simulated and result.final_state.position_raw <= 5
        record('mock move, busy exclusion, explicit cancellation rejection')
    finally:
        s.close()

    g = Gateway(initialized=0)
    s = Session(g, enabled=False)
    try:
        s.fresh()
        assert not s.states[-1].simulated and not s.send(operation=0).accepted
        assert not g.writes and g.reads >= 3
        record('hardware read-only mode emits zero register writes')
    finally:
        s.close(); g.stop()

    g = Gateway(initialized=0)
    s = Session(g)
    try:
        s.fresh()
        s.result(s.send(operation=0), 0)
        s.result(s.send(position=900), 1)
        result = s.result(s.send(position=0, contact=True), 11)
        assert result.command_attempted and result.final_state.commands_enabled
        s.result(s.send(position=1000), 1)
        assert g.writes == [[0x100,1],[0x101,20],[0x103,900],[0x101,20],[0x103,0],[0x101,20],[0x103,1000]]
        record('fragmented RTU over TCP, confirmed move, empty grasp and explicit recovery')
    finally:
        s.close(); g.stop()

    for mode, code in [('contact',2), ('drop',13), ('bad_echo',15), ('stall',14), ('old_contact',14)]:
        g = Gateway(mode, initialized=1)
        s = Session(g)
        try:
            s.fresh()
            result = s.result(s.send(position=0, contact=True, timeout=1.), code)
            assert result.command_attempted and not result.simulated
            assert g.writes == [[0x101,20],[0x103,0]], g.writes
            if code in (14,15):
                s.until(lambda: s.states[-1].feedback_valid and not s.states[-1].commands_enabled)
                assert not s.states[-1].commands_enabled
                assert not s.send().accepted
            else:
                if code == 2:
                    g.mode = 'slow_read'
                    s.until(lambda: not s.states[-1].feedback_valid)
                    assert not s.states[-1].object_detected
                    assert s.states[-1].position_raw == 65535
                    assert not s.send().accepted
                    g.mode = 'contact'
                    s.fresh()
                    record('held flag invalidates on communication loss, read-only reconnect')
                s.result(s.send(position=1000), 1)
            record(mode + ': correct outcome, no automatic command replay')
        finally:
            s.close(); g.stop()

    g = Gateway('slow_read', initialized=1)
    s = Session(g)
    try:
        s.until(lambda: len(s.states) >= 5)
        state = s.states[-1]
        assert not state.feedback_valid and state.position_raw == 65535
        assert state.grip_status == 255 and not state.object_detected
        assert not s.send().accepted and not g.writes
        record('bounded I/O timeout, invalid-feedback sentinels, goal rejection')
    finally:
        s.close(); g.stop()


if __name__ == '__main__':
    rclpy.init()
    outcome = {'passed':False, 'checks':checks}
    try:
        run()
        outcome['passed'] = True
    finally:
        Path('runs/gripper-integration').mkdir(exist_ok=True)
        Path('runs/gripper-integration/result.json').write_text(json.dumps(outcome, indent=2))
        rclpy.shutdown()
