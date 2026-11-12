#!/usr/bin/env python3
"""Reproduce initialization-reference transient with a loopback RTU gateway only."""
import json, os, struct
os.environ['ROS_DOMAIN_ID']='89'
os.environ['ROS_LOCALHOST_ONLY']='1'
import rclpy
from gripper_ros_integration import Gateway, Session

class HomingGateway(Gateway):
    def __init__(self):
        self.remaining=0
        self.unreferenced_position_reads=0
        super().__init__(initialized=0)

    def reply(self,q):
        _,fun,address,value=struct.unpack('>BBHH',q[:-2])
        if fun==6 and address==0x100:
            with self.lock:
                self.writes.append([address,value])
                self.initialized=2; self.status=0; self.position=1925; self.remaining=6
            return q
        if fun==3 and address==0x200 and self.remaining:
            with self.lock:
                self.remaining-=1
                if self.remaining==0:
                    self.initialized=1; self.status=1; self.position=1000
        if fun==3 and address==0x202 and self.initialized==2:
            self.unreferenced_position_reads+=1
        return super().reply(q)

if __name__=='__main__':
    rclpy.init()
    gateway=HomingGateway()
    session=Session(gateway)
    try:
        session.fresh()
        result=session.result(session.send(operation=0),0)
        assert result.command_attempted and result.final_state.commands_enabled
        assert gateway.unreferenced_position_reads==0
        assert gateway.writes==[[0x100,1]],gateway.writes
        assert any(s.connected and not s.feedback_valid and s.position_raw==65535 for s in session.states)
        assert not session.result(session.send(operation=0),0).command_attempted
        assert gateway.writes==[[0x100,1]]
        session.result(session.send(position=900),1)
        gateway.position=1074
        session.until(lambda:session.states and not session.states[-1].feedback_valid)
        assert not session.send(position=800).accepted
        print(json.dumps({'passed':True,'checks':['initializing with physical position=1925 does not latch fault',
            'unreferenced position register skipped and ROS position marked unknown',
            'two settled valid polls complete initialization; commands remain enabled',
            'initialization write occurs once; already-initialized request never re-homes',
            'later ordinary MOVE succeeds', 'out-of-range feedback after initialization still rejected']}))
    finally:
        session.close();gateway.stop();rclpy.shutdown()
