"""Independent ROS end-to-end acceptance; binds only 127.0.0.1 fixture ports.

Run on AGX after sourcing ROS and workspace; ROS_DOMAIN_ID=93,
ROS_LOCALHOST_ONLY=1. Never opens an outbound robot connection.
"""
import json
import argparse
import os
import pathlib
import signal
import subprocess
import sys
import time
import yaml
import rclpy
from rclpy.action import ActionClient
from chess_robot_interfaces.action import MoveArm
from chess_robot_interfaces.msg import ArmState
from chess_robot_interfaces.srv import InitializeReference, ConfirmGrasp
from arm_controller_fixture import ControllerFixture
from scipy.spatial.transform import Rotation

parser=argparse.ArgumentParser(description='Loopback-only ROS arm acceptance; no hardware connections')
parser.add_argument('--project-root',type=pathlib.Path,default=pathlib.Path(__file__).resolve().parents[2])
parser.add_argument('--out',required=True,type=pathlib.Path)
args=parser.parse_args()
PROJECT=args.project_root.resolve()
OUT=args.out.resolve()
OUT.mkdir(parents=True,exist_ok=False)
assert os.environ.get('ROS_DOMAIN_ID')=='93'
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
rclpy.init()
node=rclpy.create_node('independent_arm_acceptance')
events=[]

def spin_until(predicate,timeout=10):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.01)
        if predicate():return
    raise AssertionError('deadline waiting for '+str(predicate))

def future(f,timeout=15):
    spin_until(f.done,timeout)
    return f.result()

class Session:
    def __init__(self,execution=False):
        self.fixture=ControllerFixture(PROJECT,OUT/('execution' if execution else 'readonly'))
        info=self.fixture.info
        assert info['host']=='127.0.0.1' and info['simulation']
        self.latest=None
        self.sub=node.create_subscription(ArmState,'/simulation/arm/state',self.receive,10)
        self.action=ActionClient(node,MoveArm,'/simulation/arm/move')
        self.init=node.create_client(InitializeReference,'/simulation/arm/initialize_reference')
        self.confirm=node.create_client(ConfirmGrasp,'/simulation/arm/confirm_grasp')
        params=yaml.safe_load((PROJECT/'src/chess_robot_arm/config/arm.yaml').read_text())
        p=params['chess_robot_arm']['ros__parameters']
        p.update(test_fixture=True,backend='hardware' if execution else 'read_only',
                 urdf_file=str(PROJECT/'src/chess_robot_description/urdf/nova5_with_ag95.urdf'),
                 approach_clearance_m=.01,gripper_timeout_s=.5)
        p['controller'].update(ip=info['host'],dashboard_port=info['dashboard_port'],feedback_port=info['feedback_port'])
        p['hardware']={key:execution for key in p['hardware']}
        parameter_file=OUT/('execution.yaml' if execution else 'readonly.yaml')
        parameter_file.write_text(yaml.safe_dump(params))
        self.log=open(OUT/('execution.log' if execution else 'readonly.log'),'w')
        binary=PROJECT/'install/chess_robot_arm/lib/chess_robot_arm/arm_node'
        self.process=subprocess.Popen([str(binary),'--ros-args','--params-file',str(parameter_file)],stdout=self.log,stderr=subprocess.STDOUT)
        assert self.action.wait_for_server(timeout_sec=10)
        assert self.init.wait_for_service(timeout_sec=5)
        spin_until(lambda:self.latest is not None and self.latest.feedback_fresh)
        assert self.latest.simulated and not self.latest.hardware_certified
        assert not self.latest.reference_valid
        spin_until(lambda:time.monotonic()-self.started>.4)
        response=future(self.init.call_async(InitializeReference.Request()))
        assert response.success,response.detail
        spin_until(lambda:self.latest.reference_valid)
    def receive(self,message):
        if self.latest is None:self.started=time.monotonic()
        self.latest=message
    def goal(self,op,plan=True):
        g=MoveArm.Goal();g.operation=op;g.profile='conservative';g.plan_only=plan
        # Nominal TCP transform independently matches the URDF convention.
        import numpy as np
        flange_tcp=np.eye(4);flange_tcp[:3,3]=[0,0,.2]
        flange_tcp[:3,:3]=Rotation.from_euler('xyz',[90,-90,0],degrees=True).as_matrix()
        home=json.loads((PROJECT/'config/arm/home_reference.json').read_text())['joint_positions_rad']
        t=self.fixture.model.transform(home)@flange_tcp
        t[2,3]-=.01
        g.target.header.frame_id='base_link';g.target.header.stamp=node.get_clock().now().to_msg()
        g.target.pose.position.x=float(t[0,3]);g.target.pose.position.y=float(t[1,3]);g.target.pose.position.z=float(t[2,3])
        q=Rotation.from_matrix(t[:3,:3]).as_quat()
        g.target.pose.orientation.x=float(q[0]);g.target.pose.orientation.y=float(q[1]);g.target.pose.orientation.z=float(q[2]);g.target.pose.orientation.w=float(q[3])
        return g
    def send(self,g):return future(self.action.send_goal_async(g))
    def result(self,h,timeout=35):return future(h.get_result_async(),timeout).result
    def ack(self,job,success=True):
        req=ConfirmGrasp.Request();req.job_id=job;req.success=success;req.detail='independent fixture'
        return future(self.confirm.call_async(req))
    def idle(self):spin_until(lambda:not self.latest.busy)
    def home(self):
        h=self.send(self.goal(MoveArm.Goal.HOME,False));assert h.accepted
        r=self.result(h);assert r.code==MoveArm.Result.COMPLETED,r.detail
        self.idle()
    def close(self):
        self.process.send_signal(signal.SIGINT)
        self.process.wait(timeout=8);self.log.close()
        self.fixture.close()
        node.destroy_subscription(self.sub);node.destroy_client(self.init);node.destroy_client(self.confirm);self.action.destroy()

session=None
try:
    session=Session()
    assert not session.fixture.actuation
    h=session.send(session.goal(MoveArm.Goal.HOME));assert h.accepted
    r=session.result(h);assert r.code==MoveArm.Result.PLAN_READY and not r.executed and len(r.trajectory.points)>=2,r.detail
    session.idle();events.append('readonly Home plan returns PLAN_READY, executed=false')
    accepted_cancels=0
    for _ in range(20):
        h=session.send(session.goal(MoveArm.Goal.HOME));assert h.accepted
        cancellation=future(h.cancel_goal_async())
        wrapped=future(h.get_result_async())
        if cancellation.goals_canceling:
            accepted_cancels+=1
            assert wrapped.status==5,(wrapped.status,wrapped.result.detail)
            assert wrapped.result.code==MoveArm.Result.CANCELED
        else:assert wrapped.status==4
        session.idle()
    assert accepted_cancels>0
    events.append(f'20 cancellation/completion boundary trials; {accepted_cancels} accepted cancellations all end CANCELED')
    h=session.send(session.goal(MoveArm.Goal.HOME,False));assert not h.accepted
    assert not session.ack('wrong').accepted
    h=session.send(session.goal(MoveArm.Goal.PREGRASP));assert h.accepted
    r=session.result(h);assert r.code==0 and not r.executed,r.detail
    session.idle();events.append('independent URDF IK/FK plan; disabled execution rejected; late confirmation rejected')
    session.fixture.controls['wrong_echo']='inversekin'
    h=session.send(session.goal(MoveArm.Goal.PREGRASP));assert h.accepted
    r=session.result(h);assert r.code!=0 and not r.executed
    session.fixture.controls['wrong_echo']='';session.idle()
    events.append('wrong SDK command echo aborts planning without actuation')
    session.fixture.controls['hold_duplicates']=True
    spin_until(lambda:not session.latest.feedback_fresh and not session.latest.reference_valid,3)
    session.fixture.controls['hold_duplicates']=False
    spin_until(lambda:session.latest.feedback_fresh)
    assert not session.latest.reference_valid
    events.append('identical duplicate stream expires freshness and reconnect does not restore reference')
    session.close();assert not session.fixture.actuation
    events.append('readonly startup/goals/feedback fault/shutdown emitted zero ServoJ/Stop')
    session=None
    session=Session(execution=True)
    h=session.send(session.goal(MoveArm.Goal.GRASP_SEQUENCE,False));assert h.accepted
    busy=session.send(session.goal(MoveArm.Goal.HOME));assert not busy.accepted
    spin_until(lambda:session.latest.phase=='WAIT_GRIPPER',25)
    assert not session.ack('wrong-job').accepted
    job=session.latest.job_id;assert session.ack(job).accepted
    assert not session.ack(job).accepted
    r=session.result(h);assert r.code==1 and r.executed,r.detail
    session.idle();events.append('ROS sequence: atomic busy rejection, matching gripper gate, duplicate refusal, measured arrival')
    h=session.send(session.goal(MoveArm.Goal.GRASP_SEQUENCE,False));assert h.accepted
    spin_until(lambda:session.latest.phase=='WAIT_GRIPPER',25)
    before=len(session.fixture.actuation);assert session.ack(session.latest.job_id,False).accepted
    r=session.result(h);assert r.code==MoveArm.Result.GRIPPER_FAILED and r.executed,r.detail
    session.idle();assert len(session.fixture.actuation)==before
    events.append('negative gripper confirmation aborts with no retreat commands')
    session.home()
    h=session.send(session.goal(MoveArm.Goal.GRASP_SEQUENCE,False));assert h.accepted
    spin_until(lambda:session.latest.phase=='WAIT_GRIPPER',25)
    before=len(session.fixture.actuation)
    r=session.result(h);assert r.code==MoveArm.Result.GRIPPER_FAILED,r.detail
    session.idle();assert len(session.fixture.actuation)==before
    events.append('gripper timeout aborts with no retreat commands')
    session.home()
    count=len(session.fixture.actuation)
    h=session.send(session.goal(MoveArm.Goal.GRASP_SEQUENCE,False));assert h.accepted
    spin_until(lambda:session.latest.phase=='DESCEND' and len(session.fixture.actuation)>count+3,25)
    cancel=future(h.cancel_goal_async());assert cancel.goals_canceling
    r=session.result(h);assert r.code==MoveArm.Result.CANCELED and r.executed,r.detail
    session.idle();assert session.fixture.stop_count>=1
    events.append('ROS cancellation during ServoJ sends Stop and verifies stationary feedback')
    session.home()
    session.fixture.controls['wrong_echo']='servoj'
    stops=session.fixture.stop_count
    h=session.send(session.goal(MoveArm.Goal.PREGRASP,False));assert h.accepted
    r=session.result(h)
    assert r.code==MoveArm.Result.EXECUTION_FAILED and r.executed,r.detail
    session.idle();assert session.fixture.stop_count>stops
    session.fixture.controls['wrong_echo']=''
    events.append('malformed ServoJ acknowledgement reports execution failure and verifies Stop')
    session.home()
    count=len(session.fixture.actuation)
    h=session.send(session.goal(MoveArm.Goal.GRASP_SEQUENCE,False));assert h.accepted
    spin_until(lambda:session.latest.phase=='DESCEND' and len(session.fixture.actuation)>count+3,25)
    session.fixture.controls['reject']='stop'
    cancel=future(h.cancel_goal_async());assert cancel.goals_canceling
    r=session.result(h);assert r.code==MoveArm.Result.STOP_FAILED and r.executed,r.detail
    spin_until(lambda:session.latest.phase=='FAULT' and not session.latest.busy)
    assert not session.send(session.goal(MoveArm.Goal.HOME)).accepted
    assert not future(session.init.call_async(InitializeReference.Request())).success
    events.append('rejected Stop latches FAULT, refuses next goal and reference initialization')
    session.close();assert not session.fixture.forbidden
    session=None
    report={'status':'PASS','simulation':True,'physical_motion_commands':0,'events':events}
    (OUT/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
except Exception as error:
    (OUT/'result.json').write_text(json.dumps({'status':'FAIL','error':repr(error),'events':events},indent=2)+'\n')
    raise
finally:
    if session:session.close()
    node.destroy_node();rclpy.shutdown()
