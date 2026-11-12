#!/usr/bin/env python3
"""Execute one approved, prepared non-contact vertical observation step."""
import argparse,json,math,time
from pathlib import Path
import numpy as np
import rclpy
from chess_robot_interfaces.action import MoveArm
from ros_io import PlanInspector
from scan_controller import position,stamp,convert,write


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--proposal',required=True);p.add_argument('--output',required=True)
    p.add_argument('--authorization',required=True);p.add_argument('--hands-clear',action='store_true',required=True)
    a=p.parse_args();proposal=json.loads(Path(a.proposal).read_text())
    target=proposal['nominal_target_tcp_m'];old=proposal['start_arm']['tcp']['pose']['position']
    if not (proposal['purpose']=='observation_only_not_pickpoint' and proposal['plan']['plan_only'] and
            proposal['plan']['status']==4 and proposal['plan']['result']['code']==0 and target[2]>=.020 and
            0<old['z']-target[2]<=.080):
        raise SystemExit('proposal is not a bounded non-contact observation step')
    out=Path(a.output);out.mkdir(parents=False,exist_ok=False)
    config=json.loads(Path(__file__).with_name('demo.current.json').read_text())
    rclpy.init();io=PlanInspector(config,out);io.execute=True
    io.report.update(schema='vertical_observation_step/v1',execute=True,authorization=a.authorization,
                     hands_and_ruler_clear=True,gripper_commands_sent=0,nominal_target_tcp_m=target,
                     purpose='observation_only_not_pickpoint',actual_finger_clearance_m=None)
    try:
        io.stopped();io.check_parameters();s,j=io.pair()
        if (np.linalg.norm(position(s)-[old['x'],old['y'],old['z']])>.001 or
                max(abs(x-y) for x,y in zip(j.position,proposal['start_joint']['position']))>.0005):
            raise RuntimeError('prepared observation start changed; do not replay/rebase automatically')
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            io.spin();g=io.gripper
            if (g and not g.simulated and g.connected and g.feedback_valid and
                    0<=g.feedback_age_sec<=.5 and 0<=(io.now()-stamp(g))*1e-9<=.2 and not g.busy and
                    g.grip_status==1 and not g.object_detected and not g.object_dropped and 995<=g.position_raw<=1000):break
        else:raise RuntimeError('fresh open/empty gripper observation unavailable')
        # Replan at the real start; a preview is not an executable trajectory.
        plan=io.probe('approved-closer-observation','DESCEND',target)
        if plan['status']!=4 or plan['result']['code']!=0 or plan['result']['executed']:
            raise RuntimeError(plan['result']['detail'])
        io.stopped();io.check_parameters();s,j=io.pair()
        if max(abs(x-y) for x,y in zip(j.position,proposal['start_joint']['position']))>.0005:
            raise RuntimeError('start changed during replanning')
        goal=MoveArm.Goal();goal.operation=goal.DESCEND;goal.profile=io.c['profile'];goal.plan_only=False
        goal.target.header.frame_id='base_link';goal.target.header.stamp=io.n.get_clock().now().to_msg()
        goal.target.pose.position.x,goal.target.pose.position.y,goal.target.pose.position.z=target
        q=config['orientation_xyzw']
        goal.target.pose.orientation.x,goal.target.pose.orientation.y,goal.target.pose.orientation.z,goal.target.pose.orientation.w=q
        io.pending=io.client.send_goal_async(goal);io.goal=io.wait(io.pending,8);io.pending=None
        if not io.goal.accepted:raise RuntimeError('physical observation goal rejected')
        io.report['physical_stop_confirmed']=False;io.terminal=io.goal.get_result_async()
        reply=io.wait(io.terminal,60,monitor=True)
        io.report['execution']=dict(status=reply.status,result=convert(reply.result))
        if not (reply.status==4 and reply.result.code==1 and reply.result.executed and reply.result.final_tcp_valid):
            raise RuntimeError(reply.result.detail)
        io.goal=io.terminal=None;s,j,t=io.stopped()
        if np.linalg.norm(position(s)-target)>io.c['position_tolerance_m']:
            raise RuntimeError('measured endpoint mismatch')
        actual=s.tcp.pose.orientation;v=np.array([actual.x,actual.y,actual.z,actual.w])
        if 2*math.acos(float(np.clip(abs(v@np.array(q)),0,1)))>io.c['rotation_tolerance_rad']:
            raise RuntimeError('measured orientation mismatch')
        io.report.update(status='HOLD_FOR_OPERATOR_OBSERVATION',arm_latest=convert(s),joint_latest=convert(j))
        print(json.dumps(dict(status=io.report['status'],nominal_tcp_m=position(s).tolist(),
                              physical_stop_confirmed=True,gripper_commands_sent=0),ensure_ascii=False))
    except BaseException as e:
        io.report.update(status='FAULT',error=str(e));io.cancel_owned();raise
    finally:
        io.report['gripper_latest']=convert(io.gripper) if io.gripper else None
        write(out/'observation-result.json',io.report);io.close();rclpy.shutdown()


if __name__=='__main__':main()
