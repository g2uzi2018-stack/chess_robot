#!/usr/bin/env python3
"""One explicitly approved empty-gripper table-contact-profile observation."""
import argparse,hashlib,json,math,time
from pathlib import Path
import numpy as np
import rclpy
from chess_robot_interfaces.action import MoveArm
from ros_io import PlanInspector
from scan_controller import stamp,position,convert,write

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',required=True);p.add_argument('--output',required=True)
    p.add_argument('--authorization',required=True);p.add_argument('--hands-clear',action='store_true',required=True)
    p.add_argument('--retreat-from',help='completed observation result to retreat from, instead of descending')
    a=p.parse_args();cfg=json.loads(Path(a.config).read_text());out=Path(a.output);out.mkdir(parents=False,exist_ok=False)
    rclpy.init();io=PlanInspector(cfg,out);io.execute=True
    io.report.update(schema='table_contact_empty_observation/v1',execute=True,authorization=a.authorization,
                     hands_clear=True,gripper_commands_sent=0,grasp_executed=False)
    try:
        io.stopped();io.check_parameters();s,j=io.pair();start=position(s).tolist()
        if io.c['profile']!='table_contact':raise RuntimeError('explicit target-local profile required')
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            io.spin();g=io.gripper
            if (g and not g.simulated and g.connected and g.feedback_valid and 0<=g.feedback_age_sec<=.5 and
                    0<=(io.now()-stamp(g))*1e-9<=.2 and not g.busy and g.grip_status==1 and
                    not g.object_detected and not g.object_dropped and
                    (0<=g.position_raw<=1000 if a.retreat_from else 995<=g.position_raw<=1000)):break
        else:raise RuntimeError('fresh real empty gripper feedback unavailable (descent additionally requires fully open)')
        op='DESCEND';target=start[:]
        if a.retreat_from:
            prior=json.loads(Path(a.retreat_from).read_text())
            if prior['status']!='HOLD_FOR_OPERATOR_OBSERVATION':raise RuntimeError('no completed observation to retreat from')
            target=prior['start_tcp_m'];op='RETREAT'
            if np.linalg.norm(position(s)-prior['nominal_target_tcp_m'])>.001:
                raise RuntimeError('observation endpoint changed; do not automatically rebase')
        else:
            if not (.015<=start[2]<=.025 and abs(start[0]-.00917)<.003 and abs(start[1]+.4658)<.003):
                raise RuntimeError('not at the specifically observed black-pawn alignment pose')
            target[2]=-.003910
        if np.linalg.norm(np.array(target[:2])-start[:2])>.001 or abs(target[2]-start[2])>.030:
            raise RuntimeError('not the approved <=30mm vertical observation step')
        io.report.update(start_tcp_m=start,start_joint=convert(j),nominal_target_tcp_m=target,operation=op)
        preview=io.probe('approved-empty-nominal-point',op,target)
        if preview['status']!=4 or preview['result']['code']!=0 or preview['result']['executed']:
            raise RuntimeError(preview['result']['detail'])
        io.stopped();io.check_parameters();now,joint=io.pair()
        if np.linalg.norm(position(now)-start)>.001 or max(abs(x-y) for x,y in zip(joint.position,j.position))>.0005:
            raise RuntimeError('start changed while SDK planned')
        goal=MoveArm.Goal();goal.operation=getattr(goal,op);goal.profile='table_contact';goal.plan_only=False
        goal.target.header.frame_id='base_link';goal.target.header.stamp=io.n.get_clock().now().to_msg()
        goal.target.pose.position.x,goal.target.pose.position.y,goal.target.pose.position.z=target
        q=cfg['orientation_xyzw'];goal.target.pose.orientation.x,goal.target.pose.orientation.y,goal.target.pose.orientation.z,goal.target.pose.orientation.w=q
        io.pending=io.client.send_goal_async(goal);io.goal=io.wait(io.pending,8);io.pending=None
        if not io.goal.accepted:raise RuntimeError('physical observation goal rejected')
        io.report['physical_stop_confirmed']=False;io.terminal=io.goal.get_result_async()
        reply=io.wait(io.terminal,60,monitor=True);io.report['execution']=dict(status=reply.status,result=convert(reply.result))
        if not (reply.status==4 and reply.result.code==1 and reply.result.executed and reply.result.final_tcp_valid):
            raise RuntimeError(reply.result.detail)
        io.goal=io.terminal=None;end,joint,t=io.stopped()
        if np.linalg.norm(position(end)-target)>.001:raise RuntimeError('measured endpoint mismatch')
        o=end.tcp.pose.orientation;actual=np.array([o.x,o.y,o.z,o.w])
        if 2*math.acos(float(np.clip(abs(actual@np.array(q)),0,1)))>.005:raise RuntimeError('measured orientation mismatch')
        io.report.update(status='HOLD_FOR_OPERATOR_OBSERVATION' if op=='DESCEND' else 'RETREATED_TO_ORDINARY_HEIGHT',
                         arm_latest=convert(end),joint_latest=convert(joint))
        print(json.dumps(dict(status=io.report['status'],nominal_tcp_m=position(end).tolist(),
                              physical_stop_confirmed=True,gripper_commands_sent=0),ensure_ascii=False))
    except BaseException as e:
        io.report.update(status='FAULT',error=str(e));io.cancel_owned();raise
    finally:
        io.report['gripper_latest']=convert(io.gripper) if io.gripper else None
        write(out/'observation-result.json',io.report);io.close();rclpy.shutdown()

if __name__=='__main__':main()
