"""Production ROS action acceptance. Scan is read-only; --execute-local opts in.

All actual goals use the same C++ Planner/Executor and real V4 driver. No native
MovJ fallback or automatic motion retry. The commissioning backend is bounded.
"""
import argparse,csv,gzip,hashlib,ipaddress,json,math,os,pathlib,signal,subprocess,time
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.action import ActionClient
from chess_robot_interfaces.action import MoveArm
from chess_robot_interfaces.msg import ArmState
from chess_robot_interfaces.srv import InitializeReference
from sensor_msgs.msg import JointState

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',required=True)
    parser.add_argument('--controller-ip',required=True,type=lambda s:str(ipaddress.IPv4Address(s)))
    parser.add_argument('--project-root',type=pathlib.Path,default=pathlib.Path(__file__).resolve().parents[2])
    parser.add_argument('--execute-local',action='store_true')
    parser.add_argument('--initialize-reference-at-home',action='store_true',
                        help='Explicitly establish one reference at the previously verified neutral Home; never reset between tasks')
    parser.add_argument('--expanded-local',action='store_true',
                        help='25/30/35 mm batches, 20 mm approach; real actions, no replay')
    parser.add_argument('--expanded-plan-only',action='store_true')
    parser.add_argument('--strict-twist',action='store_true')
    parser.add_argument('--negative-local',action='store_true')
    parser.add_argument('--scan',action='store_true');args=parser.parse_args()
    if (args.scan or args.negative_local) and args.execute_local:parser.error('scan/negative tests never execute')
    if args.expanded_local and not args.execute_local:parser.error('expanded-local requires execute-local')
    if args.expanded_plan_only and args.execute_local:parser.error('expanded-plan-only never executes')
    if not args.initialize_reference_at_home:parser.error('explicit --initialize-reference-at-home required; no automatic zeroing')
    assert os.environ['ROS_DOMAIN_ID']=='96' and os.environ['ROS_LOCALHOST_ONLY']=='1'
    root=args.project_root.resolve();out=pathlib.Path(args.out)
    out.mkdir(parents=True,exist_ok=False)
    cfg=root/'src/chess_robot_arm/config';model=root/'src/chess_robot_description/urdf/nova5_with_ag95.urdf'
    home=np.array(json.loads((root/'config/arm/home_reference.json').read_text())['joint_positions_rad'])
    reference_sha=hashlib.sha256((root/'config/arm/home.yaml').read_bytes()).hexdigest()
    rclpy.init();node=rclpy.create_node('business_hardware_acceptance')
    commissioning=args.execute_local or args.negative_local or args.expanded_plan_only
    state=None;joint=None;rows=[];states=[];events=[];active=None;report={'status':'STARTING','mode':'commissioning' if commissioning else 'read_only','motion_goals_attempted':0,'goals':events,'home_sha256':reference_sha}
    def receive_state(m):
        nonlocal state
        state=m
        states.append({'host_utc_ns':time.time_ns(),'mode':m.controller_mode,'fresh':m.feedback_fresh,'reference':m.reference_valid,'phase':m.phase,'diagnostic':m.diagnostic})
    def receive_joints(m):
        nonlocal joint
        joint=m
        rows.append([time.time_ns(),m.header.stamp.sec+m.header.stamp.nanosec/1e9,*(list(m.position)+list(m.velocity)),state.controller_mode if state else -1,state.phase if state else '',state.signed_twist_rad if state else 0,state.total_twist_rad if state else 0])
    node.create_subscription(ArmState,'/arm/state',receive_state,100)
    node.create_subscription(JointState,'/arm/joint_states',receive_joints,100)
    client=ActionClient(node,MoveArm,'/arm/move');initialize=node.create_client(InitializeReference,'/arm/initialize_reference')
    log=(out/'node.log').open('w')
    cmd=[str(root/'install/chess_robot_arm/lib/chess_robot_arm/arm_node'),'--ros-args','--params-file',str(cfg/'arm.yaml'),'--params-file',str(cfg/'home.yaml')]
    if commissioning:cmd+=['--params-file',str(cfg/'commissioning.yaml')]
    if args.execute_local:cmd+=['-p','hardware.enable:=true']
    if args.expanded_local or args.expanded_plan_only:
        cmd+=['-p','commissioning.tcp_radius_m:=0.04','-p','commissioning.joint_radius_rad:=0.12',
              '-p','approach_clearance_m:=0.02']
    cmd+=['-p','controller.ip:='+args.controller_ip,'-p','urdf_file:='+str(model),'-p','safety.model_file:='+str(cfg/'nova5_ag95_geometry.yaml')]
    cmd+=['-p','audit.directory:='+str(out/'audit')]
    if args.strict_twist:cmd+=['-p','twist.net_limit_rad:=0.001','-p','twist.total_limit_rad:=0.001']
    (out/'command.json').write_text(json.dumps(cmd,indent=2)+'\n')
    proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT)
    def save():
        (out/'results.json').write_text(json.dumps(report,indent=2)+'\n')
    def wait(pred,timeout=15):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.005)
            if pred():return
            if proc.poll() is not None:raise RuntimeError('arm_node exited')
        raise TimeoutError('ROS operation deadline')
    def future(f,timeout=40):wait(f.done,timeout);return f.result()
    def pose_values():
        p=state.tcp.pose
        return [p.position.x,p.position.y,p.position.z],[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
    def goal(label,operation,position=None,quat=None,execute=False,cancel=False,allow_reject=False):
        nonlocal active
        g=MoveArm.Goal();g.operation=operation;g.profile='commissioning_slow' if commissioning else 'conservative';g.plan_only=not execute
        if position is not None:
            g.target.header.frame_id='base_link';g.target.header.stamp=node.get_clock().now().to_msg()
            g.target.pose.position.x,g.target.pose.position.y,g.target.pose.position.z=map(float,position)
            g.target.pose.orientation.x,g.target.pose.orientation.y,g.target.pose.orientation.z,g.target.pose.orientation.w=map(float,quat)
        before=pose_values()[0];started=time.monotonic();start_frame=len(rows)
        if execute:report['motion_goals_attempted']+=1;save()
        active=future(client.send_goal_async(g));assert active.accepted,'goal rejected'
        pending=active.get_result_async();cancelled_at=None
        deadline=time.monotonic()+180
        while not pending.done():
            rclpy.spin_once(node,timeout_sec=.005)
            if cancel and cancelled_at is None and state and state.tcp.pose.position.z>before[2]+.0004:
                cancelled_at=time.monotonic();future(active.cancel_goal_async(),5)
            if time.monotonic()>deadline:raise TimeoutError('action deadline; abort further tests')
        r=pending.result().result
        item={'label':label,'operation':operation,'target_m':position,'quaternion_xyzw':quat,'plan_only':not execute,'code':r.code,'detail':r.detail,'executed':r.executed,'samples':len(r.trajectory.points),'elapsed_s':time.monotonic()-started,'feedback_rows':len(rows)-start_frame,'cancel_requested':cancelled_at is not None}
        if cancelled_at is not None:item['cancel_to_verified_terminal_s']=time.monotonic()-cancelled_at
        if r.final_tcp_valid:
            p=r.final_tcp.pose.position;item['final_tcp_m']=[p.x,p.y,p.z]
        if execute:
            item['arrival_feedback']={'fresh':state.feedback_fresh,'reference_valid':state.reference_valid,
                                     'mode':state.controller_mode,'joints_rad':list(joint.position),
                                     'net_twist_rad':state.signed_twist_rad,'total_twist_rad':state.total_twist_rad}
        if r.trajectory.points:
            times=[p.time_from_start.sec+p.time_from_start.nanosec/1e9 for p in r.trajectory.points]
            item['duration_s']=times[-1]
            # Save exact production trajectories, compressed, for independent QA.
            with gzip.open(out/(label+'_trajectory.json.gz'),'wt') as f:json.dump({'q':[list(p.positions) for p in r.trajectory.points],'t':times},f)
        events.append(item);save();print(json.dumps(item),flush=True)
        active=None;wait(lambda:not state.busy)
        assert state.feedback_fresh and state.reference_valid, state.diagnostic
        if not execute:assert not r.executed
        if execute:
            expected=MoveArm.Result.CANCELED if cancel else MoveArm.Result.COMPLETED
            assert r.code==expected and r.executed,(label,r.code,r.detail)
            assert state.feedback_fresh and state.reference_valid and state.controller_mode==5
        elif not allow_reject:assert r.code==MoveArm.Result.PLAN_READY,(label,r.code,r.detail)
        return r
    try:
        assert client.wait_for_server(timeout_sec=15)
        wait(lambda:state is not None and state.feedback_fresh and joint is not None)
        assert not state.simulated and not state.hardware_certified
        assert state.controller_mode==5 and np.max(np.abs(np.asarray(joint.position)-home))<.003
        p,q=pose_values();report['initial']={'tcp_m':p,'quaternion_xyzw':q,'joints_rad':list(joint.position),'mode':state.controller_mode}
        tick=time.monotonic();wait(lambda:time.monotonic()-tick>.6)
        response=future(initialize.call_async(InitializeReference.Request()));assert response.success,response.detail
        wait(lambda:state.reference_valid);report['status']='RUNNING';save()
        if args.expanded_local or args.expanded_plan_only:
            origin=np.asarray(p);clearance=.02
            report['local_origin_tcp_m']=p;report['approach_clearance_m']=clearance
            report['reference_initializations']=1;report['batches']=[]
            if args.execute_local:
                goal('initial_home',4,execute=True)
                stop_target=(origin+[0,0,.003-clearance]).tolist()
                goal('cancel_precheck',0,stop_target,q)
                goal('cancel_verified',0,stop_target,q,execute=True,cancel=True)
                goal('home_after_cancel',4,execute=True)
            # Every target's entire PREGRASP -> DESCEND -> RETREAT chain is
            # planned first. Actual phase actions each require measured arrival.
            # No gripper confirmation is fabricated: no grip is attempted here.
            batches=[('25mm',[[.025,0,0],[0,0,.025]]),
                     ('30mm',[[0,.030,0],[0,0,-.030]]),
                     ('35mm',[[-.035,0,0],[0,-.035,0],[0,0,.035],[.020,.020,.020]])]
            # 2026-10-25 plan03 rejected front 30 / back 35 mm on the
            # joint envelope. Preserve them in planning evidence; never execute.
            if args.execute_local:
                batches=[('25mm',[[.025,0,0]]),('30mm',[[0,0,-.030]]),
                         ('35mm',[[-.035,0,0],[0,0,.035],[.020,.020,.020]])]
            for batch,deltas in batches:
                record={'batch':batch,'status':'RUNNING'};report['batches'].append(record);save()
                for index,delta in enumerate(deltas):
                    label=batch+'_'+str(index);pre=origin+delta
                    grasp=(pre-[0,0,clearance]).tolist()
                    # Some pregrasp points would descend outside the local
                    # sphere; validate/execute only PREGRASP at those points.
                    full=np.linalg.norm(np.asarray(delta)-[0,0,clearance])<.038
                    goal(label+'_precheck',3 if full else 0,grasp,q,allow_reject=args.expanded_plan_only)
                    if not args.execute_local:continue
                    goal(label+'_pregrasp',0,grasp,q,execute=True)
                    if full:
                        goal(label+'_descend_precheck',1,grasp,q)
                        goal(label+'_descend',1,grasp,q,execute=True)
                        goal(label+'_retreat_precheck',2,grasp,q)
                        goal(label+'_retreat',2,grasp,q,execute=True)
                    goal(label+'_home',4,execute=True)
                record['status']='PASS';save()
            if args.execute_local:goal('final_home',4,execute=True)
        elif args.execute_local:
            # First real ServoJ is a measured Home hold; then exercise Stop by
            # cancelling a 2 mm lift only after measured displacement appears.
            goal('initial_home',4,execute=True)
            p,q=pose_values();report['local_origin_tcp_m']=p
            up=[p[0],p[1],p[2]+.002-.25]
            goal('cancel_up_2mm_precheck',0,up,q)
            goal('cancel_up_2mm',0,up,q,execute=True,cancel=True)
            settle=time.monotonic();wait(lambda:time.monotonic()-settle>3)
            assert state.feedback_fresh and state.reference_valid and state.controller_mode==5,state.diagnostic
            goal('home_after_cancel',4,execute=True)
            for label,delta in [('up_5mm',[0,0,.005]),('left_5mm',[-.005,0,0]),('right_5mm',[.005,0,0]),('front_5mm',[0,.005,0]),('back_5mm',[0,-.005,0]),('up_10mm',[0,0,.010]),('left_10mm',[-.010,0,0]),('right_10mm',[.010,0,0]),('front_10mm',[0,.010,0]),('back_10mm',[0,-.010,0]),('up_20mm',[0,0,.020])]:
                target=(np.array(p)+delta).tolist();target[2]-=.25
                goal(label+'_precheck',0,target,q)
                goal(label,0,target,q,execute=True)
                goal(label+'_home',4,execute=True)
            current,orientation=pose_values();down=[current[0],current[1],current[2]-.002]
            goal('descend_2mm',1,down,orientation,execute=True)
            grasp=[current[0],current[1],current[2]-.25]
            goal('retreat_2mm',2,grasp,orientation,execute=True)
            goal('final_home',4,execute=True)
        elif args.negative_local:
            goal('home',4)
            r=goal('outside_local_envelope',0,[p[0],p[1],p[2]+.04-.25],q,allow_reject=True)
            assert r.code==14 and 'commissioning Home envelope' in r.detail
            r=goal('misaligned_descend',1,[p[0]+.02,p[1],p[2]-.002],q,allow_reject=True)
            assert r.code==14 and 'aligned' in r.detail
            r=goal('ascending_descend',1,[p[0],p[1],p[2]+.002],q,allow_reject=True)
            assert r.code==14 and 'direction mismatch' in r.detail
        elif args.scan:
            # Full production sequence at coarse grid; fixed Home yaw and
            # 250 mm pregrasp/descend/retreat. No physical scan.
            for z in [.025,.10,.20]:
                for y in [-.75,-.65,-.55,-.45,-.35,-.25]:
                    for x in [-.35,-.25,-.15,-.05,.05,.15,.25,.35]:
                        if z!=.025 and (x not in [-.25,-.05,.15,.35] or y not in [-.65,-.45,-.25]):continue
                        goal(f'grid_{x}_{y}_{z}',3,[x,y,z],q,allow_reject=True)
            for index,point in enumerate([[0,-.10,.025],[0,-.15,.025],[0,-.20,.025],[0,-.79,.025],[-.5,-.5,.025],[.5,-.5,.025],[-.65,-.25,.025],[.65,-.25,.025],[0,.3,.025]]):
                goal('edge_'+str(index),3,point,q,allow_reject=True)
            for angle in [-180,-90,-45,45,90,180]:
                quat=(Rotation.from_euler('z',angle,degrees=True)*Rotation.from_quat(q)).as_quat().tolist()
                goal('yaw_'+str(angle),3,[p[0],p[1],.025],quat,allow_reject=True)
        else:
            goal('home',4)
            goal('up_5mm',0,[p[0],p[1],p[2]+.005-.25],q,allow_reject=args.strict_twist)
            goal('descend_2mm',1,[p[0],p[1],p[2]-.002],q,allow_reject=args.strict_twist)
            if not args.strict_twist:goal('sequence',3,[p[0],p[1],.025],q)
        report['final']={'mode':state.controller_mode,'reference_valid':state.reference_valid,'fresh':state.feedback_fresh,'joints_rad':list(joint.position),'tcp_m':pose_values()[0],'net_twist_rad':state.signed_twist_rad,'total_twist_rad':state.total_twist_rad}
        assert hashlib.sha256((root/'config/arm/home.yaml').read_bytes()).hexdigest()==reference_sha
        report['status']='PASS';save()
    except Exception as e:
        report['status']='FAILED';report['error']=repr(e)
        for batch in report.get('batches',[]):
            if batch['status']=='RUNNING':batch.update(status='FAILED',error=repr(e))
        if state is not None and joint is not None:
            report['failure_feedback']={'fresh':state.feedback_fresh,'reference_valid':state.reference_valid,
                                        'mode':state.controller_mode,'joints_rad':list(joint.position),
                                        'tcp_m':pose_values()[0],'net_twist_rad':state.signed_twist_rad,
                                        'total_twist_rad':state.total_twist_rad}
        save();raise
    finally:
        if active is not None:
            try:future(active.cancel_goal_async(),5)
            except Exception:pass
        proc.send_signal(signal.SIGINT)
        try:proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            if args.execute_local:
                report['shutdown_error']='node still running; no force kill of motion owner';save()
            else:proc.kill();proc.wait()
        with (out/'joint_feedback.csv').open('w') as f:
            w=csv.writer(f);w.writerow(['host_utc_ns','ros_stamp_s',*[f'q{i}_rad' for i in range(1,7)],*[f'dq{i}_rad_s' for i in range(1,7)],'mode','phase','net_twist_rad','total_twist_rad']);w.writerows(rows)
        (out/'state_history.json').write_text(json.dumps(states,indent=2)+'\n')
        log.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
