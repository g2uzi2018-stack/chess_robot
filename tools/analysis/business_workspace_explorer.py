"""Explicit real-business exploration. No MovJ, reference zeroing or motion retry.

Planning rejects are evidence and may be bypassed via a separately checked route.
Execution faults terminate this process/batch; restarting needs a new output and
an explicit recorded-history handoff, not replay of saved trajectories.
"""
import argparse,csv,gzip,hashlib,ipaddress,json,math,pathlib,shutil,signal,socket,subprocess,time
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.action import ActionClient
from chess_robot_interfaces.action import MoveArm
from chess_robot_interfaces.msg import ArmState
from chess_robot_interfaces.srv import RestoreReference
from sensor_msgs.msg import JointState

def main():
    p=argparse.ArgumentParser();p.add_argument('--controller-ip',required=True,type=lambda s:str(ipaddress.IPv4Address(s)))
    p.add_argument('--out',type=pathlib.Path,required=True)
    p.add_argument('--project-root',type=pathlib.Path,default=pathlib.Path(__file__).resolve().parents[2])
    p.add_argument('--history-audit',type=pathlib.Path,action='append',required=True)
    p.add_argument('--attest-no-unobserved-motion',action='store_true',required=True)
    p.add_argument('--execute',action='store_true');p.add_argument('--enable-disabled',action='store_true')
    p.add_argument('--completed-plan',type=pathlib.Path,help='Explicit new batch after diagnosis; skip already reached checklist items')
    p.add_argument('--skip-recovery-home',action='store_true',help='Only with an earlier measured Home and completed-plan; continue the restored pose')
    p.add_argument('--validate-speed',action='store_true',help='New up/left/right trajectories and raw-speed verification before continuing a repaired batch')
    args=p.parse_args()
    import os
    assert os.environ.get('ROS_DOMAIN_ID')=='96' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    root=args.project_root.resolve();out=args.out.resolve();out.mkdir(parents=True,exist_ok=False)
    assert shutil.disk_usage(out).free>2_000_000_000,'insufficient space for raw feedback'
    config=root/'src/chess_robot_arm/config';clearance=.05
    stages=['near','inner','middle','outer','full']
    report={'status':'STARTING','execute':args.execute,'reference_initializations':0,'reference_restores':0,
            'started_utc_ns':time.time_ns(),'no_unobserved_motion_attested':args.attest_no_unobserved_motion,
            'goals':[],'checklist':[],'planning_scan':[],'boundaries':[],'batches':[],'history':[]}
    for path in args.history_audit:
        path=path.resolve();report['history'].append({'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    home_sha=hashlib.sha256((root/'config/arm/home.yaml').read_bytes()).hexdigest();report['home_sha256']=home_sha
    report['source_sha256']={str(path.relative_to(root)):hashlib.sha256(path.read_bytes()).hexdigest() for path in [root/'src/chess_robot_arm/src/core.cpp',root/'src/chess_robot_arm/src/driver.cpp',config/'exploration.yaml']}
    report['node_binary_sha256']=hashlib.sha256((root/'install/chess_robot_arm/lib/chess_robot_arm/arm_node').read_bytes()).hexdigest()
    completed={}
    if args.skip_recovery_home:assert args.completed_plan,'continuation needs previous measured Home provenance'
    if args.completed_plan:
        previous=json.loads(args.completed_plan.read_text());completed={c['id']:c for c in previous['checklist'] if c['status']=='reached'}
        report['previous_batch']=str(args.completed_plan.resolve())
    def save():
        temporary=out/'results.tmp';temporary.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');temporary.replace(out/'results.json')
    save();rclpy.init();node=rclpy.create_node('business_workspace_explorer')
    state=None;joint=None;active=None;last_key=None;feedback_count=0
    jf=(out/'joint_feedback.csv').open('w');jw=csv.writer(jf)
    jw.writerow(['host_utc_ns','ros_stamp_s',*[f'q{i}_rad' for i in range(1,7)],*[f'dq{i}_rad_s' for i in range(1,7)],'mode','phase','net_twist_rad','total_twist_rad'])
    sf=(out/'state_history.jsonl').open('w')
    def receive_state(m):
        nonlocal state,last_key
        state=m;key=(m.phase,m.feedback_fresh,m.reference_valid,m.controller_mode,m.diagnostic)
        if key!=last_key:
            sf.write(json.dumps({'host_utc_ns':time.time_ns(),'phase':m.phase,'fresh':m.feedback_fresh,'reference':m.reference_valid,'mode':m.controller_mode,'diagnostic':m.diagnostic})+'\n');sf.flush();last_key=key
    def receive_joint(m):
        nonlocal joint,feedback_count
        joint=m;feedback_count+=1
        jw.writerow([time.time_ns(),m.header.stamp.sec+m.header.stamp.nanosec/1e9,*m.position,*m.velocity,state.controller_mode if state else -1,state.phase if state else '',state.signed_twist_rad if state else 0,state.total_twist_rad if state else 0])
        if feedback_count%20==0:jf.flush()
    node.create_subscription(ArmState,'/arm/state',receive_state,100)
    node.create_subscription(JointState,'/arm/joint_states',receive_joint,100)
    client=ActionClient(node,MoveArm,'/arm/move');restore=node.create_client(RestoreReference,'/arm/restore_reference')
    command=[str(root/'install/chess_robot_arm/lib/chess_robot_arm/arm_node'),'--ros-args']
    for file in ['arm.yaml','home.yaml','exploration.yaml']:command+=['--params-file',str(config/file)]
    command+=['-p','controller.ip:='+args.controller_ip,'-p','urdf_file:='+str(root/'src/chess_robot_description/urdf/nova5_with_ag95.urdf'),'-p','safety.model_file:='+str(config/'nova5_ag95_geometry.yaml'),'-p','audit.directory:='+str(out/'audit')]
    if args.execute:command+=['-p','hardware.enable:=true']
    (out/'command.json').write_text(json.dumps(command,indent=2)+'\n')
    log=(out/'node.log').open('w');proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
    def wait(predicate,timeout=15):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.005)
            if predicate():return
            if proc.poll() is not None:raise RuntimeError('arm_node exited')
        raise TimeoutError('ROS deadline')
    def future(f,timeout=90):wait(f.done,timeout);return f.result()
    def pose():
        t=state.tcp.pose
        return np.array([t.position.x,t.position.y,t.position.z]),np.array([t.orientation.x,t.orientation.y,t.orientation.z,t.orientation.w])
    def snapshot():
        return {'tcp_m':pose()[0].tolist(),'quaternion_xyzw':pose()[1].tolist(),'joints_rad':list(joint.position),'mode':state.controller_mode,'fresh':state.feedback_fresh,'reference_valid':state.reference_valid,'net_twist_rad':state.signed_twist_rad,'total_twist_rad':state.total_twist_rad}
    def goal(label,operation,xyz=None,quat=None,stage='full',execute=False,cancel=False):
        nonlocal active
        assert not execute or args.execute
        before=snapshot();g=MoveArm.Goal();g.operation=operation;g.profile=stage;g.plan_only=not execute
        if xyz is not None:
            g.target.header.frame_id='base_link';g.target.header.stamp=node.get_clock().now().to_msg()
            g.target.pose.position.x,g.target.pose.position.y,g.target.pose.position.z=map(float,xyz)
            g.target.pose.orientation.x,g.target.pose.orientation.y,g.target.pose.orientation.z,g.target.pose.orientation.w=map(float,quat)
        start=time.monotonic();start_row=feedback_count
        rejection_log_offset=(out/'node.log').stat().st_size
        report['active_goal']={'label':label,'operation':operation,'stage':stage,'execute':execute};save()
        active=future(client.send_goal_async(g))
        if not active.accepted:
            active=None
            with (out/'node.log').open('rb') as rejected_log:
                rejected_log.seek(rejection_log_offset);diagnostic=rejected_log.read().decode(errors='replace')
            reasons=[line.split('goal rejected:',1)[1].strip() for line in diagnostic.splitlines() if 'goal rejected:' in line]
            reason=reasons[-1] if reasons else 'no node diagnostic'
            if execute or not any(x in reason for x in ['TCP outside configured workspace','grasp axis is not downward']):
                raise RuntimeError('action admission rejected: '+label+' '+reason)
            # -1 is an explicit audit sentinel, not an invented Action Result.
            record=dict(label=label,operation=operation,stage=stage,target_m=list(map(float,xyz)),quaternion_xyzw=list(map(float,quat)),
                        plan_only=True,code=-1,detail='ADMISSION_REJECTED: '+reason,admission_rejected=True,action_result_received=False,
                        executed=False,elapsed_s=time.monotonic()-start,feedback_rows=feedback_count-start_row,before=before,after=snapshot())
            report['goals'].append(record);report.pop('active_goal',None);save();print(json.dumps(record,ensure_ascii=False),flush=True)
            assert state.feedback_fresh and state.reference_valid,state.diagnostic
            return record
        pending=active.get_result_async();cancel_at=None
        deadline=start+1200
        while not pending.done():
            rclpy.spin_once(node,timeout_sec=.005)
            if cancel and cancel_at is None and np.linalg.norm(pose()[0]-np.asarray(before['tcp_m']))>.001:
                cancel_at=time.monotonic();reply=future(active.cancel_goal_async(),5);assert reply.goals_canceling
            if time.monotonic()>deadline:raise TimeoutError('action execution deadline')
        result=pending.result().result;active=None
        wait(lambda:not state.busy)
        record={'label':label,'operation':operation,'stage':stage,'target_m':list(map(float,xyz)) if xyz is not None else None,'quaternion_xyzw':list(map(float,quat)) if quat is not None else None,'plan_only':not execute,'code':result.code,'detail':result.detail,'executed':result.executed,'elapsed_s':time.monotonic()-start,'feedback_rows':feedback_count-start_row,'before':before,'after':snapshot()}
        if result.final_tcp_valid:
            t=result.final_tcp.pose.position;record['final_tcp_m']=[t.x,t.y,t.z]
        if cancel_at:record['cancel_to_verified_terminal_s']=time.monotonic()-cancel_at
        if result.trajectory.points:
            points=result.trajectory.points
            times=[x.time_from_start.sec+x.time_from_start.nanosec/1e9 for x in points]
            record['planned_duration_s']=times[-1];record['samples']=len(points)
            with gzip.open(out/(label+'_trajectory.json.gz'),'wt') as f:json.dump({'q':[list(x.positions) for x in points],'t':times},f)
        report['goals'].append(record);report.pop('active_goal',None);save();print(json.dumps(record,ensure_ascii=False),flush=True)
        assert state.feedback_fresh and state.reference_valid,state.diagnostic
        if execute:
            expected=MoveArm.Result.CANCELED if cancel else MoveArm.Result.COMPLETED
            if result.code!=expected or not result.executed:raise RuntimeError('EXECUTION FAULT: '+label+' '+result.detail)
            assert state.controller_mode==5 and result.final_tcp_valid
        else:
            assert not result.executed
            if result.code in [12,13,19] or any(t in result.detail for t in ['socket deadline','partial reply','Control Mode Is Not Tcp']):
                raise RuntimeError('reference/transport fault during planning: '+result.detail)
        return record
    def item(label,target,group,quat=None):
        c={'id':label,'target_m':list(map(float,target)),'group':group,'status':'pending'}
        if quat is not None:c['quaternion_xyzw']=list(map(float,quat))
        if label in completed and np.linalg.norm(np.asarray(completed[label]['target_m'])-np.asarray(target))<=.0005:
            c.update(status='reached',previous_batch=str(args.completed_plan),previous_result=completed[label])
        report['checklist'].append(c);return c
    def attempt(c,stage,full=True,bridge=True):
        if c['status']=='reached':return True
        target=c['target_m'];quat=c.get('quaternion_xyzw',base_quat)
        check=goal(c['id']+'_'+stage+'_plan',3 if full else 0,target,quat,stage)
        c.setdefault('attempts',[]).append({'stage':stage,'code':check['code'],'detail':check['detail']});save()
        if check['code']!=0:
            # A target planned from Home may require a high transfer from the
            # current region. A bridge is its own checked, measured Action.
            if bridge and stage=='full' and c.get('scan_pass') and not any(x in check['detail'] for x in ['joint limit','outside joint bounds','twist limit','singularity']):
                safe=[0,-.50,.35-clearance]
                b=goal(c['id']+'_bridge_plan',0,safe,base_quat,stage)
                if b['code']==0:
                    if args.execute:goal(c['id']+'_bridge',0,safe,base_quat,stage,True)
                    check=goal(c['id']+'_after_bridge_plan',3 if full else 0,target,quat,stage)
            if check['code']!=0:
                c['status']='planning_rejected' if stage=='full' else 'deferred';c['last_reason']=check['detail'];save();return False
        c.pop('last_reason',None)
        if args.execute:
            pre=goal(c['id']+'_'+stage+'_pregrasp',0,target,quat,stage,True)
            if full:
                descend_check=goal(c['id']+'_'+stage+'_descend_plan',1,target,quat,stage)
                if descend_check['code']!=0:raise RuntimeError('prevalidated descent no longer plannable: '+descend_check['detail'])
                goal(c['id']+'_'+stage+'_descend',1,target,quat,stage,True)
                retreat_check=goal(c['id']+'_'+stage+'_retreat_plan',2,target,quat,stage)
                if retreat_check['code']!=0:raise RuntimeError('prevalidated retreat no longer plannable: '+retreat_check['detail'])
                goal(c['id']+'_'+stage+'_retreat',2,target,quat,stage,True)
            c['status']='reached';c['measured_pregrasp_m']=pre['final_tcp_m'];c['achieved_stage']=stage
        else:c['status']='planning_passed'
        save();return True
    try:
        assert client.wait_for_server(timeout_sec=20)
        wait(lambda:state is not None and state.feedback_fresh and joint is not None)
        assert not state.simulated and not state.hardware_certified
        if state.controller_mode==4 and args.enable_disabled:
            # Separate, explicitly requested torque preparation. Never clears
            # errors, changes control mode, sends motion, or touches reference.
            with socket.create_connection((args.controller_ip,29999),timeout=2) as sock:
                sock.settimeout(3)
                def call(command):
                    sock.sendall(command.encode());raw=b''
                    while not raw.endswith(b';'):
                        chunk=sock.recv(4096);assert chunk;raw+=chunk
                    text=raw.decode();assert text.startswith('0,'),text
                    report.setdefault('preparation',[]).append({'command':command,'reply':text});save();return text
                assert call('GetErrorID()').split('{',1)[1].split('}',1)[0] in ['[]','[[],[],[],[],[],[],[]]']
                assert '{4}' in call('RobotMode()');call('EnableRobot()')
            wait(lambda:state.controller_mode==5)
        assert state.controller_mode==5
        stable=time.monotonic();wait(lambda:time.monotonic()-stable>.6)
        report['initial']=snapshot();save()
        assert restore.wait_for_service(timeout_sec=10)
        request=RestoreReference.Request();request.audit_files=[str(x.resolve()) for x in args.history_audit];request.no_unobserved_motion=True
        response=future(restore.call_async(request),600);assert response.success,response.detail
        wait(lambda:state.reference_valid);report['reference_restores']=1;report['restored']=snapshot();save()
        if args.skip_recovery_home:
            original=previous;original_path=args.completed_plan.resolve();visited=set()
            while not any(g['label']=='recovery_home' and g['code']==1 and g['executed'] for g in original['goals']):
                assert original_path not in visited and len(visited)<16,'cyclic/long Home provenance chain'
                visited.add(original_path);assert original.get('previous_batch'),'no measured Home provenance'
                original_path=pathlib.Path(original['previous_batch']);original=json.loads(original_path.read_text())
            assert previous.get('home_pose',{}).get('reference_valid') and original['home_sha256']==home_sha
            report['home_pose']=previous['home_pose'];report['continued_without_returning_home']=True
            report['measured_home_provenance']=str(original_path)
            home_position=np.asarray(previous['home_pose']['tcp_m']);base_quat=previous['home_pose']['quaternion_xyzw']
        else:
            check=goal('recovery_home_plan',4,stage='near');assert check['code']==0,check['detail']
            if args.execute:goal('recovery_home',4,stage='near',execute=True)
            home_position,base_quat=pose();base_quat=base_quat.tolist();report['home_pose']=snapshot()
        # Manifest is fixed before workspace actuation: 6 progressive setup
        # points, 27 full-channel grid points, 6 cross-region transfers, 12 yaw
        # changes, plus the explicitly generated boundary-bracketing checklist.
        near=[item('near_'+str(i),(home_position+delta-[0,0,clearance]),'near') for i,delta in enumerate([[.05,0,0],[-.05,0,0],[0,.05,.02]])]
        inner=[item('inner_'+str(i),(home_position+delta-[0,0,clearance]),'inner') for i,delta in enumerate([[.10,0,-.03],[-.10,0,0],[0,.12,0]])]
        grid=[]
        for iz,z in enumerate([.025,.20,.40]):
            for iy,y in enumerate([-.70,-.50,-.30]):
                xs=[-.30,0,.30] if (iy+iz)%2==0 else [.30,0,-.30]
                for x in xs:grid.append(item(f'grid_{x}_{y}_{z}',[x,y,z],'grid'))
        cross=[item('cross_'+str(i),[x,-.50,.20],'cross') for i,x in enumerate([-.25,.25,-.25,.25,-.25,.25])]
        yaws=[]
        for i,angle in enumerate([30,60,90,60,30,0,-30,-60,-90,-60,-30,0]):
            quat=(Rotation.from_euler('z',angle,degrees=True)*Rotation.from_quat(base_quat)).as_quat()
            c=item('yaw_'+str(i),[0,-.50,.20],'yaw',quat);c['yaw_degrees']=angle;yaws.append(c)
        report['coverage_plan']={'grid_shape':[3,3,3],'grid_x_m':[-.30,0,.30],'grid_y_m':[-.70,-.50,-.30],'grasp_z_m':[.025,.20,.40],
                                  'approach_clearance_m':clearance,'boundary_heights_m':[.025,.40],'boundary_refinement_m':.025,
                                  'cross_transfers':6,'yaw_sequence_degrees':[c['yaw_degrees'] for c in yaws],
                                  'home_between_points':False,'planning_success_is_execution_success':False}
        report['status']='RUNNING';save()
        if args.execute and args.validate_speed:
            initial_tcp,initial_quat=pose();initial_quat=initial_quat.tolist()
            for label,delta in [('up',[0,0,.05]),('left',[-.05,0,.05]),('right',[0,0,.05])]:
                target=(initial_tcp+delta-[0,0,clearance]).tolist()
                planned=goal('speed_validation_'+label+'_plan',0,target,initial_quat,'full');assert planned['code']==0,planned['detail']
                goal('speed_validation_'+label,0,target,initial_quat,'full',True)
            # Validate the current real business Stop path after a measured
            # displacement. This is cancellation, never a replay or Home reset.
            stop_target=(pose()[0]+[0,0,.02-clearance]).tolist()
            planned=goal('repair_stop_validation_plan',0,stop_target,initial_quat,'full')
            assert planned['code']==0,planned['detail']
            goal('repair_stop_validation',0,stop_target,initial_quat,'full',True,True)
            calm=time.monotonic();wait(lambda:time.monotonic()-calm>.3)
            raw_path=out/'audit/feedback.bin';raw=raw_path.read_bytes();record_size=1448
            raw=raw[:len(raw)//record_size*record_size]
            frames=np.frombuffer(raw,dtype=np.dtype([('host','<i8'),('frame','u1',(1440,))]))['frame']
            peak=float(np.max(abs(frames[:,480:528].copy().view('<f8')))*math.pi/180)
            report['speed_validation']={'raw_frames':len(frames),'peak_raw_speed_rad_s':peak,'unchanged_monitor_limit_rad_s':.032,'status':'PASS' if peak<=.032 else 'FAILED'};save()
            assert peak<=.032,'raw speed excursion before next batch; no threshold relaxation'
        if args.execute and not args.skip_recovery_home:
            cancel_target=(pose()[0]+[0,0,.02-clearance]).tolist()
            check=goal('stop_check_plan',0,cancel_target,base_quat,'near');assert check['code']==0
            goal('stop_check',0,cancel_target,base_quat,'near',True,True)
        # Replan the entire grid through the CURRENT production planner/SDK.
        for c in grid:
            scan=goal(c['id']+'_initial_scan',3,c['target_m'],base_quat,'full')
            c['scan_pass']=scan['code']==0
            report['planning_scan'].append({'id':c['id'],'target_m':c['target_m'],'code':scan['code'],'detail':scan['detail']});save()
        for stage,group in [('near',near),('inner',inner),('middle',grid),('outer',grid),('full',grid)]:
            batch={'stage':stage,'status':'RUNNING','started_utc_ns':time.time_ns()};report['batches'].append(batch);save()
            for c in group:
                if c['status']!='reached':attempt(c,stage)
            batch.update(status='PASS',finished_utc_ns=time.time_ns(),reached=sum(c['status']=='reached' for c in group));save()
            if args.execute and stage in ['near','inner'] and batch['reached']<2:
                raise RuntimeError('progression gate: fewer than two measured setup points in '+stage)
        for c in near+inner:
            if c['status']!='reached':attempt(c,'full')
        # Four horizontal spokes at two heights. First bracket each rejection
        # with 150 mm steps, then bisect to <=25 mm; execute the farthest checked
        # candidate. Workspace limits are labelled as software boundaries.
        for z in [.025,.40]:
            for direction in [(1,0),(-1,0),(0,1),(0,-1)]:
                center=np.array([0,-.50,z]);lo=0;hi=None;best=None;probes=[]
                for distance in [.15,.30,.45,.60,.75]:
                    target=center+np.array([direction[0]*distance,direction[1]*distance,0])
                    label=f'boundary_scan_{z}_{direction[0]}_{direction[1]}_{distance:.4f}'
                    r=goal(label,3,target,base_quat,'full');probes.append({'distance_m':distance,'target_m':target.tolist(),'code':r['code'],'detail':r['detail']})
                    if r['code']==0:lo=distance;best=target
                    else:hi=distance;break
                if hi is not None:
                    for refinement in range(5):
                        if hi-lo<=.025:break
                        distance=(lo+hi)/2;target=center+np.array([direction[0]*distance,direction[1]*distance,0])
                        label=f'boundary_refine_{z}_{direction[0]}_{direction[1]}_{refinement}'
                        r=goal(label,3,target,base_quat,'full');probes.append({'distance_m':distance,'target_m':target.tolist(),'code':r['code'],'detail':r['detail']})
                        if r['code']==0:lo=distance;best=target
                        else:hi=distance
                record={'grasp_z_m':z,'direction_xy':direction,'passed_distance_m':lo,'rejected_distance_m':hi,'probes':probes}
                report['boundaries'].append(record);save()
                if best is not None:
                    c=item(f'boundary_actual_{z}_{direction[0]}_{direction[1]}',best,'boundary');c['scan_pass']=True
                    attempt(c,'full')
                else:record['status']='planning_rejected';save()
        for c in cross:attempt(c,'full',full=False)
        for c in yaws:attempt(c,'full',full=False)
        final=goal('final_home_plan',4,stage='full')
        if final['code']==0 and args.execute:goal('final_home',4,stage='full',execute=True)
        report['final']=snapshot();report['feedback_rows']=feedback_count
        assert hashlib.sha256((root/'config/arm/home.yaml').read_bytes()).hexdigest()==home_sha
        assert all(c['status'] in ['reached','planning_rejected','planning_passed'] for c in report['checklist'])
        report['status']='COMPLETE';save()
    except Exception as e:
        report.update(status='FAILED',error=repr(e),feedback_rows=feedback_count)
        for batch in report['batches']:
            if batch['status']=='RUNNING':batch.update(status='FAILED',error=repr(e))
        if state is not None and joint is not None:report['failure_feedback']=snapshot()
        save();raise
    finally:
        if active is not None:
            try:future(active.cancel_goal_async(),5)
            except Exception:pass
        proc.send_signal(signal.SIGINT)
        try:proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            report['shutdown_error']='motion owner still alive; no force kill';save()
        jf.close();sf.close();log.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
