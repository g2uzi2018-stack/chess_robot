#!/usr/bin/env python3
"""Finite table scan through existing arm Action and qualified scene output."""
import argparse,hashlib,json,math,signal,time,uuid
from collections import deque
from pathlib import Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from chess_robot_interfaces.msg import ArmState
from chess_robot_interfaces.action import MoveArm
from rosidl_runtime_py.convert import message_to_ordereddict as convert

def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
def write(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False))
def position(s):return np.array([s.tcp.pose.position.x,s.tcp.pose.position.y,s.tcp.pose.position.z])

class ScanLedger:
    """Local scan records only; never grant snapshot consumption or motion readiness."""
    def __init__(self):self.targets=[]
    def summarize_view(self,view,frames):
        groups=[];unlocated=[]
        for index,r in enumerate(frames):
            ds=[d for d in r['detections'] if d.get('base_geometry_valid') and d.get('complete_center_target')]
            used=set()
            # Reject ties on either side; same-frame objects never share a group.
            costs=np.array([[np.linalg.norm(np.array(d['base_surface_center_m'])-np.median([x['d']['base_surface_center_m'] for x in g],axis=0)) for g in groups] for d in ds])
            for j,d in enumerate(ds):
                close=[] if not groups else list(np.flatnonzero(costs[j]<.006))
                chosen=None
                if len(close)==1:
                    k=close[0]
                    if np.count_nonzero(costs[:,k]<.006)==1 and k not in used:chosen=k
                row=dict(index=index,stamp_ns=r['stamp_ns'],d=d,source_sequence=r['sequence'])
                if chosen is None:
                    row['association_ambiguous']=bool(close);groups.append([row])
                else:groups[chosen].append(row);used.add(chosen)
            for d in r['detections']:
                if not d.get('base_geometry_valid') or not d.get('complete_center_target'):
                    unlocated.append(dict(stamp_ns=r['stamp_ns'],box_xyxy=d.get('box_xyxy'),class_name=d.get('class_name'),reason=d.get('reason','incomplete center')))
        summaries=[]
        for group in groups:
            pts=np.array([x['d']['base_surface_center_m'] for x in group]);center=np.median(pts,axis=0)
            stamps=[x['stamp_ns'] for x in group];gaps=np.diff(stamps)*1e-9
            span=(stamps[-1]-stamps[0])*1e-9
            fraction=len(group)/len(frames) if frames else 0
            p95=float(np.percentile(np.linalg.norm(pts-center,axis=1),95))
            ambiguous=any(x.get('association_ambiguous') or x['d'].get('identity_ambiguous') for x in group)
            stable=span>=2 and len(group)>=20 and fraction>=.8 and (len(gaps)==0 or max(gaps)<=.3) and p95<=.003 and not ambiguous
            labels={x['d']['class_name'] for x in group if x['d'].get('classification_valid')}
            all_classified=all(x['d'].get('classification_valid') for x in group)
            label=next(iter(labels)) if len(labels)==1 and all_classified else 'unknown'
            summaries.append(dict(view_id=view['id'],base_surface_center_m=center.tolist(),position_state='stable_repeatability_only' if stable else 'unresolved',class_name=label,classification_state='observed_consistent' if label!='unknown' else 'unresolved',association_state='ambiguous' if ambiguous else 'within_view_unique',first_stamp_ns=stamps[0],last_stamp_ns=stamps[-1],samples=len(group),valid_fraction=fraction,span_s=span,position_p95_m=p95,source_sequences=[x['source_sequence'] for x in group],motion_ready=False,grasp_ready=False))
        # Cross-view proximity is only a possible-duplicate relation until absolute
        # error and close-object separation have been validated; never force merge.
        for s in summaries:
            s['scan_target_id']='candidate-'+str(len(self.targets)+1)
            near=[t['scan_target_id'] for t in self.targets if t['view_id']!=s['view_id'] and np.linalg.norm(np.array(t['base_surface_center_m'])-s['base_surface_center_m'])<.012]
            s['possible_duplicate_of']=near
            if near:s['cross_view_identity']='unresolved_proximity_hypothesis'
            else:s['cross_view_identity']='new_hypothesis'
            self.targets.append(s)
        return dict(view_id=view['id'],independent_frames=len(frames),targets=summaries,unlocated_observations=unlocated,coverage_state='SEEN_UNRESOLVED' if unlocated or any(t['class_name']=='unknown' or t['position_state']!='stable_repeatability_only' for t in summaries) else 'OBSERVED_VALID_CANDIDATES_ONLY',all_physical_objects_confirmed=False)

class Controller:
    def __init__(self,config,out,execute):
        self.c=config;self.out=out;self.execute=execute;self.n=rclpy.create_node('xiangqi_scan_controller')
        self.arm={};self.joint={};self.scene=deque(maxlen=300);self.goal=None;self.pending=None;self.terminal=None;self.abort=False
        self.subs=[self.n.create_subscription(ArmState,'/arm/state',lambda m:self.receive('arm',m),100),self.n.create_subscription(JointState,'/arm/joint_states',lambda m:self.receive('joint',m),100),self.n.create_subscription(String,'/pose/xiangqi/detections',self.receive_scene,10)]
        self.client=ActionClient(self.n,MoveArm,'/arm/move')
        self.params=self.n.create_client(GetParameters,'/chess_robot_arm/get_parameters')
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub=self.n.create_publisher(String,'/scan/xiangqi/status',qos)
        self.result_pub=self.n.create_publisher(String,'/scan/xiangqi/result',qos)
        self.report=dict(schema='xiangqi_scan_result/v1',scan_id=uuid.uuid4().hex,execute=execute,started_ns=self.now(),actions=[],views=[],config=config,physical_stop_confirmed=False,motion_ready=False,grasp_ready=False)
        self.ledger=ScanLedger();self.events=(out/'events.jsonl').open('w')
    def now(self):return self.n.get_clock().now().nanoseconds
    def receive(self,kind,m):
        cache=getattr(self,kind);cache[stamp(m)]=m
        while len(cache)>150:del cache[min(cache)]
    def receive_scene(self,m):
        r=json.loads(m.data)
        self.scene.append(r)
    def phase(self,phase,**kw):
        data=dict(scan_id=self.report['scan_id'],stamp_ns=self.now(),phase=phase,**kw)
        wire=json.dumps(data);self.events.write(wire+'\n');self.events.flush();self.pub.publish(String(data=wire));print(wire,flush=True)
    def spin(self,check_abort=True):
        rclpy.spin_once(self.n,timeout_sec=.005)
        if check_abort and self.abort:raise RuntimeError('operator interrupted')
    def wait(self,f,seconds,monitor=False,check_abort=True):
        deadline=time.monotonic()+seconds
        while not f.done() and time.monotonic()<deadline:
            self.spin(check_abort)
            if monitor:
                s,j=self.pair()
                if not (0<=(self.now()-stamp(s))*1e-9<=1. and s.feedback_fresh and s.reference_valid and not s.simulated and s.controller_mode in (5,7)):
                    raise RuntimeError('motion feedback/reference invalid')
        if not f.done():raise TimeoutError('ROS operation timeout')
        return f.result()
    def pair(self):
        common=self.arm.keys()&self.joint.keys()
        if not common:raise RuntimeError('paired arm feedback unavailable')
        t=max(common);return self.arm[t],self.joint[t]
    def stopped(self,timeout=12,check_abort=True):
        deadline=time.monotonic()+timeout;previous=None;start=None;anchor=None
        while time.monotonic()<deadline:
            self.spin(check_abort)
            if not (self.arm.keys()&self.joint.keys()):continue
            s,j=self.pair();t=stamp(s)
            if previous and t<=previous[0]:continue
            good=(0<=(self.now()-t)*1e-9<=.2 and s.feedback_fresh and s.reference_valid and not s.simulated and not s.busy and s.controller_mode==5 and len(j.position)==len(j.velocity)==6 and max(map(abs,j.velocity))<=.002)
            if previous:
                dt=(t-previous[0])*1e-9
                good=good and 0<dt<=.2 and max(abs(a-b) for a,b in zip(j.position,previous[1]))/dt<=.002
            if anchor is not None:good=good and max(abs(a-b) for a,b in zip(j.position,anchor))<=.001
            if not good:start=anchor=None
            elif start is None:start=t;anchor=list(j.position)
            previous=t,list(j.position)
            if start is not None and (t-start)*1e-9>=.6:
                self.report['physical_stop_confirmed']=True
                return s,j,t
        raise RuntimeError('fresh measured stationary interval unavailable')
    def check_parameters(self):
        assert self.params.wait_for_service(timeout_sec=5),'arm parameters unavailable'
        names=['approach_clearance_m','tcp.translation_m','tcp.rpy_rad','safety.model_file','motion.paused']
        req=GetParameters.Request();req.names=names
        vs=self.wait(self.params.call_async(req),5).values
        assert vs[0].type==3 and abs(vs[0].double_value-self.c['expected_approach_clearance_m'])<1e-9,'clearance changed'
        assert list(vs[1].double_array_value)==[0.,0.,.2],'TCP translation changed'
        assert np.allclose(vs[2].double_array_value,[math.pi/2,-math.pi/2,0],atol=1e-9),'TCP rotation changed'
        assert vs[3].string_value==self.c['guard_file'] and not vs[4].bool_value,'guard or motion policy changed'
        for path,digest in self.c['source_sha256'].items():assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==digest,'model/config changed: '+path
    def action(self,view,plan):
        if not self.report['physical_stop_confirmed']:self.stopped()
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            self.spin();s,j=self.pair()
            if not s.busy and 0<=(self.now()-stamp(s))*1e-9<=.2:break
        assert 0<=(self.now()-stamp(s))*1e-9<=.2 and s.feedback_fresh and s.reference_valid and not s.busy and not s.simulated and s.controller_mode==5 and max(map(abs,j.velocity))<=.002,'fresh stopped start unavailable'
        self.check_parameters()
        g=MoveArm.Goal();g.operation=g.HOME if view['id']=='home' else g.PREGRASP;g.profile=self.c['profile'];g.plan_only=plan;g.target.header.frame_id='base_link';g.target.header.stamp=self.n.get_clock().now().to_msg()
        if view['id']!='home':
            p=list(view['tcp_position_m']);p[2]-=self.c['expected_approach_clearance_m']
            g.target.pose.position.x,g.target.pose.position.y,g.target.pose.position.z=p
            g.target.pose.orientation.x,g.target.pose.orientation.y,g.target.pose.orientation.z,g.target.pose.orientation.w=view['tcp_quaternion_xyzw']
        self.phase('PLAN' if plan else 'MOVE',view_id=view['id'])
        self.pending=self.client.send_goal_async(g);self.goal=self.wait(self.pending,8);self.pending=None
        assert self.goal.accepted,'arm rejected goal'
        self.terminal=self.goal.get_result_async()
        if not plan:self.report['physical_stop_confirmed']=False
        reply=self.wait(self.terminal,60,monitor=not plan)
        self.report['actions'].append(dict(view_id=view['id'],plan_only=plan,status=reply.status,result=convert(reply.result)))
        assert reply.status==4 and reply.result.code==(0 if plan else 1) and reply.result.executed== (not plan),reply.result.detail
        if not plan:
            self.arrived_monotonic=time.monotonic()
            self.arrived_ns=self.now()
            assert reply.result.final_tcp_valid,'invalid measured final TCP'
            s,j,t=self.stopped()
            if view['id']!='home':
                assert np.linalg.norm(position(s)-view['tcp_position_m'])<=self.c['position_tolerance_m'],'measured final TCP position mismatch'
                q=s.tcp.pose.orientation;actual=np.array([q.x,q.y,q.z,q.w]);target=np.array(view['tcp_quaternion_xyzw'])
                assert 2*math.acos(float(np.clip(abs(actual@target),0,1)))<=self.c['rotation_tolerance_rad'],'measured final TCP orientation mismatch'
        self.goal=self.terminal=None
        write(self.out/'run-progress.json',self.report)
    def collect(self,view,attempt=0):
        s,j=self.pair();start=self.arrived_ns;self.phase('COLLECT',view_id=view['id'])
        fresh_mode=self.c.get('sampling_mode')=='proposed_wait_fresh_after_stop'
        coarse=fresh_mode or self.c.get('sampling_mode')=='coarse_one_second'
        if fresh_mode:
            deadline=self.arrived_monotonic+self.c['view_deadline_s']
        else:deadline=self.arrived_monotonic+self.c['point_stop_s'] if coarse else time.monotonic()+self.c['collection_s']
        self.scene.clear();frames=[];seen=set();generation=None;identity=None;rejected={}
        stem=view['id']+('-retry'+str(attempt) if attempt else '')
        with (self.out/(stem+'-scene.jsonl')).open('w') as stream:
            while time.monotonic()<deadline:
                self.spin()
                if fresh_mode:
                    current,joint=self.pair()
                    if not (0<=(self.now()-stamp(current))*1e-9<=.2 and current.reference_valid and current.feedback_fresh and not current.busy and current.controller_mode==5 and max(map(abs,joint.velocity))<=.002):raise RuntimeError('fresh stopped feedback unavailable during view wait')
                while self.scene:
                    r=self.scene.popleft();t=r['stamp_ns'];key=tuple(r.get('rgbd_frame_key') or [r.get('source_epoch'),r.get('sequence'),t])
                    if key in seen or t<start:continue
                    seen.add(key)
                    reason=None
                    if not (r.get('detection_valid') and r.get('base_transform_valid') and r.get('full_scene_coverage') and 0<=(self.now()-t)*1e-9<=.5):reason=r.get('base_transform_reason','unqualified scene frame')
                    if not reason:
                        state,joint=self.pair()
                        if not (0<=(self.now()-stamp(state))*1e-9<=.2 and state.reference_valid and state.feedback_fresh and not state.busy and state.controller_mode==5 and max(map(abs,joint.velocity))<=.002):raise RuntimeError('feedback lost while collecting')
                        if np.linalg.norm(position(state)-view['tcp_position_m'])>.001:raise RuntimeError('view moved during collection')
                        g=(r['stationary_generation'],r['controller_epoch'],r['track_namespace'])
                        ident=json.dumps(dict(identity=r.get('identity'),K=r.get('source_K')),sort_keys=True)
                        if generation is None:generation=g;identity=ident
                        if g!=generation or ident!=identity:raise RuntimeError('scene epoch/configuration changed within view')
                        frames.append(r);stream.write(json.dumps(r,ensure_ascii=False)+'\n')
                    else:rejected[reason]=rejected.get(reason,0)+1
                if fresh_mode and time.monotonic()-self.arrived_monotonic>=self.c['minimum_stop_s'] and len(frames)>=self.c['min_fresh_qualified_frames']:break
        if not coarse:
            assert frames and len(frames)>=20 and (frames[-1]['stamp_ns']-frames[0]['stamp_ns'])*1e-9>=2,'not enough independent qualified scene frames'
        v=self.ledger.summarize_view(view,frames);v.update(rejected_frames=rejected,stationary_generation=generation,source_identity=json.loads(identity) if identity else None,footprint_ls_m=view['footprint_ls_m'],all_pixels_validated=False,sampling_mode=self.c.get('sampling_mode','stable_window'),arrival_ns=self.arrived_ns,point_stop_elapsed_s=time.monotonic()-self.arrived_monotonic)
        if coarse:
            v['observation_state']='coarse_only_requires_reconfirmation'
            if not frames:v['coverage_state']='SEEN_UNRESOLVED_NO_QUALIFIED_FRAMES'
        qualified=not fresh_mode or len(frames)>=self.c['min_fresh_qualified_frames']
        v.update(attempt=attempt,sampling_requirement_met=qualified)
        if not qualified:v['coverage_state']='SEEN_UNRESOLVED_INSUFFICIENT_QUALIFIED_FRAMES'
        self.report['views'].append(v);write(self.out/(stem+'-summary.json'),v)
        self.phase('VIEW_RECORDED',view_id=view['id'],frames=len(frames),candidate_hypotheses=len(v['targets']),unlocated_observations=len(v['unlocated_observations']))
        return qualified
    def cancel_owned(self):
        if self.pending is not None:
            try:self.goal=self.wait(self.pending,8,check_abort=False)
            except Exception as e:self.report['acceptance_unknown']=str(e)
        if self.goal is not None and self.goal.accepted and self.execute:
            try:
                self.report['cancel_response']=convert(self.wait(self.goal.cancel_goal_async(),5,check_abort=False))
                if self.terminal is None:self.terminal=self.goal.get_result_async()
                reply=self.wait(self.terminal,10,check_abort=False)
                self.report['terminal_after_cancel']=dict(status=reply.status,result=convert(reply.result))
            except Exception as e:self.report['cancel_error']=str(e)
        try:self.stopped(check_abort=False)
        except Exception as e:self.report['physical_stop_confirmed']=False;self.report['stop_error']=str(e)
    def run(self):
        try:
            self.phase('PRECHECK');assert len(self.c['views'])<=17 and self.c['table_z_m']==-.01
            assert self.client.wait_for_server(timeout_sec=5),'arm action unavailable'
            self.stopped();self.check_parameters()
            # Require live camera/scene before initiating any physical motion.
            deadline=time.monotonic()+5
            while time.monotonic()<deadline:
                self.spin()
                if self.scene and self.scene[-1].get('base_transform_valid') and 0<=(self.now()-self.scene[-1]['stamp_ns'])*1e-9<=.5:break
            else:raise RuntimeError('fresh qualified scene unavailable')
            for v in [dict(id='home')]+self.c['views']:
                self.action(v,True)
                if self.execute:
                    self.action(v,False)
                    if v['id']!='home':
                        qualified=self.collect(v)
                        if not qualified and self.c.get('extra_rescan_per_missing_view',0):
                            self.phase('RETRY_VIEW',view_id=v['id'])
                            self.arrived_monotonic=time.monotonic();self.arrived_ns=self.now()
                            self.collect(v,attempt=1)
            self.phase('RETURN_HOME')
            self.action(dict(id='home'),True)
            if self.execute:self.action(dict(id='home'),False)
            self.report['returned_home']=bool(self.execute)
            self.report['status']='PARTIAL' if self.execute else 'PLAN_READY'
            self.report['completion_reason']='finite scan sequence completed; physical coverage/identity not fully verified' if self.execute else 'current-start plans only; no movement'
            self.phase(self.report['status'])
        except BaseException as e:
            self.report['status']='FAULT';self.report['error']=str(e);self.phase('FAULT',reason=str(e))
            self.cancel_owned()
        finally:
            self.report.update(ended_ns=self.now(),target_hypotheses=self.ledger.targets,unique_physical_target_count=None,physical_coverage_fraction=None)
            write(self.out/'scan-result.json',self.report)
            self.result_pub.publish(String(data=json.dumps(dict(scan_id=self.report['scan_id'],status=self.report['status'],result_file=str(self.out/'scan-result.json'),motion_ready=False,grasp_ready=False))))
            self.events.close();self.n.destroy_node()
        return self.report['status']!='FAULT'

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',required=True);p.add_argument('--output',required=True);p.add_argument('--execute',action='store_true');a=p.parse_args()
    config=json.loads(Path(a.config).read_text());out=Path(a.output);out.mkdir(parents=False,exist_ok=False);write(out/'config.json',config)
    rclpy.init();controller=Controller(config,out,a.execute)
    signal.signal(signal.SIGINT,lambda *_:setattr(controller,'abort',True));signal.signal(signal.SIGTERM,lambda *_:setattr(controller,'abort',True))
    okay=controller.run();rclpy.shutdown();raise SystemExit(0 if okay else 1)
if __name__=='__main__':main()
