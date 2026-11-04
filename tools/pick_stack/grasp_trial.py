#!/usr/bin/env python3
"""One supervised commissioning pick and 20mm lift; never stack or auto release."""
import argparse,json,time,math,signal
from pathlib import Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from chess_robot_interfaces.action import MoveArm,MoveGripper
from scan_controller import position,convert,stamp,write
from ros_io import PlanInspector,RealIO
from demo import qualified_target,distance

class Trial(PlanInspector):
    gripper_state=RealIO.gripper_state
    gripper_move=RealIO.gripper_move
    def __init__(self,cfg,out):
        self.held=False;self.prechecked=False;self.latest_scene=None
        super().__init__(cfg,out);self.execute=True
        self.grip_client=ActionClient(self.n,MoveGripper,'/gripper/command')
        self.grip_pending=self.grip_goal=self.grip_terminal=None
        self.report.update(schema='single_pick_commissioning/v1',gripper_actions=[],stacking_enabled=False)
    def receive_scene(self,msg):
        super().receive_scene(msg);self.latest_scene=self.scene[-1]
    def spin(self,check_abort=True):
        super().spin(check_abort)
        if check_abort and self.held:self.gripper_state(held=True)
    def fresh(self,seed):
        self.stopped();self.scene.clear();start=self.now();end=time.monotonic()+4
        frames=[];raw=[];seen=set()
        while time.monotonic()<end:
            self.spin();state,joint=self.pair()
            if not (state.feedback_fresh and state.reference_valid and not state.busy and state.controller_mode==5 and 0<=(self.now()-stamp(state))*1e-9<=.2 and max(map(abs,joint.velocity))<=.002):raise RuntimeError('stationary reference lost during observation')
            while self.scene:
                row=self.scene.popleft();key=tuple(row['rgbd_frame_key'])
                if row['stamp_ns']<=start or key in seen:continue
                seen.add(key);raw.append(row)
                if not (row['detection_valid'] and row['full_scene_coverage']):raise RuntimeError('incomplete scene')
                if not row['base_transform_valid']:continue
                if not 0<=(self.now()-row['stamp_ns'])*1e-9<=.5:continue
                frames.append(row)
            if len(frames)>=20 and (frames[-1]['stamp_ns']-frames[0]['stamp_ns'])*1e-9>=2:break
        write(self.out/('fresh-window-'+str(time.time_ns())+'.json'),raw)
        # Missing base transforms are never observations. The original qualifier
        # still enforces <=300ms gaps, same epoch, all targets, labels and drift.
        return qualified_target(self.demo,seed,frames)
    def move(self,op,target,profile,before_send=None):
        self.c['profile']=profile
        preview=self.probe(op+'-trial',op,target)
        if preview['status']!=4 or preview['result']['code']!=0:raise RuntimeError(preview['result']['detail'])
        self.stopped();self.check_parameters();self.gripper_state(held=self.held,idle=True)
        if before_send is not None:before_send()
        g=MoveArm.Goal();g.operation=getattr(g,op);g.profile=profile;g.plan_only=False
        g.target.header.frame_id='base_link';g.target.header.stamp=self.n.get_clock().now().to_msg()
        g.target.pose.position.x,g.target.pose.position.y,g.target.pose.position.z=target
        q=self.demo['orientation_xyzw'];g.target.pose.orientation.x,g.target.pose.orientation.y,g.target.pose.orientation.z,g.target.pose.orientation.w=q
        self.pending=self.client.send_goal_async(g);self.goal=self.wait(self.pending,8);self.pending=None
        if not self.goal.accepted:raise RuntimeError('arm goal rejected')
        self.report['physical_stop_confirmed']=False;self.terminal=self.goal.get_result_async();reply=self.wait(self.terminal,60,monitor=True)
        self.report['actions'].append(dict(operation=op,profile=profile,requested_tcp_m=target,plan_only=False,status=reply.status,result=convert(reply.result)))
        if reply.status!=4 or reply.result.code!=1 or not reply.result.executed or not reply.result.final_tcp_valid:raise RuntimeError(reply.result.detail)
        self.goal=self.terminal=None;s,j,t=self.stopped()
        if np.linalg.norm(position(s)-target)>.001:raise RuntimeError('endpoint mismatch')
        o=s.tcp.pose.orientation
        if 2*math.acos(float(np.clip(abs(np.array([o.x,o.y,o.z,o.w])@np.array(q)),0,1)))>.005:raise RuntimeError('orientation mismatch')
    def radius(self,seed):
        # Ray/observed top-plane intersection, using the same RGBD frame.
        latest=self.latest_scene
        if latest is None:raise RuntimeError('no current piece outline')
        matches=[d for d in latest['detections'] if d.get('base_geometry_valid') and distance(d['base_surface_center_m'],seed['base_surface_center_m'])<.012]
        if len(matches)!=1:raise RuntimeError('outline association ambiguous')
        d=matches[0];K=np.array(latest['source_K']).reshape(3,3);center=np.array(d['camera_surface_center_m']);normal=np.array(d['camera_surface_normal'])
        outline=d['mask_outline_xy'];radii=[]
        for u,v in outline:
            ray=np.linalg.solve(K,np.array([u,v,1.]));point=ray*(normal@center)/(normal@ray);radii.append(np.linalg.norm(point-center))
        radius=max(radii)
        if not .005<=radius<=.020:raise RuntimeError('observed piece radius outside 5..20mm trial envelope')
        self.report['observed_top_radius_m']=radius
    def close(self):self.grip_client.destroy();super().close()

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',required=True);p.add_argument('--scan',required=True);p.add_argument('--height',required=True);p.add_argument('--output',required=True);p.add_argument('--authorization',required=True);p.add_argument('--seed',required=True);a=p.parse_args()
    cfg=json.loads(Path(a.config).read_text());cfg['neighbor_clearance_m']=.110
    scan=json.loads(Path(a.scan).read_text());h=json.loads(Path(a.height).read_text());out=Path(a.output);out.mkdir(exist_ok=False)
    if not scan['returned_home'] or scan['status']=='FAULT':raise RuntimeError('scan must return Home')
    seeds=[t for t in scan['target_hypotheses'] if t['scan_target_id']==a.seed and t['class_name']!='unknown' and t['association_state']=='within_view_unique' and distance(t['base_surface_center_m'][:2],h['removed_seed_xy_m'])>.030 and all(t is other or distance(t['base_surface_center_m'][:2],other['base_surface_center_m'][:2])>.110 for other in scan['target_hypotheses'])]
    if len(seeds)!=1:raise RuntimeError('exactly one isolated new selected seed required')
    seed=seeds[0];rclpy.init();io=Trial(cfg,out)
    signal.signal(signal.SIGINT,lambda *_:setattr(io,'abort',True));signal.signal(signal.SIGTERM,lambda *_:setattr(io,'abort',True))
    io.report.update(authorization=a.authorization,search_seed=seed,provisional_height=h,status='PRECHECK')
    try:
        io.stopped();io.check_parameters()
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            io.spin()
            if io.gripper and 0<=(io.now()-stamp(io.gripper))*1e-9<=.2 and io.gripper.feedback_valid and 0<=io.gripper.feedback_age_sec<=.5:break
        s=io.gripper_state(idle=True)
        if s.object_detected or s.grip_status!=1:raise RuntimeError('initial empty gripper required')
        if not io.grip_client.wait_for_server(timeout_sec=5):raise RuntimeError('gripper action unavailable')
        # This commissioning scope creates grip evidence; it does not claim full
        # calibrated stack workflow readiness or bypass that workflow's precheck.
        io.prechecked=True
        io.gripper_move(1000,False)
        view=next(v for v in io.c['views'] if v['id']==seed['view_id'])
        current=position(io.pair()[0])
        if not (.090<=current[2]<=.110 and distance(current[:2],seed['base_surface_center_m'][:2])<.012):
            io.move('PREGRASP',view['tcp_position_m'],'roundtrip_low')
        measured=io.fresh(seed)
        hover=[*measured['base_surface_center_m'][:2],.100]
        io.move('PREGRASP',hover,'roundtrip_low')
        measured=io.fresh(measured)
        io.move('PREGRASP', [*measured['base_surface_center_m'][:2],.100],'roundtrip_low')
        measured=io.fresh(measured);io.radius(measured)
        current=position(io.pair()[0])
        if distance(current[:2],measured['base_surface_center_m'][:2])>.003:raise RuntimeError('alignment error exceeds 3mm')
        target=[*current[:2].tolist(),h['nominal_grasp_tcp_base_z_m']]
        io.c['profile']='table_contact';preview=io.probe('descent-before-fresh-check','DESCEND',target)
        if preview['status']!=4 or preview['result']['code']!=0:raise RuntimeError(preview['result']['detail'])
        def refresh():
            check=io.fresh(measured)
            if (check['scene_identity'][1:]!=measured['scene_identity'][1:] or
                    distance(check['base_surface_center_m'],measured['base_surface_center_m'])>.003 or
                    not 0<=(io.now()-check['last_stamp_ns'])*1e-9<=.5):raise RuntimeError('fresh target changed/expired')
            io.report['fresh_target']=check
        io.move('DESCEND',target,'table_contact',before_send=refresh)
        io.report['status']='GRIP_ATTEMPT';io.gripper_move(0,True);io.held=True
        # Retention under stationary load before any lift.
        end=time.monotonic()+1.
        while time.monotonic()<end:io.spin();io.gripper_state(held=True,idle=True)
        lift=position(io.pair()[0]).tolist();lift[2]+=.020
        io.move('RETREAT',lift,'table_contact')
        end=time.monotonic()+1.
        while time.monotonic()<end:io.spin();io.gripper_state(held=True,idle=True)
        io.report['status']='GRASPED_LIFTED_20MM_HOLD';io.report['held']=True
        print(json.dumps(dict(status=io.report['status'],class_name=seed['class_name'],target_tcp_m=target,lift_tcp_m=lift)),flush=True)
    except BaseException as e:
        io.report.update(status='FAULT',error=str(e),held=io.held);io.cancel_owned();raise
    finally:
        io.report.update(arm_latest=convert(io.pair()[0]),joint_latest=convert(io.pair()[1]),gripper_latest=convert(io.gripper) if io.gripper else None)
        write(out/'trial-result.json',io.report);io.close();rclpy.shutdown()

if __name__=='__main__':main()
