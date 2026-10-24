"""Offline Nova5 workspace, ground-envelope and Jacobian analysis.

No robot connections, ROS publishers or motion commands. Python orchestrates
geometry and plots; a small Eigen kernel evaluates sampled configurations.
"""
import argparse
import json
import math
import pathlib
import subprocess
import shlex
import sys
import time
import numpy as np
import yaml
from scipy.spatial import ConvexHull
from scipy.stats import qmc


def save(path,data):
    path.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n')


class Study:
    def __init__(self,project,out,ground=-.01,clearance=.02,descent_speed=.01):
        self.project=project;self.out=out;out.mkdir(parents=True,exist_ok=True)
        sys.path.insert(0,str(project/'tools/description'))
        from geometry_report import RobotGeometry
        from rigid_geometry import origin
        self.model=RobotGeometry(project/'src/chess_robot_description/urdf/nova5_with_ag95.urdf',[project/'src'])
        self.home=np.array(json.loads((project/'config/arm/home_reference.json').read_text())['joint_positions_rad'])
        self.joints=[self.model.joints[f'joint{i}'] for i in range(1,7)]
        self.origins=np.array([origin(j.find('origin')) for j in self.joints])
        self.lower=np.array([float(j.find('limit').get('lower')) for j in self.joints])
        self.upper=np.array([float(j.find('limit').get('upper')) for j in self.joints])
        self.axes=np.array([np.fromstring(j.find('axis').get('xyz'),sep=' ') for j in self.joints]);self.axes/=np.linalg.norm(self.axes,axis=1)[:,None]
        if not np.allclose(self.axes,[0,0,1],atol=1e-12):raise ValueError('analytic down-pose solver requires the audited local-Z six-axis chain')
        poses=self.model.transforms(dict(zip([f'joint{i}' for i in range(1,7)],self.home)))
        self.tcp=np.linalg.inv(poses['Link6'])@poses['grasp_link']
        if not np.allclose(self.tcp[:3,3],[0,0,.2],atol=1e-9):raise ValueError('rederive down-pose solver for a nonaxial TCP')
        self.home_flange=self.fk(self.home[None,:])[0][0]
        self.ground=ground;self.clearance=clearance
        self.joint_margin=math.radians(5);self.sigma_threshold=.05;self.condition_limit=80.;self.descent_speed=descent_speed
        self.L=.85
        self.hulls=self.make_hulls()
        config=dict(joint_origins=self.origins.reshape(6,16).tolist(),axes=self.axes.tolist(),lower=self.lower.tolist(),upper=self.upper.tolist(),tcp=self.tcp.reshape(16).tolist(),hulls=self.hulls,ground_z_m=ground,clearance_m=clearance,characteristic_length_m=self.L)
        (out/'model.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
        self.binary=out/'workspace_eval'
        fcl_flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','fcl'],text=True));fcl_flags=[f for f in fcl_flags if not f.startswith('-std=')]
        subprocess.run(['g++','-O3','-DNDEBUG','-std=c++17',str(project/'tools/analysis/workspace_eval.cpp'),'-lyaml-cpp',*fcl_flags,'-o',str(self.binary)],check=True)
        self.verify()

    def fk(self,q):
        q=np.asarray(q);n=len(q);t=np.broadcast_to(np.eye(4),(n,4,4)).copy();frames=[t.copy()];positions=[];axes=[]
        for i in range(6):
            before=t@self.origins[i];positions.append(before[:,:3,3].copy());axes.append(before[:,:3,2].copy())
            r=np.broadcast_to(np.eye(4),(n,4,4)).copy();c=np.cos(q[:,i]);s=np.sin(q[:,i]);r[:,0,0]=c;r[:,0,1]=-s;r[:,1,0]=s;r[:,1,1]=c
            t=before@r;frames.append(t.copy())
        return t,t@self.tcp,np.stack(frames,axis=1),np.stack(positions,axis=1),np.stack(axes,axis=1)

    def make_hulls(self):
        # The minimum of a linear height functional over the STL occurs at a
        # vertex; its convex hull preserves that exact ground extreme.
        result=[]
        def add(points,link,name):
            hull=ConvexHull(points);indices=hull.vertices;mapping={int(old):new for new,old in enumerate(indices)};faces=[]
            for simplex,equation in zip(hull.simplices,hull.equations):
                ids=list(map(int,simplex));a,b,c=points[ids]
                if np.dot(np.cross(b-a,c-a),equation[:3])<0:ids[1],ids[2]=ids[2],ids[1]
                faces.append([mapping[i] for i in ids])
            result.append(dict(link=link,name=name,vertices=points[indices].tolist(),faces=faces))
        for i in range(7):
            name='base_link' if i==0 else f'Link{i}';meshes=self.model.geometry[name]['collision']
            points=np.vstack([m.vertices for m in meshes]);add(points,i,name)
        distal=[]
        for opening in np.linspace(0,.94,21):
            poses=self.model.transforms({'left_outer_knuckle_joint':float(opening)})
            inverse=np.linalg.inv(poses['Link6'])
            for name,geometries in self.model.geometry.items():
                if name=='base_link' or name in [f'Link{i}' for i in range(1,7)]:continue
                relative=inverse@poses[name]
                for mesh in geometries['collision']:distal.append(mesh.vertices@relative[:3,:3].T+relative[:3,3])
        points=np.vstack(distal);add(points,6,'camera_and_gripper_21_state_union')
        return result

    def evaluate(self,q,label):
        q=np.asarray(q,dtype='<f8');source=self.out/(label+'_q.bin');target=self.out/(label+'_metrics.bin');q.tofile(source)
        subprocess.run([str(self.binary),str(self.out/'model.yaml'),str(source),str(target)],check=True)
        return np.fromfile(target,dtype='<f8').reshape(-1,12)

    def precise_ground(self,q):
        frames=self.fk(np.asarray(q)[None,:])[2][0];values={}
        for hull in self.hulls:
            if hull['link']==0:continue
            t=frames[hull['link']];vertices=np.array(hull['vertices']);values[hull['name']]=float((vertices@t[2,:3]+t[2,3]).min())
        return values

    def jacobian(self,q):
        _,tcp,_,positions,axes=self.fk(np.asarray(q)[None,:]);linear=np.cross(axes[0],tcp[0,:3,3]-positions[0]);return np.r_[linear.T,axes[0].T]

    def verify(self):
        from scipy.spatial.transform import Rotation
        rng=np.random.default_rng(428);q=rng.uniform(self.lower,self.upper,(12,6));max_fk=0.;max_j=0.
        for row in q:
            expected=self.model.transforms(dict(zip([f'joint{i}' for i in range(1,7)],row)))['grasp_link'];actual=self.fk(row[None,:])[1][0];max_fk=max(max_fk,float(np.max(np.abs(actual-expected))))
            analytic=self.jacobian(row);numerical=np.zeros((6,6));eps=1e-6
            for j in range(6):
                plus=row.copy();minus=row.copy();plus[j]+=eps;minus[j]-=eps
                a=self.fk(plus[None,:])[1][0];b=self.fk(minus[None,:])[1][0]
                numerical[:3,j]=(a[:3,3]-b[:3,3])/(2*eps);numerical[3:,j]=Rotation.from_matrix(a[:3,:3]@b[:3,:3].T).as_rotvec()/(2*eps)
            max_j=max(max_j,float(np.max(np.abs(analytic-numerical))))
        assert max_fk<1e-10 and max_j<1e-7,(max_fk,max_j)
        special=np.repeat(self.home[None,:],3,axis=0);special[1,4]=0;special[2,2]=0
        metrics=self.evaluate(special,'verification')
        for i,row in enumerate(special):
            J=self.jacobian(row);J[:3]/=self.L;singular=np.linalg.svd(J,compute_uv=False)
            assert abs(metrics[i,6]-singular[-1])<1e-10
        assert metrics[1,6]<1e-5 and metrics[2,6]<1e-5,metrics.tolist()
        assert metrics[0,10]>.5,'conservative self-collision envelope unexpectedly rejects Home'
        self.verification=dict(fk_max_abs_error=max_fk,jacobian_max_abs_error=max_j,home_metrics=metrics[0].tolist(),wrist_singular_metrics=metrics[1].tolist(),elbow_singular_metrics=metrics[2].tolist(),home_precise_moving_geometry_min_z=self.precise_ground(self.home))
        save(self.out/'verification.json',self.verification)

    @staticmethod
    def wrap(q,reference=0.):return reference+np.remainder(q-reference+np.pi,2*np.pi)-np.pi

    def inverse_down(self,points,wrist,shoulder,elbow):
        # Audited Nova/UR topology: axes 2/3/4 are parallel. With fixed downward
        # wrist, their sum is fixed, leaving a planar 2R position problem.
        p=np.asarray(points);n=len(p);constant=self.home[1:4].sum()-(np.pi if wrist else 0)
        seed=np.array([0.,0.,0.,constant,np.pi/2 if wrist else -np.pi/2,0.])
        A=self.origins[2,:3,3];B=self.origins[3,:3,3];R0=(self.origins[0]@self.origins[1])[:3,:3]
        C0=self.fk(seed[None,:])[1][0,:3,3]-R0@np.array([A[0]+B[0],A[1]+B[1],0.])
        normal=R0[:,2];coefficient_a=p[:,0]*normal[0]+p[:,1]*normal[1];coefficient_b=-p[:,0]*normal[1]+p[:,1]*normal[0]
        amplitude=np.hypot(coefficient_a,coefficient_b);rhs=np.dot(normal,C0)-normal[2]*p[:,2]
        valid=(amplitude>1e-12)&(np.abs(rhs)<=amplitude+1e-12)
        q1=np.arctan2(coefficient_b,coefficient_a)+(1 if shoulder else -1)*np.arccos(np.clip(rhs/np.maximum(amplitude,1e-12),-1,1));q1=self.wrap(q1,self.home[0])
        c=np.cos(q1);s=np.sin(q1);Rz=np.zeros((n,3,3));Rz[:,0,0]=c;Rz[:,0,1]=-s;Rz[:,1,0]=s;Rz[:,1,1]=c;Rz[:,2,2]=1
        R=Rz@R0;C=Rz@C0;planar=np.einsum('nji,nj->ni',R,p-C)
        a=np.linalg.norm(A[:2]);b=np.linalg.norm(B[:2]);alpha=np.arctan2(A[1],A[0]);delta=np.arctan2(B[1],B[0])-alpha
        cos_elbow=(np.sum(planar[:,:2]**2,axis=1)-a*a-b*b)/(2*a*b);valid&=(np.abs(cos_elbow)<=1+1e-10)
        angle=(1 if elbow else -1)*np.arccos(np.clip(cos_elbow,-1,1));q3=self.wrap(angle-delta)
        q2=self.wrap(np.arctan2(planar[:,1],planar[:,0])-np.arctan2(b*np.sin(angle),a+b*np.cos(angle))-alpha)
        q=np.zeros((n,6));q[:,0]=q1;q[:,1]=q2;q[:,2]=q3;q[:,3]=self.wrap(constant-q2-q3,self.home[3]);q[:,4]=seed[4]
        flange=self.fk(q)[0];D=np.einsum('nji,jk->nik',flange[:,:3,:3],self.home_flange[:3,:3]);q[:,5]=self.wrap(np.arctan2(D[:,1,0],D[:,0,0]),self.home[5])
        valid&=((q>=self.lower)&(q<=self.upper)).all(axis=1)
        flange,tcp,*_=self.fk(q);position=np.linalg.norm(tcp[:,:3,3]-p,axis=1)
        # Frobenius difference avoids branch-dependent Euler angles and a fragile
        # trace expression; small URDF CAD rounding creates ~1e-5 rad residuals.
        rotation=np.linalg.norm(flange[:,:3,:3]-self.home_flange[:3,:3],axis=(1,2))/np.sqrt(2)
        valid&=(position<1e-4)&(rotation<1e-4)
        return q,valid,position,rotation

    def run(self,step=.025,raw_power=18):
        start=time.monotonic();raw=qmc.Sobol(6,scramble=True,seed=428).random_base2(raw_power);raw=self.lower+raw*(self.upper-self.lower)
        raw_metrics=self.evaluate(raw,'general');save(self.out/'run_progress.json',{'phase':'general_complete'})
        x=np.arange(-1.1,1.100001,step);y=x.copy();z=np.arange(0,1.400001,step)
        grid=np.stack(np.meshgrid(x,y,z,indexing='ij'),axis=-1);p=grid.reshape(-1,3);shape=grid.shape[:-1];N=len(p)
        reachable=np.zeros(N,bool);floor_reachable=np.zeros(N,bool);healthy=np.zeros(N,bool);corridor=np.zeros(N,bool);best_sigma=np.zeros(N);best_margin=np.zeros(N);best_gain=np.full(N,np.inf)
        best_q=np.full((N,6),np.nan);branch_counts=[];audit_candidates=[];max_residual=[0.,0.]
        k=int(round(.25/step));assert abs(k*step-.25)<1e-9
        for branch in range(8):
            wrist,shoulder,elbow=(branch>>2)&1,(branch>>1)&1,branch&1
            q,valid,position,rotation=self.inverse_down(p,wrist,shoulder,elbow);idx=np.flatnonzero(valid);metrics=self.evaluate(q[idx],f'branch_{branch}')
            if len(idx):max_residual=[max(max_residual[0],float(position[idx].max())),max(max_residual[1],float(rotation[idx].max()))]
            floor=metrics[:,9]>.5;floor_reachable[idx[floor]]=True
            clear=floor&(metrics[:,10]>.5);good=clear&(metrics[:,5]>=self.joint_margin)&(metrics[:,6]>=self.sigma_threshold)&(metrics[:,7]<=self.condition_limit)&(metrics[:,8]*self.descent_speed<=.25)
            reachable[idx[clear]]=True;healthy[idx[good]]=True
            improve=clear&(metrics[:,6]>best_sigma[idx]);best_sigma[idx[improve]]=metrics[improve,6];best_q[idx[improve]]=q[idx[improve]];best_margin[idx[improve]]=metrics[improve,5];best_gain[idx[improve]]=metrics[improve,8]
            b_good=np.zeros(N,bool);b_good[idx[good]]=True;b_good=b_good.reshape(shape)
            qgrid=q.reshape((*shape,6));continuous=np.zeros(shape,bool);continuous[:,:,:-1]=np.max(np.abs(np.diff(qgrid,axis=2)),axis=3)<.5
            # Same analytic shoulder/elbow/wrist branch through every 25 mm
            # height sample; this is an offline corridor screen, not a 2 mm
            # execution trajectory or a certified continuous collision proof.
            b_corridor=np.zeros(shape,bool);b_corridor[:,:,:-k]=True
            for offset in range(k+1):b_corridor[:,:,:-k]&=b_good[:,:,offset:shape[2]-k+offset]
            for offset in range(k):b_corridor[:,:,:-k]&=continuous[:,:,offset:shape[2]-k+offset]
            corridor|=b_corridor.reshape(-1)
            unsafe=np.flatnonzero((metrics[:,9]<.5)&(p[idx,2]>=.015))
            if len(unsafe):audit_candidates.append(dict(branch=branch,q=q[idx[unsafe[0]]].tolist(),tcp=p[idx[unsafe[0]]].tolist(),reason='TCP_above_table_but_geometry_hits_margin'))
            branch_counts.append(dict(branch=branch,valid_pose=len(idx),ground_clear=int(floor.sum()),self_clear=int(clear.sum()),healthy=int(good.sum()),corridor=int(b_corridor.sum())))
            save(self.out/'run_progress.json',{'phase':'branch_complete','branches':branch_counts})
        np.savez_compressed(self.out/'workspace_data.npz',x=x,y=y,z=z,reachable=reachable.reshape(shape),floor_reachable=floor_reachable.reshape(shape),healthy=healthy.reshape(shape),corridor=corridor.reshape(shape),best_sigma=best_sigma.reshape(shape),best_margin=best_margin.reshape(shape),best_gain=best_gain.reshape(shape),best_q=best_q.reshape((*shape,6)),raw_q=raw,raw_metrics=raw_metrics)
        summary=dict(ground_z_m=self.ground,clearance_m=self.clearance,static_base_excluded=True,self_collision_method='FCL conservative convex envelopes; >=10mm separation between nonadjacent groups; fixed/end-effector and adjacent groups excluded',gripper_samples=21,grid_step_m=step,grid_shape=list(shape),grid_targets=N,general_samples=len(raw),general_ground_clear=int(raw_metrics[:,9].sum()),general_self_clear=int(np.sum((raw_metrics[:,9]>.5)&(raw_metrics[:,10]>.5))),floor_only_down_targets=int(floor_reachable.sum()),ground_and_self_clear_down_targets=int(reachable.sum()),healthy_down_targets=int(healthy.sum()),corridor_down_targets=int(corridor.sum()),branches=branch_counts,max_accepted_ik_position_residual_m=max_residual[0],max_accepted_ik_rotation_residual_rad_approx=max_residual[1],characteristic_length_m=self.L,sigma_threshold=self.sigma_threshold,condition_limit=self.condition_limit,joint_margin_deg=5,reference_descent_speed_m_s=self.descent_speed,configured_joint_speed_cap_rad_s=.25,elapsed_s=time.monotonic()-start,audit_candidates=audit_candidates,home_tcp=self.fk(self.home[None,:])[1][0].tolist())
        for name,mask in [('down',reachable),('healthy',healthy),('corridor',corridor)]:
            selected=p[mask];summary[name+'_bounds_m']=[selected.min(axis=0).tolist(),selected.max(axis=0).tolist()] if len(selected) else None
            radial=np.hypot(selected[:,0],selected[:,1]);summary[name+'_radial_range_m']=[float(radial.min()),float(radial.max())] if len(selected) else None
        save(self.out/'summary.json',summary)
        return summary


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--project',type=pathlib.Path,required=True);parser.add_argument('--out',type=pathlib.Path,required=True);parser.add_argument('--step',type=float,default=.025);parser.add_argument('--raw-power',type=int,default=18);parser.add_argument('--probe-only',action='store_true');parser.add_argument('--ground-z',type=float,default=-.01);parser.add_argument('--clearance',type=float,default=.02);parser.add_argument('--descent-speed',type=float,default=.01);args=parser.parse_args()
    if not 0<args.step<=.05 or not 0<=args.clearance<=.1 or not 0<args.descent_speed<=.1 or not 1<=args.raw_power<=22:parser.error('invalid analysis resolution/margins/speed')
    if not np.isfinite([args.ground_z,args.clearance,args.descent_speed,args.step]).all():parser.error('finite parameters required')
    if abs(round(.25/args.step)*args.step-.25)>1e-9:parser.error('step must divide the configured 250 mm approach clearance')
    study=Study(args.project.resolve(),args.out.resolve(),args.ground_z,args.clearance,args.descent_speed)
    home_matches=0
    for branch in range(8):
        q,valid,position,rotation=study.inverse_down(study.fk(study.home[None,:])[1][:,:3,3],(branch>>2)&1,(branch>>1)&1,branch&1)
        home_matches+=int(valid.sum())
        print('Home IK probe',branch,q.tolist(),valid.tolist(),position.tolist(),rotation.tolist(),flush=True)
    assert home_matches>0,'analytic IK did not reconstruct Home'
    if args.probe_only:return
    summary=study.run(args.step,args.raw_power);print(json.dumps(summary,indent=2),flush=True)


if __name__=='__main__':main()
