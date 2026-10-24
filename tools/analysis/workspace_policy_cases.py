"""Produce offline counterexamples and run the actual current Planner."""
import argparse,json,pathlib,subprocess
import numpy as np

p=argparse.ArgumentParser();p.add_argument('--project',type=pathlib.Path,required=True);p.add_argument('--baseline',type=pathlib.Path,required=True);p.add_argument('--out',type=pathlib.Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
home=np.array(json.loads((a.project/'config/arm/home_reference.json').read_text())['joint_positions_rad']);c=home[1:4].sum()
cases=[];labels=[]
for angle in np.linspace(0,1.5,9):
    q=home.copy();q[1]=angle;q[2]=0.;q[3]=c-angle;cases.append(q);labels.append('elbow_singularity')
for branch in range(8):
    source=np.fromfile(a.baseline/f'branch_{branch}_q.bin').reshape(-1,6);metrics=np.fromfile(a.baseline/f'branch_{branch}_metrics.bin').reshape(len(source),-1)
    mask=(metrics[:,4]<-.01)&(metrics[:,2]>.015)&(metrics[:,2]<1.1)&(np.abs(metrics[:,:2])<.75).all(axis=1)
    indices=np.flatnonzero(mask);indices=indices[np.argsort(np.linalg.norm(source[indices]-home,axis=1))][:3]
    for index in indices:cases.append(source[index]);labels.append('table_penetration_pose_validation_probe')
q=np.asarray(cases);q.tofile(a.out/'policy_cases.bin')
result=subprocess.run([str(a.project/'runs/workspace_policy_probe'),str(a.project/'src/chess_robot_description/urdf/nova5_with_ag95.urdf'),str(a.out/'policy_cases.bin')],capture_output=True,text=True,check=True)
rows=[json.loads(s) for s in result.stdout.splitlines()];
for i,row in enumerate(rows):row.update(kind=labels[i],q_rad=q[i].tolist())
(a.out/'policy_probe.json').write_text(json.dumps({'scope':'offline actual production Planner with a single FK-verified known inverse pose; no SDK/network/motion','cases':rows},indent=2)+'\n');print(result.stdout)
