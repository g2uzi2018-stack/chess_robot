"""Offline one-joint singularity diagnostic sweeps; no hardware interfaces."""
import argparse,json,pathlib,subprocess
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--project',type=pathlib.Path,required=True);p.add_argument('--run',type=pathlib.Path,required=True);a=p.parse_args()
h=np.array(json.loads((a.project/'config/arm/home_reference.json').read_text())['joint_positions_rad']);q=np.tile(h,(802,1))
q[:401,2]=np.linspace(-2.79,2.79,401);q[:401,3]=h[1:4].sum()-q[:401,1]-q[:401,2]
q[401:,4]=np.linspace(-np.pi,np.pi,401);source=a.run/'singularity_sweep_q.bin';target=a.run/'singularity_sweep_metrics.bin';q.tofile(source)
subprocess.run([str(a.run/'workspace_eval'),str(a.run/'model.yaml'),str(source),str(target)],check=True)
