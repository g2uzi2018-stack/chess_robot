"""Independent numerical checks of generated workspace outputs (offline)."""
import argparse,json,pathlib,sys
import numpy as np
import yaml

p=argparse.ArgumentParser();p.add_argument('--project',type=pathlib.Path,required=True);p.add_argument('--run',type=pathlib.Path,required=True);a=p.parse_args()
sys.path.insert(0,str(a.project/'tools/analysis'));from workspace_analysis import Study
model=yaml.safe_load((a.run/'model.yaml').read_text());s=Study.__new__(Study);s.origins=np.array(model['joint_origins']).reshape(6,4,4);s.tcp=np.array(model['tcp']).reshape(4,4);s.hulls=model['hulls']
data=np.load(a.run/'workspace_data.npz');q=data['raw_q'];metrics=data['raw_metrics'];rng=np.random.default_rng(915)
indices=rng.choice(len(q),64,replace=False);fk_error=0.;floor_matches=0
for i in indices:
    tcp=s.fk(q[i:i+1])[1][0,:3,3];fk_error=max(fk_error,float(abs(tcp-metrics[i,:3]).max()))
    exact=min(s.precise_ground(q[i]).values());floor_matches+=int((exact>=model['ground_z_m']+model['clearance_m'])==bool(metrics[i,9]))
assert fk_error<1e-10 and floor_matches==len(indices),(fk_error,floor_matches)
reachable=data['reachable'];healthy=data['healthy'];corridor=data['corridor'];assert np.all(~healthy|reachable);assert np.all(~corridor|healthy)
shift=int(round(.25/json.loads((a.run/'summary.json').read_text())['grid_step_m']))
assert np.all(~corridor[:,:,:-shift]|healthy[:,:,shift:])
rows=json.loads((a.run/'policy_probe.json').read_text())['cases'];singular=[r for r in rows if r['kind']=='elbow_singularity' and r['accepted']]
result=dict(status='PASS',sampled_ground_checks=len(indices),ground_classifier_agreement=floor_matches,cpp_python_fk_max_abs_error_m=fk_error,subset_invariants=True,production_planner_singular_endpoints_accepted=len(singular),not_hardware_validation=True)
(a.run/'analysis_checks.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
