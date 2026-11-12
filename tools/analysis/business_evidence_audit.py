"""Independent offline margins for exact business trajectories and feedback."""
import argparse,csv,gzip,json,pathlib,subprocess
import numpy as np
p=argparse.ArgumentParser();p.add_argument('run',type=pathlib.Path)
p.add_argument('--evaluator',type=pathlib.Path,required=True)
p.add_argument('--project-root',type=pathlib.Path,default=pathlib.Path(__file__).resolve().parents[2]);a=p.parse_args()
root=a.project_root;run=a.run
result=json.loads((run/'results.json').read_text());home=np.array(json.loads((root/'config/arm/home_reference.json').read_text())['joint_positions_rad'])
sets={}
for path in sorted(run.glob('*_trajectory.json.gz')):
    with gzip.open(path,'rt') as f:sets[path.name.removesuffix('_trajectory.json.gz')]=np.array(json.load(f)['q'])
rows=list(csv.DictReader((run/'joint_feedback.csv').open()))
if rows:sets['measured_feedback']=np.array([[float(r[f'q{i}_rad']) for i in range(1,7)] for r in rows])
summary={}
for label,q in sets.items():
    q.astype('<f8').tofile(run/'margin_q.bin')
    subprocess.run([str(a.evaluator),str(root/'src/chess_robot_arm/config/nova5_ag95_geometry.yaml'),str(run/'margin_q.bin'),str(run/'margin_metrics.bin')],check=True,stdout=subprocess.DEVNULL)
    m=np.fromfile(run/'margin_metrics.bin',dtype='<f8').reshape(-1,12)
    item={'samples':len(q),'tcp_min_m':m[:,:3].min(0).tolist(),'tcp_max_m':m[:,:3].max(0).tolist(),
          'table_clearance_lower_bound_m':float(m[:,4].min()+.01),
          'joint_limit_margin_rad':float(m[:,5].min()),'sigma_min':float(m[:,6].min()),
          'condition_max':float(m[:,7].max()),'self_clearance_lower_bound_m':float(m[:,11].min()),
          'max_joint_distance_home_rad':float(np.max(abs(q-home))),
          'table_pass':bool(np.all(m[:,9]==1)),'self_pass':bool(np.all(m[:,10]==1))}
    assert item['table_pass'] and item['self_pass'] and item['joint_limit_margin_rad']>=np.pi/36 and item['sigma_min']>=.05 and item['condition_max']<=80,item
    summary[label]=item
if rows:
    summary['feedback_stats']={'frames':len(rows),'peak_speed_rad_s':max(abs(float(r[f'dq{i}_rad_s'])) for r in rows for i in range(1,7)),
                               'peak_abs_net_twist_rad':max(abs(float(r['net_twist_rad'])) for r in rows),
                               'peak_total_twist_rad':max(float(r['total_twist_rad']) for r in rows)}
(run/'margins.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
