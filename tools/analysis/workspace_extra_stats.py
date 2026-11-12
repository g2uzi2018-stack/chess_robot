import json,pathlib,sys
import numpy as np

run=pathlib.Path(sys.argv[1]);d=np.load(run/'workspace_data.npz');x,y,z=[d[k] for k in ['x','y','z']]
report=json.loads((run/'policy_probe.json').read_text());metrics=np.fromfile(run/'policy_case_metrics.bin').reshape(-1,12)
for row,m in zip(report['cases'],metrics):row.update(tcp_xyz_m=m[:3].tolist(),moving_geometry_z_evidence_m=float(m[4]),sigma_min=float(m[6]),condition=float(m[7]),ground_margin_pass=bool(m[9]))
(run/'policy_probe.json').write_text(json.dumps(report,indent=2)+'\n')
rect_x=(x>=-.250001)&(x<=.250001);rect_y=(y>=-.650001)&(y<=-.249999);k=int(np.argmin(abs(z-.025)))
mask=np.ix_(rect_x,rect_y,[k]);coverage=d['corridor'][mask]
stats={'candidate_board_rectangle':{'x_m':[-.25,.25],'y_m':[-.65,-.25],'tcp_z_m':float(z[k]),'size_m':[.5,.4],'tested_grid_points':int(coverage.size),'corridor_pass_points':int(coverage.sum()),'scope':'25mm endpoint/corridor samples only, fixed Home orientation; no transfer path from Home or physical certification'}}
for key in ['reachable','healthy','corridor']:
    a=d[key];eligible=np.zeros_like(a);eligible[np.ix_(np.abs(x)<=.800001,np.abs(y)<=.800001,z<=1.200001)]=True
    stats[key+'_inside_current_workspace_box']=int((a&eligible).sum())
(run/'extra_stats.json').write_text(json.dumps(stats,indent=2)+'\n');print(json.dumps(stats,indent=2))
