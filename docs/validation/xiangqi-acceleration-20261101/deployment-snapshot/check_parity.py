from pathlib import Path
import json
import numpy as np
from scipy.optimize import linear_sum_assignment

O=Path(__file__).resolve().parent;results=[]
for case in range(2):
    a=np.load(O/'parity-pt'/(str(case)+'.npz'));b=np.load(O/'parity-trt'/(str(case)+'.npz'))
    pa,pb=a['predictions'],b['predictions'];assert len(pa)==len(pb),'Raw target count differs'
    score=np.zeros((len(pa),len(pb)))
    for i,x in enumerate(pa):
        for j,y in enumerate(pb):
            wh=np.maximum(0,np.minimum(x[2:4],y[2:4])-np.maximum(x[:2],y[:2]));inter=float(wh.prod());union=float(np.prod(x[2:4]-x[:2])+np.prod(y[2:4]-y[:2])-inter);score[i,j]=inter/max(1e-9,union)
    ri,ci=linear_sum_assignment(1-score)
    mask_ious=[];classes=[];boxes=[]
    for i,j in zip(ri,ci):
        assert score[i,j]>.99
        classes.append(int(pa[i,-1])==int(pb[j,-1]));boxes.append(float(np.max(abs(pa[i,:4]-pb[j,:4]))))
        ma,mb=a['masks'][i].astype(bool),b['masks'][j].astype(bool)
        mask_ious.append(float(np.sum(ma&mb))/max(1,int(np.sum(ma|mb))))
    assert all(classes),'Returned target class differs'
    assert all(v>=.99 for v in mask_ious),'Mask changed materially'
    ra=json.loads((O/'parity-pt'/(str(case)+'.json')).read_text());rb=json.loads((O/'parity-trt'/(str(case)+'.json')).read_text())
    assert ra['deduplicated_count']==rb['deduplicated_count']
    da=sorted(ra['detections'],key=lambda d:d['center_pixel']);db=sorted(rb['detections'],key=lambda d:d['center_pixel'])
    deltas=[]
    for x,y in zip(da,db):
        assert x['class_name']==y['class_name'] and x['valid']==y['valid'] and x['camera_geometry_valid']==y['camera_geometry_valid']
        for field in ['camera_surface_center_m','base_surface_center_m']:
            if field in x or field in y:
                assert field in x and field in y
                delta=float(np.linalg.norm(np.array(x[field])-np.array(y[field])));assert delta<1e-6;deltas.append(delta)
    results.append(dict(case=case,matched_targets=len(ri),minimum_mask_iou=min(mask_ious) if mask_ious else None,maximum_box_delta_px=max(boxes) if boxes else None,maximum_position_delta_m=max(deltas) if deltas else None))
result=dict(passed=True,cases=results,scope='Same fresh saved image in full/ROI; not labelled accuracy acceptance')
assert sum(r['matched_targets'] for r in results)>=3,'Insufficient visible targets for a useful detector check'
(O/'parity-result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
