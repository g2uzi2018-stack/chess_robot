"""Independent offline two-model timing; does not publish or grant geometry qualification."""
import argparse,fcntl,json,signal,sys,time
from pathlib import Path
import cv2
import numpy as np

p=argparse.ArgumentParser()
p.add_argument('--candidate',type=Path,required=True)
p.add_argument('--imgsz',type=int,required=True)
p.add_argument('--conf',type=float,required=True)
p.add_argument('--captures',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
ROOT=Path('/home/kuang/workspace/chess_robot')
SOURCE=ROOT/'runs/xiangqi_candidate_roi_deploy_20261102_01/source'
sys.path.insert(0,str(SOURCE))
from candidate_roi import plan_crops,MAX_CROPS,recognize,items
from measurement_contract import producer_json_value
from recognize_tracked import Experiment

handle=(ROOT/'runs/perf_ceiling_20261031/benchmark.lock').open('a')
def timeout(*args):raise TimeoutError('GPU lease did not become available')
signal.signal(signal.SIGALRM,timeout);signal.alarm(30)
try:fcntl.flock(handle,fcntl.LOCK_EX)
finally:signal.alarm(0)
try:
    import torch
    from ultralytics import YOLO
    torch.set_num_threads(2);cv2.setNumThreads(1)
    torch.set_float32_matmul_precision('highest')
    torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
    candidate=YOLO(str(a.candidate),task='detect')
    roi_model=Experiment(camera_only=True)
    frames=[]
    for folder in sorted(a.captures.glob('[0-9][0-9][0-9]')):
        frames.append((folder.name,cv2.imread(str(folder/'color.png'))))
    input_size=[{640:384,512:288,416:256}[a.imgsz],a.imgsz]
    def detect(image):
        return candidate.predict(image,device=0,imgsz=input_size,conf=a.conf,
                                 verbose=False,half=False)[0]
    for _ in range(3):
        detect(frames[0][1]);roi_model.detect(frames[0][1],[320,0,1040,720])
    for _ in range(2):recognize(frames[0][1],roi_model.detect)
    records=[]
    for i,(name,image) in enumerate(frames):
        order=('baseline','lite') if i%2==0 else ('lite','baseline')
        for mode in order:
            start=time.monotonic()
            if mode=='baseline':
                trace=[]
                def recorded_detect(color,crop=None):
                    result=roi_model.detect(color,crop)
                    trace.append(dict(roi=crop,wall_ms=result['detector_s']*1000))
                    return result
                result=recognize(image,recorded_detect)
                elapsed=(time.monotonic()-start)*1000
                records.append(dict(sample=name,mode=mode,total_stage_ms=elapsed,
                    candidate_count=result['full_candidate_count'],
                    roi_count=len(result['classification_rois']),trace=trace))
                continue
            t=time.monotonic();result=detect(image)
            boxes=result.boxes.data.cpu().numpy()
            raw_count=len(boxes)
            proposals=items(dict(predictions=boxes,masks=None,names=dict(result.names),
                                 roi_xyxy=[0,0,image.shape[1],image.shape[0]]),'full_candidate')
            boxes=np.array([[*d['box'],d['confidence'],0] for d in proposals],dtype=float).reshape(-1,6)
            candidate_ms=(time.monotonic()-t)*1000
            anchors=np.array([[537.54,431.73],[682.20,449.88],[827.41,468.38],[955.81,490.23]])
            centers=(boxes[:,:2]+boxes[:,2:4])/2
            anchor_hits=[bool(np.any(np.linalg.norm(centers-anchor,axis=1)<35)) for anchor in anchors]
            crops=plan_crops(boxes[:,:4].tolist(),image.shape[1],image.shape[0])
            overflow=len(crops)>MAX_CROPS
            observations=[];trace=[]
            for crop in crops[:MAX_CROPS]:
                packet=roi_model.detect(image,crop)
                trace.append(dict(roi=crop,wall_ms=packet['detector_s']*1000))
                offset=np.array(crop[:2]*2)
                observations.extend(dict(box_xyxy=(row[:4]+offset).tolist(),
                    confidence=float(row[-2]),class_name=packet['names'][int(row[-1])])
                    for row in packet['predictions'])
            elapsed=(time.monotonic()-start)*1000
            records.append(dict(sample=name,mode=mode,total_stage_ms=elapsed,
                candidate_ms=candidate_ms,candidate_count=len(boxes),
                raw_candidate_count=raw_count,candidate_dedup_iou=.65,
                diagnostic_anchor_hits=anchor_hits,
                candidates=boxes.tolist(),roi_count=min(len(crops),MAX_CROPS),
                roi_overflow=overflow,trace=trace,roi_observations=observations))
    summary={}
    for mode in ('baseline','lite'):
        rows=[r for r in records if r['mode']==mode]
        summary[mode]=dict(frames=len(rows),p50_ms=float(np.median([r['total_stage_ms'] for r in rows])),
            p95_ms=float(np.percentile([r['total_stage_ms'] for r in rows],95)),
            candidate_count_range=[min(r['candidate_count'] for r in rows),max(r['candidate_count'] for r in rows)],
            roi_count_range=[min(r['roi_count'] for r in rows),max(r['roi_count'] for r in rows)])
        if mode=='lite':
            summary[mode].update(candidate_p50_ms=float(np.median([r['candidate_ms'] for r in rows])),
                candidate_p95_ms=float(np.percentile([r['candidate_ms'] for r in rows],95)),
                diagnostic_all_four_anchors_frames=sum(all(r['diagnostic_anchor_hits']) for r in rows),
                diagnostic_exactly_four_frames=sum(len(r['candidates'])==4 and all(r['diagnostic_anchor_hits']) for r in rows))
    report=producer_json_value(dict(summary=summary,samples=records,
        candidate_artifact=str(a.candidate),imgsz=a.imgsz,conf=a.conf,
        same_input_frames=True,scene_hint_used=False,production_code_changed=False,
        geometry_qualified=False,robot_commands=0,
        limitation='offline timing only; candidate/ROI association and scene qualification are not implemented here'))
    a.output.write_text(json.dumps(report,indent=2,allow_nan=False))
    print(json.dumps(summary))
finally:
    fcntl.flock(handle,fcntl.LOCK_UN);handle.close()
