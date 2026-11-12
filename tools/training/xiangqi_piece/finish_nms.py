import json,subprocess,time,traceback
from pathlib import Path
R=Path('/home/kuang/workspace/xiangqi-yolo-training/20261102-yolo26n-piece-v1')
s={'state':'running','started_unix':time.time()}
def save():
 p=R/'runtime/nms-pipeline-status.json';p.write_text(json.dumps(s,indent=2))
try:
 for size in [640,512,416]:
  s['size']=size;save()
  subprocess.run([str(R/'runtime/python-piece'),str(R/'evaluate_piece_nms.py'),'--artifact',
    str(R/f'artifacts/yolo26n-piece-fp32-v1/imgsz{size}-fp32/yolo26n-piece-imgsz{size}-fp32.engine'),
    '--sizes',str(size),'--benchmark-conf','.10','--nms-iou','.65','--output',str(R/f'evaluation/trt-nms-{size}')],check=True)
 s.update(state='completed',finished_unix=time.time());save()
except BaseException as exc:
 s.update(state='failed',error=repr(exc),traceback=traceback.format_exc());save();raise
