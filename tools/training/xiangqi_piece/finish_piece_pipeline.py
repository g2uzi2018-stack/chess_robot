"""Wait for bounded training, then sequentially export, evaluate and compare."""
import json,time,subprocess,sys,traceback
from pathlib import Path
R=Path('/home/kuang/workspace/xiangqi-yolo-training/20261102-yolo26n-piece-v1')
PY=R/'runtime/python-piece';S=R/'runtime/pipeline-status.json'
state=dict(state='waiting_for_training',started_unix=time.time(),physical_commands=0,deployed=False)
def update(**kwargs):
    state.update(kwargs,updated_unix=time.time());p=S.with_suffix('.tmp');p.write_text(json.dumps(state,indent=2));p.replace(S)
def run(phase,args):
    update(state='running',phase=phase)
    subprocess.run([str(PY),*[str(x) for x in args]],check=True,cwd=R)
try:
    update()
    deadline=time.monotonic()+1800
    while True:
        t=json.loads((R/'runtime/train-status.json').read_text())
        if t['state']=='completed':break
        if t['state']=='failed':raise RuntimeError('Training failed: '+str(t.get('error')))
        if time.monotonic()>deadline:raise TimeoutError('Training wait exceeded 30 minutes')
        time.sleep(5)
    run('export', [R/'export_piece.py'])
    A=R/'artifacts/yolo26n-piece-fp32-v1'
    run('evaluate_pt',[R/'evaluate_piece.py','--artifact',A/'yolo26n-piece.pt',
                        '--output',R/'evaluation/pt','--sizes','640,512,416'])
    for size in [640,512,416]:
        run('evaluate_trt_'+str(size),[R/'evaluate_piece.py','--artifact',A/f'imgsz{size}-fp32/yolo26n-piece-imgsz{size}-fp32.engine',
                '--output',R/f'evaluation/trt-{size}','--sizes',size])
        run('compare_'+str(size),[R/'compare_piece_backends.py','--pt-dir',R/'evaluation/pt',
                '--engine-dir',R/f'evaluation/trt-{size}','--imgsz',size,'--output',R/f'evaluation/parity-{size}.json'])
    update(state='completed',phase='completed')
except BaseException as exc:
    update(state='failed',error=repr(exc),traceback=traceback.format_exc());raise
