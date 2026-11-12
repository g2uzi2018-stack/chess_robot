"""Read-only final artifact/data/service verification; no GPU imports."""
import json,hashlib,subprocess,time,struct
from pathlib import Path
R=Path('/home/kuang/workspace/xiangqi-yolo-training/20261102-yolo26n-piece-v1')
def read(p):return json.loads((R/p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert read('runtime/pipeline-status.json')['state']=='completed'
t=read('runtime/train-status.json');e=read('runtime/export-status.json')
assert t['state']=='completed' and e['state']=='completed' and not e['gpu_lock_held']
m=read('dataset/manifest.json');a=read('artifacts/yolo26n-piece-fp32-v1/export-manifest.json')
assert len(m['items'])==239 and sum(x['objects'] for x in m['items'])==983
assert m['split_counts']=={'train':{'images':190,'objects':785},'val':{'images':49,'objects':198}}
for rel,h in m['fixed_data_files_sha256'].items():assert sha(R/'dataset'/rel)==h,rel
O=Path(m['source_dataset'])
for rel,h in m['source_files_sha256'].items():assert sha(O/rel)==h,rel
assert sha(R/'dataset/manifest.json')==read('dataset/conversion-audit.json')['conversion_manifest_sha256']
data=read('dataset/data.yaml');assert data['names']=={'0':'piece'} and data['train']=='images/train' and data['val']=='images/val'
assert sha(a['copied_weights'])==a['source_weights_sha256']==sha(a['source_weights'])
assert sha(a['onnx']['onnx_path'])==a['onnx']['onnx_sha256']
assert {x['imgsz'] for x in a['engines']}=={640,512,416}
for engine in a['engines']:
 p=Path(engine['engine_path']);assert sha(p)==engine['engine_sha256']
 assert not any(engine['precision_flags_enabled'].values())
 with p.open('rb') as f:
  n=struct.unpack('<i',f.read(4))[0];meta=json.loads(f.read(n))
 assert meta['nc']==1 and meta['task']=='detect' and meta['names']=={'0':'piece'}
 assert meta['imgsz']==engine['actual_input_shape'][2:] and meta['precision']=='FP32-no-TF32'
for size in [640,512,416]:
 for backend in ['pt',f'trt-{size}']:
  d=R/'evaluation'/backend
  s=json.loads((d/'status.json').read_text());assert s['state']=='completed' and s['gpu_lock_released']
  val=json.loads((d/f'val-raw-{size}.json').read_text());diag=json.loads((d/f'diagnostic-raw-{size}.json').read_text())
  assert len(val['frames'])==49 and len(diag['frames'])==12
  assert sum(len(x['instances']) for x in val['frames'])==198
  v=json.loads((d/f'evaluation-{size}.json').read_text());assert v['benchmark']['calls']==100
 p=read(f'evaluation/parity-{size}.json');assert p['numerical_gate_pass'],f'parity failed {size}'
baseline=read('runtime/service-baseline.json')
source=Path('/home/kuang/workspace/chess_robot/runs/xiangqi_candidate_roi_deploy_20261102_01/source')
now={str(p.relative_to(source)):sha(p) for p in sorted(source.rglob('*')) if p.is_file() and p.suffix in ['.py','.sh','.yaml','.json'] and '__pycache__' not in str(p)}
assert now==baseline['source_files']
for p,h in baseline['original_models'].items():assert sha(p)==h
service=subprocess.check_output(['systemctl','--user','show','chess-robot-xiangqi-preview-20261031.service','-p','MainPID','-p','NRestarts','-p','ActiveState'],text=True)
assert service==baseline['service'],service
report=dict(status='passed',time_unix=time.time(),fixed_new_files_verified=len(m['fixed_data_files_sha256']),
 source_files_verified=len(m['source_files_sha256']),all_three_pt_trt_parity_passed=True,
 production_source_files_unchanged=len(now),production_service=service,original_models_unchanged=True,
 deployed=False,physical_commands=0,gpu_initialized=False,
 scripts={p.name:sha(p) for p in sorted(R.glob('*.py'))},runtime_wrapper_sha256=sha(R/'runtime/python-piece'))
(R/'artifact-verification.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
