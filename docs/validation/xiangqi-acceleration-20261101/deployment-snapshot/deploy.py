from pathlib import Path
import hashlib,json,subprocess,sys

R=Path('/home/kuang/workspace/chess_robot');O=R/'runs/vision_acceleration_20261101_01'
UNIT='chess-robot-xiangqi-preview-20261031.service'
LAUNCH=R/'runs/stable_observation_20261101_01/start_visual.sh'
DROP=Path.home()/'.config/systemd/user'/f'{UNIT}.d/zz-vision-acceleration.conf'
CONFIG=dict(fixed_clocks=True,model_artifact=str(R/'runs/perf_ceiling_20261031/01_backend/weights/20261031T143600930755709/best.engine'),artifact_sha256='26103821bc0788d1dc0a2d0051c91362530d5b6b1cc92347f2289d12c9469d35',source_weights='/home/kuang/workspace/xiangqi-yolo-training/20261031-yolo26s-seg-v1/runs/train/weights/best.pt',source_weights_sha256='f4fce2c0217b2ce292820e45f1779654b4cf58da3d156aff6306d56ae6aabf7e')
drop_text=f'[Service]\nTimeoutStopSec=45s\nExecStopPost=/usr/bin/python3 {O}/clock_lease.py release\n'
def run(*args):return subprocess.run(args,check=True,capture_output=True,text=True).stdout
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def start_unit(accelerated=True):
    load=run('systemctl','--user','show',UNIT,'-p','LoadState','--value').strip()
    if load!='not-found':return run('systemctl','--user','start',UNIT)
    folder=O/'production' if accelerated else R/'runs/stable_observation_20261101_01/production'
    folder.mkdir(parents=True,exist_ok=True)
    return run('systemd-run','--user','--unit='+UNIT,'--property=KillMode=mixed','--property=TimeoutStopSec='+('45s' if accelerated else '8s'),'--property=Restart=no','--property=StandardOutput=append:'+str(folder/'service.log'),'--property=StandardError=append:'+str(folder/'service.log'),'/bin/bash',str(LAUNCH))
def stop_unit():
    if run('systemctl','--user','show',UNIT,'-p','LoadState','--value').strip()!='not-found':run('systemctl','--user','stop',UNIT)
if sys.argv[1]=='verify-lifecycle':
    stop_unit()
    state=json.loads((R/'.runtime/vision-performance-clock.json').read_text());assert not state['active'] and 'PASS restored' in state['release_verification']
    (O/'lifecycle-stop-verified.json').write_text(json.dumps(state,indent=2))
    start_unit();print('Stop hook restored and verified original clocks; accelerated visual service restarted');sys.exit(0)
if sys.argv[1]=='finalize':
    assert str(O/'start_accelerated.sh') in LAUNCH.read_text()
    assert DROP.read_text()==drop_text
    for filename in ['recognize_tracked.py','live_tracked.py']:
        target=R/'tools/testing/xiangqi_tracking'/filename
        assert sha(target)==sha(O/'before'/filename),'Working source changed after staging'
        target.write_bytes((O/'source'/filename).read_bytes())
    (O/'deployment.json').write_text(json.dumps(dict(service=UNIT,launcher=str(LAUNCH),source=str(O/'source'),drop_in=str(DROP),config=CONFIG,repo_source_updated=True,git_committed=False,transient_visual_unit_recreated=True),indent=2))
    print('Optional backend support synchronized to working source; previous fixes retained');sys.exit(0)
if sys.argv[1]=='rollback':
    stop_unit()
    state=R/'.runtime/vision-performance-clock.json'
    if state.exists():assert not json.loads(state.read_text()).get('active'),'Clock lease not released'
    LAUNCH.write_bytes((O/'before/start_visual.sh').read_bytes())
    if DROP.exists():
        assert DROP.read_text()==drop_text,'Drop-in changed; not deleting unrelated edits'
        DROP.unlink()
    run('systemctl','--user','daemon-reload');start_unit(False)
    print('Original PT visual launcher and dynamic clocks restored');sys.exit(0)
assert json.loads((O/'parity-result.json').read_text())['passed']
preliminary=DROP.with_name('90-vision-acceleration.conf')
if preliminary.exists():
    assert preliminary.read_text()==drop_text,'Preliminary hook was edited externally'
    preliminary.unlink()
assert not DROP.exists(),'Performance drop-in already exists; inspect instead of overwrite'
assert LAUNCH.read_bytes()==(O/'before/start_visual.sh').read_bytes(),'Visual launcher changed after backup'
(O/'performance.json').write_text(json.dumps(CONFIG,indent=2))
for file in ['recognize_tracked.py','live_tracked.py']:
    assert sha(R/'tools/testing/xiangqi_tracking'/file)==sha(O/'before'/file),'Working source changed after staging'
DROP.parent.mkdir(parents=True,exist_ok=True);DROP.write_text(drop_text)
run('systemctl','--user','daemon-reload')
props=run('systemctl','--user','show',UNIT,'-p','TimeoutStopUSec','-p','ExecStopPost')
assert str(O/'clock_lease.py') in props and '45s' in props,props
(O/'lifecycle-properties.txt').write_text(props)
stop_unit()
try:
    LAUNCH.write_text(f'#!/bin/bash\nset -eo pipefail\nexec bash {O}/start_accelerated.sh\n')
    start_unit()
except BaseException:
    stop_unit()
    LAUNCH.write_bytes((O/'before/start_visual.sh').read_bytes());DROP.unlink();run('systemctl','--user','daemon-reload');start_unit(False);raise
(O/'deployment.json').write_text(json.dumps(dict(service=UNIT,launcher=str(LAUNCH),source=str(O/'source'),drop_in=str(DROP),config=CONFIG,repo_source_committed=False),indent=2))
print('TRT FP32 visual service started with lifecycle-scoped fixed clocks')
