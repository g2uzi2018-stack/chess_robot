"""Switch only the existing read-only visual unit to an accepted fixed snapshot."""
from pathlib import Path
import hashlib,json,os,shutil,subprocess,sys

R=Path('/home/kuang/workspace/chess_robot')
N=R/'runs/xiangqi_node_split_20261101_01'
A=R/'runs/vision_acceleration_20261101_01'
UNIT='chess-robot-xiangqi-preview-20261031.service'
TOP=R/'runs/stable_observation_20261101_01/start_visual.sh'
BACKUP=N/'deployment-before'
SOURCE=N/'source-final-02'
SOURCE_MANIFEST=N/'production-source-manifest-02.json'
CLOCK_DROPIN=Path.home()/'.config/systemd/user'/f'{UNIT}.d/zz-vision-acceleration.conf'

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def run(*args):return subprocess.run(args,check=True,capture_output=True,text=True,timeout=65).stdout
def load(p):return json.loads(p.read_text())
def stop():
    if run('systemctl','--user','show',UNIT,'-p','LoadState','--value').strip()!='not-found':
        run('systemctl','--user','stop',UNIT)
    state=load(R/'.runtime/vision-performance-clock.json')
    assert not state['active'] and 'PASS restored' in state.get('release_verification',''),state
    (N/'deployment-clock-stop.json').write_text(json.dumps(state,indent=2))
def start(accelerated):
    # This persistent same-name drop-in also applies when the transient unit is
    # recreated. Refuse to acquire clocks if its release handler is absent.
    policy=CLOCK_DROPIN.read_text()
    assert f'ExecStopPost=/usr/bin/python3 {A}/clock_lease.py release' in policy and 'TimeoutStopSec=45s' in policy,policy
    if run('systemctl','--user','show',UNIT,'-p','LoadState','--value').strip()!='not-found':
        return run('systemctl','--user','start',UNIT)
    out=N/'production' if accelerated else A/'production'
    out.mkdir(parents=True,exist_ok=True)
    return run('systemd-run','--user','--unit='+UNIT,'--property=KillMode=mixed','--property=TimeoutStopSec=45s','--property=Restart=no',
               '--property=StandardOutput=append:'+str(out/'service.log'),
               '--property=StandardError=append:'+str(out/'service.log'),'/bin/bash',str(TOP))

action=sys.argv[1]
if action=='prepare':
    accepted=load(N/'parent-acceptance.json')
    assert accepted['accepted'] is True
    assert not SOURCE.exists()
    if BACKUP.exists():
        assert TOP.read_bytes()==(BACKUP/'start_visual.sh').read_bytes(),'A prior rollback must restore the original launcher first'
    for name,digest in accepted['code_sha256'].items():assert sha(R/name)==digest,name
    source=R/'tools/testing/xiangqi_tracking'
    SOURCE.mkdir()
    manifest={}
    for p in source.iterdir():
        if not p.is_file() or p.suffix not in ('.py','.sh','.md') or p.name=='test_history_loopback.py':continue
        target=SOURCE/p.name;shutil.copy2(p,target);manifest[p.name]=sha(target)
    SOURCE_MANIFEST.write_text(json.dumps(manifest,indent=2))
    old=(A/'start_accelerated.sh').read_text()
    expected=['export XIANGQI_SOURCE_DIR="$HERE/source"','export XIANGQI_RUN_DIR="$HERE/production"','exec bash "$HERE/source/start_live_tracked.sh"']
    assert all(old.count(s)==1 for s in expected)
    new=old.replace(expected[0],f'export XIANGQI_SOURCE_DIR={SOURCE}').replace(expected[1],f'export XIANGQI_RUN_DIR={N}/production').replace(expected[2],'exec bash "$XIANGQI_SOURCE_DIR/start_live_tracked.sh"')
    (N/'start_split_accelerated.sh').write_text(new)
    run('bash','-n',str(N/'start_split_accelerated.sh'))
    BACKUP.mkdir(exist_ok=True)
    if not (BACKUP/'start_visual.sh').exists():shutil.copy2(TOP,BACKUP/'start_visual.sh')
    assert str(A/'start_accelerated.sh') in TOP.read_text(),'Unexpected live launcher; inspect instead of overwrite'
    if not (BACKUP/'service.txt').exists():
        (BACKUP/'service.txt').write_text(run('systemctl','--user','show',UNIT,'-p','Id','-p','MainPID','-p','ExecStart','-p','ExecStopPost','-p','TimeoutStopUSec'))
    if not (BACKUP/'clock-state.json').exists():
        (BACKUP/'clock-state.json').write_text((R/'.runtime/vision-performance-clock.json').read_text())
    print('Prepared accepted fixed source and rollback backup; no service changes')
elif action=='apply':
    accepted=load(N/'parent-acceptance.json');assert accepted['accepted'] is True
    for name,digest in load(SOURCE_MANIFEST).items():assert sha(SOURCE/name)==digest,name
    assert TOP.read_bytes()==(BACKUP/'start_visual.sh').read_bytes(),'Live launcher changed since backup'
    properties=run('systemctl','--user','show',UNIT,'-p','ExecStopPost','-p','TimeoutStopUSec')
    assert str(A/'clock_lease.py') in properties and '45s' in properties,properties
    desired=f'#!/bin/bash\nset -eo pipefail\nexec bash {N}/start_split_accelerated.sh\n'
    stop()
    try:
        TOP.write_text(desired);start(True)
    except BaseException:
        stop();TOP.write_bytes((BACKUP/'start_visual.sh').read_bytes());start(False);raise
    (N/'deployment.json').write_text(json.dumps(dict(status='split_visual_service_started_pending_observation',service=UNIT,launcher=str(TOP),source=str(SOURCE),launcher_sha256=sha(TOP),clock_helper=str(A/'clock_lease.py'),no_boot_enable_added=True,actuator_commands_sent=False),indent=2))
    print('Only visual unit switched to fixed split source; clock lifecycle retained')
elif action=='rollback':
    assert str(N/'start_split_accelerated.sh') in TOP.read_text(),'Refusing to overwrite a later launcher'
    stop();TOP.write_bytes((BACKUP/'start_visual.sh').read_bytes());start(False)
    print('Prior TRT visual source restored; no controller/camera/renderer changes')
else:
    raise ValueError('Expected prepare, apply or rollback')
