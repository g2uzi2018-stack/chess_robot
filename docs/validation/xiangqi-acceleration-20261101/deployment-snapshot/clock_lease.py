"""Fixed clocks belong to the visual systemd activation, not the arm driver."""
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

O=Path(__file__).resolve().parent;ROOT=Path('/home/kuang/workspace/chess_robot')
STATE=ROOT/'.runtime/vision-performance-clock.json'
LOCK=ROOT/'runs/perf_ceiling_20261031/benchmark.lock'
RESOURCE=ROOT/'runs/perf_ceiling_20261031/02_resources'
BOOT=Path('/proc/sys/kernel/random/boot_id').read_text().strip()

def run(args):
    p=subprocess.run([str(x) for x in args],capture_output=True,text=True,timeout=20)
    if p.returncode:raise RuntimeError(f'{args}: {p.stdout} {p.stderr}')
    return p.stdout

def save(data):
    tmp=STATE.with_suffix('.json.tmp');tmp.write_text(json.dumps(data,indent=2));tmp.replace(STATE)

def release(state):
    if not state.get('active'):return
    if state.get('boot_id')!=BOOT:
        state.update(active=False,release_reason='old boot ended; stale sysfs snapshot not applied');save(state);return
    snap=Path(state['snapshot']);folder=Path(state['folder'])
    run(['sudo','-n','jetson_clocks','--restore',snap])
    after=run(['sudo','-n','jetson_clocks','--show']);(folder/'clock-restored.txt').write_text(after)
    verified=run(['sudo','-n','python3',RESOURCE/'verify_restore.py',snap,folder/'clock-before.txt',folder/'clock-restored.txt'])
    (folder/'restore-verification.txt').write_text(verified)
    state.update(active=False,released_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),release_verification=verified);save(state)

def main():
    invocation=os.environ.get('INVOCATION_ID')
    if sys.argv[1]=='acquire' and not invocation:raise RuntimeError('Fixed clocks require the managed systemd visual service')
    handle=LOCK.open('a')
    def timeout(*args):raise TimeoutError('Could not own benchmark lock for visual clock transition')
    signal.signal(signal.SIGALRM,timeout);signal.alarm(30)
    fcntl.flock(handle,fcntl.LOCK_EX);signal.alarm(0)
    try:
        state=json.loads(STATE.read_text()) if STATE.exists() else {}
        if sys.argv[1]=='release':
            if state.get('active') and invocation and state.get('invocation_id')!=invocation:raise RuntimeError('Refusing to release another activation clocks')
            release(state);return
        if state.get('active'):
            if state.get('boot_id')==BOOT and Path('/proc/'+str(state['owner_pid'])).exists():raise RuntimeError('An active visual clock owner already exists')
            release(state)
        config=json.loads((O/'performance.json').read_text())
        for key,path_key in [('artifact_sha256','model_artifact'),('source_weights_sha256','source_weights')]:
            actual=hashlib.sha256(Path(config[path_key]).read_bytes()).hexdigest()
            if actual!=config[key]:raise RuntimeError(f'{path_key} SHA256 mismatch')
        if not config['fixed_clocks']:return
        assert 'MAXN' in run(['nvpmodel','-q']),'Existing MAXN mode required; not changing power mode'
        folder=O/'clock-sessions'/invocation;folder.mkdir(parents=True,exist_ok=False)
        snapshot=folder/'original.conf'
        run(['sudo','-n','jetson_clocks','--store',snapshot]);assert snapshot.stat().st_size>0
        (folder/'clock-before.txt').write_text(run(['sudo','-n','jetson_clocks','--show']))
        state=dict(active=True,owner_pid=os.getppid(),invocation_id=invocation,boot_id=BOOT,folder=str(folder),snapshot=str(snapshot),created_at=datetime.datetime.now(datetime.timezone.utc).isoformat());save(state)
        try:
            (folder/'apply.txt').write_text(run(['sudo','-n','jetson_clocks']))
            run(['sudo','-n','python3',RESOURCE/'restore_cpuidle.py',snapshot,'--apply'])
            (folder/'clock-fixed.txt').write_text(run(['sudo','-n','jetson_clocks','--show']))
            (folder/'fixed-verification.txt').write_text(run(['sudo','-n','python3',RESOURCE/'verify_fixed.py',folder/'clock-fixed.txt',snapshot,folder/'clock-before.txt']))
        except BaseException:
            release(state);raise
        print(json.dumps(state),flush=True)
    finally:fcntl.flock(handle,fcntl.LOCK_UN);handle.close()

if __name__=='__main__':main()
