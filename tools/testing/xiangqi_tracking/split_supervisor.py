"""Foreground owner of the two read-only node process groups and their cleanup."""
import json
import os
import re
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path


def group_signal(process, sig):
    try:
        os.killpg(process.pid, sig)
    except ProcessLookupError:
        pass


def cleanup_slots(path, token, owner_pid):
    """Only unlink names explicitly registered by this supervisor's own child."""
    if not path.exists():
        return []
    data = json.loads(path.read_text())
    if data.get('token') != token or data.get('owner_pid') != owner_pid:
        raise RuntimeError('shared-memory cleanup ownership mismatch')
    names = data.get('names')
    if not isinstance(names, list) or len(names) != 3 or not all(
            isinstance(name, str) and re.fullmatch(r'psm_[a-zA-Z0-9_]{1,48}', name) for name in names):
        raise RuntimeError('invalid owned shared-memory manifest')
    from multiprocessing import shared_memory
    removed = []
    for name in names:
        try:
            block = shared_memory.SharedMemory(name=name)
        except FileNotFoundError:
            continue
        try:
            block.unlink()
            removed.append(name)
        finally:
            block.close()
    path.unlink()
    return removed


def run_split(commands=None, shutdown_timeout=12.):
    out = Path(os.environ.get('XIANGQI_RUN_DIR', str(Path(__file__).resolve().parent)))
    out.mkdir(parents=True, exist_ok=True)
    source = Path(os.environ.get('XIANGQI_SOURCE_DIR', str(Path(__file__).resolve().parent)))
    args = sys.argv[1:] if commands is None else []
    if any('__node:=' in value or '__name:=' in value for value in args):
        raise ValueError('Use the independent node launchers to override node names')
    commands = commands or {'scene': [sys.executable, str(source/'scene_node.py'), *args],
                            'perception': [sys.executable, str(source/'perception_node.py'), *args]}
    token = uuid.uuid4().hex
    manifest = out/('split-shm-'+token+'.json')
    env = os.environ.copy()
    env.update(XIANGQI_RUN_DIR=str(out), XIANGQI_SUPERVISOR_TOKEN=token, XIANGQI_SHM_MANIFEST=str(manifest))
    children = {}
    stopping = False
    exit_code = 1
    reason = 'startup failed'
    previous_handlers = {}

    def stop(signum, frame):
        nonlocal stopping, reason
        stopping = True
        reason = 'supervisor received '+signal.Signals(signum).name

    for sig in (signal.SIGTERM, signal.SIGINT):
        previous_handlers[sig] = signal.signal(sig, stop)
    try:
        for role, command in commands.items():
            children[role] = subprocess.Popen(command, env=env, start_new_session=True)
        while not stopping:
            ended = [(role, p.returncode) for role, p in children.items() if p.poll() is not None]
            if ended:
                reason = 'child exited unexpectedly: '+repr(ended)
                break
            time.sleep(.05)
        exit_code = 0 if stopping else 1
    finally:
        # First let each parent drain/unlink its normal pipeline.
        for child in children.values():
            if child.poll() is None:
                child.send_signal(signal.SIGTERM)
        deadline = time.monotonic()+shutdown_timeout
        while any(child.poll() is None for child in children.values()) and time.monotonic() < deadline:
            time.sleep(.05)
        # A crashed parent can leave workers behind. These groups belong only
        # to the subprocesses above, never the production service's group.
        for child in children.values():
            group_signal(child, signal.SIGTERM)
        time.sleep(.1)
        for child in children.values():
            group_signal(child, signal.SIGKILL)
            try:
                child.wait(timeout=2)
            except subprocess.TimeoutExpired:
                exit_code = 1
                reason += '; child did not exit'
        removed = []
        if 'perception' in children:
            removed = cleanup_slots(manifest, token, children['perception'].pid)
        report = dict(token=token, reason=reason, exit_code=exit_code,
                      children={role: dict(pid=p.pid, returncode=p.returncode) for role, p in children.items()},
                      forcibly_unlinked_owned_slots=removed, robot_commands=0)
        (out/'split-supervisor.json').write_text(json.dumps(report, indent=2))
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
    return exit_code


def main():
    return run_split()


if __name__ == '__main__':
    sys.exit(main())
