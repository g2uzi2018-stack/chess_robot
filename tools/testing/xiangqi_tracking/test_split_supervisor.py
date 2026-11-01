"""Bounded subprocess shutdown/crash checks; tiny shared slots, no ROS or model."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent


def child(role, scenario):
    out = Path(os.environ['XIANGQI_RUN_DIR'])
    running = True
    def stop(*args):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, stop)
    if role == 'perception':
        from shared_pipeline import FrameSlots
        slots = FrameSlots(16, 12, max_masks=2)
        grandchild = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'])
        (out/'created.json').write_text(json.dumps(dict(names=[b.name for b in slots.blocks],
                                                       child=os.getpid(), grandchild=grandchild.pid)))
        if scenario == 'perception-crash':
            time.sleep(.2)
            os._exit(23)
        try:
            while running:
                time.sleep(.01)
        finally:
            grandchild.terminate()
            grandchild.wait(timeout=2)
            slots.close()
    else:
        if scenario == 'scene-crash':
            deadline = time.monotonic()+3
            while not (out/'created.json').exists() and time.monotonic() < deadline:
                time.sleep(.01)
            sys.exit(17)
        while running:
            time.sleep(.01)


def active(pid):
    path = Path('/proc')/str(pid)/'stat'
    try:
        return path.read_text().split()[2] not in ('Z', 'X')
    except FileNotFoundError:
        return False


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--child', choices=['scene', 'perception'])
    p.add_argument('--supervisor', action='store_true')
    p.add_argument('--scenario', default='normal-stop')
    p.add_argument('--output')
    a = p.parse_args()
    if a.child:
        child(a.child, a.scenario)
        return
    if a.supervisor:
        from split_supervisor import run_split
        commands = {role: [sys.executable, str(Path(__file__).resolve()), '--child', role, '--scenario', a.scenario]
                    for role in ('scene', 'perception')}
        sys.exit(run_split(commands, shutdown_timeout=3))
    out = Path(a.output)
    out.mkdir(parents=True, exist_ok=False)
    results = []
    for scenario in ('normal-stop', 'perception-crash', 'scene-crash'):
        case = out/scenario
        case.mkdir()
        env = dict(os.environ, XIANGQI_RUN_DIR=str(case))
        with (case/'supervisor.log').open('w') as log:
            worker = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--supervisor', '--scenario', scenario],
                                      env=env, stdout=log, stderr=subprocess.STDOUT)
            try:
                if scenario == 'normal-stop':
                    deadline = time.monotonic()+4
                    while not (case/'created.json').exists():
                        assert worker.poll() is None and time.monotonic() < deadline
                        time.sleep(.01)
                    worker.send_signal(signal.SIGTERM)
                code = worker.wait(timeout=8)
            finally:
                if worker.poll() is None:
                    worker.kill(); worker.wait(timeout=2)
        assert code == (0 if scenario == 'normal-stop' else 1), (scenario, code)
        report = json.loads((case/'split-supervisor.json').read_text())
        created = json.loads((case/'created.json').read_text())
        from multiprocessing import shared_memory
        for name in created['names']:
            try:
                block = shared_memory.SharedMemory(name=name)
            except FileNotFoundError:
                continue
            block.close()
            raise AssertionError('owned shared slot leaked: '+name)
        assert all(not active(r['pid']) for r in report['children'].values()), report
        deadline = time.monotonic()+2
        while active(created['grandchild']) and time.monotonic() < deadline:
            time.sleep(.01)
        assert not active(created['grandchild']), 'grandchild survived supervisor exit'
        results.append(dict(scenario=scenario, exit_code=code, passed=True,
                            shared_slots_unlinked=3, child_groups_stopped=True))
    result = dict(passed=True, cases=results, physical_commands=0)
    (out/'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result))


if __name__ == '__main__':
    main()
