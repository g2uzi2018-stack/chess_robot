#!/usr/bin/env python3
"""Verify max CPU/GPU/EMC clocks while preserving CPU idle/governor policy."""
import argparse
import re
import subprocess
from pathlib import Path

CPU_RE = re.compile(r'cpu(\d+):\s+Online=(\d+)\s+Governor=(\S+)\s+MinFreq=(\d+)\s+MaxFreq=(\d+)\s+CurrentFreq=(\d+)')


def snapshot_values(path):
    values = {}
    for line in Path(path).read_text().splitlines():
        if ':' in line:
            key, value = line.split(':', 1)
            if key.startswith('/sys/'):
                values[key] = value.strip()
    return values


def non_target_lines(text):
    selected = []
    prefixes = ('Online CPUs:', 'Active GPU TPCs:', 'DLA0_', 'DLA1_', 'PVA0_',
                'FAN Dynamic Speed Control=', 'NV Power Mode:')
    for line in text.splitlines():
        if line.startswith(prefixes):
            line = re.sub(r'\s+hwmon\S+=\d+', '', line)
            selected.append(line.strip())
    return selected


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('fixed_show', type=Path)
    parser.add_argument('snapshot', type=Path)
    parser.add_argument('before_show', type=Path)
    args = parser.parse_args()
    text = args.fixed_show.read_text()
    baseline = args.before_show.read_text()
    stored = snapshot_values(args.snapshot)
    expected_cpus = {int(match.group(1)): match for match in CPU_RE.finditer(baseline)}
    actual_cpus = {int(match.group(1)): match for match in CPU_RE.finditer(text)}
    errors = []
    if set(actual_cpus) != set(range(12)):
        errors.append(f'online CPU rows are incomplete: {sorted(actual_cpus)}')
    for cpu, match in actual_cpus.items():
        online, governor, minimum, maximum, _current = match.groups()[1:]
        if online != '1':
            errors.append(f'cpu{cpu} is offline')
        if minimum != maximum:
            errors.append(f'cpu{cpu} not fixed: min={minimum}, max={maximum}')
        old = expected_cpus.get(cpu)
        if old and governor != old.group(3):
            errors.append(f'cpu{cpu} governor changed: {old.group(3)} -> {governor}')

    gpu = re.search(r'GPU MinFreq=(\d+) MaxFreq=(\d+) CurrentFreq=(\d+)', text)
    if not gpu:
        errors.append('GPU clock row missing')
    elif gpu.group(1) != gpu.group(2):
        errors.append(f'GPU min/max are not fixed: {gpu.group(1)}/{gpu.group(2)}')

    emc = re.search(r'EMC MinFreq=(\d+) MaxFreq=(\d+) CurrentFreq=(\d+) FreqOverride=(\d+)', text)
    if not emc:
        errors.append('EMC clock row missing')
    else:
        minimum, maximum, current, override = map(int, emc.groups())
        if override != 1:
            errors.append(f'EMC override is not locked: {override}')
        if current != maximum:
            errors.append(f'EMC current is not at cap: current={current}, max={maximum}')

    mode = subprocess.run(['nvpmodel', '-q'], check=True, text=True, capture_output=True).stdout.strip()
    if 'MAXN' not in mode:
        errors.append(f'nvpmodel changed or is not MAXN: {mode}')
    if non_target_lines(baseline) != non_target_lines(text):
        errors.append('non-target DLA/PVA, fan policy, CPU online, or power-mode report changed')

    # Restore the pre-test CPU idle-state flags after jetson_clocks disables them.
    for target, expected in stored.items():
        path = Path(target)
        try:
            actual = path.read_text().strip()
        except OSError as exc:
            errors.append(f'cannot read saved setting {target}: {exc}')
            continue
        if '/cpuidle/state' in target and target.endswith('/disable'):
            if actual != expected:
                errors.append(f'cpuidle changed: {target} expected={expected}, actual={actual}')
        elif target.endswith('/cpufreq/scaling_min_freq'):
            maximum = path.with_name('scaling_max_freq').read_text().strip()
            if actual != maximum:
                errors.append(f'CPU max not pinned: {target} actual={actual}, max={maximum}')
        elif target.endswith('/min_freq') and '/devfreq/' in target:
            maximum = path.with_name('max_freq').read_text().strip()
            if actual != maximum:
                errors.append(f'GPU max not pinned: {target} actual={actual}, max={maximum}')
        elif target.endswith('/mrq_rate_locked'):
            if actual != '1':
                errors.append(f'EMC lock is not enabled: {target}={actual}')
        elif target.endswith('/bwmgr_halt'):
            if actual != '1':
                errors.append(f'EMC bandwidth-manager hold is not enabled: {target}={actual}')
        elif target.endswith('/railgate_enable'):
            if actual != '0':
                errors.append(f'GPU rail gate is not open: {target}={actual}')
        elif actual != expected:
            errors.append(f'unexpected non-target resource change: {target} expected={expected}, actual={actual}')

    if errors:
        raise SystemExit('FAIL fixed-clock verification:\n' + '\n'.join(errors))
    print('PASS fixed clock envelope: CPU/GPU/EMC pinned; MAXN, governors, CPU idle states, fan policy and non-target saved settings preserved')
    print(f'CPU policies verified: {len(actual_cpus)}; EMC override={emc.group(4)}; nvpmodel={mode}')


if __name__ == '__main__':
    main()
