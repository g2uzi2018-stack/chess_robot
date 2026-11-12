#!/usr/bin/env python3
"""Compare saved sysfs policy values and normalized Jetson clock reports."""
import re
import subprocess
import sys
from pathlib import Path


def read_snapshot(path):
    rows = []
    for line in Path(path).read_text().splitlines():
        if ':' not in line:
            continue
        target, expected = line.split(':', 1)
        if target.startswith('/'):
            rows.append((target, expected.strip()))
    return rows


def normalized_show(path):
    lines = []
    for line in Path(path).read_text(errors='replace').splitlines():
        if line.startswith('cpu') and 'Governor=' in line:
            line = re.sub(r'CurrentFreq=\d+\s*', '', line)
        elif line.startswith(('GPU MinFreq=', 'EMC MinFreq=')):
            line = re.sub(r'CurrentFreq=\d+\s*', '', line)
        elif line.startswith('FAN Dynamic Speed Control='):
            line = re.sub(r'\s+hwmon\S+=\d+', '', line)
        lines.append(line.strip())
    return [line for line in lines if line]


def main():
    if len(sys.argv) != 4:
        raise SystemExit('usage: verify_restore.py SNAPSHOT BEFORE_SHOW AFTER_SHOW')
    snapshot, before_show, after_show = map(Path, sys.argv[1:])
    errors = []
    checked = 0
    for target, expected in read_snapshot(snapshot):
        path = Path(target)
        try:
            actual = path.read_text().strip()
        except OSError as exc:
            errors.append(f'{target}: unreadable after restore ({exc})')
            continue
        if actual != expected:
            errors.append(f'{target}: expected={expected!r}, actual={actual!r}')
        checked += 1
    if normalized_show(before_show) != normalized_show(after_show):
        errors.append('normalized jetson_clocks --show differs from pre-test state')
    mode = subprocess.run(['nvpmodel', '-q'], check=True, text=True, capture_output=True).stdout.strip()
    if 'MAXN' not in mode:
        errors.append(f'nvpmodel is no longer MAXN: {mode}')
    if errors:
        raise SystemExit('FAIL restore verification:\n' + '\n'.join(errors))
    print(f'PASS restored {checked} saved sysfs values; normalized CPU/GPU/EMC/fan report and MAXN match the pre-test snapshot')


if __name__ == '__main__':
    main()
