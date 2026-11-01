#!/usr/bin/env python3
"""Restore the baseline CPU idle-state switches while max clocks are held."""
import argparse
from pathlib import Path


def read_snapshot(path):
    rows = {}
    for line in Path(path).read_text().splitlines():
        if ':' not in line:
            continue
        target, value = line.split(':', 1)
        if '/sys/devices/system/cpu/cpu' in target and '/cpuidle/state' in target and target.endswith('/disable'):
            if value.strip() not in ('0', '1'):
                raise ValueError(f'Unexpected cpuidle state {target}: {value!r}')
            rows[target] = value.strip()
    if not rows:
        raise RuntimeError('Snapshot contains no CPU cpuidle state entries')
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('snapshot', type=Path)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    rows = read_snapshot(args.snapshot)
    errors = []
    for target, expected in rows.items():
        path = Path(target)
        if not path.exists():
            errors.append(f'missing {target}')
            continue
        if args.apply:
            path.write_text(expected)
        actual = path.read_text().strip()
        if actual != expected:
            errors.append(f'{target}: expected={expected}, actual={actual}')
    if errors:
        raise SystemExit('FAIL cpuidle preservation:\n' + '\n'.join(errors))
    print(f"PASS cpuidle states {'restored' if args.apply else 'match baseline'}: {len(rows)} entries")


if __name__ == '__main__':
    main()
