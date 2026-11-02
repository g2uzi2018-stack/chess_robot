#!/usr/bin/env python3
"""Offline Home consistency gate. Reads files only; never talks to a controller."""
import argparse
import json
import math
from pathlib import Path
import yaml


def validate(root):
    root = Path(root).resolve()
    canonical = root / 'config/arm/home.yaml'
    if (canonical.parent / '.home_update_pending').exists():
        raise ValueError('unfinished Home update; inspect capture backups and marker')
    def load(path, field):
        return yaml.safe_load(path.read_text())['chess_robot_arm']['ros__parameters'][field]
    home = load(canonical, 'home')
    mirror = load(root / 'src/chess_robot_arm/config/home.yaml', 'home')
    if home != mirror or canonical.read_bytes() != (root / 'src/chess_robot_arm/config/home.yaml').read_bytes():
        raise ValueError('daily Home YAML drift; reconcile canonical and package mirror before restart/build')
    if home.get('taught') is not True or home.get('joint_names') != ['joint'+str(i) for i in range(1, 7)]:
        raise ValueError('taught daily Home with canonical joint order required')
    def joints(value):
        if not isinstance(value, list) or len(value) != 6 or any(type(q) not in (int, float) or not math.isfinite(q) for q in value):
            raise ValueError('six finite joint radians required')
        # Pinned Nova5 limits; runtime independently checks the installed URDF.
        lower = [-6.28, -3.14, -2.79, -6.28, -6.28, -6.28]
        upper = [6.28, 3.14, 2.79, 6.28, 6.28, 6.28]
        if any(not lo <= q <= hi for q, lo, hi in zip(value, lower, upper)):
            raise ValueError('Home/reference outside pinned joint limits')
    joints(home['joint_positions_rad'])
    reference = load(root / 'config/arm/twist_reference.yaml', 'reference')['joint_positions_rad']
    joints(reference)
    metadata = json.loads((canonical.parent / 'home_reference.json').read_text())
    if metadata.get('source') != 'operator_stationary_feedback_capture' or metadata.get('capture_frames', 0) < 2:
        raise ValueError('trusted stationary capture provenance missing; capture a new candidate')
    if metadata.get('joint_positions_rad') != home['joint_positions_rad']:
        raise ValueError('daily Home metadata does not match canonical target')
    return {'status': 'PASS', 'canonical': str(canonical), 'reference_basis_rad': reference, 'history_compatibility': 'runtime checkpoint identity required', 'motion_commands': 0}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    args = parser.parse_args()
    try:
        print(json.dumps(validate(args.root)))
    except (ValueError, KeyError, OSError, TypeError) as exc:
        parser.exit(2, str(exc) + '\n')
