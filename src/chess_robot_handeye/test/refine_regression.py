#!/usr/bin/env python3
"""Saved-image regression through the actual C++ product detector; no ROS/USB."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import cv2
import numpy as np
import yaml


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    for key in ['probe', 'raw', 'capture', 'output']:
        parser.add_argument('--' + key, required=True)
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.mkdir(exist_ok=False)
    raw, capture, probe = (Path(p).resolve() for p in [args.raw, args.capture, args.probe])
    original_hash = sha(raw)
    blank = np.full((480, 640, 3), 255, dtype=np.uint8)
    assert cv2.imwrite(str(output / 'blank.png'), blank)
    foreign = blank.copy()
    marker = cv2.aruco.drawMarker(cv2.aruco.getPredefinedDictionary(5), 75, 160)
    foreign[140:300, 240:400] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    assert cv2.imwrite(str(output / 'unrelated_marker.png'), foreign)
    cases = [
        ('raw', raw, 12, 9, 5, [], True),
        ('wrong_board_11x8', raw, 11, 8, 5, [], False),
        ('wrong_dictionary_4x4_100', raw, 12, 9, 1, [], False),
        ('blank', output / 'blank.png', 12, 9, 5, [], False),
        ('unrelated_id_75', output / 'unrelated_marker.png', 12, 9, 5, [], False),
        ('occluded_raw', raw, 12, 9, 5, [350, 260, 130, 150], True),
    ]
    summary = {'raw_sha256': original_hash, 'capture_sha256': sha(capture),
               'probe_sha256': sha(probe), 'static_recognition_only': True,
               'cases': {}}
    for name, image, x, y, dictionary, mask, expected in cases:
        command = [str(probe), str(image), str(capture), str(x), str(y), str(dictionary)]
        command += [str(v) for v in mask]
        run = subprocess.run(command, capture_output=True, text=True, timeout=15)
        (output / (name + '.yaml')).write_text(run.stdout)
        (output / (name + '.stderr')).write_text(run.stderr)
        result = yaml.safe_load(run.stdout)
        passed = isinstance(result, dict) and result.get('product_called') is True
        passed = passed and bool(result.get('accepted')) == expected
        passed = passed and run.returncode == (0 if expected else 2)
        if expected:
            passed = passed and result.get('product_charuco_corner_count', 0) >= 12
            passed = passed and result.get('reprojection_rms_px', float('inf')) < 1.
        passed = passed and result.get('minimum_corners') == 12
        passed = passed and result.get('maximum_reprojection_px') == 1.
        passed = passed and result.get('adaptive_threshold_constant') == 7.
        summary['cases'][name] = {'test_pass': bool(passed), 'expected_accept': expected,
                                  'returncode': run.returncode, 'command': command,
                                  'product_result': result}
        print(name, 'PASS' if passed else 'FAIL', result)
    assert sha(raw) == original_hash, 'original raw image was modified'
    summary['all_passed'] = all(c['test_pass'] for c in summary['cases'].values())
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    raise SystemExit(0 if summary['all_passed'] else 1)


if __name__ == '__main__':
    main()
