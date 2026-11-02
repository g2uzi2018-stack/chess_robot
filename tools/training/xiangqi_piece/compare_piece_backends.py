#!/usr/bin/env python3
"""Pure-CPU comparison of saved evaluate_piece.py PT/TRT outputs; no inference."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from evaluate_piece import THRESHOLDS, iou, percentile, save_json


def match_predictions(a, b):
    # Match by geometry, using score closeness to break ties among identical
    # boxes. This is independent of class (the single-class policy is checked).
    candidates = [(iou(p, q), -abs(p[4] - q[4]), ai, bi)
                  for ai, p in enumerate(a) for bi, q in enumerate(b) if iou(p, q) >= .90]
    chosen_a, chosen_b, matched = set(), set(), []
    for overlap, _, ai, bi in sorted(candidates, reverse=True):
        if ai in chosen_a or bi in chosen_b:
            continue
        chosen_a.add(ai); chosen_b.add(bi)
        p, q = a[ai], b[bi]
        matched.append({'pt_index': ai, 'engine_index': bi, 'box_iou': overlap,
                        'class_equal': int(p[5]) == int(q[5]) == 0,
                        'confidence_pt': p[4], 'confidence_engine': q[4],
                        'confidence_abs_delta': abs(p[4] - q[4]),
                        'box_linf_delta_px': max(abs(x - y) for x, y in zip(p[:4], q[:4])),
                        'threshold_flips': [t for t in THRESHOLDS if (p[4] >= t) != (q[4] >= t)]})
    return matched, sorted(set(range(len(a))) - chosen_a), sorted(set(range(len(b))) - chosen_b)


def compare_frames(pt, engine, key):
    a = {r[key]: r for r in pt}; b = {r[key]: r for r in engine}
    if len(a) != len(pt) or len(b) != len(engine) or a.keys() != b.keys():
        raise ValueError('Frame identities differ or contain duplicates')
    pairs = []
    for frame in a:
        x, y = a[frame], b[frame]
        if x['image_sha256'] != y['image_sha256'] or (x['width'], x['height']) != (y['width'], y['height']):
            raise ValueError(f'Different source pixels for {frame}')
        if x['stage_times_ms']['input_shape_nchw'] != y['stage_times_ms']['input_shape_nchw']:
            raise ValueError(f'Different actual input shapes for {frame}')
        if not x['stage_times_ms'].get('input_tensor_sha256') or x['stage_times_ms']['input_tensor_sha256'] != y['stage_times_ms'].get('input_tensor_sha256'):
            raise ValueError(f'Different or missing normalized input tensor SHA256 for {frame}')
        if 'instances' in x and x['instances'] != y['instances']:
            raise ValueError(f'Different GT instances for {frame}')
        matched, unmatched_pt, unmatched_engine = match_predictions(x['predictions'], y['predictions'])
        pairs.append({'frame': frame, 'matched': matched, 'unmatched_pt_indices': unmatched_pt,
                      'unmatched_engine_indices': unmatched_engine, 'count_pt': len(x['predictions']),
                      'count_engine': len(y['predictions'])})
    matches = [m for row in pairs for m in row['matched']]
    unmatched_pt_count = sum(len(r['unmatched_pt_indices']) for r in pairs)
    unmatched_engine_count = sum(len(r['unmatched_engine_indices']) for r in pairs)
    flips = sum(len(m['threshold_flips']) for m in matches)
    class_errors = sum(not m['class_equal'] for m in matches)
    max_confidence = max((m['confidence_abs_delta'] for m in matches), default=0)
    max_box = max((m['box_linf_delta_px'] for m in matches), default=0)
    min_iou = min((m['box_iou'] for m in matches), default=1)
    gate = unmatched_pt_count == unmatched_engine_count == flips == class_errors == 0 and max_confidence <= 1e-4 and max_box <= .01 and min_iou >= .9999
    return {'frames': len(pairs), 'matched_predictions': len(matches), 'unmatched_pt': unmatched_pt_count,
            'unmatched_engine': unmatched_engine_count, 'matched_threshold_flips': flips,
            'class_errors': class_errors, 'max_confidence_abs_delta': max_confidence,
            'max_box_linf_delta_px': max_box, 'min_matched_box_iou': min_iou,
            'confidence_abs_delta': percentile(m['confidence_abs_delta'] for m in matches),
            'box_linf_delta_px': percentile(m['box_linf_delta_px'] for m in matches),
            'box_iou': percentile(m['box_iou'] for m in matches), 'numerical_gate_pass': gate,
            'input_tensor_sha256_equal_for_every_frame': True, 'pairs': pairs}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--pt-dir', type=Path, required=True)
    p.add_argument('--engine-dir', type=Path, required=True)
    p.add_argument('--imgsz', type=int, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    pt_status = json.loads((args.pt_dir / 'status.json').read_text())
    engine_status = json.loads((args.engine_dir / 'status.json').read_text())
    if pt_status['state'] != 'completed' or engine_status['state'] != 'completed':
        raise ValueError('Evaluation is not completed')
    if pt_status['dataset_manifest_sha256'] != engine_status['dataset_manifest_sha256']:
        raise ValueError('Dataset manifests differ')
    n = args.imgsz
    a = json.loads((args.pt_dir / f'val-raw-{n}.json').read_text())
    b = json.loads((args.engine_dir / f'val-raw-{n}.json').read_text())
    validation = compare_frames(a['frames'], b['frames'], 'id')
    a = json.loads((args.pt_dir / f'diagnostic-raw-{n}.json').read_text())
    b = json.loads((args.engine_dir / f'diagnostic-raw-{n}.json').read_text())
    diagnostic = compare_frames(a['frames'], b['frames'], 'sample')
    report = {'mode': 'saved_outputs_cpu_only', 'imgsz': n,
              'pt_artifact': pt_status['artifact'], 'pt_sha256': pt_status['artifact_sha256'],
              'engine_artifact': engine_status['artifact'], 'engine_sha256': engine_status['artifact_sha256'],
              'dataset_manifest_sha256': pt_status['dataset_manifest_sha256'],
              'validation': validation, 'known_scene_regression': diagnostic,
              'numerical_gate_pass': validation['numerical_gate_pass'] and diagnostic['numerical_gate_pass'],
              'gate': {'unmatched_predictions': 0, 'matched_threshold_flips': 0,
                       'max_confidence_abs_delta': 1e-4, 'max_box_linf_delta_px': .01, 'min_box_iou': .9999},
              'limitations': ['Both backends returned detections after conf=.01. This does not compare all pre-threshold logits.',
                              'Unmatched predictions cannot be attributed solely to confidence crossings from returned boxes.',
                              'Only saved frames are covered. This does not certify arbitrary images or deployment accuracy.',
                              'No instance masks or 3D geometry exist in this single-class detector.'],
              'physical_commands': 0, 'gpu_initialized': False}
    save_json(args.output, report)
    print(json.dumps({k: v for k, v in report.items() if k not in ['validation', 'known_scene_regression']}, ensure_ascii=False))


if __name__ == '__main__':
    main()
