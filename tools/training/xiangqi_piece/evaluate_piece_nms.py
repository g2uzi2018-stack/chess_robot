#!/usr/bin/env python3
"""New bounded evaluation/benchmark with CPU geometric NMS, no deployment.

Original raw evaluations and model artifacts are never modified. The measured
total brackets the entire adapter predict/CPU-transfer/formatting plus NMS.
"""
from __future__ import annotations

import argparse
import json
import signal
import time
from pathlib import Path

from evaluate_piece import (ROOT, THRESHOLDS, CENTERS, DetectAdapter, contains,
                            gpu_lease, iou, percentile, save_json, sha256,
                            summarize_validation)


def cpu_nms(predictions, confidence, iou_threshold):
    """Strict descending-score IoU>threshold suppression, one class, pure CPU."""
    kept = []
    for p in sorted((p for p in predictions if p[4] >= confidence), key=lambda p: -p[4]):
        if not any(iou(p, q) > iou_threshold for q in kept):
            kept.append(p)
    return kept


def infer(detector, image, confidence, nms_iou, capture_hash=False):
    begin = time.perf_counter()
    raw, timing = detector.detect(image, confidence, capture_input_hash=capture_hash)
    pre_nms = time.perf_counter()
    filtered = cpu_nms(raw, confidence, nms_iou)
    end = time.perf_counter()
    timing.update(nms_cpu_ms=(end - pre_nms) * 1000,
                  api_total_with_nms_wall_ms=(end - begin) * 1000,
                  raw_count=len(raw), filtered_count=len(filtered))
    return raw, filtered, timing


def run_size(args, size, manifest):
    import cv2
    val = [r for r in manifest['items'] if r['split'] == 'val']
    images = {r['id']: cv2.imread(str(args.dataset / r['image_relative_path'])) for r in val}
    if any(v is None for v in images.values()):
        raise ValueError('Failed to decode validation RGB')
    detector = DetectAdapter(args.artifact, size, .01, True)
    _, _, cold = infer(detector, images[val[0]['id']], args.benchmark_conf, args.nms_iou)
    for _ in range(args.warmup):
        infer(detector, images[val[0]['id']], args.benchmark_conf, args.nms_iou)
    frames = []
    for row in val:
        raw, filtered, timing = infer(detector, images[row['id']], .01, args.nms_iou, True)
        frames.append({**{k: row[k] for k in ['id', 'split', 'group_id', 'width', 'height', 'instances']},
                       'image_sha256': sha256(args.dataset / row['image_relative_path']),
                       'raw_predictions': raw, 'predictions': filtered,
                       'raw_count': len(raw), 'filtered_count': len(filtered), 'stage_times_ms': timing})
    save_json(args.output / f'val-raw-and-nms-{size}.json', {'imgsz': size, 'raw_confidence': .01,
              'cpu_nms_iou': args.nms_iou, 'frames': frames})
    quality = summarize_validation(frames, manifest['source_classes'])
    diag = []
    for p in sorted(args.diagnostic.glob('[0-9][0-9][0-9]/color.png')):
        image = cv2.imread(str(p))
        if image is None:
            raise ValueError(p)
        raw, filtered, timing = infer(detector, image, .01, args.nms_iou, True)
        thresholds = []
        for confidence in THRESHOLDS:
            selected = [r for r in filtered if r[4] >= confidence]
            counts = [sum(contains(row, center) for row in selected) for center in CENTERS]
            merged = sum(sum(contains(row, center) for center in CENTERS) > 1 for row in selected)
            thresholds.append({'confidence_threshold': confidence, 'candidate_count': len(selected),
                               'center_coverage_counts': counts, 'boxes_containing_multiple_centers': merged,
                               'boxes_containing_no_known_center': sum(not any(contains(row, center) for center in CENTERS) for row in selected),
                               'exactly_one_box_per_center': len(selected) == 4 and counts == [1, 1, 1, 1] and merged == 0})
        diag.append({'sample': p.parent.name, 'image_sha256': sha256(p), 'width': image.shape[1], 'height': image.shape[0],
                     'raw_predictions': raw, 'predictions': filtered, 'raw_count': len(raw), 'filtered_count': len(filtered),
                     'stage_times_ms': timing, 'threshold_results': thresholds})
    save_json(args.output / f'diagnostic-raw-and-nms-{size}.json', {'frames': diag, 'cpu_nms_iou': args.nms_iou,
              'scope': 'Known four-piece stationary scene, approximate center containment only; not independent GT boxes/mAP.'})
    chosen = [val[round(i * (len(val) - 1) / 8)]['id'] for i in range(9)]
    timings = []
    for i in range(args.benchmark_calls):
        image_id = chosen[i % len(chosen)]
        raw, filtered, timing = infer(detector, images[image_id], args.benchmark_conf, args.nms_iou)
        if 'input_tensor_sha256' in timing:
            raise RuntimeError('Hashing leaked into performance microbenchmark')
        timings.append({'image_id': image_id, **timing})
    fields = ['api_total_with_nms_wall_ms', 'api_total_wall_ms', 'detector_wall_ms', 'predict_wall_ms',
              'preprocess_host_ms', 'forward_host_ms', 'forward_cuda_ms', 'postprocess_host_ms', 'boxes_to_cpu_ms', 'nms_cpu_ms']
    result = {'imgsz': size, 'predict_imgsz_hw': detector.predict_size, 'confidence_for_raw_quality_pass': .01,
              'cpu_nms_iou': args.nms_iou, 'cpu_nms_rule': 'descending confidence; suppress if IoU>threshold with any higher-score kept box',
              'validation': quality, 'diagnostic_frames': len(diag),
              'benchmark': {'calls': len(timings), 'confidence_threshold': args.benchmark_conf, 'cpu_nms_iou': args.nms_iou,
                            'stage_ms': {f: percentile(r.get(f) for r in timings) for f in fields},
                            'raw_count': percentile(r['raw_count'] for r in timings),
                            'filtered_count': percentile(r['filtered_count'] for r in timings),
                            'cold_first_call_excluded': cold, 'warmup_calls': args.warmup + 1,
                            'input_tensor_hashing': False, 'samples': timings,
                            'scope': 'Full public predict pre/postprocessing + blocking CPU boxes transfer + Python formatting/adapter validation + pure CPU score-ordered NMS, bracketed by one independent wall-clock interval. Excludes image disk decode, model load/cold warmup, camera, ROI segmenter, depth geometry, ROS and display. This is not the sum of stage quantiles.',
                            'timing_note': 'Ultralytics native profiling retains stage boundary CUDA synchronizations. GPU forward is also measured with CUDA events.'}}
    save_json(args.output / f'evaluation-nms-{size}.json', result)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--artifact', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sizes', required=True)
    parser.add_argument('--dataset', type=Path, default=ROOT / 'dataset')
    parser.add_argument('--diagnostic', type=Path, default=Path('/home/kuang/workspace/chess_robot/runs/xiangqi_depth_eval_20261102_01/captures'))
    parser.add_argument('--benchmark-conf', type=float, default=.10)
    parser.add_argument('--nms-iou', type=float, default=.65)
    parser.add_argument('--warmup', type=int, default=5)
    parser.add_argument('--benchmark-calls', type=int, default=100)
    parser.add_argument('--lock-timeout', type=float, default=900)
    args = parser.parse_args()
    sizes = [int(s) for s in args.sizes.split(',')]
    if not sizes or any(s not in (640, 512, 416) for s in sizes) or not args.artifact.is_file():
        raise ValueError('Invalid artifact or size')
    if args.output.exists() and any(args.output.iterdir()):
        raise FileExistsError(args.output)
    if not .01 <= args.benchmark_conf <= 1 or not 0 < args.nms_iou < 1 or not 1 <= args.benchmark_calls <= 300:
        raise ValueError('Invalid bounded parameters')
    args.output.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((args.dataset / 'manifest.json').read_text())
    if len([r for r in manifest['items'] if r['split'] == 'val']) != 49:
        raise ValueError('Expected 49 frozen val images')
    status = {'state': 'waiting_for_gpu', 'artifact': str(args.artifact), 'artifact_sha256': sha256(args.artifact),
              'script_sha256': sha256(__file__), 'dataset_manifest_sha256': sha256(args.dataset / 'manifest.json'),
              'physical_commands': 0, 'deployed': False, 'sizes': sizes, 'cpu_nms_iou': args.nms_iou, 'reports': []}
    save_json(args.output / 'status.json', status)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(InterruptedError('SIGTERM')))
    try:
        with gpu_lease(args.lock_timeout) as lease:
            import torch
            import ultralytics
            import cv2
            torch.set_num_threads(2);cv2.setNumThreads(1)
            torch.set_float32_matmul_precision('highest')
            torch.backends.cuda.matmul.allow_tf32 = False;torch.backends.cudnn.allow_tf32 = False
            if ultralytics.__version__ != '8.4.32':
                raise RuntimeError('Fixed Ultralytics 8.4.32 required')
            status.update(state='running', gpu_lease=lease,
                          precision={'fp16': False, 'matmul_allow_tf32': False, 'cudnn_allow_tf32': False})
            save_json(args.output / 'status.json', status)
            for size in sizes:
                status['current_size'] = size;save_json(args.output / 'status.json', status)
                report = run_size(args, size, manifest)
                status['reports'].append({'imgsz': size, 'path': str(args.output / f'evaluation-nms-{size}.json'),
                                          'api_total_with_nms_wall_ms': report['benchmark']['stage_ms']['api_total_with_nms_wall_ms']})
                torch.cuda.empty_cache()
        status.update(state='completed', gpu_lock_released=True)
        save_json(args.output / 'status.json', status)
        print(json.dumps(status, ensure_ascii=False))
    except BaseException as exc:
        status.update(state='failed', gpu_lock_released=True, error=f'{type(exc).__name__}: {exc}')
        save_json(args.output / 'status.json', status)
        raise


if __name__ == '__main__':
    main()
