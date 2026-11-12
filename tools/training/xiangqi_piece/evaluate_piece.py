#!/usr/bin/env python3
"""Full-frame single-class detector evaluation; no camera or robot operations.

CUDA is initialized only after a timed, blocking flock on the production
benchmark lock. Run this process outside any other process holding that lock.
"""
from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import time

ROOT = Path('/home/kuang/workspace/xiangqi-yolo-training/20261102-yolo26n-piece-v1')
LOCK = Path('/home/kuang/workspace/chess_robot/runs/perf_ceiling_20261031/benchmark.lock')
THRESHOLDS = (.05, .10, .15, .20, .25, .35, .50)
CENTERS = [(537.5, 431.7), (682.2, 449.9), (827.4, 468.4), (955.8, 490.2)]


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + f'.tmp-{os.getpid()}')
    temp.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    temp.replace(path)


@contextlib.contextmanager
def gpu_lease(timeout_seconds):
    """Blocking queue, not LOCK_NB polling; SIGALRM bounds the wait."""
    import fcntl
    handle = LOCK.open('a')
    acquired = False
    old_alarm = signal.getsignal(signal.SIGALRM)

    def expired(*_):
        raise TimeoutError(f'GPU lock wait exceeded {timeout_seconds}s: {LOCK}')

    started = time.monotonic()
    try:
        signal.signal(signal.SIGALRM, expired)
        signal.setitimer(signal.ITIMER_REAL, timeout_seconds)
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX)
        acquired = True
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, old_alarm)
        yield {'path': str(LOCK), 'wait_seconds': time.monotonic() - started,
               'policy': 'blocking LOCK_EX with SIGALRM timeout; held for entire GPU process'}
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, old_alarm)
        if acquired:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        handle.close()


def iou(a, b):
    intersection = max(0., min(a[2], b[2]) - max(a[0], b[0])) * max(0., min(a[3], b[3]) - max(a[1], b[1]))
    aa = max(0., a[2] - a[0]) * max(0., a[3] - a[1])
    bb = max(0., b[2] - b[0]) * max(0., b[3] - b[1])
    return intersection / max(aa + bb - intersection, 1e-12)


def contains(box, center):
    return box[0] <= center[0] <= box[2] and box[1] <= center[1] <= box[3]


def percentile(values):
    import numpy as np
    a = [float(x) for x in values if x is not None and math.isfinite(x)]
    return {'n': len(a), **({f'p{p}': float(np.percentile(a, p)) for p in (5, 50, 95, 99)} if a else {})}


def match_frame(predictions, instances, threshold, match_iou=.5):
    """Score-ordered one-to-one IoU matching, with geometric duplicate counts."""
    pred = sorted((p for p in predictions if p[4] >= threshold), key=lambda p: -p[4])
    available = set(range(len(instances)))
    matches, false_positives, duplicates = [], [], []
    for pi, p in enumerate(pred):
        ranked = sorted(((iou(p, g['box_xyxy']), gi) for gi, g in enumerate(instances)), reverse=True)
        candidate = next(((overlap, gi) for overlap, gi in ranked if gi in available and overlap >= match_iou), None)
        if candidate is not None:
            overlap, gi = candidate
            available.remove(gi)
            matches.append({'prediction_index': pi, 'instance_index': gi, 'iou': overlap, 'confidence': p[4]})
        else:
            false_positives.append(pi)
            if ranked and ranked[0][0] >= match_iou:
                duplicates.append(pi)
    return {'predictions': pred, 'matches': matches, 'missed_indices': sorted(available),
            'false_positive_indices': false_positives, 'duplicate_indices': duplicates}


def gt_groups(instances, width, height):
    groups = []
    for i, g in enumerate(instances):
        a = g['box_xyxy']
        small = max(a[2] - a[0], a[3] - a[1]) * 640 / max(width, height) <= 48
        adjacent = False
        for j, other in enumerate(instances):
            if i == j:
                continue
            b = other['box_xyxy']
            dx = max(0., max(a[0], b[0]) - min(a[2], b[2]))
            dy = max(0., max(a[1], b[1]) - min(a[3], b[3]))
            unit = min(a[2] - a[0], a[3] - a[1], b[2] - b[0], b[3] - b[1])
            if math.hypot(dx, dy) <= .25 * unit:
                adjacent = True
                break
        groups.append({'small_at640': small, 'adjacent': adjacent})
    return groups


def summarize_validation(frames, source_classes):
    summaries = []
    for threshold in THRESHOLDS:
        total = tp = fp = duplicate = merged = 0
        matched_ious, pred_counts, misses = [], [], []
        classes = {c['name']: {'gt': 0, 'matched': 0} for c in source_classes}
        groups = {n: {'gt': 0, 'matched': 0} for n in ['small_at640', 'adjacent', 'not_small_at640', 'not_adjacent']}
        for f in frames:
            gt = f['instances']
            result = match_frame(f['predictions'], gt, threshold)
            group_flags = gt_groups(gt, f['width'], f['height'])
            matched = {m['instance_index'] for m in result['matches']}
            total += len(gt); tp += len(matched); fp += len(result['false_positive_indices'])
            duplicate += len(result['duplicate_indices']); pred_counts.append(len(result['predictions']))
            matched_ious.extend(m['iou'] for m in result['matches'])
            centers = [((g['box_xyxy'][0] + g['box_xyxy'][2]) / 2,
                        (g['box_xyxy'][1] + g['box_xyxy'][3]) / 2) for g in gt]
            merged += sum(sum(contains(p, c) for c in centers) > 1 for p in result['predictions'])
            for gi, g in enumerate(gt):
                found = int(gi in matched)
                c = classes[g['source_class_name']]; c['gt'] += 1; c['matched'] += found
                for group_name, flag in group_flags[gi].items():
                    key = group_name if flag else 'not_' + group_name
                    groups[key]['gt'] += 1; groups[key]['matched'] += found
            for gi in result['missed_indices']:
                g = gt[gi]
                misses.append({'id': f['id'], 'instance_index': gi, 'source_class_name': g['source_class_name'],
                               'box_xyxy': g['box_xyxy']})
        for row in list(classes.values()) + list(groups.values()):
            row['recall'] = row['matched'] / row['gt'] if row['gt'] else None
        summaries.append({'confidence_threshold': threshold, 'iou_match_threshold': .5,
                          'images': len(frames), 'ground_truth_instances': total,
                          'true_positives': tp, 'false_positives': fp, 'misses': total - tp,
                          'precision': tp / (tp + fp) if tp + fp else None, 'recall': tp / total if total else None,
                          'false_positives_per_image': fp / len(frames),
                          'duplicates_at_iou50': duplicate, 'duplicates_per_image': duplicate / len(frames),
                          'boxes_containing_multiple_gt_centers': merged,
                          'candidate_count_per_image': percentile(pred_counts),
                          'matched_box_iou': percentile(matched_ious), 'recall_by_original_class': classes,
                          'recall_by_geometry_group': groups, 'missed_instances': misses})
    return summaries


class DetectAdapter:
    def __init__(self, artifact, imgsz, conf, rect):
        # Instantiated only while holding gpu_lease.
        import torch
        from ultralytics import YOLO
        self.torch, self.imgsz, self.conf, self.rect = torch, imgsz, conf, rect
        # The official rectangular imgsz argument is identical for PT and TRT.
        # It also makes the official first warmup match the exact engine profile.
        self.predict_size = [math.ceil((720 * imgsz / 1280) / 32) * 32, imgsz] if rect else [imgsz, imgsz]
        self.model = YOLO(str(artifact), task='detect')
        self.fields, self.events = {}, None
        self.hooked = False
        self.capture_input_hash = False

    def install_hooks(self):
        predictor = self.model.predictor
        for method, field in [('preprocess', 'preprocess_host_ms'), ('inference', 'forward_host_ms'),
                              ('postprocess', 'postprocess_host_ms')]:
            original = getattr(predictor, method)

            def measured(*args, _method=method, _field=field, _original=original, **kwargs):
                started = time.perf_counter()
                pair = None
                if _method == 'inference':
                    pair = (self.torch.cuda.Event(enable_timing=True), self.torch.cuda.Event(enable_timing=True))
                    pair[0].record()
                out = _original(*args, **kwargs)
                if pair is not None:
                    pair[1].record(); self.events = pair
                self.fields[_field] = (time.perf_counter() - started) * 1000
                if _method == 'preprocess':
                    self.fields['input_shape_nchw'] = list(out.shape)
                    self.fields['input_dtype'] = str(out.dtype)
                    if max(out.shape[-2:]) != self.imgsz:
                        raise ValueError(f'Actual input {list(out.shape)} does not match requested imgsz {self.imgsz}')
                    if self.capture_input_hash:
                        # Validation/parity evidence only. This blocking copy
                        # and hash are deliberately absent from timed benchmark.
                        self.fields['input_tensor_sha256'] = hashlib.sha256(out.detach().cpu().numpy().tobytes()).hexdigest()
                return out

            setattr(predictor, method, measured)
        self.hooked = True

    def detect(self, image, confidence=None, capture_input_hash=False):
        if tuple(image.shape[:2]) != (720, 1280):
            raise ValueError('This bounded full-frame experiment requires original 1280x720 RGB images')
        self.fields, self.events = {}, None
        self.capture_input_hash = capture_input_hash
        start = time.perf_counter()
        result = self.model.predict(image, device=0, imgsz=self.predict_size,
                                    conf=self.conf if confidence is None else confidence,
                                    half=False, rect=self.rect, verbose=False, save=False,
                                    stream=False, max_det=300, agnostic_nms=False)[0]
        predict_done = time.perf_counter()
        predictions = result.boxes.data.detach().cpu().numpy().copy()
        end = time.perf_counter()
        timing = dict(self.fields)
        timing['predict_wall_ms'] = (predict_done - start) * 1000
        timing['boxes_to_cpu_ms'] = (end - predict_done) * 1000
        timing['detector_wall_ms'] = (end - start) * 1000
        timing['api_total_wall_ms'] = timing['detector_wall_ms']
        if self.events:
            self.events[1].synchronize()
            timing['forward_cuda_ms'] = self.events[0].elapsed_time(self.events[1])
        if not self.hooked:
            self.install_hooks()
        if not self.model.names == {0: 'piece'}:
            raise ValueError(f'Expected single class piece, got {self.model.names}')
        if any(len(p) != 6 or int(p[5]) != 0 or not all(math.isfinite(float(v)) for v in p) for p in predictions):
            raise ValueError('Invalid or non-single-class output')
        backend = self.model.predictor.model
        if bool(getattr(backend, 'fp16', False)):
            raise ValueError('Unexpected FP16 backend')
        return predictions.tolist(), timing


def evaluate_size(args, size, manifest, output):
    import cv2
    started = time.monotonic()
    val = [r for r in manifest['items'] if r['split'] == 'val']
    images = {r['id']: cv2.imread(str(args.dataset / r['image_relative_path'])) for r in val}
    if any(v is None for v in images.values()):
        raise ValueError('Unable to decode a validation image')
    detector = DetectAdapter(args.artifact, size, .01, args.rect)
    cold_predictions, cold_timing = detector.detect(images[val[0]['id']])
    for _ in range(args.warmup):
        detector.detect(images[val[0]['id']], args.benchmark_conf)
    frames = []
    for row in val:
        predictions, timing = detector.detect(images[row['id']], capture_input_hash=True)
        frames.append({**{k: row[k] for k in ['id', 'split', 'group_id', 'width', 'height', 'instances']},
                       'image_sha256': sha256(args.dataset / row['image_relative_path']),
                       'predictions': predictions, 'stage_times_ms': timing})
    save_json(output / f'val-raw-{size}.json', {'imgsz': size, 'confidence': .01, 'frames': frames})
    summary = summarize_validation(frames, manifest['source_classes'])
    diagnostic_frames = []
    if args.diagnostic:
        for path in sorted(args.diagnostic.glob('[0-9][0-9][0-9]/color.png')):
            color = cv2.imread(str(path))
            if color is None:
                raise ValueError(path)
            predictions, timing = detector.detect(color, capture_input_hash=True)
            decisions = []
            for threshold in THRESHOLDS:
                selected = [p for p in predictions if p[4] >= threshold]
                center_counts = [sum(contains(p, c) for p in selected) for c in CENTERS]
                merged_count = sum(sum(contains(p, c) for c in CENTERS) > 1 for p in selected)
                decisions.append({'confidence_threshold': threshold, 'candidate_count': len(selected),
                                  'center_coverage_counts': center_counts,
                                  'exactly_one_box_per_center': center_counts == [1, 1, 1, 1] and len(selected) == 4 and merged_count == 0,
                                  'boxes_containing_multiple_centers': merged_count,
                                  'boxes_containing_no_known_center': sum(not any(contains(p, c) for c in CENTERS) for p in selected)})
            diagnostic_frames.append({'sample': path.parent.name, 'path': str(path), 'image_sha256': sha256(path),
                                      'width': color.shape[1], 'height': color.shape[0],
                                      'predictions': predictions, 'stage_times_ms': timing, 'threshold_results': decisions})
    save_json(output / f'diagnostic-raw-{size}.json', {'frames': diagnostic_frames, 'known_centers': CENTERS,
              'scope': 'Known four-piece stationary regression; approximate center containment, no annotated GT boxes; not independent test or box AP.'})
    # Distinct bounded microbenchmark. Images decoded before timing; full public
    # predict() call and blocking CPU box transfer are included on every call.
    bench = []
    chosen = [val[round(i * (len(val) - 1) / min(8, len(val) - 1))]['id'] for i in range(min(9, len(val)))]
    for i in range(args.benchmark_calls):
        image_id = chosen[i % len(chosen)]
        _, timing = detector.detect(images[image_id], args.benchmark_conf)
        bench.append({'image_id': image_id, **timing})
    fields = ['api_total_wall_ms', 'detector_wall_ms', 'predict_wall_ms', 'preprocess_host_ms', 'forward_host_ms', 'forward_cuda_ms',
              'postprocess_host_ms', 'boxes_to_cpu_ms']
    benchmark = {'calls': args.benchmark_calls, 'warmup_calls': args.warmup + 1,
                 'confidence_threshold': args.benchmark_conf, 'source_image_ids': chosen,
                 'stage_ms': {f: percentile(r.get(f) for r in bench) for f in fields},
                 'input_shapes': sorted({tuple(r['input_shape_nchw']) for r in bench}),
                 'cold_first_call_ms_excluded': cold_timing,
                 'scope': 'Offline isolated synchronous detector calls on saved full RGB. Includes public predict pre/postprocessing and CPU box transfer, excludes file decode, model load, camera, ROI classifier, depth geometry, ROS and display.',
                 'timing_note': 'Ultralytics Profile synchronizes CUDA around stages (six native sync boundaries). Host wrappers exclude outer profile boundaries; CUDA events measure forward. End-to-end wall is measured independently; do not sum quantiles.',
                 'samples': bench}
    report = {'imgsz': size, 'predict_imgsz_hw': detector.predict_size, 'rect': args.rect, 'validation': summary,
              'preprocessing_parity_evidence': 'SHA256 of exact normalized NCHW float input tensor is recorded for every validation/diagnostic image. Hashing adds blocking CPU copy only in these quality passes, never in the separate microbenchmark. Use this same official imgsz=[H,W] for both PT and engine; exact-profile engines cannot accept square ROI input.',
              'group_definitions': {'small_at640': 'GT max side after aspect-preserving resize to max side 640 <=48px, before padding, fixed across candidate input sizes',
                                    'adjacent': 'GT box-to-box Euclidean edge gap <=0.25*minimum side of the two boxes; overlaps count adjacent'},
              'matching': 'Confidence-descending one-to-one greedy IoU>=0.5; unmatched detections overlapping any GT by >=0.5 count duplicates and false positives. Per-source recall only: false positives cannot be assigned a true original class.',
              'benchmark': benchmark, 'diagnostic_frames': len(diagnostic_frames),
              'elapsed_seconds': time.monotonic() - started}
    save_json(output / f'evaluation-{size}.json', report)
    return report


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--artifact', type=Path, required=True)
    p.add_argument('--dataset', type=Path, default=ROOT / 'dataset')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--sizes', default='640,512,416')
    p.add_argument('--lock-timeout', type=float, default=900)
    p.add_argument('--warmup', type=int, default=5)
    p.add_argument('--benchmark-calls', type=int, default=100)
    p.add_argument('--benchmark-conf', type=float, default=.10)
    p.add_argument('--rect', action=argparse.BooleanOptionalAction, default=True)
    p.add_argument('--diagnostic', type=Path, default=Path('/home/kuang/workspace/chess_robot/runs/xiangqi_depth_eval_20261102_01/captures'))
    args = p.parse_args()
    sizes = [int(s) for s in args.sizes.split(',')]
    if any(s not in (640, 512, 416) for s in sizes) or not args.artifact.is_file():
        raise ValueError('Expected existing artifact and bounded sizes 640/512/416')
    if args.warmup < 1 or not 1 <= args.benchmark_calls <= 300 or args.lock_timeout <= 0:
        raise ValueError('Invalid benchmark or lock bounds')
    if args.output.exists() and any(args.output.iterdir()):
        raise FileExistsError(f'Refusing nonempty output directory: {args.output}')
    args.output.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((args.dataset / 'manifest.json').read_text())
    if len([i for i in manifest['items'] if i['split'] == 'val']) != 49:
        raise ValueError('Expected frozen 49-image validation set')
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(InterruptedError('SIGTERM')))
    report = {'state': 'waiting_for_gpu', 'artifact': str(args.artifact), 'artifact_sha256': sha256(args.artifact),
              'script_sha256': sha256(__file__), 'dataset_manifest_sha256': sha256(args.dataset / 'manifest.json'),
              'physical_commands': 0, 'deployment_changed': False, 'sizes': sizes, 'reports': []}
    save_json(args.output / 'status.json', report)
    try:
        with gpu_lease(args.lock_timeout) as lease:
            import torch
            import ultralytics
            import cv2
            torch.set_num_threads(2); cv2.setNumThreads(1)
            torch.set_float32_matmul_precision('highest')
            torch.backends.cuda.matmul.allow_tf32 = False
            torch.backends.cudnn.allow_tf32 = False
            if ultralytics.__version__ != '8.4.32':
                raise RuntimeError('Expected fixed Ultralytics 8.4.32')
            report.update(state='running', gpu_lease=lease, versions={'torch': torch.__version__, 'ultralytics': ultralytics.__version__},
                          precision={'fp16': False, 'matmul_allow_tf32': torch.backends.cuda.matmul.allow_tf32,
                                     'cudnn_allow_tf32': torch.backends.cudnn.allow_tf32})
            save_json(args.output / 'status.json', report)
            for size in sizes:
                report['current_size'] = size; save_json(args.output / 'status.json', report)
                result = evaluate_size(args, size, manifest, args.output)
                report['reports'].append({'imgsz': size, 'path': str(args.output / f'evaluation-{size}.json'),
                                          'detector_wall_ms': result['benchmark']['stage_ms']['detector_wall_ms']})
                torch.cuda.empty_cache()
            report['state'] = 'completed'
        report['gpu_lock_released'] = True
        save_json(args.output / 'status.json', report)
        print(json.dumps(report, ensure_ascii=False))
    except BaseException as exc:
        report.update(state='failed', error=f'{type(exc).__name__}: {exc}', gpu_lock_released=True)
        save_json(args.output / 'status.json', report)
        raise


if __name__ == '__main__':
    main()
