#!/usr/bin/env python3
"""Independent CPU-only integrity/metric audit of saved piece model results."""
from __future__ import annotations

import collections
import hashlib
import json
import math
from pathlib import Path
import time

ROOT = Path('/home/kuang/workspace/xiangqi-yolo-training/20261102-yolo26n-piece-v1')
EXPECTED_PT_SHA = '980551baa725bc1796d3d5ef1644224a33256cb99890f2b971b4eac93fc558b4'
SIZES = (640, 512, 416)
SHAPES = {640: [1, 3, 384, 640], 512: [1, 3, 288, 512], 416: [1, 3, 256, 416]}


def read(path):
    return json.loads(Path(path).read_text())


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def overlap(a, b):
    ax, ay, xx, yy = a[:4]
    bx, by, xxx, yyy = b[:4]
    common = max(min(xx, xxx) - max(ax, bx), 0) * max(min(yy, yyy) - max(ay, by), 0)
    return common / max((xx - ax) * (yy - ay) + (xxx - bx) * (yyy - by) - common, 1e-12)


def optimal_indices(pred, gt):
    """Maximum-cardinality IoU>=.5 matching, an independent recall upper bound."""
    graph = {i: [j for j, g in enumerate(gt) if overlap(p, g['box_xyxy']) >= .5] for i, p in enumerate(pred)}
    owner = {}

    def place(i, seen):
        for j in graph[i]:
            if j in seen:
                continue
            seen.add(j)
            if j not in owner or place(owner[j], seen):
                owner[j] = i
                return True
        return False

    for i in graph:
        place(i, set())
    return set(owner)


def optimal_count(pred, gt):
    return len(optimal_indices(pred, gt))


def suppress_duplicates(pred, threshold, nms_iou):
    selected = sorted([p for p in pred if p[4] >= threshold], key=lambda p: -p[4])
    kept = []
    for p in selected:
        if not any(overlap(p, q) > nms_iou for q in kept):
            kept.append(p)
    return kept


def cpu_nms_review():
    # This is an offline quality ablation only, not a production modification.
    centers = [(537.5, 431.7), (682.2, 449.9), (827.4, 468.4), (955.8, 490.2)]
    rows = []
    for size in SIZES:
        directory = ROOT / f'evaluation/trt-{size}'
        raw = read(directory / f'val-raw-{size}.json')['frames']
        diag = read(directory / f'diagnostic-raw-{size}.json')['frames']
        for threshold in [.05, .10, .15, .20, .25, .35, .50]:
            for nms_iou in [.65, .70, .80, .90]:
                tp = fp = duplicates = merged = adjacent_gt = adjacent_tp = 0
                missed = []
                classes = collections.defaultdict(lambda: {'gt': 0, 'matched': 0})
                for f in raw:
                    p = suppress_duplicates(f['predictions'], threshold, nms_iou)
                    matched = optimal_indices(p, f['instances'])
                    tp += len(matched); fp += len(p) - len(matched)
                    duplicates += sum(any(overlap(x, g['box_xyxy']) >= .5 for g in f['instances']) for x in p) - len(matched)
                    for gi, g in enumerate(f['instances']):
                        classes[g['source_class_name']]['gt'] += 1
                        classes[g['source_class_name']]['matched'] += gi in matched
                        a = g['box_xyxy']
                        close = False
                        for gj, other in enumerate(f['instances']):
                            if gi == gj:
                                continue
                            b = other['box_xyxy']
                            gap = math.hypot(max(0., max(a[0], b[0]) - min(a[2], b[2])), max(0., max(a[1], b[1]) - min(a[3], b[3])))
                            close |= gap <= .25 * min(a[2] - a[0], a[3] - a[1], b[2] - b[0], b[3] - b[1])
                        adjacent_gt += close; adjacent_tp += close and gi in matched
                        if gi not in matched:
                            missed.append({'id': f['id'], 'instance_index': gi, 'source_class_name': g['source_class_name']})
                    for x in p:
                        inside = sum(x[0] <= (g['box_xyxy'][0] + g['box_xyxy'][2]) / 2 <= x[2] and
                                     x[1] <= (g['box_xyxy'][1] + g['box_xyxy'][3]) / 2 <= x[3] for g in f['instances'])
                        merged += inside > 1
                exact = 0
                for f in diag:
                    p = suppress_duplicates(f['predictions'], threshold, nms_iou)
                    counts = [sum(x[0] <= cx <= x[2] and x[1] <= cy <= x[3] for x in p) for cx, cy in centers]
                    merger = any(sum(x[0] <= cx <= x[2] and x[1] <= cy <= x[3] for cx, cy in centers) > 1 for x in p)
                    exact += len(p) == 4 and counts == [1, 1, 1, 1] and not merger
                rows.append({'imgsz': size, 'confidence_threshold': threshold, 'nms_iou': nms_iou,
                             'tp': tp, 'fp': fp, 'precision': tp / (tp + fp) if tp + fp else None,
                             'recall': tp / 198, 'duplicates': duplicates, 'merged_center_boxes': merged,
                             'diagnostic_all_four_exact': exact, 'missed_instances': missed,
                             'adjacent_gt': adjacent_gt, 'adjacent_tp': adjacent_tp,
                             'recall_by_source_class': dict(classes)})
    return {'scope': 'CPU-only offline score-ordered single-class IoU NMS quality ablation on saved predictions. Does not add NMS to actual model artifacts or measured API pipeline. Max-cardinality IoU>=.5 GT matching is used as geometric recall upper bound.',
            'rows': rows, 'measured_end_to_end_with_nms': False,
            'limitation': 'Suppressing boxes can suppress real overlapping/stacked instances in unseen scenes. Existing adjacent scenes and 12 known frames do not certify arbitrary crowding or occlusion.'}


def main():
    pipe = read(ROOT / 'runtime/pipeline-status.json')
    if pipe['state'] != 'completed':
        raise RuntimeError(f'Pipeline not completed: {pipe}')
    manifest = read(ROOT / 'dataset/manifest.json')
    gt_rows = {r['id']: r for r in manifest['items'] if r['split'] == 'val'}
    gt_classes = collections.Counter(g['source_class_name'] for r in gt_rows.values() for g in r['instances'])
    errors, warnings = [], []

    def require(ok, message):
        if not ok:
            errors.append(message)

    require(len(gt_rows) == 49 and sum(gt_classes.values()) == 198, 'Validation size is not 49/198')
    small = sum(max(g['box_xyxy'][2] - g['box_xyxy'][0], g['box_xyxy'][3] - g['box_xyxy'][1]) * 640 / max(r['width'], r['height']) <= 48
                for r in gt_rows.values() for g in r['instances'])
    minimum_maxside = min(max(g['box_xyxy'][2] - g['box_xyxy'][0], g['box_xyxy'][3] - g['box_xyxy'][1]) * 640 / max(r['width'], r['height'])
                          for r in gt_rows.values() for g in r['instances'])
    if not small:
        warnings.append(f'No validation GT falls in small_at640<=48px group; small-target recall is N/A, not 100%. Smallest GT max side at640 is {minimum_maxside:.2f}px.')
    summaries = {}
    all_tensor_hashes = True
    for label in ['pt', 'trt-640', 'trt-512', 'trt-416']:
        directory = ROOT / 'evaluation' / label
        status = read(directory / 'status.json')
        require(status['state'] == 'completed', label + ' evaluation not completed')
        require(status.get('gpu_lock_released') is True, label + ' lock-release flag missing')
        require(status['precision'] == {'fp16': False, 'matmul_allow_tf32': False, 'cudnn_allow_tf32': False}, label + ' precision controls mismatch')
        require(status['dataset_manifest_sha256'] == digest(ROOT / 'dataset/manifest.json'), label + ' manifest SHA mismatch')
        if label == 'pt':
            require(status['artifact_sha256'] == EXPECTED_PT_SHA, 'PT artifact SHA mismatch')
        else:
            require(digest(status['artifact']) == status['artifact_sha256'], label + ' artifact content mismatch')
        for size in SIZES if label == 'pt' else [int(label.split('-')[1])]:
            raw = read(directory / f'val-raw-{size}.json')['frames']
            diag = read(directory / f'diagnostic-raw-{size}.json')['frames']
            report = read(directory / f'evaluation-{size}.json')
            require(len(raw) == 49 and {r['id'] for r in raw} == gt_rows.keys(), f'{label}/{size} frame identities differ')
            require(len(diag) == 12, f'{label}/{size} diagnostic count is not 12')
            for frame in raw + diag:
                require(frame['stage_times_ms']['input_shape_nchw'] == SHAPES[size], f'{label}/{size} wrong tensor shape')
                require(frame['stage_times_ms']['input_dtype'] == 'torch.float32', f'{label}/{size} input not float32')
                all_tensor_hashes &= len(frame['stage_times_ms'].get('input_tensor_sha256', '')) == 64
            for frame in raw:
                require(frame['instances'] == gt_rows[frame['id']]['instances'], f'{label}/{size} source GT changed')
                require(frame['width'] == 1280 and frame['height'] == 720, f'{label}/{size} source resolution changed')
            table = []
            for q in report['validation']:
                threshold = q['confidence_threshold']
                selected = [[p for p in f['predictions'] if p[4] >= threshold] for f in raw]
                count = sum(len(x) for x in selected)
                optimum = sum(optimal_count(p, f['instances']) for p, f in zip(selected, raw))
                require(q['true_positives'] + q['false_positives'] == count, f'{label}/{size}/{threshold} prediction accounting mismatch')
                require(q['true_positives'] + q['misses'] == 198 and len(q['missed_instances']) == q['misses'], f'{label}/{size}/{threshold} GT accounting mismatch')
                require(abs(q['recall'] - q['true_positives'] / 198) < 1e-12, f'{label}/{size}/{threshold} recall mismatch')
                require(q['true_positives'] <= optimum, f'{label}/{size}/{threshold} matches exceed optimal bound')
                require(q['duplicates_at_iou50'] <= q['false_positives'], f'{label}/{size}/{threshold} duplicate FP mismatch')
                require(q['recall_by_geometry_group']['small_at640']['gt'] == small, f'{label}/{size} small group count mismatch')
                require(q['recall_by_geometry_group']['small_at640']['recall'] is None if small == 0 else True, f'{label}/{size} empty small group is not N/A')
                for name, n in gt_classes.items():
                    c = q['recall_by_original_class'][name]
                    require(c['gt'] == n, f'{label}/{size}/{threshold} class GT count mismatch {name}')
                known = []
                for d in diag:
                    known.append(next(r for r in d['threshold_results'] if r['confidence_threshold'] == threshold))
                row = {'threshold': threshold, 'tp': q['true_positives'], 'fp': q['false_positives'], 'misses': q['misses'],
                       'precision': q['precision'], 'recall': q['recall'], 'duplicates': q['duplicates_at_iou50'],
                       'merged_center_boxes': q['boxes_containing_multiple_gt_centers'],
                       'optimal_matching_tp_upper_bound': optimum,
                       'red_pawn': q['recall_by_original_class']['red_pawn'], 'black_elephant': q['recall_by_original_class']['black_elephant'],
                       'adjacent': q['recall_by_geometry_group']['adjacent'],
                       'diagnostic_all_four_exact': sum(r['exactly_one_box_per_center'] for r in known),
                       'diagnostic_total_extra_or_merged': sum(r['boxes_containing_multiple_centers'] + r['boxes_containing_no_known_center'] for r in known),
                       'missed_instances': q['missed_instances']}
                table.append(row)
                if optimum != q['true_positives']:
                    warnings.append(f'{label}/{size}/{threshold}: greedy TP={q["true_positives"]}, maximum-cardinality TP={optimum}; ambiguous matching exists.')
            bench = report['benchmark']
            require(bench['calls'] == 100 and len(bench['samples']) == 100, f'{label}/{size} benchmark count differs')
            require(not any('input_tensor_sha256' in b for b in bench['samples']), f'{label}/{size} hashing leaked into benchmark')
            require(all(b['input_shape_nchw'] == SHAPES[size] for b in bench['samples']), f'{label}/{size} benchmark shape differs')
            require(all(b['api_total_wall_ms'] == b['detector_wall_ms'] and b['api_total_wall_ms'] >= b['boxes_to_cpu_ms'] for b in bench['samples']), f'{label}/{size} API timing accounting error')
            summaries[f'{label}/{size}'] = {'artifact': status['artifact'], 'artifact_sha256': status['artifact_sha256'],
                                           'threshold_table': table, 'benchmark_conf': bench['confidence_threshold'],
                                           'stage_ms': bench['stage_ms'], 'scope': bench['scope'],
                                           'hash_recorded_for_quality_passes_only': True}
    require(all_tensor_hashes, 'Some validation/diagnostic input tensor hashes are missing')
    parity = {}
    for size in SIZES:
        p = read(ROOT / f'evaluation/parity-{size}.json')
        require(p['numerical_gate_pass'], f'{size} backend numerical gate failed')
        require(p['validation']['frames'] == 49 and p['known_scene_regression']['frames'] == 12, f'{size} parity frame count mismatch')
        for scope in ['validation', 'known_scene_regression']:
            require(p[scope]['input_tensor_sha256_equal_for_every_frame'], f'{size} preprocessing tensor mismatch')
            require(p[scope]['unmatched_pt'] == p[scope]['unmatched_engine'] == p[scope]['matched_threshold_flips'] == 0, f'{size} parity output mismatch')
        parity[size] = {k: v for k, v in p.items() if k not in ['validation', 'known_scene_regression']}
        parity[size]['quality_summaries'] = {scope: {k: v for k, v in p[scope].items() if k != 'pairs'} for scope in ['validation', 'known_scene_regression']}
    baseline = next(r for r in summaries['trt-640/640']['threshold_table'] if r['threshold'] == .1)
    usable = []
    for size in SIZES:
        row = next(r for r in summaries[f'trt-{size}/{size}']['threshold_table'] if r['threshold'] == .1)
        if row['tp'] >= baseline['tp'] and row['fp'] <= baseline['fp'] and row['duplicates'] <= baseline['duplicates'] and row['red_pawn']['matched'] >= baseline['red_pawn']['matched'] and row['black_elephant']['matched'] >= baseline['black_elephant']['matched'] and row['adjacent']['matched'] >= baseline['adjacent']['matched'] and row['diagnostic_all_four_exact'] == 12:
            usable.append(size)
    recommendation = {'threshold': .10, 'smallest_size_without_recall_or_fp_regression_vs640_in_this_validation': min(usable) if usable else None,
                      'promotion_scope': 'Candidate integration only; not independent deployment acceptance. Size choice is conditional on this fixed held-out scene set and 12 known four-piece stationary frames.',
                      'criterion': 'At conf=.10: TP/source-red-pawn/source-black-elephant/adjacent >=640 baseline, FP/duplicates <=640, all 12 diagnostics exactly four independent center-containing boxes.'}
    warnings.extend(['Original split is same acquisition day, camera and physical chess set; model tuning used this validation set.',
                     'Twelve diagnostic frames are the same known four-piece stationary scene and are not training or independent generalization tests.',
                     'Offline speed includes official predict/preprocessing/postprocessing/blocking CPU box transfer; no file decode, camera, ROI segmenter, depth geometry or live ROS pipeline.',
                     'One single class piece does not prove detection of boards or arbitrary objects; stacked/face-down/full-board/dynamic occlusion remain unvalidated.',
                     'Zero observed false positives/duplicates in this dataset is not a general false-positive guarantee.'])
    nms = cpu_nms_review()
    (ROOT / 'runtime/nms-quality-review.json').write_text(json.dumps(nms, indent=2, ensure_ascii=False, allow_nan=False))
    report = {'state': 'passed' if not errors else 'failed', 'reviewer': 'piece_eval independent CPU audit', 'created_unix': time.time(),
              'gpu_initialized': False, 'physical_commands': 0, 'production_changed': False,
              'expected_pt_sha256': EXPECTED_PT_SHA, 'val_images': len(gt_rows), 'val_instances': sum(gt_classes.values()),
              'source_class_counts': dict(gt_classes), 'small_group_gt': small, 'minimum_gt_max_side_at640': minimum_maxside,
              'summaries': summaries, 'parity': parity, 'recommendation': recommendation, 'cpu_nms_offline_ablation': nms, 'warnings': warnings, 'errors': errors,
              'review_script_sha256': digest(__file__)}
    output = ROOT / 'runtime/evaluation-independent-review.json'
    temp = output.with_suffix('.tmp');temp.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False));temp.replace(output)
    print(json.dumps({'state': report['state'], 'recommendation': recommendation, 'errors': errors, 'warnings': warnings}, ensure_ascii=False))


if __name__ == '__main__':
    main()
