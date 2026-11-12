#!/usr/bin/env python3
"""Copy the frozen v1 split and convert its polygons to one-class detect labels.

Uses only the Python standard library. Never opens the live annotation database.
The existing source files and any existing destination dataset are never changed.
"""
from __future__ import annotations

import argparse
import collections
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import struct
import time


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def save_json(path: Path, value) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def require(ok: bool, message: str) -> None:
    if not ok:
        raise ValueError(message)


def build(source: Path, destination: Path) -> dict:
    source = source.resolve(strict=True)
    destination = destination.resolve()
    require(source != destination and source not in destination.parents,
            'Destination must be outside the frozen source dataset')
    require(not destination.exists(), f'Refusing to overwrite existing dataset: {destination}')
    destination.parent.mkdir(parents=True, exist_ok=True)
    stage = destination.parent / f'.{destination.name}-build-{os.getpid()}-{time.time_ns()}'
    stage.mkdir()
    source_manifest_path = source / 'manifest.json'
    source_manifest_sha = digest(source_manifest_path)
    manifest = json.loads(source_manifest_path.read_text(encoding='utf-8'))
    require(len(manifest['items']) == 239, 'Expected original 239 eligible images')
    expected_counts = {'train': {'images': 190, 'objects': 785}, 'val': {'images': 49, 'objects': 198}}
    require(manifest['split_counts'] == expected_counts, 'Unexpected original split counts')
    expected_classes = {int(c['id']): c['name'] for c in manifest['source_classes']}
    require(set(expected_classes) == set(range(14)), 'Expected all 14 original source classes')
    counts = {s: {'images': 0, 'objects': 0} for s in expected_counts}
    classes = {s: collections.Counter() for s in expected_counts}
    ids = set()
    group_splits = collections.defaultdict(set)
    component_splits = collections.defaultdict(set)
    image_hash_splits = collections.defaultdict(set)
    records = []
    source_files = {str(source_manifest_path): source_manifest_sha}
    frozen_output_hashes = {}
    for split in expected_counts:
        (stage / 'images' / split).mkdir(parents=True)
        (stage / 'labels' / split).mkdir(parents=True)
    for item in manifest['items']:
        iid, split = item['id'], item['split']
        require(split in expected_counts, f'Unexpected split {split}')
        require(iid not in ids, f'Duplicate image id {iid}')
        require(iid and iid.isalnum(), f'Unexpected image id characters {iid}')
        ids.add(iid)
        group_splits[item['group_id']].add(split)
        component_splits[item['leakage_component']].add(split)
        image_path = source / 'images' / split / f'{iid}.png'
        label_path = source / 'labels' / split / f'{iid}.txt'
        image_sha = digest(image_path)
        require(image_sha == item['source_sha256'], f'Original image hash mismatch: {iid}')
        source_label_sha = digest(label_path)
        source_files[str(image_path)] = image_sha
        source_files[str(label_path)] = source_label_sha
        image_hash_splits[image_sha].add(split)
        with image_path.open('rb') as stream:
            header = stream.read(24)
        require(header[:8] == b'\x89PNG\r\n\x1a\n' and header[12:16] == b'IHDR', f'Expected PNG: {iid}')
        width, height = struct.unpack('>II', header[16:24])
        require(width > 0 and height > 0, f'Invalid PNG dimensions: {iid}')
        out_image = stage / 'images' / split / image_path.name
        out_label = stage / 'labels' / split / label_path.name
        shutil.copyfile(image_path, out_image)
        require(digest(out_image) == image_sha, f'Copied image mismatch: {iid}')
        lines = label_path.read_text(encoding='utf-8').splitlines()
        require(len(lines) == item['objects'] and lines, f'Object count mismatch: {iid}')
        converted, instances, instance_counts = [], [], collections.Counter()
        for line_index, line in enumerate(lines):
            cells = line.split()
            require(len(cells) >= 7 and (len(cells) - 1) % 2 == 0,
                    f'Not a polygon label: {iid}:{line_index}')
            cid = int(cells[0])
            require(cid in expected_classes, f'Unknown source class {cid}')
            coords = [float(v) for v in cells[1:]]
            require(all(math.isfinite(v) and 0 <= v <= 1 for v in coords),
                    f'Nonfinite/out-of-bounds polygon: {iid}:{line_index}')
            points = list(zip(coords[::2], coords[1::2]))
            area2 = abs(sum(x * points[(k + 1) % len(points)][1] -
                            points[(k + 1) % len(points)][0] * y
                            for k, (x, y) in enumerate(points)))
            require(area2 > 0, f'Degenerate polygon: {iid}:{line_index}')
            x1, y1 = min(coords[::2]), min(coords[1::2])
            x2, y2 = max(coords[::2]), max(coords[1::2])
            require(x2 > x1 and y2 > y1, f'Degenerate bbox: {iid}:{line_index}')
            xywh = [(x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1]
            output = '0 ' + ' '.join(f'{v:.10f}' for v in xywh)
            parsed = list(map(float, output.split()[1:]))
            restored = [parsed[0] - parsed[2] / 2, parsed[1] - parsed[3] / 2,
                        parsed[0] + parsed[2] / 2, parsed[1] + parsed[3] / 2]
            require(max(abs(a - b) for a, b in zip(restored, [x1, y1, x2, y2])) <= 1e-9,
                    f'Label serialization error: {iid}:{line_index}')
            require(all(0 <= v <= 1 for v in parsed), 'Serialized coordinates out of bounds')
            converted.append(output)
            instances.append({'instance_index': line_index, 'source_class_id': cid,
                              'source_class_name': expected_classes[cid], 'class_id': 0,
                              'class_name': 'piece', 'source_polygon_normalized_xy': points,
                              'bbox_normalized_xyxy': [x1, y1, x2, y2],
                              'bbox_normalized_xywh': xywh,
                              'box_xyxy': [x1 * width, y1 * height, x2 * width, y2 * height],
                              'source_polygon_line_sha256': hashlib.sha256(line.encode()).hexdigest()})
            instance_counts[str(cid)] += 1
            classes[split][str(cid)] += 1
        require(dict(instance_counts) == item['class_counts'], f'Source class counts differ: {iid}')
        out_label.write_text('\n'.join(converted) + '\n', encoding='utf-8')
        label_sha = digest(out_label)
        out_image.chmod(0o444)
        out_label.chmod(0o444)
        frozen_output_hashes[str(out_image.relative_to(stage))] = image_sha
        frozen_output_hashes[str(out_label.relative_to(stage))] = label_sha
        counts[split]['images'] += 1
        counts[split]['objects'] += len(instances)
        records.append({**item, 'source_segmentation_label_path': str(label_path),
                        'width': width, 'height': height,
                        'source_segmentation_label_sha256': source_label_sha,
                        'image_relative_path': str(out_image.relative_to(stage)),
                        'label_relative_path': str(out_label.relative_to(stage)),
                        'image_sha256': image_sha, 'label_sha256': label_sha,
                        'instances': instances})
    require(counts == expected_counts, f'Final split counts differ: {counts}')
    for split in expected_counts:
        require(dict(classes[split]) == manifest['objects_by_class'][split], f'Source class totals differ: {split}')
        actual_images = {p.stem for p in (source / 'images' / split).iterdir() if p.suffix == '.png'}
        actual_labels = {p.stem for p in (source / 'labels' / split).iterdir() if p.suffix == '.txt'}
        expected_ids = {i['id'] for i in manifest['items'] if i['split'] == split}
        require(actual_images == actual_labels == expected_ids, f'Source fixed file set differs: {split}')
    require(all(len(v) == 1 for v in group_splits.values()), 'Group crosses splits')
    require(all(len(v) == 1 for v in component_splits.values()), 'Leakage component crosses splits')
    require(all(len(v) == 1 for v in image_hash_splits.values()), 'Same image content crosses splits')
    excluded_ids = {i['id'] for i in manifest['excluded']}
    require(not excluded_ids & ids, 'Excluded source image included')
    require(len(excluded_ids) == 39, 'Expected all 37 uncertain plus 2 rejected exclusions')
    for path, expected in source_files.items():
        require(digest(Path(path)) == expected, f'Original changed during conversion: {path}')
    output_manifest = {
        'schema': 'xiangqi-piece-detect-dataset/v1',
        'created_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'source_dataset': str(source), 'destination_dataset': str(destination),
        'source_manifest_sha256': source_manifest_sha,
        'source_annotation_hash': manifest['dataset_annotation_hash'],
        'source_classes': manifest['source_classes'], 'names': {'0': 'piece'},
        'conversion': 'Each frozen normalized segmentation polygon -> exact axis-aligned min/max bbox -> class 0 xywh; no relabeling, no cropping, no augmentation during conversion',
        'split_counts': counts, 'source_objects_by_class': {s: dict(v) for s, v in classes.items()},
        'groups': manifest['groups'], 'items': records, 'excluded': manifest['excluded'],
        'split_limitations': manifest['split_limitations'],
        'source_files_sha256': source_files, 'fixed_data_files_sha256': frozen_output_hashes,
    }
    save_json(stage / 'manifest.json', output_manifest)
    save_json(stage / 'data.yaml', {'path': str(destination), 'train': 'images/train',
                                   'val': 'images/val', 'nc': 1, 'names': {'0': 'piece'}})
    audit = {
        'status': 'passed', 'source_dataset_unchanged': True, 'source_fixed_files_verified': len(source_files),
        'source_manifest_sha256': source_manifest_sha, 'conversion_manifest_sha256': digest(stage / 'manifest.json'),
        'split_counts': counts, 'total_images': len(ids), 'total_objects': sum(v['objects'] for v in counts.values()),
        'source_objects_by_class': {s: dict(v) for s, v in classes.items()},
        'excluded_images_preserved': len(excluded_ids), 'same_source_image_cross_split': 0,
        'cross_split_groups': 0, 'cross_split_leakage_components': 0,
        'finite_in_bounds_nondegenerate_polygons_and_boxes': True,
        'polygon_bbox_serialization_max_tolerance_normalized': 1e-9,
        'copied_image_sha256_verified': True, 'labels_and_images_read_only': True,
        'new_dataset': str(destination), 'original_database_opened': False, 'gpu_used': False,
        'deployment_ready': False,
    }
    save_json(stage / 'conversion-audit.json', audit)
    # Publish only a complete dataset; destination has been checked absent.
    require(not destination.exists(), f'Destination appeared during conversion: {destination}')
    stage.rename(destination)
    return audit


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, default=Path('/home/kuang/workspace/xiangqi-yolo-training/20261031-yolo26s-seg-v1/dataset'))
    parser.add_argument('--destination', type=Path, default=Path('/home/kuang/workspace/xiangqi-yolo-training/20261102-yolo26n-piece-v1/dataset'))
    args = parser.parse_args()
    print(json.dumps(build(args.source, args.destination), ensure_ascii=False, indent=2))
