#!/usr/bin/env python3
"""Reproduce the reviewed camera/mount selection (needs numpy and trimesh).

This does not identify installation datums or calibrate the physical assembly.
The original composite is retained unchanged for comparison.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[2]


def extract():
    directory = ROOT / 'src/chess_robot_description/meshes/tool_mount'
    meta = json.loads((directory / 'camera_mount_assembly_m.json').read_text())
    source = directory / meta['source']
    assert hashlib.sha256(source.read_bytes()).hexdigest() == meta['source_sha256']
    legacy = ET.parse(ROOT / 'reference/ag95/nova5_with_ag95.original.xacro')
    visual = legacy.find(".//link[@name='camera_bracket_link']/visual")
    mesh = trimesh.load_mesh(source, process=True)
    mesh.apply_scale(np.fromstring(visual.find('geometry/mesh').get('scale'), sep=' '))
    origin = visual.find('origin')
    matrix = trimesh.transformations.euler_matrix(*np.fromstring(origin.get('rpy'), sep=' '), axes='sxyz')
    matrix[:3, 3] = np.fromstring(origin.get('xyz'), sep=' ')
    mesh.apply_transform(matrix)
    components = sorted(mesh.split(only_watertight=False), key=lambda m: len(m.faces), reverse=True)
    selected = [m for m in components if m.bounds[1, 1] < -0.028]
    assert len(components) == meta['original_components']
    assert len(selected) == meta['retained_components']
    result = trimesh.util.concatenate(selected)
    assert len(result.faces) == meta['triangles']
    content = result.export(file_type='stl')
    assert hashlib.sha256(content).hexdigest() == meta['output_sha256'], 'Extraction changed; review before updating the manifest'
    return directory / meta['output'], content


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    path, content = extract()
    if args.check:
        assert path.read_bytes() == content, 'Derived mesh is stale'
    else:
        path.write_bytes(content)
    print(f"{'check' if args.check else 'write'}: {path.relative_to(ROOT)}")


if __name__ == '__main__':
    main()
