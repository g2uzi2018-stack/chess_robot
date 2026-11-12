#!/usr/bin/env python3
"""Reproduce the registered user-provided camera/mount (numpy and trimesh).

This applies the reviewed CAD datums; it does not calibrate the physical assembly.
The user source and older composite are retained unchanged for comparison.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[2]


def extract():
    directory = ROOT / 'src/chess_robot_description/meshes/tool_mount'
    meta = json.loads((directory / 'camera_mount_assembly_m.json').read_text())
    source = ROOT / meta['source_path']
    assert hashlib.sha256(source.read_bytes()).hexdigest() == meta['source_sha256']
    mesh = trimesh.load_mesh(source, process=True)
    components = sorted(mesh.split(only_watertight=False), key=lambda m: len(m.faces), reverse=True)
    assert len(components) == meta['original_components']
    assert len(components) == meta['retained_components']
    result = mesh.copy()
    result.apply_transform(np.asarray(meta['raw_to_Link6_mm'],dtype=float))
    result.apply_scale(0.001)
    # Recompute normals from registered vertices, rather than retaining normals
    # cached by connected-component analysis before registration.
    result = trimesh.Trimesh(vertices=result.vertices, faces=result.faces, process=False)
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
