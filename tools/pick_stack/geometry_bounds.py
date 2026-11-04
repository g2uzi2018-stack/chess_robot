#!/usr/bin/env python3
"""Necessary endpoint floor bound from unchanged Link6 hulls, not full planning."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml


def bounds(config, guard_path):
    raw = guard_path.read_bytes()
    guard = yaml.safe_load(raw)
    x, y, z, w = config['orientation_xyzw']
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    link6_tcp = np.array(guard['tcp']).reshape(4, 4)
    tcp_link6 = np.linalg.inv(link6_tcp)
    minimum = min((rotation @ (tcp_link6 @ np.r_[vertex, 1])[:3])[2]
                  for hull in guard['hulls'] if hull['link'] == 6 for vertex in hull['vertices'])
    floor = guard['ground_z_m']+guard['clearance_m']
    return dict(schema='xiangqi_pick_stack_floor_bound/v1', guard_path=str(guard_path),
                guard_sha256=hashlib.sha256(raw).hexdigest(),
                orientation_xyzw=config['orientation_xyzw'], floor_z_m=floor,
                lowest_fixed_link6_hull_offset_from_nominal_tcp_z_m=float(minimum),
                necessary_nominal_tcp_z_m=float(floor-minimum),
                full_arm_path_validated=False, real_tcp_calibrated=False,
                payload_stack_collision_checked=False,
                uncalibrated_top_endpoint_probes=[
                    dict(layer=k, nominal_tcp_z_m=config['table_z_m']+k*config['piece_thickness_m'],
                         fixed_link6_hull_min_z_m=float(config['table_z_m']+k*config['piece_thickness_m']+minimum),
                         necessary_floor_condition_met=bool(config['table_z_m']+k*config['piece_thickness_m']+minimum >= floor))
                    for k in (1, 2, 8)])


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', required=True)
    p.add_argument('--guard', required=True)
    p.add_argument('--output', required=True)
    a = p.parse_args()
    result = bounds(json.loads(Path(a.config).read_text()), Path(a.guard))
    Path(a.output).write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
