"""Offline URDF/mesh evidence only. Does not connect to any robot."""
import argparse
import csv
import hashlib
import json
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p = argparse.ArgumentParser()
p.add_argument('repository', type=Path)
p.add_argument('run', type=Path)
a = p.parse_args()
root = a.repository
run = a.run
rows = list(csv.DictReader((run / 'model_fk.csv').open()))
urdf = ET.parse(root / 'src/chess_robot_description/urdf/nova5.urdf').getroot()
meshes = []
references = urdf.findall('.//mesh')
for file in sorted((root / 'src/chess_robot_description/meshes/nova5').glob('*.STL')):
    data = file.read_bytes()
    triangles = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84 + 50 * triangles:
        raise ValueError(f'Unexpected binary STL size: {file}')
    vertices = [struct.unpack_from('<fff', data, 84 + i*50 + 12 + j*12)
                for i in range(triangles) for j in range(3)]
    low = [min(v[k] for v in vertices) for k in range(3)]
    high = [max(v[k] for v in vertices) for k in range(3)]
    meshes.append({'file': file.name, 'triangles': triangles, 'min_assuming_m': low,
                   'max_assuming_m': high, 'extent_assuming_m': [h-l for h,l in zip(high,low)],
                   'sha256': hashlib.sha256(data).hexdigest()})
missing = []
for m in references:
    uri = m.attrib['filename']
    if not uri.startswith('package://chess_robot_description/'):
        missing.append(uri)
    elif not (root/'src/chess_robot_description'/uri.split('/',3)[3]).is_file():
        missing.append(uri)
summary = {'label': 'OFFLINE MODEL ONLY; no physical trajectory or actual feedback',
           'mesh_references': len(references), 'missing_meshes': missing, 'mesh_count':len(meshes),
           'meshes': meshes, 'poses':len(rows),
           'max_kdl_eigen_position_error_m':max(float(r['kdl_eigen_position_error_m']) for r in rows),
           'max_kdl_eigen_rotation_matrix_error':max(float(r['kdl_eigen_rotation_matrix_error']) for r in rows),
           'candidate_min_scaled_sv': min(float(r['jacobian_min_sv_scaled']) for r in rows[1:]),
           'candidate_max_scaled_condition':max(float(r['jacobian_condition_scaled']) for r in rows[1:]),
           'actual_robot_frames':0, 'mesh_units_note':'STL does not encode units. Metre interpretation agrees with URDF link lengths; physical scale unverified.'}
(run/'model_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
fig=plt.figure(figsize=(13,5), constrained_layout=True)
ax=fig.add_subplot(121, projection='3d')
ax.scatter([float(r['x_m']) for r in rows[1:]],[float(r['y_m']) for r in rows[1:]],[float(r['z_m']) for r in rows[1:]],s=35,label='12 screened model poses')
zero=rows[0];ax.scatter([float(zero['x_m'])],[float(zero['y_m'])],[float(zero['z_m'])],marker='x',c='crimson',s=70,label='zero: singular diagnostic')
ax.set(xlabel='Base X (m)',ylabel='Base Y (m)',zlabel='Base Z (m)',title='URDF Link6 positions; no connecting path')
ax.legend(loc='upper left',fontsize=8)
ax=fig.add_subplot(122)
x=list(range(13))
ax.semilogy(x,[max(1e-18,float(r['kdl_eigen_position_error_m'])) for r in rows],'o-',label='position norm (m)')
ax.semilogy(x,[max(1e-18,float(r['kdl_eigen_rotation_matrix_error'])) for r in rows],'s-',label='rotation matrix norm')
ax.axhline(1e-10,color='crimson',linestyle='--',label='offline numeric thresholds')
ax.set(xlabel='Synthetic pose index (0 = zero diagnostic)',ylabel='Numerical residual (separate units)',title='Same-URDF KDL vs Eigen consistency')
ax.grid(alpha=.25);ax.legend(fontsize=8)
fig.suptitle('OFFLINE MODEL CHECK ONLY - no robot motion, TCP precision or live feedback claim',fontsize=12)
fig.savefig(run/'offline_model.png',dpi=160)
print(json.dumps({k:v for k,v in summary.items() if k!='meshes'},indent=2))
if missing or len(meshes)!=7 or len(references)!=14:
    raise ValueError('Mesh reference check failed')
