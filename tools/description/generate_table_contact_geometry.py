#!/usr/bin/env python3
"""Split the unchanged 21-aperture distal envelope for target-local pad checks.

Offline model generation only: no ROS, SDK or device connections.
"""
import argparse,hashlib,json,sys
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial import ConvexHull


def hull(points,name,role=None):
    points=np.unique(points,axis=0);ch=ConvexHull(points)
    indices=ch.vertices;mapping={int(old):new for new,old in enumerate(indices)};faces=[]
    for simplex,equation in zip(ch.simplices,ch.equations):
        ids=list(map(int,simplex));a,b,c=points[ids]
        if np.dot(np.cross(b-a,c-a),equation[:3])<0:ids[1],ids[2]=ids[2],ids[1]
        faces.append([mapping[i] for i in ids])
    result=dict(link=6,name=name,vertices=points[indices].tolist(),faces=faces)
    if role:result['role']=role
    return result


def maximum_outside(points,equations):
    return max(float(np.max(points[i:i+2048]@equations[:,:3].T+equations[:,3]))
               for i in range(0,len(points),2048))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--report',type=Path,required=True);a=p.parse_args()
    sys.path.insert(0,str(a.root/'tools/description'))
    from rigid_geometry import fk,origin,stl_vertices,numbers
    source=a.root/'src/chess_robot_arm/config/nova5_ag95_geometry.yaml'
    old=yaml.safe_load(source.read_text());assert len(old['hulls'])==8
    assert [h['link'] for h in old['hulls'][:7]]==list(range(7)) and old['hulls'][7]['link']==6
    urdf=a.root/'src/chess_robot_description/urdf/nova5_with_ag95.urdf'
    robot=ET.parse(urdf).getroot();collision={}
    for link in robot.findall('link'):
        meshes=[]
        for item in link.findall('collision'):
            mesh=item.find('geometry/mesh')
            if mesh is None:raise ValueError('non-mesh collision must be included explicitly')
            package,relative=mesh.get('filename').removeprefix('package://').split('/',1)
            points=np.unique(np.array(list(stl_vertices(a.root/'src'/package/relative))),axis=0)
            points*=numbers(mesh.get('scale','1 1 1'))
            transform=np.array(origin(item.find('origin')))
            meshes.append(points@transform[:3,:3].T+transform[:3,3])
        collision[link.get('name')]=meshes
    groups={name:[] for name in ('body','left_finger_assembly','right_finger_assembly')}
    excluded={'base_link',*[f'Link{i}' for i in range(1,7)]}
    for angle in np.linspace(0,.94,21):
        poses={k:np.array(v) for k,v in fk(robot,{'left_outer_knuckle_joint':float(angle)}).items()}
        inv=np.linalg.inv(poses['Link6'])
        for name,meshes in collision.items():
            if name in excluded:continue
            relative=inv@poses[name]
            group='left_finger_assembly' if name.startswith('left_') else ('right_finger_assembly' if name.startswith('right_') else 'body')
            for points in meshes:
                groups[group].append(points@relative[:3,:3].T+relative[:3,3])
    meshes={name:np.vstack(points) for name,points in groups.items()}
    split=[hull(meshes['body'],'camera_and_fixed_gripper_body')]
    split += [hull(meshes[name],name+'_21_state_union','finger_assembly') for name in ('left_finger_assembly','right_finger_assembly')]
    old_points=np.array(old['hulls'][7]['vertices']);old_equations=ConvexHull(old_points).equations
    joined=np.vstack([h['vertices'] for h in split]);joined_equations=ConvexHull(joined).equations
    lost=maximum_outside(old_points,joined_equations);added=maximum_outside(joined,old_equations)
    assert lost<1e-8 and added<1e-8,(lost,added)
    coverage={name:maximum_outside(points,ConvexHull(np.array(h['vertices'])).equations)
              for (name,points),h in zip(meshes.items(),split)}
    assert max(coverage.values())<1e-8
    new=dict(old);new['hulls']=old['hulls'][:7]+split
    new['table_contact']=dict(finger_clearance_m=.003,target_radius_m=.080)
    a.output.write_text(yaml.safe_dump(new,sort_keys=False))
    report=dict(source_geometry_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                candidate_geometry_sha256=hashlib.sha256(a.output.read_bytes()).hexdigest(),
                urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),
                original_arm_hulls_unchanged=True,aperture_states=21,raw_to_angle_calibrated=False,
                split_hulls=[dict(name=h['name'],vertices=len(h['vertices']),role=h.get('role','normal')) for h in split],
                recombined_old_envelope_max_outside_m=lost,new_envelope_max_outside_old_m=added,
                mesh_containment_max_outside_m=coverage,robot_commands_sent=0)
    a.report.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))


if __name__=='__main__':main()
