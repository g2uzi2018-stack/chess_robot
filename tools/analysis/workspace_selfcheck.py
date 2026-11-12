"""Exact triangle-surface self-collision diagnostics for selected configurations.

No ACM is invented: fixed groups and directly connected joint groups are listed
explicitly, and remaining contacts are reported rather than silently ignored.
"""
import argparse
import collections
import json
import pathlib
import sys
import numpy as np
import fcl


def main():
    p=argparse.ArgumentParser();p.add_argument('--project',type=pathlib.Path,required=True);p.add_argument('--out',type=pathlib.Path,required=True);args=p.parse_args()
    sys.path.insert(0,str(args.project/'tools/description'))
    from geometry_report import RobotGeometry
    model=RobotGeometry(args.project/'src/chess_robot_description/urdf/nova5_with_ag95.urdf',[args.project/'src'])
    home=np.array(json.loads((args.project/'config/arm/home_reference.json').read_text())['joint_positions_rad'])
    group={'base_link':0,**{f'Link{i}':i for i in range(1,7)}}
    links=[name for name,data in model.geometry.items() if data['collision']]
    for name in links:group.setdefault(name,6)
    objects={}
    for name in links:
        objects[name]=[]
        for mesh in model.geometry[name]['collision']:
            geom=fcl.BVHModel();geom.beginModel(len(mesh.vertices),len(mesh.faces));geom.addSubModel(mesh.vertices,mesh.faces);geom.endModel();objects[name].append(fcl.CollisionObject(geom))
    pairs=[(a,b) for i,a in enumerate(links) for b in links[i+1:] if abs(group[a]-group[b])>1]
    def hits(q,opening):
        values={f'joint{i+1}':float(x) for i,x in enumerate(q)};values['left_outer_knuckle_joint']=opening;poses=model.transforms(values)
        for name in links:
            t=poses[name]
            for obj in objects[name]:obj.setTransform(fcl.Transform(t[:3,:3],t[:3,3]))
        return [[a,b] for a,b in pairs if any(fcl.collide(x,y,fcl.CollisionRequest(),fcl.CollisionResult()) for x in objects[a] for y in objects[b])]
    result={'engine':'python-fcl triangle BVH','scope':'selected poses; no volumetric containment certificate; fixed/end-effector groups and direct adjacent groups excluded explicitly','checked_pairs':[list(x) for x in pairs],'group_assignment':group,'home':[]}
    for opening in [0.,.47,.94]:result['home'].append({'opening_rad':opening,'contacts':hits(home,opening)})
    result['sampled_candidate_checks']=[]
    if (args.out/'workspace_data.npz').exists():
        data=np.load(args.out/'workspace_data.npz');candidates=data['best_q'][data['healthy']]
        indices=np.random.default_rng(428).choice(len(candidates),min(16,len(candidates)),replace=False)
        for index in indices:
            for opening in [0.,.47,.94]:
                contacts=hits(candidates[index],opening)
                result['sampled_candidate_checks'].append({'candidate':int(index),'opening_rad':opening,'contacts':contacts})
                if contacts:raise AssertionError('a conservative-clear candidate has a triangle contact: '+str(contacts))
    args.out.mkdir(parents=True,exist_ok=True);(args.out/'self_collision_diagnostics.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':main()
