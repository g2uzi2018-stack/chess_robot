#!/usr/bin/env python3
"""Generic pure-URDF/FK report, no-WebGL views, and explicit FCL pair sweeps."""
import argparse,json,sys
from pathlib import Path


def collisions(robot, supplied, pairs, sweep):
    import numpy as np
    try:
        import fcl
    except ImportError as exc:
        raise RuntimeError('FCL NOT_RUN: install python-fcl in the analysis environment') from exc
    from rigid_geometry import origin
    required={name for pair in pairs for name in pair}
    if any(a==b for a,b in pairs):raise ValueError('Collision pairs require two distinct links')
    if not required<=set(robot.links):raise ValueError('Unknown collision-pair link')
    if any(not robot.geometry[name]['collision'] for name in required):raise ValueError('Selected link has no collision geometry')
    objects={}
    for name in required:
        objects[name]=[]
        for i,node in enumerate(robot.links[name].findall('collision')):
            shape=node.find('geometry')[0]
            local=np.eye(4)
            if shape.tag=='mesh':
                mesh=robot.geometry[name]['collision'][i]
                geom=fcl.BVHModel();geom.beginModel(len(mesh.vertices),len(mesh.faces))
                geom.addSubModel(mesh.vertices,mesh.faces);geom.endModel()
            else:
                local=np.asarray(origin(node.find('origin')))
                if shape.tag=='box':geom=fcl.Box(*[float(x) for x in shape.get('size').split()])
                elif shape.tag=='sphere':geom=fcl.Sphere(float(shape.get('radius')))
                elif shape.tag=='cylinder':geom=fcl.Cylinder(float(shape.get('radius')),float(shape.get('length')))
                else:raise ValueError('Unsupported collision shape')
            objects[name].append((fcl.CollisionObject(geom),local))
    positions=[dict(supplied)]
    if sweep:
        joint,low,high,count=sweep
        if joint not in robot.joints or robot.joints[joint].get('type')=='fixed' or robot.joints[joint].find('mimic') is not None:
            raise ValueError('Sweep a supported independent movable joint')
        low,high,count=float(low),float(high),int(count)
        if not np.isfinite([low,high]).all() or low>high or not 2<=count<=10000:raise ValueError('Invalid sweep interval/sample count')
        limit=robot.joints[joint].find('limit')
        if limit is not None and robot.joints[joint].get('type')!='continuous':
            if low<float(limit.get('lower')) or high>float(limit.get('upper')):raise ValueError('Sweep outside declared joint limits')
        positions=[dict(supplied,**{joint:float(q)}) for q in np.linspace(low,high,count)]
    hits=[]
    for index,values in enumerate(positions):
        poses=robot.transforms(values)
        for name in required:
            for obj,local in objects[name]:
                t=poses[name]@local;obj.setTransform(fcl.Transform(t[:3,:3],t[:3,3]))
        for a,b in pairs:
            hit=any(fcl.collide(x,y,fcl.CollisionRequest(),fcl.CollisionResult())>0
                    for x,_ in objects[a] for y,_ in objects[b])
            if hit:hits.append({'sample':index,'positions':values,'links':[a,b]})
    return {'status':'performed','engine':'python-fcl','primitive_shapes':'native FCL',
        'mesh_shapes':'triangle BVH surface intersection; containment without crossing is not certified',
        'pairs':pairs,'sample_count':len(positions),'sweep':sweep,'contacts':hits,
        'scope':'Selected pairs and discrete sampled poses only; no ACM filters, trajectory/CCD or physical calibration.'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf',type=Path)
    parser.add_argument('--package-root',action='append',type=Path,default=[],help='Directory containing ROS package folders; repeat if needed')
    parser.add_argument('--joint',action='append',default=[],help='Independent joint NAME=VALUE; radians or metres as appropriate')
    parser.add_argument('--positions',type=Path,help='JSON object of independent joint positions')
    parser.add_argument('--frame',help='Reference link for reported geometry/frames; default root')
    parser.add_argument('--subtree',help='Restrict displayed/reported geometry to descendants of this link')
    parser.add_argument('--show-frame',action='append',default=[])
    parser.add_argument('--json',type=Path,required=True)
    parser.add_argument('--visual-png',type=Path)
    parser.add_argument('--collision-png',type=Path)
    parser.add_argument('--pair',nargs=2,action='append',default=[],metavar=('LINK_A','LINK_B'))
    parser.add_argument('--sweep',nargs=4,metavar=('JOINT','LOW','HIGH','SAMPLES'))
    parser.add_argument('--require-clear',action='store_true',help='Exit 1 if a requested pair contacts in sampled poses')
    args=parser.parse_args()
    if args.json.suffix.lower()!='.json' or args.json.resolve()==args.urdf.resolve():parser.error('Choose a separate .json report path')
    if args.positions and args.positions.resolve()==args.json.resolve():parser.error('Report must not overwrite positions input')
    for path in (args.visual_png,args.collision_png):
        if path and (path.suffix.lower()!='.png' or path.resolve()==args.urdf.resolve()):parser.error('Choose separate .png image paths')
    try:
        import numpy as np
        from geometry_report import RobotGeometry,render_views,save_json,sha256,versions,mesh_stats
        from rigid_geometry import joint_values
        if (args.sweep or args.require_clear) and not args.pair:raise ValueError('Specify --pair before sweep/require-clear; unrequested collisions are NOT_RUN')
        supplied=json.loads(args.positions.read_text(encoding='utf-8-sig')) if args.positions else {}
        if not isinstance(supplied,dict):raise ValueError('Positions JSON must be an object')
        for item in args.joint:
            name,value=item.split('=',1);supplied[name]=float(value)
        robot=RobotGeometry(args.urdf,args.package_root)
        poses=robot.transforms(supplied);values=joint_values(robot.joints,supplied)
        frame=args.frame or robot.root_link
        if frame not in poses:raise ValueError('Unknown reference frame')
        relative={name:np.linalg.inv(poses[frame])@t for name,t in poses.items()}
        warnings=[];joints=[]
        for name,node in robot.joints.items():
            limit=node.find('limit');mimic=node.find('mimic')
            row={'name':name,'type':node.get('type'),'position':values[name],
                 'position_units':'m' if node.get('type')=='prismatic' else ('rad' if node.get('type') in ('revolute','continuous') else None),
                 'mimic':None if mimic is None else mimic.attrib,'limit':None if limit is None else limit.attrib}
            joints.append(row)
            if limit is not None:
                if node.get('type')!='continuous' and not float(limit.get('lower'))<=values[name]<=float(limit.get('upper')):
                    warnings.append(f'{name}: supplied/default position is outside limits')
                if any(float(limit.get(k,'0'))<=0 for k in ('effort','velocity')):
                    warnings.append(f'{name}: effort/velocity is nonpositive or missing; dynamic execution not validated')
        visual,_=robot.posed(supplied,'visual',frame,args.subtree)
        collision,_=robot.posed(supplied,'collision',frame,args.subtree)
        geometry={}
        for kind,items in (('visual',visual),('collision',collision)):
            geometry[kind]=[{'name':name,'triangles':len(m.faces),'bounds_m':m.bounds.tolist(),'watertight':bool(m.is_watertight)} for name,m in items]
        inertia=[]
        for name,node in robot.links.items():
            body=node.find('inertial')
            if body is None:
                if node.findall('visual') or node.findall('collision'):warnings.append(f'{name}: physical geometry without inertia')
                continue
            mass=float(body.find('mass').get('value'));i=body.find('inertia')
            x={k:float(v) for k,v in i.attrib.items()}
            mat=np.array([[x['ixx'],x['ixy'],x['ixz']],[x['ixy'],x['iyy'],x['iyz']],[x['ixz'],x['iyz'],x['izz']]])
            if not np.isfinite(mat).all() or not np.isfinite(mass):raise ValueError('Non-finite inertia/mass')
            eigen=np.linalg.eigvalsh(mat)
            valid=bool(mass>0 and eigen.min()>0 and eigen[-1]<=eigen[0]+eigen[1]+1e-12)
            inertia.append({'link':name,'mass_kg':mass,'principal_moments_kg_m2':eigen.tolist(),'basic_physical_inequalities':valid})
            if not valid:warnings.append(f'{name}: invalid basic mass/inertia inequalities')
        if robot.root.findall('gazebo'):warnings.append('Gazebo/SDF fragments present; this tool does not validate closed-loop/contact dynamics')
        requested=args.show_frame or [frame]
        if not set(requested)<=set(relative):raise ValueError('Unknown --show-frame link')
        frames={n:relative[n] for n in requested}
        if args.visual_png:render_views(visual,args.visual_png,title=f'URDF visual; frame={frame}; zero for unspecified joints',units='m',frames=frames)
        if args.collision_png:render_views(collision,args.collision_png,title=f'URDF collision geometry; frame={frame}; no padding implied beyond source',units='m',frames=frames)
        report={'schema_version':1,'status':'file_and_fk_report_generated','source':str(args.urdf.resolve()),'sha256':sha256(args.urdf),
            'root_link':robot.root_link,'links':len(robot.links),'joints':joints,'supplied_positions':supplied,'reference_frame':frame,
            'link_poses_in_reference_frame':{n:t.tolist() for n,t in relative.items()},'geometry':geometry,'inertia':inertia,
            'mesh_assets':{str(p):{'sha256':v['sha256'],'quantity_units':'raw mesh coordinates; URDF scale is applied only in posed geometry',**mesh_stats(v['mesh'])} for p,v in robot.assets.items()},
            'warnings':warnings,'physical_calibration':'not_verified','collision':{'status':'NOT_RUN'},'environment':versions(),
            'limitations':['Pure URDF only; fixed/revolute/continuous/prismatic supported, floating/planar rejected.',
                'Unspecified joint positions are zero, not a measured home pose.',
                'Rendering is orthographic triangle projection, not a collision test or proof of dimensional accuracy.']}
        if args.pair:report['collision']=collisions(robot,supplied,args.pair,args.sweep)
        save_json(args.json,report)
        contacts=report['collision'].get('contacts',[])
        print(json.dumps({'status':report['status'],'links':len(robot.links),'warnings':len(warnings),
            'collision_status':report['collision']['status'],'contact_samples':len(contacts),'report':str(args.json)},ensure_ascii=True))
        return 1 if args.require_clear and contacts else 0
    except Exception as exc:
        error={'status':'error','error':str(exc),'physical_calibration':'not_verified','collision':{'status':'NOT_RUN'}}
        args.json.parent.mkdir(parents=True,exist_ok=True)
        args.json.write_text(json.dumps(error,ensure_ascii=True,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(error,ensure_ascii=True),file=sys.stderr)
        return 2


if __name__=='__main__':sys.exit(main())
