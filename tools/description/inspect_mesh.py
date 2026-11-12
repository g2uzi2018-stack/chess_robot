#!/usr/bin/env python3
"""Read mesh dimensions/topology and render orthographic views; never edits input."""
import argparse
import json
import sys
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mesh',type=Path)
    parser.add_argument('--units',choices=('unknown','mm','m'),default='unknown',help='Declared by user/CAD, never inferred from file name')
    parser.add_argument('--json',type=Path,required=True)
    parser.add_argument('--png',type=Path)
    args=parser.parse_args()
    if args.json.suffix.lower()!='.json' or args.json.resolve()==args.mesh.resolve():parser.error('Choose a separate .json report path')
    if args.png and (args.png.suffix.lower()!='.png' or args.png.resolve()==args.mesh.resolve()):parser.error('Choose a separate .png image path')
    try:
        from geometry_report import load_mesh,components,mesh_stats,plane_candidates,render_views,sha256,versions,save_json
        mesh=load_mesh(args.mesh); parts=components(mesh)
        factor={'mm':.001,'m':1.}.get(args.units)
        report={'schema_version':1,'status':'report_generated','scope':'file_geometry_only',
            'source':str(args.mesh.resolve()),'sha256':sha256(args.mesh),'declared_units':args.units,
            'unit_source':'not_defined' if factor is None else 'explicit_user_argument',
            'quantity_units':{'mesh_and_components':'raw source coordinates; lengths U, area U^2, volume U^3',
                              'bounds_m_and_extents_m':'metres only when units explicitly supplied'},
            'mesh':mesh_stats(mesh),'component_count':len(parts),
            'components':[{'index':i,**mesh_stats(m)} for i,m in enumerate(parts)],
            'axis_aligned_plane_candidates':plane_candidates(mesh),
            'bounds_m':None if factor is None else (mesh.bounds*factor).tolist(),
            'extents_m':None if factor is None else (mesh.extents*factor).tolist(),
            'limitations':['STL does not provide reliable units, semantic part names, installation datums, material or measured mass.',
                'Connected components are geometric groups, not automatically physical parts.',
                'Plane candidates are not automatically flange/slot reference surfaces.',
                'Mesh volume requires suitable topology; overlapping component volumes are not a union volume or a measured mass.'],
            'environment':versions()}
        if factor is None:report['limitations'].append('Units unconfirmed: no SI dimensions have been asserted.')
        if args.png:render_views([(str(i),m) for i,m in enumerate(parts)],args.png,
            title='Mesh components; numeric labels match JSON indices; no semantic classification',units=args.units,labels=True)
        save_json(args.json,report)
        print(json.dumps({'status':report['status'],'triangles':len(mesh.faces),'components':len(parts),
                          'declared_units':args.units,'raw_extents':mesh.extents.tolist(),'report':str(args.json)},ensure_ascii=True))
        return 0
    except Exception as exc:
        error={'status':'error','error':str(exc)}
        args.json.parent.mkdir(parents=True,exist_ok=True)
        args.json.write_text(json.dumps(error,ensure_ascii=True,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(error,ensure_ascii=True),file=sys.stderr)
        return 2


if __name__=='__main__':sys.exit(main())
