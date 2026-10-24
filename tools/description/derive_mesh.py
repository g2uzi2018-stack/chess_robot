#!/usr/bin/env python3
"""Create a new SI mesh from explicit component IDs/rigid transform and source SHA."""
import argparse,json,sys
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mesh',type=Path)
    parser.add_argument('--spec',type=Path,required=True,help='JSON: input_sha256, units, components (all/list), transform_m (4x4)')
    parser.add_argument('--output',type=Path,required=True,help='A new .stl path; existing files are never overwritten')
    args=parser.parse_args()
    try:
        import numpy as np,trimesh
        from geometry_report import load_mesh,components,sha256,versions,save_json
        spec=json.loads(args.spec.read_text(encoding='utf-8-sig'))
        if sha256(args.mesh)!=spec['input_sha256']:raise ValueError('Input SHA mismatch; re-inspect the correct source version')
        factor={'mm':.001,'m':1.}.get(spec.get('units'))
        if factor is None:raise ValueError('Explicit source units mm/m are required')
        t=np.asarray(spec['transform_m'],dtype=float)
        if t.shape!=(4,4) or not np.isfinite(t).all() or not np.allclose(t[3],[0,0,0,1],atol=1e-10):raise ValueError('Invalid homogeneous transform')
        if not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-8) or abs(np.linalg.det(t[:3,:3])-1)>1e-8:
            raise ValueError('Transform must be a proper rigid rotation; reflection or fitted scaling is not allowed')
        sidecar=args.output.with_suffix(args.output.suffix+'.json')
        if args.output.resolve()==args.mesh.resolve() or args.output.exists() or sidecar.exists():
            raise ValueError('Choose new output/sidecar paths; never overwrite source or existing products')
        if args.output.suffix.lower()!='.stl':raise ValueError('Derived output must be .stl')
        mesh=load_mesh(args.mesh); selected=spec.get('components','all')
        count=None
        if selected!='all':
            parts=components(mesh);count=len(parts)
            if not isinstance(selected,list) or not selected or any(type(i) is not int or not 0<=i<len(parts) for i in selected) or len(set(selected))!=len(selected):
                raise ValueError('Invalid/duplicate component indices')
            mesh=trimesh.util.concatenate([parts[i] for i in selected])
        mesh.apply_scale(factor);mesh.apply_transform(t)
        mesh=trimesh.Trimesh(vertices=mesh.vertices,faces=mesh.faces,process=False)
        content=mesh.export(file_type='stl')
        args.output.parent.mkdir(parents=True,exist_ok=True)
        with args.output.open('xb') as file:file.write(content)
        roundtrip=trimesh.load_mesh(args.output,process=False)
        quantization=float(np.max(np.abs(mesh.triangles-roundtrip.triangles)))
        save_json(sidecar,{'schema_version':1,'source':str(args.mesh.resolve()),'input_sha256':spec['input_sha256'],
            'output_sha256':sha256(args.output),'spec':spec,'source_component_count_if_selected':count,
            'output_units':'m','triangles':len(mesh.faces),'bounds_m':mesh.bounds.tolist(),
            'max_stl_coordinate_quantization_m':quantization,'environment':versions(),
            'scope':'Explicit CAD transform only. Does not identify correct parts, datums, units or physical calibration.'})
        print(json.dumps({'status':'derived','output':str(args.output),'provenance':str(sidecar),'triangles':len(mesh.faces)}))
        return 0
    except Exception as exc:
        print(json.dumps({'status':'error','error':str(exc)},ensure_ascii=True),file=sys.stderr)
        return 2


if __name__=='__main__':sys.exit(main())
