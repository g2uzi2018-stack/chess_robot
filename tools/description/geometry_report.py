"""Shared read-only mesh/URDF report helpers. No robot or browser interface."""
import hashlib
import importlib.metadata
import json
from pathlib import Path
from urllib.parse import unquote
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from rigid_geometry import fk, joint_values, origin


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def versions():
    found = {}
    for name in ('numpy', 'trimesh', 'scipy', 'matplotlib', 'python-fcl'):
        try:
            found[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            found[name] = None
    return found


def save_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=True, indent=2, allow_nan=False)+'\n', encoding='utf-8', newline='\n')


def load_mesh(path):
    raw = trimesh.load_mesh(Path(path), process=False)
    if not isinstance(raw, trimesh.Trimesh) or not len(raw.faces):
        raise ValueError('Expected one nonempty triangle mesh; export a mesh file first')
    if not np.isfinite(raw.vertices).all():
        raise ValueError('Mesh contains non-finite vertex coordinates')
    # Weld coincident vertices for topology analysis, but do not fill holes,
    # simplify, flip normals or modify the source file.
    return trimesh.Trimesh(vertices=raw.vertices, faces=raw.faces, process=True, validate=False)


def components(mesh):
    return sorted(mesh.split(only_watertight=False, repair=False), key=lambda m: len(m.faces), reverse=True)


def mesh_stats(mesh):
    valid = bool(mesh.is_volume)
    return {
        'triangles': len(mesh.faces), 'welded_vertices': len(mesh.vertices),
        'bounds': mesh.bounds.tolist(), 'extents': mesh.extents.tolist(),
        'surface_area': float(mesh.area), 'aabb_volume': float(np.prod(mesh.extents)),
        'watertight': bool(mesh.is_watertight), 'consistent_winding': bool(mesh.is_winding_consistent),
        'is_volume': valid, 'signed_mesh_volume': float(mesh.volume) if valid else None,
        'zero_area_faces': int(np.sum(mesh.area_faces == 0)),
    }


def plane_candidates(mesh):
    """Large axis-aligned plane groups; candidates, never semantic datums."""
    tolerance = max(float(mesh.extents.max()), 1e-12)*1e-6
    result = []
    for axis in range(3):
        idx = np.flatnonzero(np.abs(mesh.face_normals[:, axis]) > 0.99999)
        grouped = {}
        for i in idx:
            value = mesh.triangles_center[i, axis]
            key = (int(np.sign(mesh.face_normals[i, axis])), int(round(value/tolerance)))
            grouped.setdefault(key, []).append(i)
        for (sign, _), faces in grouped.items():
            area = float(mesh.area_faces[faces].sum())
            if area <= 0:
                continue
            coordinate = float(np.average(mesh.triangles_center[faces, axis], weights=mesh.area_faces[faces]))
            result.append({'axis': 'xyz'[axis], 'normal_sign': sign, 'coordinate': coordinate,
                           'area': area, 'triangle_count': len(faces)})
    return sorted(result, key=lambda row: row['area'], reverse=True)[:24]


def render_views(items, path, *, title, units, frames=None, labels=False):
    # Standard raster plots work without WebGL, a display server, or browser GPU.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    items = [(label, mesh) for label, mesh in items if len(mesh.faces)]
    if not items:
        raise ValueError('No geometry selected to render')
    all_bounds = np.array([m.bounds for _, m in items])
    lower, upper = all_bounds[:, 0].min(axis=0), all_bounds[:, 1].max(axis=0)
    colors = plt.get_cmap('tab20')
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.7), constrained_layout=True)
    for ax, projection, axis_names in zip(axes, ((0,1),(0,2),(1,2)), ('XY','XZ','YZ')):
        depth = ({0,1,2}-set(projection)).pop()
        triangles = np.vstack([m.triangles for _, m in items])
        face_colors = np.vstack([np.tile(colors(i % 20), (len(m.faces), 1)) for i,(_,m) in enumerate(items)])
        order = np.argsort(triangles[:, :, depth].mean(axis=1))
        ax.add_collection(PolyCollection(triangles[order][:,:,list(projection)],
                          facecolors=face_colors[order], edgecolor='none', rasterized=True))
        span = max(float((upper-lower)[list(projection)].max()), 1e-6)*1.12
        center = (lower+upper)/2
        for coordinate, setter in zip(projection, (ax.set_xlim, ax.set_ylim)):
            setter(center[coordinate]-span/2, center[coordinate]+span/2)
        if labels:
            for label, mesh in items[:20]:
                center_point = mesh.bounds.mean(axis=0)
                ax.text(center_point[projection[0]], center_point[projection[1]], label, fontsize=7)
        if frames:
            length = max(float((upper-lower).max())*.12, 1e-6)
            for name,t in frames.items():
                t = np.asarray(t)
                p = t[:3, 3]
                ax.scatter([p[projection[0]]], [p[projection[1]]], s=12, color='black')
                ax.text(p[projection[0]], p[projection[1]], name, fontsize=7)
                for axis, color in enumerate(('red','green','blue')):
                    delta=t[:3,axis]*length
                    ax.plot([p[projection[0]],p[projection[0]]+delta[projection[0]]],
                            [p[projection[1]],p[projection[1]]+delta[projection[1]]], color=color, lw=1)
        ax.set_aspect('equal'); ax.grid(alpha=.2); ax.set_axisbelow(True)
        ax.set(xlabel=f'{axis_names[0]} ({units})', ylabel=f'{axis_names[1]} ({units})', title=axis_names)
    fig.suptitle(title, fontsize=11)
    path=Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160); plt.close(fig)


class RobotGeometry:
    def __init__(self, path, package_roots):
        self.path = Path(path).resolve()
        self.root = ET.parse(self.path).getroot()
        if self.path.suffix == '.xacro' or any('xacro' in n.tag for n in self.root.iter()):
            raise ValueError('Expand Xacro to pure URDF first')
        if self.root.tag != 'robot':
            raise ValueError('Expected a URDF robot root')
        names = [n.get('name') for n in self.root.findall('link')]
        jnames = [n.get('name') for n in self.root.findall('joint')]
        if None in names or None in jnames or len(names)!=len(set(names)) or len(jnames)!=len(set(jnames)):
            raise ValueError('Missing or duplicate link/joint names')
        self.links={n.get('name'):n for n in self.root.findall('link')}
        self.joints={n.get('name'):n for n in self.root.findall('joint')}
        children=set()
        for joint in self.joints.values():
            a,b=joint.find('parent').get('link'),joint.find('child').get('link')
            if a not in self.links or b not in self.links or b in children:
                raise ValueError('Missing endpoint or multiple parents')
            children.add(b)
            if joint.get('type') not in ('fixed','revolute','continuous','prismatic'):
                raise ValueError('Unsupported planar/floating/unknown joint type')
            for origin_node in joint.findall('origin'):
                values=[float(x) for k in ('xyz','rpy') for x in origin_node.get(k,'0 0 0').split()]
                if len(values)!=6 or not np.isfinite(values).all():
                    raise ValueError('Invalid joint origin')
            if joint.get('type')!='fixed':
                axis=joint.find('axis')
                vector=np.fromstring(axis.get('xyz','1 0 0') if axis is not None else '1 0 0',sep=' ')
                if len(vector)!=3 or not np.isfinite(vector).all() or np.linalg.norm(vector)==0:
                    raise ValueError('Invalid joint axis')
            if joint.get('type') in ('revolute','prismatic'):
                limit=joint.find('limit')
                if limit is None:
                    raise ValueError('Missing joint limits')
                bounds=[float(limit.get(k)) for k in ('lower','upper')]
                if not np.isfinite(bounds).all() or bounds[0]>bounds[1]:
                    raise ValueError('Missing or reversed joint limits')
        self.root_link=next(iter(set(names)-children)) if len(set(names)-children)==1 else None
        if self.root_link is None:
            raise ValueError('Expected exactly one root link')
        self.package_roots=[Path(p).resolve() for p in package_roots]
        self.assets={}; self.geometry={}
        for name,link in self.links.items():
            self.geometry[name]={kind:[self.read_geometry(n) for n in link.findall(kind)] for kind in ('visual','collision')}
        self.transforms({})

    def resolve(self, uri):
        if uri.startswith('package://'):
            package,relative=uri[len('package://'):].split('/',1)
            candidates=[root/package/unquote(relative) for root in self.package_roots]
            found={p.resolve() for p in candidates if p.is_file()}
            if len(found)!=1:
                raise ValueError(f'Missing/ambiguous package asset {uri}; supply --package-root paths explicitly')
            return next(iter(found))
        if '://' in uri:
            raise ValueError('Only package:// and URDF-relative/local mesh paths are supported')
        p=(self.path.parent/unquote(uri)).resolve()
        if not p.is_file():
            raise FileNotFoundError(f'Missing relative mesh: {p}')
        return p

    def read_geometry(self, node):
        geo=node.find('geometry')
        if geo is None or len(geo)!=1:
            raise ValueError('Expected exactly one geometry shape')
        shape=geo[0]
        if shape.tag=='mesh':
            path=self.resolve(shape.get('filename'))
            if path not in self.assets:
                self.assets[path]={'sha256':sha256(path),'mesh':load_mesh(path)}
            result=self.assets[path]['mesh'].copy()
            scale=np.fromstring(shape.get('scale','1 1 1'),sep=' ')
            if len(scale)!=3 or not np.isfinite(scale).all() or np.min(scale)<=0:
                raise ValueError('Mesh scale must have three positive finite values')
            result.apply_scale(scale)
        elif shape.tag=='box':
            size=np.fromstring(shape.get('size'),sep=' ')
            if len(size)!=3 or not np.isfinite(size).all() or min(size)<=0:
                raise ValueError('Invalid box size')
            result=trimesh.creation.box(extents=size)
        elif shape.tag=='cylinder':
            radius,height=float(shape.get('radius')),float(shape.get('length'))
            if not np.isfinite([radius,height]).all() or min(radius,height)<=0:raise ValueError('Invalid cylinder')
            result=trimesh.creation.cylinder(radius=radius,height=height,sections=64)
        elif shape.tag=='sphere':
            radius=float(shape.get('radius'))
            if not np.isfinite(radius) or radius<=0:raise ValueError('Invalid sphere')
            result=trimesh.creation.icosphere(radius=radius,subdivisions=2)
        else:
            raise ValueError(f'Unsupported geometry {shape.tag}')
        element=node.find('origin')
        if element is not None:
            for key in ('xyz','rpy'):
                v=np.fromstring(element.get(key,'0 0 0'),sep=' ')
                if len(v)!=3 or not np.isfinite(v).all():raise ValueError('Invalid geometry origin')
        matrix=np.asarray(origin(element))
        if not np.isfinite(matrix).all():raise ValueError('Non-finite geometry origin')
        result.apply_transform(matrix)
        return result

    def transforms(self, supplied):
        unknown=set(supplied)-set(self.joints)
        if unknown:raise ValueError(f'Unknown joints: {sorted(unknown)}')
        if not all(np.isfinite(float(x)) for x in supplied.values()):raise ValueError('Non-finite joint position')
        if any(self.joints[n].find('mimic') is not None for n in supplied):
            raise ValueError('Set the mimic master joint, not its dependent joint')
        if any(self.joints[n].get('type')=='fixed' for n in supplied):raise ValueError('Cannot set a fixed joint')
        return {n:np.asarray(t) for n,t in fk(self.root,supplied).items()}

    def selected_links(self, subtree=None):
        if subtree is None:return set(self.links)
        if subtree not in self.links:raise ValueError('Unknown subtree link')
        selected={subtree}
        while True:
            more={j.find('child').get('link') for j in self.joints.values() if j.find('parent').get('link') in selected}
            if more<=selected:return selected
            selected|=more

    def posed(self, supplied, kind, frame=None, subtree=None):
        transforms=self.transforms(supplied)
        frame=frame or self.root_link
        if frame not in transforms:raise ValueError('Unknown reference frame')
        inv=np.linalg.inv(transforms[frame]); selected=self.selected_links(subtree)
        meshes=[]
        for name in self.links:
            if name not in selected:continue
            for i,mesh in enumerate(self.geometry[name][kind]):
                m=mesh.copy(); m.apply_transform(inv@transforms[name]);meshes.append((f'{name}/{i}',m))
        return meshes,{n:inv@t for n,t in transforms.items() if n in selected}
