"""Small standard-library FK/STL helpers for description checks (not control)."""
import math
import struct


def eye():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def rotation(axis, angle):
    length = math.sqrt(sum(x*x for x in axis))
    assert length > 0, 'Zero joint axis'
    x, y, z = (v/length for v in axis)
    c, s, v = math.cos(angle), math.sin(angle), 1-math.cos(angle)
    result = eye()
    result[:3] = [[x*x*v+c, x*y*v-z*s, x*z*v+y*s, 0],
                  [y*x*v+z*s, y*y*v+c, y*z*v-x*s, 0],
                  [z*x*v-y*s, z*y*v+x*s, z*z*v+c, 0]]
    return result


def numbers(text):
    return tuple(float(x) for x in text.split())


def origin(node):
    if node is None:
        return eye()
    roll, pitch, yaw = numbers(node.get('rpy', '0 0 0'))
    result = multiply(multiply(rotation((0, 0, 1), yaw), rotation((0, 1, 0), pitch)), rotation((1, 0, 0), roll))
    for i, value in enumerate(numbers(node.get('xyz', '0 0 0'))):
        result[i][3] = value
    return result


def joint_values(joints, supplied):
    values = {}
    def get(name, seen=frozenset()):
        assert name not in seen, 'Mimic cycle'
        if name in values:
            return values[name]
        mimic = joints[name].find('mimic')
        values[name] = (supplied.get(name, 0.) if mimic is None else
            float(mimic.get('multiplier', '1'))*get(mimic.get('joint'), seen | {name}) + float(mimic.get('offset', '0')))
        return values[name]
    for name in joints:
        get(name)
    return values


def fk(robot, supplied):
    joints = {j.get('name'): j for j in robot.findall('joint')}
    roots = {l.get('name') for l in robot.findall('link')} - {j.find('child').get('link') for j in joints.values()}
    assert len(roots) == 1
    poses = {next(iter(roots)): eye()}
    values = joint_values(joints, supplied)
    pending = list(joints.values())
    while pending:
        ready = [j for j in pending if j.find('parent').get('link') in poses]
        assert ready, 'Disconnected or cyclic joint graph'
        for joint in ready:
            t = origin(joint.find('origin'))
            kind = joint.get('type')
            if kind in ('revolute', 'continuous', 'prismatic'):
                axis = joint.find('axis')
                axis = numbers(axis.get('xyz', '1 0 0')) if axis is not None else (1, 0, 0)
                if kind == 'prismatic':
                    length = math.sqrt(sum(x*x for x in axis))
                    assert length > 0, 'Zero joint axis'
                    motion = eye()
                    for i in range(3):
                        motion[i][3] = axis[i]/length*values[joint.get('name')]
                else:
                    motion = rotation(axis, values[joint.get('name')])
                t = multiply(t, motion)
            elif kind != 'fixed':
                raise ValueError(f'Unsupported joint type: {kind}; use a suitable kinematics backend')
            poses[joint.find('child').get('link')] = multiply(poses[joint.find('parent').get('link')], t)
            pending.remove(joint)
    return poses


def transform_point(t, p):
    return tuple(sum(t[i][j]*p[j] for j in range(3)) + t[i][3] for i in range(3))


def stl_vertices(path):
    data = path.read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    assert len(data) == 84+50*count
    for offset in range(84, len(data), 50):
        xyz = struct.unpack_from('<9f', data, offset+12)
        yield from (xyz[:3], xyz[3:6], xyz[6:])


def visual_vertices(link, poses, resolve):
    for visual in link.findall('visual'):
        mesh = visual.find('geometry/mesh')
        assert mesh is not None
        scale = numbers(mesh.get('scale', '1 1 1'))
        t = multiply(poses[link.get('name')], origin(visual.find('origin')))
        for point in stl_vertices(resolve(mesh.get('filename'))):
            yield transform_point(t, tuple(a*b for a, b in zip(point, scale)))


def pad_gap(robot, q, resolve):
    poses = fk(robot, {'left_outer_knuckle_joint': q})
    # Express the two meshes along the ag95_base_link local X axis, including
    # a possible rotation of the complete arm or standalone mounting frame.
    base = poses['ag95_base_link']
    axis = tuple(base[i][0] for i in range(3))
    def project(name):
        link = robot.find(f"link[@name='{name}']")
        return [sum(axis[i]*(p[i]-base[i][3]) for i in range(3)) for p in visual_vertices(link, poses, resolve)]
    return min(project('left_finger_pad')) - max(project('right_finger_pad'))
