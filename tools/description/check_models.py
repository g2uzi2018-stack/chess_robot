#!/usr/bin/env python3
"""Offline integrity, URDF tree, mesh and mounting checks (standard library)."""
import hashlib
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]


def numbers(text):
    return tuple(float(value) for value in text.split())


def origin(joint):
    node = joint.find("origin")
    xyz = numbers(node.get("xyz", "0 0 0")) if node is not None else (0, 0, 0)
    rpy = numbers(node.get("rpy", "0 0 0")) if node is not None else (0, 0, 0)
    return xyz, rpy


def same_xml(left, right):
    # Ignore whitespace and attribute order, retain every value and child.
    def canonical(node):
        return (node.tag, sorted(node.attrib.items()), (node.text or "").strip(),
                [canonical(child) for child in node])
    return canonical(left) == canonical(right)


def check_tree(robot):
    links = [node.get("name") for node in robot.findall("link")]
    joints = robot.findall("joint")
    assert len(links) == len(set(links)), "Duplicate link names"
    names = [node.get("name") for node in joints]
    assert len(names) == len(set(names)), "Duplicate joint names"
    parent_of = {}
    for joint in joints:
        parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
        assert parent in links and child in links, f"Missing joint endpoint: {joint.get('name')}"
        assert child not in parent_of, f"Multiple parents: {child}"
        parent_of[child] = parent
        mimic = joint.find("mimic")
        if mimic is not None:
            assert mimic.get("joint") in names, "Missing mimic source"
            assert mimic.get("joint") != joint.get("name"), "Self mimic"
        if joint.get("type") == "revolute":
            limit = joint.find("limit")
            assert limit is not None and float(limit.get("lower")) <= float(limit.get("upper"))
    roots = set(links) - set(parent_of)
    assert len(roots) == 1, f"Expected one root, found {roots}"
    for link in links:
        seen = set()
        current = link
        while current in parent_of:
            assert current not in seen, f"Joint cycle at {current}"
            seen.add(current)
            current = parent_of[current]
        assert current in roots
    assert robot.find("ros2_control") is None, "Hardware config in preview"
    assert robot.find(".//plugin") is None, "Plugin in preview"
    return len(links), len(joints), next(iter(roots))


def resolve_mesh(uri):
    assert uri.startswith("package://"), f"Unexpected mesh URI: {uri}"
    package, relative = uri[len("package://"):].split("/", 1)
    path = (ROOT / "src" / package / relative).resolve()
    assert path.is_relative_to(ROOT / "src") and path.is_file(), f"Missing mesh: {uri}"
    return path


def check_stl(path):
    data = path.read_bytes()
    assert len(data) >= 84, f"Truncated STL: {path}"
    triangles = struct.unpack_from("<I", data, 80)[0]
    assert triangles > 0 and len(data) == 84 + 50 * triangles, f"Invalid binary STL: {path}"
    for offset in range(84, len(data), 50):
        assert all(math.isfinite(value) for value in struct.unpack_from("<12f", data, offset))
    return triangles


def main():
    meta = json.loads((ROOT / "reference/ag95/SOURCES.json").read_text(encoding="utf-8"))
    for entry in meta["files"]:
        target = ROOT / entry["target_path"]
        assert hashlib.sha256(target.read_bytes()).hexdigest() == entry["sha256"], str(target)
    assert (ROOT / "src/dh_ag95_description/SOURCES.json").read_bytes() == (
        ROOT / "reference/ag95/SOURCES.json").read_bytes()

    base = ET.parse(ROOT / "src/chess_robot_description/urdf/nova5.urdf").getroot()
    assembly = ET.parse(ROOT / "src/chess_robot_description/urdf/nova5_with_ag95.urdf").getroot()
    gripper = ET.parse(ROOT / "src/dh_ag95_description/urdf/ag95.urdf").getroot()
    legacy = ET.parse(ROOT / "reference/ag95/nova5_with_ag95.original.xacro").getroot()
    summary = {"source_files_unchanged": len(meta["files"]), "models": {}}
    assets = set()
    references = 0
    for name, robot in (("nova5", base), ("ag95", gripper), ("nova5_with_ag95", assembly)):
        count_links, count_joints, root = check_tree(robot)
        summary["models"][name] = {"links": count_links, "joints": count_joints, "root": root}
        for mesh in robot.findall(".//mesh"):
            assets.add(resolve_mesh(mesh.get("filename")))
            assert all(value > 0 for value in numbers(mesh.get("scale", "1 1 1")))
            references += 1
    triangle_count = sum(check_stl(asset) for asset in sorted(assets))

    # Check the whole original arm subtree is unchanged in the assembly.
    for node in list(base):
        if node.tag in ("link", "joint"):
            copy = assembly.find(f"{node.tag}[@name='{node.get('name')}']")
            assert copy is not None and same_xml(node, copy), f"Changed Nova5 node: {node.get('name')}"
    # Check standalone and assembled gripper share identical kinematics/geometries.
    for node in list(gripper):
        if node.tag in ("link", "joint") and node.get("name") not in ("world", "gripper_base_joint"):
            copy = assembly.find(f"{node.tag}[@name='{node.get('name')}']")
            assert copy is not None and same_xml(node, copy), f"Changed AG95 node: {node.get('name')}"
    for name in ("camera_bracket_joint", "safety_padding_joint"):
        assert same_xml(legacy.find(f"joint[@name='{name}']"), assembly.find(f"joint[@name='{name}']"))
    # Compare inherited bracket geometry after the intentional mesh-URI migration.
    old_bracket = legacy.find("link[@name='camera_bracket_link']")
    new_bracket = assembly.find("link[@name='camera_bracket_link']")
    for mesh in old_bracket.findall(".//mesh"):
        mesh.set("filename", "package://chess_robot_description/meshes/tool_mount/new_jiazhua1.STL")
    assert same_xml(old_bracket, new_bracket), "Bracket geometry/origin changed"
    assert same_xml(legacy.find("link[@name='safety_padding_link']"),
                    assembly.find("link[@name='safety_padding_link']")), "Padding changed"
    assert assembly.find("link[@name='env_ground']") is None, "Default includes scene ground"
    mount = assembly.find("joint[@name='gripper_base_joint']")
    assert mount.find("parent").get("link") == "camera_bracket_link"
    assert origin(mount) == ((0.0, 0.0, 0.0085), (0.0, 0.0, 0.0))
    grasp = assembly.find("joint[@name='grasp_joint']")
    xyz, rpy = origin(grasp)
    assert xyz == (0.0, 0.0, 0.20)
    assert max(abs(a-b) for a, b in zip(rpy, (math.pi/2, -math.pi/2, 0))) < 1e-12
    summary.update({"unique_meshes": len(assets), "mesh_references": references,
                    "triangles": triangle_count,
                    "Link6_to_grasp_link_xyz_m": [0.0, 0.0, 0.2085],
                    "legacy_mount_geometry_and_padding": "unchanged"})
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
