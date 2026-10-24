#!/usr/bin/env python3
"""Export pure URDF and relative STL paths for external viewers (stdlib only)."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import zipfile

ROOT = Path(__file__).resolve().parents[2]
MODELS = {
    "nova5_with_ag95.urdf": ROOT / "src/chess_robot_description/urdf/nova5_with_ag95.urdf",
    "ag95.urdf": ROOT / "src/dh_ag95_description/urdf/ag95.urdf",
}


def export(output):
    output = output.resolve()
    # Reject locations inside source packages; exporting never edits model sources.
    if output == ROOT or output == ROOT / "src" or (ROOT / "src") in output.parents:
        raise ValueError("Choose an output directory outside src/, e.g. runs/browser_model")
    output.mkdir(parents=True, exist_ok=True)
    # Retire only the known managed, byte-identical obsolete composite. It is no
    # longer referenced; importing it as a loose STL could reintroduce duplicates.
    obsolete = output / "assets/chess_robot_description/meshes/tool_mount/new_jiazhua1.STL"
    original = ROOT / "src/chess_robot_description/meshes/tool_mount/new_jiazhua1.STL"
    if obsolete.is_file() and not obsolete.is_symlink() and obsolete.resolve().is_relative_to(output):
        if hashlib.sha256(obsolete.read_bytes()).digest() == hashlib.sha256(original.read_bytes()).digest():
            obsolete.unlink()
    # Remove only the known entrypoints from older exports of our retired viewer.
    for name in ("index.html", "viewer.bundle.js", "LICENSES.txt"):
        retired = output / name
        if retired.is_file():
            retired.unlink()
    exported = set()
    for filename, source in MODELS.items():
        doc = ET.parse(source)
        for mesh in doc.findall(".//mesh"):
            uri = mesh.attrib["filename"]
            if not uri.startswith("package://"):
                raise ValueError(f"Unexpected mesh URI: {uri}")
            package, relative = uri[len("package://"):].split("/", 1)
            asset = (ROOT / "src" / package / relative).resolve()
            if not asset.is_relative_to(ROOT / "src") or not asset.is_file():
                raise ValueError(f"Missing or invalid mesh: {uri}")
            destination = Path("assets") / package / relative
            target = output / destination
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(asset, target)
            exported.add(target)
            mesh.set("filename", destination.as_posix())
        ET.indent(doc, space="  ")
        target = output / filename
        doc.write(target, encoding="utf-8", xml_declaration=True)
        exported.add(target)

    for package in ("chess_robot_description", "dh_ag95_description"):
        for name in ("LICENSE", "SOURCES.json"):
            source = ROOT / "src" / package / name
            if source.exists():
                target = output / f"{package}_{name}"
                shutil.copyfile(source, target)
                exported.add(target)
    provenance = ROOT / "src/chess_robot_description/meshes/tool_mount/camera_mount_assembly_m.json"
    provenance_target = output / "camera_mount_extraction.json"
    shutil.copyfile(provenance, provenance_target)
    exported.add(provenance_target)
    guide = output / "README.txt"
    guide.write_text(
        "Nova5 + AG95 model assets (description only)\n\n"
        "Import this entire folder into your chosen URDF viewer/parser.\n"
        "Select nova5_with_ag95.urdf for the assembly, or ag95.urdf for the gripper.\n"
        "Use Joints to change left_outer_knuckle_joint (0..0.94 rad).\n"
        "   The other finger joints are coupled by mimic.\n"
        "Legacy finger padding boxes and the blue safety sheet have been removed.\n"
        "Original collision STL geometry remains active.\n"
        "Only rear camera/mount geometry is fixed; the AG95 fingers move once.\n\n"
        "This package contains model assets only; no bundled viewer or server.\n"
        "All meshes use relative paths; no ROS install or controller connection is needed.\n"
        "Mounting and grasp frame offsets are inherited, not calibrated measurements.\n"
        "0.94 rad is a geometry-only model limit, not a hardware command or calibration.\n"
        "No robot control code is included. Default model has no ground plane.\n",
        encoding="utf-8", newline="\n")
    exported.add(guide)
    checksums = {
        file.relative_to(output).as_posix(): hashlib.sha256(file.read_bytes()).hexdigest()
        for file in sorted(exported)
    }
    manifest = output / "SHA256SUMS.json"
    manifest.write_text(json.dumps(checksums, indent=2) + "\n", encoding="utf-8")
    exported.add(manifest)
    archive = output.with_name(output.name + ".zip")
    with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as bundle:
        for file in sorted(exported):
            bundle.write(file, Path(output.name) / file.relative_to(output))
    print(f"Exported {len(checksums)} files + manifest: {output}")
    print(f"ZIP: {archive}")
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "runs/browser_model")
    args = parser.parse_args()
    export(args.output)


if __name__ == "__main__":
    main()
