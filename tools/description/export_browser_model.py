#!/usr/bin/env python3
"""Export pure URDF and relative STL paths for browser viewers (stdlib only)."""
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
    for name in ("index.html", "viewer.bundle.js", "LICENSES.txt"):
        target = output / name
        shutil.copyfile(ROOT / "tools/description/viewer" / name, target)
        exported.add(target)
    guide = output / "README.txt"
    guide.write_text(
        "Nova5 + AG95 browser preview (description only)\n\n"
        "Local viewer (no internet or ROS required):\n"
        "  cd into this extracted folder, then run:\n"
        "  python3 -m http.server 8765 --bind 127.0.0.1\n"
        "  Open http://127.0.0.1:8765 in a WebGL-capable browser on this machine.\n\n"
        "Alternatively open https://urdf.deyuf.org/, click Open Folder and choose\n"
        "this entire folder. Select nova5_with_ag95.urdf or ag95.urdf.\n"
        "Use Joints to change left_outer_knuckle_joint (0..1.10 rad).\n"
        "   The other finger joints are coupled by mimic.\n"
        "Enable collision display to inspect the inherited padding boxes.\n\n"
        "All meshes use relative paths; no ROS install or controller connection is needed.\n"
        "Mounting and grasp frame offsets are inherited, not calibrated measurements.\n"
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
