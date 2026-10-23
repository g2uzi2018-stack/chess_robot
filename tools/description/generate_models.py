#!/usr/bin/env python3
"""Expand description-only Xacro entries; never loads robot hardware."""
import argparse
import difflib
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import xacro

ROOT = Path(__file__).resolve().parents[2]
ENTRIES = (
    ROOT / "src/dh_ag95_description/urdf/ag95.urdf.xacro",
    ROOT / "src/chess_robot_description/urdf/nova5_with_ag95.urdf.xacro",
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Fail if generated URDF is stale")
    args = parser.parse_args()
    stale = False
    for source in ENTRIES:
        target = source.with_suffix("")
        # Use the Python API so output has no machine-specific command/path banner.
        mappings = {"ag95_macro_file": str(
            ROOT / "src/dh_ag95_description/urdf/dh_ag95_macro.xacro")}
        doc = xacro.process_file(str(source), mappings=mappings)
        # Normalize formatting and discard auto-generated absolute-path comments.
        robot = ET.fromstring(doc.toxml())
        ET.indent(robot, space="  ")
        content = '<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(
            robot, encoding="unicode") + "\n"
        current = target.read_text(encoding="utf-8") if target.exists() else ""
        if args.check:
            if content != current:
                stale = True
                sys.stderr.writelines(difflib.unified_diff(
                    current.splitlines(True), content.splitlines(True),
                    fromfile=str(target.relative_to(ROOT)), tofile="regenerated"))
        else:
            target.write_text(content, encoding="utf-8", newline="\n")
        print(f"{'check' if args.check else 'write'}: {target.relative_to(ROOT)}")
    return int(stale)


if __name__ == "__main__":
    raise SystemExit(main())
