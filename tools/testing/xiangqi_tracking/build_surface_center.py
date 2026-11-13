"""Compile the native inverse-depth center fitter on its deployment machine."""
import argparse
import hashlib
import json
import platform
import subprocess
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    source = Path(__file__).with_name('surface_center.cpp')
    legacy = Path(__file__).with_name('surface_fit.cpp')
    artifact = args.output/'surface_center.so'
    command = ['g++', '-O3', '-std=c++17', '-march=native', '-fPIC', '-shared',
               '-I/usr/include/eigen3', str(source), str(legacy), '-o', str(artifact)]
    subprocess.run(command, check=True)
    metadata = dict(source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        legacy_source_sha256=hashlib.sha256(legacy.read_bytes()).hexdigest(),
        artifact_sha256=hashlib.sha256(artifact.read_bytes()).hexdigest(),
        architecture=platform.machine(), command=command,
        compiler=subprocess.check_output(['g++', '--version'], text=True).splitlines()[0])
    (args.output/'build.json').write_text(json.dumps(metadata, indent=2))
    print(json.dumps(metadata))


if __name__ == '__main__':
    main()
