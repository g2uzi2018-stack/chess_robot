"""Add Ultralytics-compatible metadata to a raw TensorRT plan built by trtexec."""
import ast
import json
import os
import sys
from pathlib import Path

import onnx

onnx_path, raw_path, output_path = map(Path, sys.argv[1:4])
graph = onnx.load(str(onnx_path))
metadata = {}
for item in graph.metadata_props:
    try:
        metadata[item.key] = json.loads(item.value)
    except (ValueError, TypeError):
        metadata[item.key] = item.value
metadata.update({"task": "segment", "batch": 1, "imgsz": [640, 640], "precision": "FP32-no-TF32"})
if isinstance(metadata.get("names"), str):
    try:
        metadata["names"] = json.loads(metadata["names"])
    except ValueError:
        metadata["names"] = ast.literal_eval(metadata["names"])
if not isinstance(metadata.get("names"), dict):
    raise RuntimeError("ONNX metadata does not contain a class-name map")
metadata["stride"] = int(metadata.get("stride", 32))
payload = json.dumps(metadata, separators=(",", ":"), allow_nan=False).encode("utf-8")
temporary = output_path.with_suffix(output_path.suffix + ".tmp")
with temporary.open("wb") as dst, raw_path.open("rb") as src:
    dst.write(len(payload).to_bytes(4, "little", signed=True))
    dst.write(payload)
    for block in iter(lambda: src.read(1024 * 1024), b""):
        dst.write(block)
    dst.flush()
    os.fsync(dst.fileno())
os.replace(temporary, output_path)
print(json.dumps({"status": "annotated", "engine": str(output_path), "metadata": metadata}, allow_nan=False))
