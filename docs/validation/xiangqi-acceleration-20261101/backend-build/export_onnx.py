"""Export the unchanged best.pt to dynamic FP32 ONNX in the experiment folder."""
from __future__ import annotations

import hashlib
import json
import resource
import time
from pathlib import Path

import onnx
import torch
import ultralytics
from ultralytics import YOLO


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def main():
    import argparse

    p = argparse.ArgumentParser()
    p.add_argument("weights", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    torch.set_float32_matmul_precision("highest")
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.set_num_threads(2)
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is unavailable in the selected runtime")
    load_start = time.monotonic()
    model = YOLO(str(a.weights), task="segment")
    load_ms = (time.monotonic() - load_start) * 1000
    if model.task != "segment":
        raise RuntimeError(f"expected segment task, got {model.task}")
    torch.cuda.synchronize()
    export_start = time.monotonic()
    output = model.export(format="onnx", imgsz=640, device=0, half=False, int8=False,
                          batch=1, dynamic=True, simplify=False, opset=18, nms=False,
                          verbose=False)
    torch.cuda.synchronize()
    export_ms = (time.monotonic() - export_start) * 1000
    path = Path(output)
    graph = onnx.load(str(path))
    onnx.checker.check_model(graph)
    dims = []
    for tensor in graph.graph.input:
        shape = []
        for d in tensor.type.tensor_type.shape.dim:
            shape.append(d.dim_param if d.dim_param else d.dim_value)
        dims.append({"name": tensor.name, "shape": shape,
                     "elem_type": tensor.type.tensor_type.elem_type})
    metadata = {item.key: item.value for item in graph.metadata_props}
    report = {
        "status": "exported",
        "source_weights": str(a.weights),
        "source_weights_sha256": sha256(a.weights),
        "onnx_path": str(path),
        "onnx_sha256": sha256(path),
        "onnx_size_bytes": path.stat().st_size,
        "onnx_opset_imports": [{"domain": x.domain, "version": x.version} for x in graph.opset_import],
        "inputs": dims,
        "metadata_props": metadata,
        "torch": torch.__version__,
        "cuda": torch.version.cuda,
        "cudnn": torch.backends.cudnn.version(),
        "ultralytics": ultralytics.__version__,
        "export_parameters": {"imgsz": 640, "dynamic": True, "batch": 1, "half": False,
                              "int8": False, "simplify": False, "opset": 18, "nms": False,
                              "task": "segment"},
        "tf32": {"matmul": torch.backends.cuda.matmul.allow_tf32,
                 "cudnn": torch.backends.cudnn.allow_tf32},
        "model_load_ms": load_ms,
        "onnx_export_wall_ms": export_ms,
        "resource_maxrss_kib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        "torch_cuda_peak_allocated_bytes": int(torch.cuda.max_memory_allocated()),
        "torch_cuda_peak_reserved_bytes": int(torch.cuda.max_memory_reserved()),
    }
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(report, indent=2, allow_nan=False))
    print(json.dumps(report, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
