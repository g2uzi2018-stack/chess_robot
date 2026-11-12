"""Second, explicit legacy PyTorch ONNX route used only if Ultralytics export fails."""
from __future__ import annotations

import hashlib
import json
import time
from pathlib import Path

import onnx
import torch
from ultralytics import YOLO


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for b in iter(lambda: f.read(1024 * 1024), b""):
            h.update(b)
    return h.hexdigest()


def main():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("weights", type=Path)
    p.add_argument("onnx", type=Path)
    p.add_argument("report", type=Path)
    a = p.parse_args()
    torch.set_float32_matmul_precision("highest")
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.set_num_threads(2)
    model = YOLO(str(a.weights), task="segment").model.float().eval().cuda()
    model.model[-1].export = True
    image = torch.zeros((1, 3, 640, 640), dtype=torch.float32, device="cuda")
    dynamic_axes = {
        "images": {0: "batch", 2: "height", 3: "width"},
        "output0": {0: "batch", 2: "anchors"},
        "output1": {0: "batch", 2: "mask_height", 3: "mask_width"},
    }
    started = time.monotonic()
    torch.onnx.export(model, image, str(a.onnx), input_names=["images"],
                      output_names=["output0", "output1"], dynamic_axes=dynamic_axes,
                      opset_version=18, do_constant_folding=True, dynamo=False, verbose=False)
    torch.cuda.synchronize()
    graph = onnx.load(str(a.onnx))
    onnx.checker.check_model(graph)
    names = getattr(model, "names", None)
    metadata = {"task": "segment", "stride": 32, "batch": 1,
                "imgsz": [640, 640], "names": json.dumps(names or {}, separators=(",", ":")),
                "export_path": "torch.onnx.export(dynamo=False)", "precision": "FP32-no-TF32"}
    for key, value in metadata.items():
        item = graph.metadata_props.add()
        item.key, item.value = key, str(value)
    onnx.save(graph, str(a.onnx))
    report = {"status": "exported", "path": str(a.onnx), "sha256": sha256(a.onnx),
              "size_bytes": a.onnx.stat().st_size, "opset": 18, "dynamic_axes": dynamic_axes,
              "input_dtype": "float32", "batch": 1, "metadata": metadata,
              "elapsed_ms": (time.monotonic() - started) * 1000}
    a.report.parent.mkdir(parents=True, exist_ok=True)
    a.report.write_text(json.dumps(report, indent=2, allow_nan=False))
    print(json.dumps(report, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
