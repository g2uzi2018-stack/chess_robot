"""Build an exact-shape-profile TensorRT 10 FP32 engine from dynamic ONNX."""
from __future__ import annotations

import ast
import hashlib
import json
import os
import resource
import time
from pathlib import Path

import onnx
import torch
import tensorrt as trt


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("onnx", type=Path)
    parser.add_argument("engine", type=Path)
    parser.add_argument("report", type=Path)
    parser.add_argument("--workspace-gib", type=float, default=1.0)
    args = parser.parse_args()
    args.engine.parent.mkdir(parents=True, exist_ok=True)
    args.report.parent.mkdir(parents=True, exist_ok=True)

    model = onnx.load(str(args.onnx))
    metadata = {}
    for item in model.metadata_props:
        try:
            metadata[item.key] = json.loads(item.value)
        except (ValueError, TypeError):
            metadata[item.key] = item.value
    metadata.update({"task": "segment", "batch": 1, "imgsz": [640, 640], "precision": "FP32-no-TF32"})
    names = metadata.get("names")
    if isinstance(names, str):
        try:
            names = json.loads(names)
        except ValueError:
            try:
                names = ast.literal_eval(names)
            except (ValueError, SyntaxError):
                names = None
    if not isinstance(names, dict) or not names:
        raise RuntimeError(f"ONNX metadata does not contain class names: {metadata.keys()}")
    metadata["names"] = names
    metadata["stride"] = int(metadata.get("stride", 32))

    logger = trt.Logger(trt.Logger.INFO)
    builder = trt.Builder(logger)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    config = builder.create_builder_config()
    workspace_bytes = int(args.workspace_gib * (1 << 30))
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, workspace_bytes)
    config.clear_flag(trt.BuilderFlag.TF32)
    cleared = ["TF32"]
    for flag_name in ("FP16", "INT8", "BF16", "FP8"):
        flag = getattr(trt.BuilderFlag, flag_name, None)
        if flag is not None:
            config.clear_flag(flag)
            cleared.append(flag_name)

    onnx_parser = trt.OnnxParser(network, logger)
    if not onnx_parser.parse_from_file(str(args.onnx)):
        errors = [str(onnx_parser.get_error(i)) for i in range(onnx_parser.num_errors)]
        raise RuntimeError("ONNX parser rejected model: " + " | ".join(errors))
    if network.num_inputs != 1:
        raise RuntimeError(f"expected one input, got {network.num_inputs}")
    input_tensor = network.get_input(0)
    if input_tensor.name != "images" or input_tensor.dtype != trt.float32:
        raise RuntimeError(f"unexpected TRT input: name={input_tensor.name}, dtype={input_tensor.dtype}")

    # Ultralytics rect=True creates exactly these three BCHW tensors for the
    # saved full-frame and ROI cases. Width and batch remain fixed at 640 and 1.
    shape_min = (1, 3, 384, 640)
    shape_opt = (1, 3, 480, 640)
    shape_max = (1, 3, 640, 640)
    profile = builder.create_optimization_profile()
    # TensorRT 10.3's Python binding returns None on success and raises
    # ValueError on invalid dimensions; it is not a boolean status API.
    profile.set_shape(input_tensor.name, shape_min, shape_opt, shape_max)
    if config.add_optimization_profile(profile) < 0:
        raise RuntimeError("TensorRT could not attach the input optimization profile")
    if hasattr(config, "builder_optimization_level"):
        config.builder_optimization_level = 3

    torch.cuda.synchronize()
    rss_before = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    started = time.monotonic()
    serialized = builder.build_serialized_network(network, config)
    torch.cuda.synchronize()
    build_ms = (time.monotonic() - started) * 1000
    if serialized is None:
        raise RuntimeError("TensorRT engine build returned None")

    # Ultralytics TensorRTBackend reads a 4-byte metadata length and JSON before
    # the serialized engine. This preserves task, class names, stride and imgsz.
    metadata_bytes = json.dumps(metadata, separators=(",", ":"), allow_nan=False).encode("utf-8")
    temporary = args.engine.with_suffix(args.engine.suffix + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(len(metadata_bytes).to_bytes(4, "little", signed=True))
        stream.write(metadata_bytes)
        stream.write(serialized)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, args.engine)

    profile_report = None
    inspector_report = None
    inspector_error = None
    with trt.Runtime(logger) as runtime:
        engine = runtime.deserialize_cuda_engine(serialized)
        if engine is None:
            raise RuntimeError("built engine could not be deserialized")
        profile_report = [list(tuple(s)) for s in engine.get_tensor_profile_shape(input_tensor.name, 0)]
        try:
            inspector = engine.create_engine_inspector()
            context = engine.create_execution_context()
            if hasattr(inspector, "execution_context"):
                inspector.execution_context = context
            elif hasattr(inspector, "set_execution_context"):
                inspector.set_execution_context(context)
            inspector_report = inspector.get_engine_information(trt.LayerInformationFormat.JSON)
            (args.engine.parent / "engine_layers.json").write_text(inspector_report)
        except Exception as exc:
            inspector_error = f"{type(exc).__name__}: {exc}"

    report = {
        "status": "built",
        "tensorrt_version": trt.__version__,
        "onnx_path": str(args.onnx),
        "onnx_sha256": sha256(args.onnx),
        "engine_path": str(args.engine),
        "engine_sha256": sha256(args.engine),
        "engine_size_bytes": args.engine.stat().st_size,
        "workspace_bytes": workspace_bytes,
        "batch": 1,
        "input_name": input_tensor.name,
        "input_dtype": str(input_tensor.dtype),
        "input_shapes_min_opt_max": [list(shape_min), list(shape_opt), list(shape_max)],
        "loaded_engine_profile_min_opt_max": profile_report,
        "precision_flags_cleared": cleared,
        "precision_flags_enabled": {
            flag_name: bool(config.get_flag(getattr(trt.BuilderFlag, flag_name)))
            for flag_name in ("TF32", "FP16", "INT8", "BF16", "FP8")
            if hasattr(trt.BuilderFlag, flag_name)
        },
        "build_ms_monotonic": build_ms,
        "workspace_gib": args.workspace_gib,
        "resource_maxrss_kib_before": rss_before,
        "resource_maxrss_kib_after": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        "torch_cuda_peak_allocated_bytes": int(torch.cuda.max_memory_allocated()),
        "torch_cuda_peak_reserved_bytes": int(torch.cuda.max_memory_reserved()),
        "platform_fast_fp16": bool(builder.platform_has_fast_fp16),
        "platform_fast_int8": bool(builder.platform_has_fast_int8),
        "layer_inspector_written": inspector_report is not None,
        "layer_inspector_error": inspector_error,
        "metadata": metadata,
    }
    args.report.write_text(json.dumps(report, indent=2, allow_nan=False))
    print(json.dumps(report, allow_nan=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
