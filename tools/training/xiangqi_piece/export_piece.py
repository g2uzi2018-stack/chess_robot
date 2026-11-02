#!/usr/bin/env python3
"""Export the new piece detector to dynamic ONNX and exact-profile FP32 TRT.

CUDA-capable imports occur only after acquiring the shared production GPU lock.
No service, camera, robot, clock, old model or SDK is changed by this program.
"""
from __future__ import annotations

import argparse
import ast
import contextlib
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import time
import traceback


ROOT = Path('/home/kuang/workspace/xiangqi-yolo-training/20261102-yolo26n-piece-v1')
GPU_LOCK = Path('/home/kuang/workspace/chess_robot/runs/perf_ceiling_20261031/benchmark.lock')
SHAPES = {640: [1, 3, 384, 640], 512: [1, 3, 288, 512], 416: [1, 3, 256, 416]}


def sha256(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + f'.tmp-{os.getpid()}')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def require(ok, message):
    if not ok:
        raise RuntimeError(message)


@contextlib.contextmanager
def gpu_lease(path, timeout):
    def timed_out(_signum, _frame):
        raise TimeoutError(f'Timed out after {timeout}s acquiring shared GPU lock: {path}')

    with path.open('a') as stream:
        previous = signal.signal(signal.SIGALRM, timed_out)
        signal.setitimer(signal.ITIMER_REAL, timeout)
        try:
            # Blocking flock cooperates with the production service's short yield
            # window. Never substitute a nonblocking polling acquisition here.
            fcntl.flock(stream, fcntl.LOCK_EX)
        finally:
            signal.setitimer(signal.ITIMER_REAL, 0)
            signal.signal(signal.SIGALRM, previous)
        try:
            yield stream
        finally:
            fcntl.flock(stream, fcntl.LOCK_UN)


def metadata_value(value):
    try:
        return json.loads(value)
    except (TypeError, ValueError):
        try:
            return ast.literal_eval(value)
        except (ValueError, SyntaxError, TypeError):
            return value


def checked_reuse(report_path, artifact_path, source_hash, script_hash, shape=None):
    require(report_path.is_file(), f'Artifact exists without completed report; refusing overwrite: {artifact_path}')
    report = json.loads(report_path.read_text())
    require(report.get('status') in ['exported', 'built'], f'Incomplete artifact report: {report_path}')
    require(report.get('source_weights_sha256') == source_hash, f'Source weights changed: {report_path}')
    require(report.get('export_script_sha256') == script_hash, f'Exporter script changed: {report_path}')
    key = 'engine_sha256' if shape else 'onnx_sha256'
    require(report.get(key) == sha256(artifact_path), f'Existing artifact SHA mismatch: {artifact_path}')
    if shape:
        require(report.get('loaded_engine_profile_min_opt_max') == [shape] * 3,
                f'Existing engine is not the expected exact profile: {artifact_path}')
        require(report.get('precision') == 'FP32-no-TF32', 'Existing engine precision differs')
    return report


def execute(args, state):
    script_hash = sha256(Path(__file__))
    source_hash = sha256(args.weights)
    args.artifacts.mkdir(parents=True, exist_ok=True)
    copied_weights = args.artifacts / 'yolo26n-piece.pt'
    if copied_weights.exists():
        require(sha256(copied_weights) == source_hash, 'Existing copied weights differ; refusing overwrite')
    else:
        shutil.copyfile(args.weights, copied_weights)
    require(sha256(copied_weights) == source_hash, 'Copied weights SHA mismatch')
    state.update(state='waiting_gpu_lock', phase='waiting_gpu_lock', source_weights_sha256=source_hash,
                 export_script_sha256=script_hash, gpu_lock=str(args.lock), lock_timeout_seconds=args.lock_timeout)
    atomic_json(args.status, state)
    wait_started = time.monotonic()
    with gpu_lease(args.lock, args.lock_timeout):
        state.update(state='running', phase='gpu_lock_acquired', lock_acquired_at_utc=now(),
                     lock_wait_seconds=time.monotonic() - wait_started, gpu_lock_held=True)
        atomic_json(args.status, state)
        # These imports and every CUDA operation remain inside the GPU lease.
        import onnx
        import torch
        import tensorrt as trt
        import ultralytics
        from ultralytics import YOLO

        require(ultralytics.__version__ == '8.4.32', f'Unexpected Ultralytics: {ultralytics.__version__}')
        torch.set_num_threads(4)
        torch.set_num_interop_threads(1)
        torch.set_float32_matmul_precision('highest')
        torch.backends.cuda.matmul.allow_tf32 = False
        torch.backends.cudnn.allow_tf32 = False
        require(torch.cuda.is_available(), 'CUDA unavailable in fixed runtime')
        state['versions'] = {'ultralytics': ultralytics.__version__, 'torch': torch.__version__,
                             'cuda': torch.version.cuda, 'tensorrt': trt.__version__, 'onnx': onnx.__version__}
        source_versions = {}
        package_root = Path(ultralytics.__file__).parent
        for rel in ['nn/modules/head.py', 'engine/exporter.py', 'engine/predictor.py',
                    'models/yolo/detect/predict.py', 'nn/backends/tensorrt.py',
                    'nn/autobackend.py', 'utils/nms.py']:
            path = package_root / rel
            if path.is_file():
                source_versions[rel] = {'path': str(path), 'sha256': sha256(path)}
        state['source_versions'] = source_versions
        onnx_path = copied_weights.with_suffix('.onnx')
        onnx_report_path = args.artifacts / 'onnx-export.json'
        if onnx_path.exists():
            onnx_report = checked_reuse(onnx_report_path, onnx_path, source_hash, script_hash)
        else:
            state.update(phase='exporting_dynamic_onnx', updated_at_utc=now())
            atomic_json(args.status, state)
            model = YOLO(str(copied_weights), task='detect')
            require(model.task == 'detect', f'Unexpected task: {model.task}')
            names = {int(k): v for k, v in model.names.items()}
            head = model.model.model[-1]
            require(names == {0: 'piece'} and head.nc == 1, f'Expected one-class piece model: {names}')
            require(bool(head.end2end), 'Expected YOLO26 end2end head')
            model.model.float()
            started = time.monotonic()
            exported = Path(model.export(format='onnx', imgsz=640, device=0, half=False,
                                          int8=False, batch=1, dynamic=True, simplify=False,
                                          opset=18, nms=False, agnostic_nms=False, verbose=False))
            torch.cuda.synchronize()
            require(exported.resolve() == onnx_path.resolve(), f'Unexpected export location: {exported}')
            graph = onnx.load(str(onnx_path))
            onnx.checker.check_model(graph)
            require(len(graph.graph.input) == 1, 'Expected one ONNX input')
            inp = graph.graph.input[0]
            shape = [d.dim_param if d.dim_param else d.dim_value for d in inp.type.tensor_type.shape.dim]
            require(inp.name == 'images' and inp.type.tensor_type.elem_type == onnx.TensorProto.FLOAT,
                    'Expected FP32 images ONNX input')
            require(len(shape) == 4 and shape[1] == 3 and all(isinstance(shape[i], str) for i in [0, 2, 3]),
                    f'Expected dynamic batch/height/width: {shape}')
            metadata = {i.key: metadata_value(i.value) for i in graph.metadata_props}
            require(str(metadata.get('end2end')).lower() == 'true', 'ONNX end2end metadata missing')
            require(metadata.get('task') == 'detect', 'ONNX task is not detect')
            onnx_report = {
                'status': 'exported', 'created_at_utc': now(), 'source_weights': str(args.weights),
                'copied_weights': str(copied_weights), 'source_weights_sha256': source_hash,
                'export_script_sha256': script_hash, 'onnx_path': str(onnx_path),
                'onnx_sha256': sha256(onnx_path), 'bytes': onnx_path.stat().st_size,
                'input_name': inp.name, 'input_shape': shape, 'input_dtype': 'float32',
                'metadata': metadata, 'versions': state['versions'], 'source_versions': source_versions,
                'parameters': {'imgsz': 640, 'batch': 1, 'dynamic': True, 'opset': 18,
                               'simplify': False, 'nms': False, 'agnostic_nms': False,
                               'half': False, 'int8': False, 'task': 'detect'},
                'precision': 'FP32-no-TF32', 'tf32': {'matmul': False, 'cudnn': False},
                'export_seconds': time.monotonic() - started,
            }
            atomic_json(onnx_report_path, onnx_report)
            del model, graph
            torch.cuda.empty_cache()
        engines = []
        for size in args.sizes:
            shape = SHAPES[size]
            directory = args.artifacts / f'imgsz{size}-fp32'
            directory.mkdir(exist_ok=True)
            engine_path = directory / f'yolo26n-piece-imgsz{size}-fp32.engine'
            report_path = directory / 'engine-build.json'
            state.update(phase='building_engine', current_size=size, actual_input_shape=shape,
                         completed_sizes=[e['imgsz'] for e in engines], updated_at_utc=now())
            atomic_json(args.status, state)
            if engine_path.exists():
                engines.append(checked_reuse(report_path, engine_path, source_hash, script_hash, shape))
                continue
            logger = trt.Logger(trt.Logger.INFO)
            trt.init_libnvinfer_plugins(logger, '')
            builder = trt.Builder(logger)
            network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
            config = builder.create_builder_config()
            config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 30)
            cleared = []
            for name in ['TF32', 'FP16', 'INT8', 'BF16', 'FP8']:
                flag = getattr(trt.BuilderFlag, name, None)
                if flag is not None:
                    config.clear_flag(flag)
                    cleared.append(name)
            enabled = {name: bool(config.get_flag(getattr(trt.BuilderFlag, name))) for name in cleared}
            require(not any(enabled.values()), f'Precision flag remained enabled: {enabled}')
            parser = trt.OnnxParser(network, logger)
            if not parser.parse_from_file(str(onnx_path)):
                raise RuntimeError('ONNX parser rejected graph: ' + ' | '.join(
                    str(parser.get_error(i)) for i in range(parser.num_errors)))
            require(network.num_inputs == 1, f'Unexpected input count: {network.num_inputs}')
            inp = network.get_input(0)
            require(inp.name == 'images' and inp.dtype == trt.float32, 'Expected FP32 images TRT input')
            profile = builder.create_optimization_profile()
            profile.set_shape(inp.name, tuple(shape), tuple(shape), tuple(shape))
            require(config.add_optimization_profile(profile) >= 0, 'Failed to attach exact input profile')
            if hasattr(config, 'builder_optimization_level'):
                config.builder_optimization_level = 3
            torch.cuda.synchronize()
            started = time.monotonic()
            serialized = builder.build_serialized_network(network, config)
            torch.cuda.synchronize()
            require(serialized is not None, 'TensorRT build returned None')
            build_seconds = time.monotonic() - started
            metadata = dict(onnx_report['metadata'])
            metadata.update(task='detect', names={'0': 'piece'}, nc=1, end2end=True, batch=1,
                            imgsz=shape[2:], requested_long_side=size, actual_input_shape=shape, stride=32,
                            precision='FP32-no-TF32', source_weights_sha256=source_hash,
                            export_script_sha256=script_hash, ultralytics_version=ultralytics.__version__)
            header = json.dumps(metadata, ensure_ascii=False, separators=(',', ':'), allow_nan=False).encode('utf-8')
            temporary = engine_path.with_name(engine_path.name + f'.tmp-{os.getpid()}')
            require(not temporary.exists(), f'Partial engine temporary file exists: {temporary}')
            with temporary.open('xb') as stream:
                stream.write(len(header).to_bytes(4, 'little', signed=True))
                stream.write(header)
                stream.write(serialized)
                stream.flush()
                os.fsync(stream.fileno())
            require(not engine_path.exists(), f'Engine appeared while building: {engine_path}')
            os.replace(temporary, engine_path)
            runtime = trt.Runtime(logger)
            engine = runtime.deserialize_cuda_engine(serialized)
            require(engine is not None, 'Built engine cannot be deserialized')
            actual_profiles = [list(s) for s in engine.get_tensor_profile_shape('images', 0)]
            require(actual_profiles == [shape] * 3, f'Built engine profile mismatch: {actual_profiles}')
            io_tensors = [{'name': engine.get_tensor_name(i),
                           'dtype': str(engine.get_tensor_dtype(engine.get_tensor_name(i))),
                           'mode': str(engine.get_tensor_mode(engine.get_tensor_name(i)))}
                          for i in range(engine.num_io_tensors)]
            require(all(engine.get_tensor_dtype(engine.get_tensor_name(i)) == trt.float32
                        for i in range(engine.num_io_tensors)), 'Non-FP32 external engine tensor')
            report = {
                'status': 'built', 'created_at_utc': now(), 'source_weights': str(args.weights),
                'source_weights_sha256': source_hash, 'export_script_sha256': script_hash,
                'onnx_path': str(onnx_path), 'onnx_sha256': sha256(onnx_path),
                'engine_path': str(engine_path), 'engine_sha256': sha256(engine_path),
                'engine_bytes': engine_path.stat().st_size, 'imgsz': size,
                'actual_input_shape': shape, 'input_shapes_min_opt_max': [shape] * 3,
                'loaded_engine_profile_min_opt_max': actual_profiles, 'io_tensors': io_tensors,
                'precision': 'FP32-no-TF32', 'precision_flags_cleared': cleared,
                'precision_flags_enabled': enabled, 'workspace_bytes': 1 << 30,
                'build_seconds': build_seconds, 'metadata': metadata, 'versions': state['versions'],
                'source_versions': source_versions,
                'restriction': 'Exact profile for full-frame 1280x720 only. Use official predict(imgsz=[height,width]) matching metadata imgsz and actual_input_shape; square or ROI tensors are unsupported.',
            }
            atomic_json(report_path, report)
            engines.append(report)
            del engine, runtime, serialized, parser, profile, config, network, builder
            torch.cuda.empty_cache()
        require(sha256(args.weights) == source_hash and sha256(copied_weights) == source_hash,
                'Source or copied checkpoint changed during export')
        artifact_manifest = {
            'status': 'completed', 'created_at_utc': now(), 'task': 'detect', 'nc': 1,
            'names': {'0': 'piece'}, 'end2end': True, 'precision': 'FP32-no-TF32',
            'source_weights': str(args.weights), 'source_weights_sha256': source_hash,
            'copied_weights': str(copied_weights), 'copied_weights_sha256': sha256(copied_weights),
            'export_script': str(Path(__file__).resolve()), 'export_script_sha256': script_hash,
            'source_versions': source_versions, 'versions': state['versions'],
            'onnx': onnx_report, 'engines': engines, 'deployed': False,
            'gpu_lock': str(args.lock), 'all_cuda_work_under_exclusive_gpu_lock': True,
        }
        atomic_json(args.artifacts / 'export-manifest.json', artifact_manifest)
        state.update(phase='gpu_work_completed', updated_at_utc=now(), completed_sizes=args.sizes,
                     manifest=str(args.artifacts / 'export-manifest.json'))
        atomic_json(args.status, state)
    state.update(state='completed', phase='completed', gpu_lock_held=False, finished_at_utc=now())
    atomic_json(args.status, state)
    print(json.dumps(state, allow_nan=False), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=Path, default=ROOT / 'runs/train/weights/best.pt')
    parser.add_argument('--artifacts', type=Path, default=ROOT / 'artifacts/yolo26n-piece-fp32-v1')
    parser.add_argument('--sizes', default='640,512,416')
    parser.add_argument('--lock', type=Path, default=GPU_LOCK)
    parser.add_argument('--lock-timeout', type=float, default=900)
    parser.add_argument('--status', type=Path, default=ROOT / 'runtime/export-status.json')
    args = parser.parse_args()
    args.sizes = [int(x) for x in args.sizes.split(',')]
    require(bool(args.sizes) and len(set(args.sizes)) == len(args.sizes) and all(x in SHAPES for x in args.sizes),
            'Sizes must be unique values selected from 640,512,416')
    require(args.lock_timeout > 0, 'Lock timeout must be positive')
    state = {'state': 'starting', 'phase': 'starting', 'pid': os.getpid(),
             'started_at_utc': now(), 'requested_sizes': args.sizes, 'gpu_lock_held': False}
    try:
        execute(args, state)
    except BaseException as exc:
        state.update(state='failed', phase='failed', gpu_lock_held=False, finished_at_utc=now(),
                     error=f'{type(exc).__name__}: {exc}', traceback=traceback.format_exc())
        atomic_json(args.status, state)
        raise


if __name__ == '__main__':
    main()
