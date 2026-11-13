"""Build the trained 96px BGR glyph classifier for bounded TensorRT batches."""
import argparse
import hashlib
import json
from pathlib import Path
import torch
import tensorrt as trt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    model = torch.jit.load(str(args.source), map_location='cpu').eval()
    onnx = args.output/'classifier.onnx'
    torch.onnx.export(model, torch.zeros(1, 3, 96, 96), str(onnx), dynamo=False,
        input_names=['images'], output_names=['logits'], opset_version=17,
        dynamic_axes={'images': {0: 'batch'}, 'logits': {0: 'batch'}})
    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, logger)
    if not parser.parse(onnx.read_bytes()):
        raise RuntimeError('\n'.join(str(parser.get_error(i)) for i in range(parser.num_errors)))
    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 512*1024*1024)
    for name in ('TF32', 'FP16', 'INT8', 'BF16', 'FP8'):
        flag = getattr(trt.BuilderFlag, name, None)
        if flag is not None:
            config.clear_flag(flag)
    config.set_flag(trt.BuilderFlag.FP16)
    profile = builder.create_optimization_profile()
    profile.set_shape('images', (1, 3, 96, 96), (32, 3, 96, 96), (32, 3, 96, 96))
    config.add_optimization_profile(profile)
    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError('Classifier TensorRT build failed')
    metadata = dict(source_sha256=hashlib.sha256(args.source.read_bytes()).hexdigest(),
        input='images', output='logits', classes=14, max_batch=32, precision='FP16-mixed',
        input_layout='BGR float32 NCHW / 255, 96x96', tensorrt=trt.__version__)
    header = json.dumps(metadata).encode()
    artifact = args.output/'classifier.engine'
    artifact.write_bytes(len(header).to_bytes(4, 'little')+header+bytes(serialized))
    metadata['artifact_sha256'] = hashlib.sha256(artifact.read_bytes()).hexdigest()
    (args.output/'build.json').write_text(json.dumps(metadata, indent=2))
    print(json.dumps(metadata), flush=True)


if __name__ == '__main__':
    main()
