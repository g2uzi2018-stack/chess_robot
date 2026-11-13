"""TensorRT logits on the same CUDA stream as crop preparation and softmax."""
import json
import torch
import tensorrt as trt


class ClassifierEngine:
    def __init__(self, path):
        with open(path, 'rb') as stream:
            size = int.from_bytes(stream.read(4), 'little')
            self.metadata = json.loads(stream.read(size))
            self.logger = trt.Logger(trt.Logger.ERROR)
            self.runtime = trt.Runtime(self.logger)
            self.engine = self.runtime.deserialize_cuda_engine(stream.read())
        self.context = self.engine.create_execution_context()
        self.output = torch.empty((32, 14), device='cuda', dtype=torch.float32)

    def __call__(self, images):
        outputs = []
        for part in images.split(32):
            part = part.contiguous()
            output = self.output[:len(part)]
            self.context.set_input_shape('images', tuple(part.shape))
            self.context.set_tensor_address('images', part.data_ptr())
            self.context.set_tensor_address('logits', output.data_ptr())
            if not self.context.execute_async_v3(torch.cuda.current_stream().cuda_stream):
                raise RuntimeError('Classifier TensorRT execution failed')
            outputs.append(output.clone() if len(images) > 32 else output)
        return torch.cat(outputs) if len(outputs) > 1 else outputs[0]
