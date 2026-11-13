"""Batch glyph classification on crops from the current RGB frame."""
import hashlib
from pathlib import Path
import cv2
import numpy as np

def crop_window(box):
    box=np.asarray(box,dtype=float)
    center=(box[:2]+box[2:])/2
    half=float(max(box[2:]-box[:2]))*1.15/2
    return [float(center[0]-half),float(center[1]-half),float(center[0]+half),float(center[1]+half)]

def piece_crop(image, box):
    box = np.asarray(box, dtype=float)
    center = (box[:2] + box[2:]) / 2
    size = float(max(box[2:] - box[:2])) * 1.15
    matrix = np.array([[size/96, 0, center[0]-size/2],
                       [0, size/96, center[1]-size/2]], dtype=np.float32)
    return cv2.warpAffine(image, matrix, (96, 96),
        flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
        borderMode=cv2.BORDER_REPLICATE)

class PieceClassifier:
    def __init__(self, path):
        import torch
        self.torch = torch
        self.sha256 = hashlib.sha256(Path(path).read_bytes()).hexdigest()
        if Path(path).suffix=='.engine':
            from classifier_engine import ClassifierEngine
            self.model=ClassifierEngine(path)
            self.precision=self.model.metadata['precision']
        else:
            self.model=torch.jit.load(str(path),map_location='cuda').eval()
            self.precision='FP32'
        # Fixed input dimensions; warming common batch sizes keeps first-use
        # specialization outside the camera's processing loop.
        with torch.inference_mode():
            for n in (1, 2, 4, 8, 16, 32):
                self.model(torch.zeros(n, 3, 96, 96, device='cuda'))

    def classify(self, image, boxes):
        if not len(boxes):
            return np.empty((0, 14), dtype=np.float32)
        torch = self.torch
        with torch.inference_mode():
            boxes = torch.as_tensor(np.asarray(boxes), device='cuda', dtype=torch.float32)
            centers = (boxes[:, :2]+boxes[:, 2:])/2
            sizes = (boxes[:, 2:]-boxes[:, :2]).amax(1)*1.15
            steps = torch.arange(96, device='cuda', dtype=torch.float32)/96
            x = (centers[:, 0]-sizes/2)[:, None]+sizes[:, None]*steps
            y = (centers[:, 1]-sizes/2)[:, None]+sizes[:, None]*steps
            # Match OpenCV's five-bit bilinear coordinate quantization.
            x = (x*32).round()/32
            y = (y*32).round()/32
            height, width = image.shape[:2]
            grid = torch.stack((x[:, None, :].expand(-1, 96, -1)*2/(width-1)-1,
                                y[:, :, None].expand(-1, -1, 96)*2/(height-1)-1), -1)
            source = torch.from_numpy(image.transpose(2, 0, 1).copy()).to('cuda').float()[None]
            crops = torch.nn.functional.grid_sample(source.expand(len(boxes), -1, -1, -1),
                grid, mode='bilinear', padding_mode='border', align_corners=True).round()/255.
            return self.model(crops).softmax(1).cpu().numpy()
