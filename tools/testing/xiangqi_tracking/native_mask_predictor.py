"""Native-resolution mask patches; avoid upsampling the empty full-image region."""
import torch
from ultralytics.models.yolo.segment.predict import SegmentationPredictor
from ultralytics.utils import ops
from ultralytics.engine.results import Results


def native_mask_patches(protos, coefficients, boxes, shape):
    channels, mask_height, mask_width = protos.shape
    height, width = shape
    masks = (coefficients @ protos.float().view(channels, -1)).view(-1, mask_height, mask_width)
    gain = min(mask_height/height, mask_width/width)
    pad_x = (mask_width-round(width*gain))/2
    pad_y = (mask_height-round(height*gain))/2
    top, left = round(pad_y-.1), round(pad_x-.1)
    bottom, right = mask_height-round(pad_y+.1), mask_width-round(pad_x+.1)
    masks = masks[:, None, top:bottom, left:right]
    bounds = boxes.ceil().long()
    bounds[:, [0, 2]] = bounds[:, [0, 2]].clamp(0, width)
    bounds[:, [1, 3]] = bounds[:, [1, 3]].clamp(0, height)
    max_width, max_height = (bounds[:, 2:]-bounds[:, :2]).amax(0).tolist()
    x = bounds[:, 0, None, None]+torch.arange(max(1, max_width), device=boxes.device)[None, None, :]
    y = bounds[:, 1, None, None]+torch.arange(max(1, max_height), device=boxes.device)[None, :, None]
    # Same align_corners=False pixel centers and border padding as native upsampling.
    grid = torch.stack(((x.expand(-1, max(1, max_height), -1)+.5)*2/width-1,
                        (y.expand(-1, -1, max(1, max_width))+.5)*2/height-1), -1)
    patches = torch.nn.functional.grid_sample(masks, grid, mode='bilinear',
        padding_mode='border', align_corners=False)[:, 0]
    valid = (x < bounds[:, 2, None, None]) & (y < bounds[:, 3, None, None])
    return ((patches > 0) & valid).byte(), bounds


class NativeMaskPredictor(SegmentationPredictor):
    def construct_result(self, pred, img, orig_img, img_path, proto):
        if not self.args.retina_masks:
            return super().construct_result(pred, img, orig_img, img_path, proto)
        masks = bounds = None
        if pred.shape[0]:
            pred[:, :4] = ops.scale_boxes(img.shape[2:], pred[:, :4], orig_img.shape)
            masks, bounds = native_mask_patches(proto, pred[:, 6:], pred[:, :4], orig_img.shape[:2])
            keep = masks.amax((-2, -1)) > 0
            if not keep.all().item():
                pred, masks, bounds = pred[keep], masks[keep], bounds[keep]
        if masks is not None:
            from fast_recognition import unique_indices
            indices=unique_indices(pred[:, :6].cpu().numpy())
            pred, masks, bounds=pred[indices], masks[indices], bounds[indices]
        result = Results(orig_img, path=img_path, names=self.model.names, boxes=pred[:, :6], masks=masks)
        result.native_mask_bounds = None if bounds is None else bounds.cpu().numpy()
        return result
