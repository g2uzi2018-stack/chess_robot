"""Same-frame ROI grouping; never wait for another camera frame."""
from collections import OrderedDict


def roi_batches(crops, batch_size):
    """Batch compatible source shapes without changing serial letterboxing.

    Square crops always become imgsz x imgsz, even if source sizes differ.
    Rectangular crops are grouped by exact source shape: Ultralytics rect=True
    otherwise disables minimum-rectangle padding for mixed source shapes.
    """
    groups = OrderedDict()
    for index, (x1, y1, x2, y2) in enumerate(crops):
        height, width = y2-y1, x2-x1
        key = ('square',) if height == width else (height, width)
        groups.setdefault(key, []).append(index)
    return [indices[start:start+batch_size] for indices in groups.values()
            for start in range(0, len(indices), batch_size)]
