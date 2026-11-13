"""Small C ABI for the unchanged three-pass robust surface plane fit."""
import ctypes
import numpy as np


class SurfaceFit:
    def __init__(self, path):
        library = ctypes.CDLL(str(path))
        self.fit = library.robust_plane_fit
        array = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
        self.fit.argtypes = [array, ctypes.c_int, array, array]
        self.fit.restype = ctypes.c_int

    def __call__(self, points, ray, result, median):
        values = np.empty(7, dtype=np.float64)
        status = self.fit(np.ascontiguousarray(points, dtype=np.float64), len(points),
                          np.ascontiguousarray(ray, dtype=np.float64), values)
        if status == 1:
            return dict(result, reason='surface plane rejected')
        if status == 2:
            return dict(result, reason='surface plane unreliable', plane_rms_m=float(values[6]))
        if status == 3:
            return dict(result, reason='invalid plane intersection')
        if status != 0:
            raise RuntimeError('Unexpected native plane-fit status: '+str(status))
        return dict(result, valid=True, camera_surface_center_m=values[:3].tolist(),
            camera_surface_normal=values[3:6].tolist(), plane_rms_m=float(values[6]),
            depth_median_m=median,
            point_semantics='visible top surface center; not cylinder center or grasp TCP')
