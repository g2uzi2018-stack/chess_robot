"""Small C ABI for inverse-depth center-consistent surface fit."""
import ctypes
import numpy as np
from surface_fit import SurfaceFit


def _diag(values):
    names = ('plane_rms_m', 'depth_rms_m', 'center_subset_spread_m',
             'close_inlier_fraction', 'half_center_spread_m')
    return {k: float(values[6 + i]) for i, k in enumerate(names) if np.isfinite(values[6 + i])}


class SurfaceCenter:
    def __init__(self, path):
        self.plane = SurfaceFit(path)
        library = ctypes.CDLL(str(path))
        self.fit = library.robust_center_fit
        array = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
        self.fit.argtypes = [array, ctypes.c_int, array, ctypes.c_double, array]
        self.fit.restype = ctypes.c_int

    def __call__(self, points, ray, result, median):
        # Preserve the established fast path; review only same-frame noisy planes.
        plane = self.plane(points, ray, result, median)
        if plane['valid']:
            return dict(plane, method='surface_plane', version='v1')
        if plane.get('reason') != 'surface plane unreliable':
            return plane
        return self.center(points, ray, result, median)

    def center(self, points, ray, result, median):
        values = np.empty(11, dtype=np.float64)
        values[:] = np.nan
        status = self.fit(np.ascontiguousarray(points, dtype=np.float64), len(points),
                          np.ascontiguousarray(ray, dtype=np.float64), float(median), values)
        extra = _diag(values)
        if status == 0:
            out = dict(result, valid=True, camera_surface_center_m=values[:3].tolist(),
                camera_surface_normal=values[3:6].tolist(), depth_median_m=float(median),
                method='inverse_depth_center', version='v2',
                point_semantics='visible top surface center; not cylinder center or grasp TCP')
            out.update(extra)
            return out
        reasons = {
            1: 'surface plane rejected',
            3: 'invalid plane intersection',
            4: 'insufficient quadrant support',
            5: 'insufficient spatial spread',
            6: 'insufficient retained fraction',
            7: 'center inconsistent across quadrants',
            8: 'surface depth rms too high',
            9: 'grazing plane intersection',
            10: 'insufficient close depth inliers',
            11: 'opposite half center inconsistent',
        }
        if status not in reasons:
            raise RuntimeError('Unexpected native center-fit status: '+str(status))
        return dict(result, reason=reasons[status], **extra)
