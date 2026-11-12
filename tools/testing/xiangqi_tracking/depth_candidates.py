"""Colour-independent, same-frame tabletop proposals; experimental, not a calibration.

The supplied seed identifies *bare table*, not merely a large plane. Pixel polygons
are valid only in their bound camera view. A successful fit cannot prove that the
operator chose the physical table rather than a board or an outside surface.
Masks describe thresholded visible height support, not exact cylinder outlines.
"""
from dataclasses import dataclass
import math
import time

import cv2
import numpy as np


@dataclass(frozen=True)
class DepthCandidateConfig:
    image_size: object = None  # [width, height]
    workspace_polygon_xy: object = None
    table_seed_polygon_xy: object = None
    min_depth_m: float = .02
    max_depth_m: float = 1.5
    plane_inlier_m: float = .008
    max_plane_rms_m: float = .004
    min_seed_inlier_fraction: float = .95
    min_seed_pixels: int = 100
    max_fit_points: int = 3000
    ransac_iterations: int = 48
    min_facing_cosine: float = .35
    min_height_m: float = .008
    max_height_m: float = .080
    min_object_height_m: object = None  # physically verified smallest board/piece height
    expected_piece_thickness_m: object = None  # nominal estimate; not a calibration
    expected_board_thickness_m: object = None  # nominal estimate; not a calibration
    min_valid_fraction: float = .995
    min_candidate_area_m2: float = .00008
    max_piece_span_m: float = .095
    single_piece_diameter_m: object = None  # physically confirmed; no merged-object guarantee otherwise
    min_board_span_m: float = .120
    min_board_area_m2: float = .008
    board_plane_band_m: float = .004
    min_board_planar_fraction: float = .55
    min_board_rectangularity: float = .90
    min_component_pixels: int = 12
    edge_margin_px: int = 2
    max_candidates: int = 300
    max_components: int = 4096
    median_kernel_px: int = 3

    def __post_init__(self):
        integers = ('min_seed_pixels', 'max_fit_points', 'ransac_iterations',
                    'min_component_pixels', 'edge_margin_px', 'max_candidates', 'max_components', 'median_kernel_px')
        for name in integers:
            v = getattr(self, name)
            if type(v) is not int or v < (0 if name == 'edge_margin_px' else 1):
                raise ValueError('invalid ' + name)
        for name in ('min_depth_m', 'max_depth_m', 'plane_inlier_m', 'max_plane_rms_m',
                     'min_height_m', 'max_height_m', 'min_candidate_area_m2',
                     'max_piece_span_m', 'min_board_span_m', 'min_board_area_m2', 'board_plane_band_m'):
            v = getattr(self, name)
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
                raise ValueError('invalid ' + name)
        for name in ('min_seed_inlier_fraction', 'min_facing_cosine', 'min_valid_fraction', 'min_board_planar_fraction', 'min_board_rectangularity'):
            v = getattr(self, name)
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not 0 < v <= 1:
                raise ValueError('invalid ' + name)
        if not self.min_depth_m < self.max_depth_m or not self.min_height_m < self.max_height_m:
            raise ValueError('invalid depth/height interval')
        if self.max_candidates > 300 or self.median_kernel_px not in (1, 3, 5):
            raise ValueError('invalid candidate limit or median kernel')
        for name in ('min_object_height_m', 'single_piece_diameter_m',
                     'expected_piece_thickness_m', 'expected_board_thickness_m'):
            value = getattr(self, name)
            if value is not None and (isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value <= 0):
                raise ValueError('invalid ' + name)


def _polygon_mask(polygon, shape, name):
    if polygon is None:
        raise ValueError(name + ' required')
    p = np.asarray(polygon, dtype=float)
    if p.ndim != 2 or p.shape[0] < 3 or p.shape[1] != 2 or not np.isfinite(p).all():
        raise ValueError('invalid ' + name)
    h, w = shape
    if np.any(p < 0) or np.any(p[:, 0] >= w) or np.any(p[:, 1] >= h):
        raise ValueError(name + ' outside image')
    mask = np.zeros(shape, np.uint8)
    cv2.fillPoly(mask, [np.round(p).astype(np.int32)], 1)
    if int(mask.sum()) < 3:
        raise ValueError(name + ' has no area')
    return mask.astype(bool)


def _fit_plane(points, cfg):
    rng = np.random.default_rng(0)
    best = None
    for _ in range(cfg.ransac_iterations):
        p = points[rng.choice(len(points), 3, replace=False)]
        n = np.cross(p[1]-p[0], p[2]-p[0]); norm = np.linalg.norm(n)
        if norm < 1e-10:
            continue
        n /= norm
        if abs(n[2]) < cfg.min_facing_cosine:
            continue
        d = -float(n @ p[0]); errors = np.abs(points @ n+d)
        keep = errors <= cfg.plane_inlier_m
        score = (int(keep.sum()), -float(np.median(errors)))
        if best is None or score > best[0]:
            best = score, keep
    if best is None:
        raise ValueError('no camera-facing table plane')
    keep = best[1]
    for _ in range(3):
        if int(keep.sum()) < 3:
            raise ValueError('insufficient table plane support')
        p = points[keep]; center = p.mean(0)
        _, vectors = np.linalg.eigh((p-center).T @ (p-center))
        n = vectors[:, 0]
        if n[2] > 0:
            n = -n
        d = -float(n @ center)
        errors = np.abs(points @ n+d)
        keep = errors <= cfg.plane_inlier_m
    rms = float(np.sqrt(np.mean(errors[keep]**2)))
    return n, d, keep, rms, errors


def detect_depth_candidates(depth_m, K, config=None, workspace_mask=None, table_seed_mask=None):
    """Return all resolved/unresolved raised regions; never turn rejection into empty success.

    ``valid`` means a usable seeded reference plane. ``coverage_valid`` also
    requires sufficient visible depth, resolvable configured minimum object
    height, and no unhandled components. It is scoped to the configured polygon,
    not the whole desk or unseen area. View/session verification is an upstream
    responsibility; the integration intentionally runs this detector in shadow.
    """
    started = time.monotonic()
    cfg = config if isinstance(config, DepthCandidateConfig) else DepthCandidateConfig(**(config or {}))
    result = dict(valid=False, coverage_valid=False, full_scene_coverage=False,
                  candidates=[], board_regions=[], unresolved_regions=[], table_plane=None, reasons=[],
                  diagnostics=dict(coverage_scope='configured_workspace_current_view',
                                   geometry_semantics='thresholded visible height support; not exact piece outline',
                                   table_identity='externally supplied bare-table seed; not independently verified'))
    reasons = result['reasons']; diag = result['diagnostics']

    def fail(reason):
        if reason not in reasons:
            reasons.append(reason)

    def finish():
        result['reason'] = '; '.join(reasons) if reasons else 'ok'
        result['full_scene_coverage'] = result['coverage_valid']
        diag['detector_ms'] = float((time.monotonic()-started)*1000)
        return result

    z = np.asarray(depth_m)
    k = np.asarray(K, dtype=float)
    if z.ndim != 2 or z.dtype != np.float32 or k.shape != (3, 3) or not np.isfinite(k).all():
        fail('invalid depth or intrinsics'); return finish()
    h, w = z.shape
    if cfg.image_size is None or list(cfg.image_size) != [w, h]:
        fail('configured image_size missing or mismatched'); return finish()
    if (k[0, 0] <= 0 or k[1, 1] <= 0 or not np.allclose(k[2], [0, 0, 1])
            or abs(float(np.linalg.det(k))) < 1e-9):
        fail('invalid intrinsics'); return finish()
    try:
        workspace = (_polygon_mask(cfg.workspace_polygon_xy, z.shape, 'workspace_polygon_xy')
                     if workspace_mask is None else np.asarray(workspace_mask, dtype=bool))
        seed = (_polygon_mask(cfg.table_seed_polygon_xy, z.shape, 'table_seed_polygon_xy')
                if table_seed_mask is None else np.asarray(table_seed_mask, dtype=bool))
        if workspace.shape != z.shape or seed.shape != z.shape or not workspace.any() or not seed.any():
            raise ValueError('invalid workspace or table seed mask')
        if np.any(seed & ~workspace):
            raise ValueError('table seed outside workspace')
    except (ValueError, TypeError) as exc:
        fail(str(exc)); return finish()
    finite = np.isfinite(z) & (z > cfg.min_depth_m) & (z < cfg.max_depth_m)
    fraction = float(np.count_nonzero(finite & workspace)/np.count_nonzero(workspace))
    diag.update(workspace_pixels=int(workspace.sum()), valid_depth_fraction=fraction,
                invalid_depth_pixels=int(np.count_nonzero(workspace & ~finite)))
    if fraction < cfg.min_valid_fraction:
        fail('workspace depth holes exceed limit')
    if np.any(workspace & ~finite):
        # A small global fraction may still hide a complete chess piece.
        # No unobserved pixels are certified empty in this conservative version.
        fail('workspace contains unobserved depth; absence cannot be certified')
    seed_valid = finite & seed
    if np.count_nonzero(seed_valid) < cfg.min_seed_pixels:
        fail('insufficient valid bare-table seed'); return finish()
    if np.count_nonzero(seed_valid)/np.count_nonzero(seed) < cfg.min_valid_fraction:
        fail('table seed depth holes exceed limit')
    yy, xx = np.indices(z.shape, dtype=np.float32)
    # General pinhole K, including skew. Normalized rays have z=1.
    inv = np.linalg.inv(k)
    rx = inv[0, 0]*xx + inv[0, 1]*yy + inv[0, 2]
    ry = inv[1, 0]*xx + inv[1, 1]*yy + inv[1, 2]
    sy, sx = np.nonzero(seed_valid)
    stride = max(1, math.ceil(len(sx)/cfg.max_fit_points)); sy, sx = sy[::stride], sx[::stride]
    sz = z[sy, sx].astype(float)
    points = np.column_stack([rx[sy, sx]*sz, ry[sy, sx]*sz, sz])
    try:
        n, d, keep, rms, errors = _fit_plane(points, cfg)
    except ValueError as exc:
        fail(str(exc)); return finish()
    inlier_fraction = float(keep.mean())
    q95 = float(np.percentile(errors, 95))
    result['table_plane'] = dict(normal=n.tolist(), d_m=d, rms_m=rms,
                                 inlier_fraction=inlier_fraction, seed_p95_residual_m=q95)
    if inlier_fraction < cfg.min_seed_inlier_fraction:
        fail('table seed contains multiple surfaces or excessive outliers')
    if rms > cfg.max_plane_rms_m or abs(n[2]) < cfg.min_facing_cosine:
        fail('table reference plane unreliable')
    if any(r.startswith('table seed') or r.startswith('table reference') for r in reasons):
        return finish()
    result['valid'] = True
    # A configured feature lower than the local noise cannot be certified absent.
    reliable_min = float(max(cfg.min_height_m, 2*q95))
    diag['minimum_resolvable_height_m'] = reliable_min
    diag['configured_foreground_threshold_m'] = float(cfg.min_height_m)
    diag['expected_piece_thickness_m'] = cfg.expected_piece_thickness_m
    diag['expected_board_thickness_m'] = cfg.expected_board_thickness_m
    diag['thickness_semantics'] = 'nominal configuration; physical dimensions require independent confirmation'
    if cfg.min_object_height_m is None:
        fail('minimum physical board/piece height unconfirmed')
    elif cfg.min_object_height_m <= reliable_min:
        fail('smallest configured object height not resolvable above table noise')
    zf = np.where(finite, z, 0).astype(np.float32)
    if cfg.median_kernel_px > 1:
        filtered = cv2.medianBlur(zf, cfg.median_kernel_px)
        # Filtering never grants validity to original depth holes.
        zf = np.where(finite, filtered, 0)
    signed = (n[0]*rx+n[1]*ry+n[2])*zf+d
    positive = finite & workspace & (signed > cfg.min_height_m)
    negative = finite & workspace & (signed < -max(cfg.min_height_m, 2*q95))
    diag['below_table_pixels'] = int(negative.sum())
    if negative.any():
        fail('configured workspace includes below-table or unsupported surface')
    # Perspective-aware approximate area on the table plane.
    ray_dot = np.abs(n[0]*rx+n[1]*ry+n[2])
    pixel_area = zf*zf/(k[0, 0]*k[1, 1]*np.maximum(ray_dot, .1))
    boundary_distance = cv2.distanceTransform(np.pad(workspace.astype(np.uint8), 1), cv2.DIST_L2, 3)[1:-1, 1:-1]
    count, labels, stats, _ = cv2.connectedComponentsWithStats(positive.astype(np.uint8), 8)
    diag['raw_raised_component_count'] = int(count-1)
    diag['small_raised_component_count'] = 0
    diag['small_raised_pixel_count'] = 0
    diag['unresolved_region_count'] = 0
    diag['candidate_budget_overflow_count'] = 0
    if count-1 > cfg.max_components:
        fail('raised component processing budget exceeded')
        diag['unprocessed_raised_component_count'] = int(count-1)
        return finish()

    def unresolved(summary):
        diag['unresolved_region_count'] += 1
        if len(result['unresolved_regions']) < cfg.max_candidates:
            result['unresolved_regions'].append(summary)

    def describe(mask, height, surface, bounds=None):
        x, y, bw, bh = cv2.boundingRect(mask.astype(np.uint8)) if bounds is None else bounds
        local = mask[y:y+bh, x:x+bw]
        ys, xs = np.nonzero(local); ys += y; xs += x
        area = float(pixel_area[ys, xs].sum())
        # Physical extent evaluated in a tangent basis, independent of table tilt.
        zs = zf[ys, xs].astype(float)
        pts = np.column_stack([rx[ys, xs]*zs, ry[ys, xs]*zs, zs])
        t = np.cross(n, [0, 1, 0]); t /= np.linalg.norm(t)
        b = np.cross(n, t)
        spans = np.ptp(np.column_stack([pts@t, pts@b]), axis=0)
        return dict(box_xyxy=[float(xs.min()), float(ys.min()), float(xs.max()+1), float(ys.max()+1)],
                    mask=mask.astype(np.uint8), height_m=float(height), reference_surface=surface,
                    area_m2=area, span_m=spans.tolist(), candidate_complete=True, reasons=[])

    def incomplete(item, why):
        item['candidate_complete'] = False; item['reasons'].append(why); fail(why)

    def emit_piece(mask, base, surface, item=None):
        if len(result['candidates']) >= cfg.max_candidates:
            diag['candidate_budget_overflow_count'] += 1
            fail('depth candidate limit exceeded')
            return
        if item is None:
            item = describe(mask, float(np.median(signed[mask])-base), surface)
        # Classification can still be useful when depth geometry is incomplete.
        # Only physical-area-supported domains reach this branch from the table;
        # board-top domains get the same explicit admission test here.
        item['roi_eligible'] = bool(item['area_m2'] >= cfg.min_candidate_area_m2)
        span = max(item['span_m'])
        if item['area_m2'] < cfg.min_candidate_area_m2:
            incomplete(item, 'raised component below configured piece area')
        if span > cfg.max_piece_span_m:
            incomplete(item, 'raised component may contain touching pieces or unresolved board')
        if cfg.single_piece_diameter_m is None:
            incomplete(item, 'single-piece dimensions unconfirmed')
        elif span > cfg.single_piece_diameter_m*1.15:
            incomplete(item, 'raised span exceeds confirmed single-piece diameter')
        x1, y1, x2, y2 = map(int, item['box_xyxy'])
        region = np.s_[max(0,y1-1):min(h,y2+1), max(0,x1-1):min(w,x2+1)]
        local_mask = mask[region]
        if np.any(boundary_distance[region][local_mask] <= cfg.edge_margin_px):
            incomplete(item, 'raised component touches workspace boundary')
        if np.any(signed[region][local_mask]-base > cfg.max_height_m):
            incomplete(item, 'raised component exceeds configured height')
        # Holes next to the foreground make its original outline uncertain.
        rim = cv2.dilate(local_mask.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
        if np.any(rim & workspace[region] & ~finite[region]):
            incomplete(item, 'raised component borders invalid depth')
        result['candidates'].append(item)

    for label in range(1, count):
        pixels = int(stats[label, cv2.CC_STAT_AREA])
        if pixels < cfg.min_component_pixels:
            diag['small_raised_component_count'] += 1
            diag['small_raised_pixel_count'] += pixels
            fail('unresolved small raised components')
            continue
        x, y, bw, bh = map(int, stats[label, :4])
        region = np.s_[y:y+bh, x:x+bw]
        local_mask = labels[region] == label
        area = float(pixel_area[region][local_mask].sum())
        if area < cfg.min_candidate_area_m2:
            # Keep the rejection visible without allocating a full-frame mask
            # and paying an inference crop for every table-noise speck.
            unresolved(dict(box_xyxy=[x, y, x+bw, y+bh], area_m2=area, pixels=pixels,
                            reason='raised component below configured piece area'))
            fail('raised component below configured piece area')
            continue
        if len(result['candidates']) >= cfg.max_candidates:
            diag['candidate_budget_overflow_count'] += 1
            unresolved(dict(box_xyxy=[x,y,x+bw,y+bh], area_m2=area, pixels=pixels,
                            reason='raised region unprocessed after candidate budget exhausted'))
            fail('depth candidate limit exceeded')
            continue
        mask = labels == label
        item = describe(mask, float(np.median(signed[region][local_mask])), 'table', (x, y, bw, bh))
        # Broad raised support can be a board. Its low height mode must itself
        # form one broad connected planar area; a group of chess tops is not a board.
        if max(item['span_m']) >= cfg.min_board_span_m and item['area_m2'] >= cfg.min_board_area_m2:
            base = float(np.percentile(signed[mask], 30))
            planar = mask & (np.abs(signed-base) <= cfg.board_plane_band_m)
            nc, plabels, pst, _ = cv2.connectedComponentsWithStats(planar.astype(np.uint8), 8)
            largest = int(np.argmax(pst[1:, cv2.CC_STAT_AREA])+1) if nc > 1 else 0
            plane_mask = plabels == largest if largest else np.zeros(z.shape, bool)
            planar_fraction = float(plane_mask.sum()/pixels)
            if largest and planar_fraction >= cfg.min_board_planar_fraction:
                board = describe(plane_mask, base, 'table')
                if (board['area_m2'] >= cfg.min_board_area_m2
                        and max(board['span_m']) >= cfg.min_board_span_m):
                    board['planar_fraction'] = planar_fraction
                    # Test the whole raised footprint: actual piece tops punch
                    # legitimate holes in the fitted board-plane inliers.
                    by, bx = np.nonzero(mask)
                    bz = (base-d)/(n[0]*rx[by,bx]+n[1]*ry[by,bx]+n[2])
                    bp = np.column_stack([rx[by,bx]*bz, ry[by,bx]*bz, bz])
                    tangent = np.cross(n, [0,1,0]); tangent /= np.linalg.norm(tangent)
                    bitangent = np.cross(n, tangent)
                    projected = np.column_stack([bp@tangent, bp@bitangent]).astype(np.float32)
                    rectangle = cv2.minAreaRect(np.ascontiguousarray(projected[::max(1,len(projected)//5000)]))
                    rect_area = float(rectangle[1][0]*rectangle[1][1])
                    rectangularity = min(1., item['area_m2']/max(rect_area, 1e-12))
                    board['rectangularity'] = rectangularity
                    if rectangularity < cfg.min_board_rectangularity:
                        # A connected grid of equal-height round chess tops is
                        # not evidence of a solid board. Keep the whole region
                        # unresolved instead of silently erasing all pieces.
                        incomplete(item, 'broad raised region is not a sufficiently solid rectangular board')
                        emit_piece(mask, 0., 'table', item=item)
                        continue
                    if (cfg.expected_board_thickness_m is not None
                            and abs(base-cfg.expected_board_thickness_m) > max(2*q95, cfg.board_plane_band_m)):
                        incomplete(item, 'broad raised plane inconsistent with nominal board thickness')
                        emit_piece(mask, 0., 'table', item=item)
                        continue
                    if base > cfg.max_height_m:
                        incomplete(board, 'board exceeds configured height')
                    if np.any(boundary_distance[mask] <= cfg.edge_margin_px):
                        incomplete(board, 'board touches workspace boundary')
                    if len(result['board_regions']) >= cfg.max_candidates:
                        fail('board region limit exceeded')
                        continue
                    result['board_regions'].append(board)
                    upper = mask & (signed > base+cfg.min_height_m)
                    upper_count, upper_labels, upper_stats, _ = cv2.connectedComponentsWithStats(upper.astype(np.uint8), 8)
                    for j in range(1, upper_count):
                        m = upper_labels == j
                        if upper_stats[j, cv2.CC_STAT_AREA] < cfg.min_component_pixels:
                            fail('unresolved small raised components on board')
                        else:
                            emit_piece(m, base, 'board')
                    middle = mask & ~planar & ~upper
                    if np.count_nonzero(middle) >= cfg.min_component_pixels:
                        fail('unresolved board height transition')
                    continue
        emit_piece(mask, 0., 'table', item=item)
    diag['candidate_count_before_limit'] = len(result['candidates'])+diag['candidate_budget_overflow_count']
    diag['candidate_count_semantics'] = 'retained candidates plus unprocessed potential candidates after budget exhaustion'
    result['coverage_valid'] = bool(result['valid'] and not reasons)
    return finish()
