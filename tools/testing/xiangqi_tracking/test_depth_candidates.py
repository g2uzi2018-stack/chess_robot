"""Geometry and conservative coverage counterexamples for the shadow scanner."""
import json
import cv2
import numpy as np
import pytest

from depth_candidates import DepthCandidateConfig, detect_depth_candidates


H, W = 240, 320
K = np.array([[400., 0., 160.], [0., 400., 120.], [0., 0., 1.]])


def config(**kwargs):
    values = dict(image_size=[W, H], workspace_polygon_xy=[[10, 10], [309, 10], [309, 229], [10, 229]],
                  table_seed_polygon_xy=[[30, 20], [280, 20], [280, 60], [30, 60]],
                  min_object_height_m=.020, single_piece_diameter_m=.055,
                  min_height_m=.008, median_kernel_px=1)
    values.update(kwargs)
    return DepthCandidateConfig(**values)


def scene(tilted=False):
    n = np.array([.25, -.18, -1.]) if tilted else np.array([0., 0., -1.])
    n /= np.linalg.norm(n)
    yy, xx = np.indices((H, W))
    rays = np.stack([(xx-160)/400., (yy-120)/400., np.ones((H, W))], axis=-1)
    denominator = rays@n
    return ((-.5/denominator).astype(np.float32), denominator)


def add_height(depth, denominator, mask, height):
    depth[mask] = ((height-.5)/denominator[mask]).astype(np.float32)


def disk(x=160, y=140, radius=18):
    m = np.zeros((H, W), np.uint8)
    cv2.circle(m, (x, y), radius, 1, -1)
    return m.astype(bool)


def test_static_piece_on_tilted_table_measures_perpendicular_height():
    z, den = scene(True); m = disk(); add_height(z, den, m, .020)
    a = detect_depth_candidates(z, K, config())
    b = detect_depth_candidates(z.copy(), K, config())
    assert a['coverage_valid'] and len(a['candidates']) == 1
    assert a['candidates'][0]['height_m'] == pytest.approx(.020, abs=2e-6)
    assert a['candidates'][0]['box_xyxy'] == b['candidates'][0]['box_xyxy']
    np.testing.assert_array_equal(a['candidates'][0]['mask'], m.astype(np.uint8))
    assert a['table_plane']['rms_m'] < 1e-6
    json.dumps({k: v for k, v in a.items() if k not in ('candidates', 'board_regions')}, allow_nan=False)


def test_empty_table_can_be_distinguished_from_missing_reference():
    z, _ = scene()
    good = detect_depth_candidates(z, K, config())
    assert good['valid'] and good['coverage_valid'] and good['candidates'] == []
    missing = detect_depth_candidates(z, K, DepthCandidateConfig(image_size=[W, H]))
    assert not missing['valid'] and not missing['coverage_valid']


def test_unconfirmed_thin_board_never_becomes_certified_empty():
    z, den = scene(); m = np.zeros((H, W), bool); m[90:210, 50:270] = True
    add_height(z, den, m, .002)
    out = detect_depth_candidates(z, K, config(min_object_height_m=None))
    assert out['valid'] and not out['coverage_valid'] and out['candidates'] == []
    assert 'minimum physical board/piece height unconfirmed' in out['reasons']
    out = detect_depth_candidates(z, K, config(min_object_height_m=.002))
    assert not out['coverage_valid']
    assert 'smallest configured object height not resolvable above table noise' in out['reasons']


def test_depth_holes_are_not_empty_space():
    z, _ = scene(); z[100:120, 140:180] = np.nan
    out = detect_depth_candidates(z, K, config())
    assert out['valid'] and not out['coverage_valid']
    assert out['diagnostics']['invalid_depth_pixels'] == 800


def test_outside_workspace_objects_do_not_create_candidates():
    z, den = scene(); m = disk(5, 130, 3); add_height(z, den, m, .03)
    out = detect_depth_candidates(z, K, config())
    assert out['coverage_valid'] and not out['candidates']
    # The same outside surface admitted into the workspace must not be ignored.
    z[150:170, 150:190] = .7
    out = detect_depth_candidates(z, K, config())
    assert not out['coverage_valid']
    assert out['diagnostics']['below_table_pixels'] == 800


def test_mixed_table_seed_cannot_silently_select_largest_surface():
    z, den = scene(); m = np.zeros((H, W), bool); m[20:61, 30:155] = True
    add_height(z, den, m, .03)
    out = detect_depth_candidates(z, K, config())
    assert not out['valid'] and not out['coverage_valid']
    assert 'table seed contains multiple surfaces or excessive outliers' in out['reasons']


def test_full_wrong_seed_identity_is_declared_external_assumption():
    # Geometry alone cannot distinguish a flat board from a flat tabletop.
    z, _ = scene()
    out = detect_depth_candidates(z, K, config())
    assert 'externally supplied' in out['diagnostics']['table_identity']


def test_small_raised_domain_is_counted_and_blocks_coverage():
    z, den = scene(); m = np.zeros((H, W), bool); m[140:142, 160:162] = True
    add_height(z, den, m, .02)
    out = detect_depth_candidates(z, K, config())
    assert not out['coverage_valid'] and not out['candidates']
    assert out['diagnostics']['small_raised_component_count'] == 1
    assert out['diagnostics']['small_raised_pixel_count'] == 4


def test_board_is_separate_reference_and_piece_height_is_above_board():
    z, den = scene(); board = np.zeros((H, W), bool); board[90:210, 50:270] = True
    add_height(z, den, board, .012)
    piece = disk(); add_height(z, den, piece, .032)
    out = detect_depth_candidates(z, K, config())
    assert out['coverage_valid'] and len(out['board_regions']) == 1
    assert len(out['candidates']) == 1
    assert out['board_regions'][0]['height_m'] == pytest.approx(.012, abs=1e-6)
    assert out['candidates'][0]['height_m'] == pytest.approx(.020, abs=1e-6)
    assert out['candidates'][0]['reference_surface'] == 'board'


def test_board_only_is_not_a_chess_piece():
    z, den = scene(); board = np.zeros((H, W), bool); board[90:210, 50:270] = True
    add_height(z, den, board, .012)
    out = detect_depth_candidates(z, K, config())
    assert out['coverage_valid'] and len(out['board_regions']) == 1 and not out['candidates']


def test_boundary_and_touching_objects_remain_unresolved():
    z, den = scene(); m = disk(15, 140); add_height(z, den, m, .020)
    out = detect_depth_candidates(z, K, config())
    assert not out['coverage_valid'] and not out['candidates'][0]['candidate_complete']
    z, den = scene(); m = disk(142, 140) | disk(176, 140); add_height(z, den, m, .020)
    out = detect_depth_candidates(z, K, config(max_piece_span_m=.060))
    assert not out['coverage_valid']
    assert 'raised component may contain touching pieces or unresolved board' in out['reasons']


@pytest.mark.parametrize('kwargs', [dict(min_height_m=float('nan')), dict(max_candidates=301),
                                   dict(min_valid_fraction=0), dict(median_kernel_px=2),
                                   dict(min_object_height_m=-.01), dict(max_height_m=.001)])
def test_invalid_configuration_rejected(kwargs):
    with pytest.raises(ValueError):
        config(**kwargs)


def test_invalid_intrinsics_and_dimensions_never_pass():
    z, _ = scene()
    for k in (np.zeros((3, 3)), np.eye(3)*np.nan):
        assert not detect_depth_candidates(z, k, config())['valid']
    assert not detect_depth_candidates(z, K, config(image_size=[W+1, H]))['valid']


def test_small_hole_in_large_workspace_cannot_be_certified_empty():
    z, _ = scene(); z[140:143, 150:153] = 0
    out = detect_depth_candidates(z, K, config())
    assert out['diagnostics']['valid_depth_fraction'] > .995
    assert not out['coverage_valid']
    assert 'workspace contains unobserved depth; absence cannot be certified' in out['reasons']


def test_full_image_workspace_still_marks_image_edge_as_truncated():
    z, den = scene(); m = disk(0, 140); add_height(z, den, m, .020)
    out = detect_depth_candidates(z, K, config(workspace_polygon_xy=[[0, 0], [W-1, 0], [W-1, H-1], [0, H-1]]))
    assert not out['coverage_valid']
    assert 'raised component touches workspace boundary' in out['reasons']


def test_high_large_plane_cannot_bypass_height_gate_as_board():
    z, den = scene(); board = np.zeros((H, W), bool); board[90:210, 50:270] = True
    add_height(z, den, board, .10)
    out = detect_depth_candidates(z, K, config())
    assert not out['coverage_valid']
    assert 'board exceeds configured height' in out['reasons']


def test_physical_area_rejection_stays_unresolved_without_full_mask():
    z, den = scene(); m = disk(radius=3); add_height(z, den, m, .020)
    out = detect_depth_candidates(z, K, config(min_component_pixels=4, min_candidate_area_m2=.0001))
    assert not out['coverage_valid'] and not out['candidates']
    assert len(out['unresolved_regions']) == 1 and 'mask' not in out['unresolved_regions'][0]


def test_candidate_without_confirmed_single_piece_size_never_passes():
    z, den = scene(); add_height(z, den, disk(), .020)
    out = detect_depth_candidates(z, K, config(single_piece_diameter_m=None))
    assert not out['coverage_valid']
    assert 'single-piece dimensions unconfirmed' in out['reasons']


def test_touching_grid_of_piece_tops_is_not_certified_as_empty_board():
    h, w = 400, 600
    depth = np.full((h,w), .5, np.float32)
    mask = np.zeros((h,w), np.uint8)
    for x in (250,310,370,430):
        for y in (200,260,320):
            cv2.circle(mask, (x,y), 30, 1, -1)
    depth[mask.astype(bool)] = .48
    k = np.array([[600.,0.,300.],[0.,600.,200.],[0.,0.,1.]])
    cfg = config(image_size=[w,h], workspace_polygon_xy=[[5,5],[594,5],[594,394],[5,394]],
                 table_seed_polygon_xy=[[30,20],[550,20],[550,100],[30,100]],
                 single_piece_diameter_m=.05)
    out = detect_depth_candidates(depth,k,cfg)
    assert not out['coverage_valid'] and not out['board_regions']
    assert len(out['candidates']) == 1 and not out['candidates'][0]['candidate_complete']


def test_nominal_dimensions_are_configuration_not_new_truth():
    z, den = scene(); add_height(z, den, disk(), .008)
    out = detect_depth_candidates(z, K, config(min_object_height_m=.008,
                                               expected_piece_thickness_m=.008,
                                               expected_board_thickness_m=.010))
    assert not out['coverage_valid']
    assert out['diagnostics']['expected_piece_thickness_m'] == .008
    assert out['diagnostics']['expected_board_thickness_m'] == .010


def test_rectangular_high_plane_not_board_when_nominal_thickness_disagrees():
    z, den = scene(); board = np.zeros((H, W), bool); board[90:210, 50:270] = True
    add_height(z, den, board, .035)
    out = detect_depth_candidates(z, K, config(expected_board_thickness_m=.010))
    assert not out['coverage_valid'] and not out['board_regions']
    assert 'broad raised plane inconsistent with nominal board thickness' in out['reasons']


def test_candidate_and_unresolved_summary_budgets_are_enforced_before_accumulation():
    z, den = scene()
    add_height(z, den, disk(80,140) | disk(160,140) | disk(240,140), .020)
    out = detect_depth_candidates(z, K, config(max_candidates=1))
    assert len(out['candidates']) == 1 and not out['coverage_valid']
    assert out['diagnostics']['candidate_budget_overflow_count'] == 2
    assert out['diagnostics']['candidate_count_before_limit'] == 3
    assert len(out['unresolved_regions']) <= 1
    assert 'depth candidate limit exceeded' in out['reasons']


def test_many_components_fail_before_per_region_masks_are_allocated():
    z, den = scene()
    add_height(z, den, disk(80,140) | disk(160,140) | disk(240,140), .020)
    out = detect_depth_candidates(z, K, config(max_components=2))
    assert not out['coverage_valid'] and not out['candidates']
    assert out['diagnostics']['unprocessed_raised_component_count'] == 3
