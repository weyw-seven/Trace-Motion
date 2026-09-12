import numpy as np
import pytest

from pc_trajectory.raster import RasterError, SkeletonConfig, skeletonize_mask


def thick_line_mask(height=21, width=31):
    mask = np.zeros((height, width), dtype=bool)
    mask[9:13, 3:28] = True
    return mask


def test_thinning_is_convergent_read_only_and_one_pixel_wide():
    mask = thick_line_mask()
    before = mask.copy()
    result = skeletonize_mask(mask)
    assert result.converged
    assert result.iterations >= 1
    assert result.skeleton_pixel_count < result.input_pixel_count
    assert np.array_equal(mask, before)
    assert result.skeleton_mask.flags.writeable is False
    assert np.all(result.skeleton_mask.sum(axis=0) <= 1)
    # The centerline of a four-pixel-wide cap has an inherently ambiguous end;
    # thinning keeps the topology and produces the medial 21-pixel span.
    assert result.skeleton_mask.sum() == 21
    assert result.diagnostics == ()


def test_zhang_suen_preserves_simple_topologies():
    line = np.zeros((7, 13), bool)
    line[3, 2:11] = True
    result = skeletonize_mask(line)
    np.testing.assert_array_equal(result.skeleton_mask, line)

    ring = np.zeros((9, 9), bool)
    ring[1, 1:8] = ring[7, 1:8] = True
    ring[1:8, 1] = ring[1:8, 7] = True
    ring_result = skeletonize_mask(ring)
    assert ring_result.skeleton_pixel_count == ring.sum()


def test_thick_intersection_remains_one_connected_component():
    """Regression: wrong diagonal terms in Zhang-Suen split this trunk."""
    mask = np.zeros((80, 100), dtype=bool)
    mask[8:15, 8:91] = True
    mask[65:73, 8:91] = True
    mask[8:73, 8:15] = True
    mask[8:73, 47:54] = True
    mask[8:73, 84:91] = True

    result = skeletonize_mask(mask)

    assert result.converged
    assert not any(
        diagnostic.code == "SKELETON_COMPONENT_COUNT_CHANGED"
        for diagnostic in result.diagnostics
    )

    # Independent 8-neighbor flood fill confirms there is one skeleton component.
    pixels = set(zip(*np.nonzero(result.skeleton_mask)))
    seen = {min(pixels)}
    stack = list(seen)
    while stack:
        y, x = stack.pop()
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                neighbor = (y + dy, x + dx)
                if neighbor in pixels and neighbor not in seen:
                    seen.add(neighbor)
                    stack.append(neighbor)
    assert seen == pixels


def test_empty_and_single_pixel_diagnostics():
    empty = skeletonize_mask(np.zeros((4, 5), dtype=bool))
    assert empty.converged
    assert {diagnostic.code for diagnostic in empty.diagnostics} == {"EMPTY_SKELETON"}

    single = skeletonize_mask(np.array([[True]], dtype=bool))
    assert {diagnostic.code for diagnostic in single.diagnostics} == {"SINGLE_PIXEL_SKELETON"}


def test_non_boolean_numeric_input_is_supported():
    result = skeletonize_mask(np.array([[0, 2, 0], [0, 2, 0], [0, 2, 0]], dtype=np.uint8))
    assert result.skeleton_pixel_count == 3


@pytest.mark.parametrize("mask", [np.array([]), np.zeros((3, 3, 1)), np.array([["x"]])])
def test_invalid_masks(mask):
    with pytest.raises(RasterError):
        skeletonize_mask(mask)


def test_iteration_limit_is_reported():
    result = skeletonize_mask(thick_line_mask(), SkeletonConfig(max_iterations=1))
    assert not result.converged
    assert any(diagnostic.code == "SKELETON_MAX_ITERATIONS" for diagnostic in result.diagnostics)


@pytest.mark.parametrize("value", [0, -1, 1.0, True])
def test_invalid_skeleton_config(value):
    with pytest.raises(RasterError):
        SkeletonConfig(max_iterations=value)
