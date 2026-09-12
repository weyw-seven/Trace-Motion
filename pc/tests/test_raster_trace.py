import numpy as np
import pytest

from pc_trajectory.raster import (
    NodeKind,
    PixelPoint,
    RasterError,
    TraceConfig,
    extract_pixel_paths,
    suppress_small_parallel_loops,
    trace_skeleton,
)
from pc_trajectory.raster.trace_preview import save_trace_preview


def horizontal_line():
    mask = np.zeros((7, 15), dtype=bool)
    mask[3, 2:13] = True
    return mask


def cross():
    mask = np.zeros((11, 11), dtype=bool)
    mask[5, 2:9] = True
    mask[2:9, 5] = True
    return mask


def ring():
    mask = np.zeros((9, 9), dtype=bool)
    mask[1, 1:8] = mask[7, 1:8] = True
    mask[1:8, 1] = mask[1:8, 7] = True
    return mask


def acute_cap_loop():
    """Two routes between degree-3 nodes plus one external arm at each end."""
    mask = np.zeros((20, 20), dtype=bool)
    outer = [
        (6, 10), (5, 10), (4, 10), (3, 10), (3, 9), (3, 8),
        (3, 7), (3, 6), (4, 6), (5, 6), (6, 6), (7, 6), (8, 6),
    ]
    inner = [(6, 10), (7, 9), (8, 8), (8, 7), (8, 6)]
    for y, x in outer + inner:
        mask[y, x] = True
    mask[6, 10:17] = True
    mask[8:17, 6] = True
    return mask


def dilate_for_source(mask, radius=2):
    source = np.zeros_like(mask)
    for y, x in zip(*np.nonzero(mask)):
        source[max(0, y - radius):y + radius + 1, max(0, x - radius):x + radius + 1] = True
    return source


def test_open_line_is_one_edge_complete_path():
    result = trace_skeleton(horizontal_line())
    assert result.path_count == 1
    path = result.paths[0]
    assert not path.closed
    assert path.points[0] == PixelPoint(2, 3)
    assert path.points[-1] == PixelPoint(12, 3)
    assert result.raw_edge_count == result.external_edge_count == result.traced_edge_count == 10
    assert result.untraced_edge_count == 0
    assert result.diagnostics == ()


def test_cross_splits_at_one_junction_without_duplicate_edges():
    result = trace_skeleton(cross())
    assert result.path_count == 4
    assert sum(node.kind is NodeKind.JUNCTION for node in result.nodes) == 1
    junction = next(node for node in result.nodes if node.kind is NodeKind.JUNCTION)
    assert junction.representative == PixelPoint(5, 5)
    assert junction.degree == 4
    assert all(not path.closed for path in result.paths)
    assert result.external_edge_count == result.traced_edge_count
    assert result.untraced_edge_count == 0


def test_closed_ring_is_one_canonical_loop():
    result = trace_skeleton(ring())
    assert result.path_count == 1
    path = result.paths[0]
    assert path.closed
    assert path.points[0] == path.points[-1] == PixelPoint(1, 1)
    assert result.nodes == ()
    assert result.external_edge_count == result.traced_edge_count
    assert result.diagnostics == ()


def test_two_components_and_component_indices_are_preserved():
    mask = np.zeros((8, 15), bool)
    mask[1, 1:4] = True
    mask[6, 8:12] = True
    result = trace_skeleton(mask)
    assert result.path_count == 2
    assert {path.component_index for path in result.paths} == {0, 1}
    assert result.component_count == 2
    assert result.external_edge_count == result.traced_edge_count == 5


def test_diagonal_chain_is_connected_without_orthogonal_shortcuts():
    mask = np.zeros((7, 7), bool)
    for index in range(1, 6):
        mask[index, index] = True
    result = trace_skeleton(mask)
    assert result.path_count == 1
    assert result.paths[0].points == tuple(PixelPoint(i, i) for i in range(1, 6))


def test_isolated_pixel_is_diagnosed_and_not_silently_drawn():
    mask = np.zeros((5, 7), bool)
    mask[1, 1] = True
    mask[3, 2:5] = True
    result = trace_skeleton(mask)
    assert result.path_count == 1
    assert any(diagnostic.code == "ISOLATED_SKELETON_PIXEL" for diagnostic in result.diagnostics)


def test_closed_loop_direction_and_order_are_deterministic():
    mask = ring()
    first = trace_skeleton(mask)
    second = trace_skeleton(mask.copy())
    assert first.paths == second.paths
    assert first.nodes == second.nodes


def test_extract_convenience_combines_skeleton_and_trace_diagnostics():
    skeleton, trace = extract_pixel_paths(np.zeros((4, 4), bool))
    assert skeleton.skeleton_pixel_count == 0
    assert trace.path_count == 0
    assert {diagnostic.code for diagnostic in trace.diagnostics} >= {"EMPTY_SKELETON"}


def test_small_parallel_cap_loop_is_removed_and_false_junctions_collapse():
    skeleton = acute_cap_loop()
    raw = trace_skeleton(skeleton)
    assert sum(node.kind is NodeKind.JUNCTION for node in raw.nodes) == 2
    assert raw.path_count == 4

    cleaned = suppress_small_parallel_loops(dilate_for_source(skeleton), raw)

    assert cleaned.path_count == 1
    assert all(node.kind is NodeKind.ENDPOINT for node in cleaned.nodes)
    assert cleaned.external_edge_count == cleaned.traced_edge_count
    assert any(
        diagnostic.code == "SMALL_PARALLEL_LOOP_SUPPRESSED"
        for diagnostic in cleaned.diagnostics
    )


def test_parallel_loop_filter_can_be_disabled():
    skeleton = acute_cap_loop()
    raw = trace_skeleton(skeleton)
    config = TraceConfig(suppress_small_parallel_loops=False)
    assert suppress_small_parallel_loops(dilate_for_source(skeleton), raw, config) is raw


def test_standalone_small_ring_is_never_suppressed():
    skeleton = ring()
    raw = trace_skeleton(skeleton)
    cleaned = suppress_small_parallel_loops(dilate_for_source(skeleton, radius=3), raw)
    assert cleaned is raw
    assert cleaned.path_count == 1
    assert cleaned.paths[0].closed


def test_preview_is_nonempty_and_does_not_mutate_result(tmp_path):
    result = trace_skeleton(cross())
    before = result.skeleton_mask.copy()
    output = save_trace_preview(result, tmp_path / "trace.png", scale=5)
    assert output.exists()
    assert output.stat().st_size > 0
    np.testing.assert_array_equal(result.skeleton_mask, before)


def test_preview_auto_scale_caps_large_image(tmp_path):
    mask = np.zeros((950, 1302), dtype=bool)
    mask[475, 10:1290] = True
    result = trace_skeleton(mask)
    output = save_trace_preview(result, tmp_path / "trace.png")
    from PIL import Image
    with Image.open(output) as image:
        assert max(image.size) <= 1600


@pytest.mark.parametrize("kwargs", [
    {"allow_diagonal": 1},
    {"allow_diagonal": None},
    {"suppress_small_parallel_loops": 1},
    {"artifact_loop_perimeter_width_factor": 0},
    {"artifact_loop_perimeter_width_factor": float("nan")},
])
def test_invalid_trace_config(kwargs):
    with pytest.raises(RasterError):
        TraceConfig(**kwargs)


@pytest.mark.parametrize("value", [0, -1, 1.5, True])
def test_invalid_preview_scale(value, tmp_path):
    result = trace_skeleton(horizontal_line())
    with pytest.raises(RasterError):
        save_trace_preview(result, tmp_path / "trace.png", scale=value)
