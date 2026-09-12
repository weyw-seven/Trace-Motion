import numpy as np

from pc_trajectory.raster.coordinates import PixelPoint
from pc_trajectory.raster.stroke_planner import plan_strokes
from pc_trajectory.raster.trace import PixelPath, TraceNode, TraceResult, NodeKind, trace_skeleton


def test_three_parallel_edges_between_two_junctions_become_one_stroke():
    paths = tuple(
        PixelPath(
            (PixelPoint(0, 1), PixelPoint(4, index), PixelPoint(8, 1)),
            False,
            0,
            0,
            1,
        )
        for index in (0, 1, 2)
    )
    trace = TraceResult(
        skeleton_mask=np.ones((3, 9), dtype=bool),
        paths=paths,
        nodes=(
            TraceNode(0, NodeKind.JUNCTION, PixelPoint(0, 1), (PixelPoint(0, 1),), 3, 0),
            TraceNode(1, NodeKind.JUNCTION, PixelPoint(8, 1), (PixelPoint(8, 1),), 3, 0),
        ),
        component_count=1,
        raw_edge_count=3,
        contracted_edge_count=0,
        external_edge_count=3,
        traced_edge_count=3,
        diagnostics=(),
    )
    plan = plan_strokes(trace)

    assert trace.path_count == 3
    assert plan.stroke_count == 1
    assert plan.strokes[0].path_count == 3
    assert len(plan.strokes[0].points) > 3


def test_t_graph_needs_two_trails_and_consumes_every_path():
    mask = np.zeros((7, 7), dtype=bool)
    mask[1, 1:6] = True
    mask[1:6, 3] = True
    trace = trace_skeleton(mask)
    plan = plan_strokes(trace)

    assert plan.stroke_count == 2
    assert sum(stroke.path_count for stroke in plan.strokes) == trace.path_count


def test_closed_loop_remains_one_closed_stroke():
    mask = np.ones((5, 5), dtype=bool)
    mask[1:4, 1:4] = False
    trace = trace_skeleton(mask)
    plan = plan_strokes(trace)

    assert plan.stroke_count == 1
    assert plan.strokes[0].closed
    assert plan.strokes[0].points[0] == plan.strokes[0].points[-1]
