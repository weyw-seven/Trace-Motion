from __future__ import annotations

import pytest

from pc_trajectory.demo.toolpath_review import review_trj2
from pc_trajectory.demo.map_model import MapDocument, MapHome, MapPath
from pc_trajectory.demo.map_planner import NavigationPlanner, PenMode, PlannerConfig
from pc_trajectory.demo.protocol import Pose
from pc_trajectory.geometry import Arc, Line, Point2D
from pc_trajectory.toolpath import Motion, PenDown, PenUp, Toolpath
from pc_trajectory.toolpath_trj2_export import encode_toolpath_trj2


def test_review_counts_exact_trj2_records_and_arc_lengths() -> None:
    toolpath = Toolpath(
        (
            PenUp(),
            Motion(Line(Point2D(0.0, 0.0), Point2D(10.0, 0.0)), 50.0),
            PenDown(),
            Motion(Arc(Point2D(10.0, 10.0), 10.0, -90.0, 90.0), 30.0),
            PenUp(),
        )
    )
    review = review_trj2(encode_toolpath_trj2(toolpath))
    assert review.record_count == 5
    assert review.line_count == 1
    assert review.circle_count == 1
    assert review.pen_up_count == 2
    assert review.pen_down_count == 1
    assert review.min_circle_length_mm == pytest.approx(10.0 * 3.141592653589793 / 2.0)


def test_review_warns_for_dense_and_short_circles() -> None:
    toolpath = Toolpath((Motion(Arc(Point2D(0.0, 10.0), 10.0, 0.0, 10.0), 30.0),))
    review = review_trj2(encode_toolpath_trj2(toolpath))
    warnings = review.warning_messages(max_circles=0, min_circle_length_mm=5.0)
    assert warnings
    assert "shorter" in warnings[0]


def test_disabling_arc_fitting_falls_back_to_line_records() -> None:
    document = MapDocument(
        name="line fallback",
        width_mm=400.0,
        height_mm=400.0,
        home=MapHome(),
        paths=(
            MapPath(
                tuple(Point2D(40.0 + index * 8.0, 80.0 + (index % 3) * 5.0) for index in range(12)),
                path_id="freehand",
            ),
        ),
    )
    plan = NavigationPlanner(PlannerConfig(enable_arc_fitting=False)).plan_selected_path(
        document,
        Pose(0.0, 0.0, 0.0),
        "freehand",
        pen_mode=PenMode.DRAW,
    )
    review = review_trj2(plan.trj2_bytes())
    assert review.circle_count == 0
    assert review.line_count > 0
