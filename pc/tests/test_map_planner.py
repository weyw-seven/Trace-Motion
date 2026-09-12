import math

import pytest

from pc_trajectory.demo.map_model import (
    PATH_KIND_FREEHAND,
    PATH_KIND_POLYLINE,
    MapDocument,
    MapHome,
    MapObstacle,
    MapPath,
)
from pc_trajectory.demo.map_planner import (
    NavigationMode,
    NavigationPlanner,
    NavigationPlanningError,
    PathDirection,
    PenMode,
    PlannerConfig,
    document_revision,
)
from pc_trajectory.demo.protocol import Pose
from pc_trajectory.geometry import Arc, Line, Point2D
from pc_trajectory.toolpath import Motion, PenDown, PenUp
from pc_trajectory.traj2_reader import decode_trj2


def demo_document() -> MapDocument:
    return MapDocument(
        name="planner-demo",
        width_mm=1000.0,
        height_mm=700.0,
        home=MapHome(100.0, 50.0, 12.0),
        paths=(
            MapPath(
                (Point2D(100.0, 50.0), Point2D(300.0, 50.0), Point2D(500.0, 200.0)),
                path_id="route",
                name="Route",
            ),
        ),
    )


def test_click_to_go_translates_map_target_and_exports_current_pose_header():
    document = demo_document()
    planner = NavigationPlanner()
    plan = planner.plan_click_to_go(document, Pose(10.0, 20.0, 7.0), Point2D(500.0, 200.0))
    assert plan.mode is NavigationMode.CLICK_TO_GO
    assert plan.target_world == Point2D(400.0, 150.0)
    assert isinstance(plan.toolpath.records[0], PenUp)
    motion = plan.toolpath.records[1]
    assert isinstance(motion, Motion)
    assert motion.geometry.start == Point2D(10.0, 20.0)
    assert motion.geometry.end == Point2D(400.0, 150.0)
    decoded = decode_trj2(plan.trj2_bytes())
    assert decoded.header.start_x_mm == 10.0
    assert decoded.header.start_y_mm == 20.0
    assert decoded.header.start_yaw_deg == 7.0


def test_saved_ui_settings_do_not_change_geometry_revision():
    document = demo_document()
    with_settings = document._replace(
        settings={"schema": 1, "navigation": {"mode": "path"}}
    )
    assert document_revision(with_settings) == document_revision(document)


def test_selected_path_draw_plan_adds_pen_events_and_travel():
    document = demo_document()
    plan = NavigationPlanner().plan_selected_path(
        document,
        Pose(-100.0, 0.0, 0.0),
        "route",
        pen_mode=PenMode.DRAW,
        direction=PathDirection.FORWARD,
    )
    assert plan.path_direction is PathDirection.FORWARD
    assert isinstance(plan.toolpath.records[0], PenUp)
    assert isinstance(plan.toolpath.records[1], Motion)  # approach travel
    assert isinstance(plan.toolpath.records[2], PenDown)
    assert isinstance(plan.toolpath.records[-1], PenUp)
    assert plan.toolpath.drawing_length_mm() > 0.0
    assert plan.toolpath.travel_length_mm() > 0.0
    assert all(isinstance(record, Motion) for record in plan.toolpath.records if isinstance(record, Motion))


def test_polyline_plan_keeps_user_vertices_as_line_junctions():
    document = MapDocument(
        name="exact",
        width_mm=500.0,
        height_mm=400.0,
        paths=(
            MapPath(
                (Point2D(0.0, 0.0), Point2D(100.0, 100.0), Point2D(200.0, 0.0)),
                path_id="exact",
                path_kind=PATH_KIND_POLYLINE,
            ),
        ),
    )
    plan = NavigationPlanner().plan_selected_path(
        document,
        Pose(0.0, 0.0, 0.0),
        "exact",
        pen_mode=PenMode.DRAW,
        direction=PathDirection.FORWARD,
    )
    motions = [record for record in plan.toolpath.records if isinstance(record, Motion)]
    assert [motion.geometry.start_point() for motion in motions] == [
        Point2D(0.0, 0.0),
        Point2D(100.0, 100.0),
    ]
    assert [motion.geometry.end_point() for motion in motions] == [
        Point2D(100.0, 100.0),
        Point2D(200.0, 0.0),
    ]


def test_freehand_line_fallback_translates_map_points_to_world():
    document = MapDocument(
        name="translated-freehand",
        width_mm=500.0,
        height_mm=900.0,
        home=MapHome(20.0, 800.0),
        paths=(
            MapPath(
                (Point2D(20.0, 800.0), Point2D(120.0, 800.0), Point2D(220.0, 700.0)),
                path_id="freehand",
                path_kind=PATH_KIND_FREEHAND,
            ),
        ),
    )
    plan = NavigationPlanner(
        PlannerConfig(
            enable_arc_fitting=False,
            smoothing_iterations=0,
            simplify_tolerance_mm=0.01,
        )
    ).plan_selected_path(
        document,
        Pose(0.0, 0.0, 0.0),
        "freehand",
        pen_mode=PenMode.DRAW,
        direction=PathDirection.FORWARD,
    )

    motions = [record for record in plan.toolpath.records if isinstance(record, Motion)]
    assert [motion.geometry.start_point() for motion in motions] == [
        Point2D(0.0, 0.0),
        Point2D(100.0, 0.0),
    ]
    assert [motion.geometry.end_point() for motion in motions] == [
        Point2D(100.0, 0.0),
        Point2D(200.0, -100.0),
    ]


def test_auto_direction_chooses_nearest_endpoint_and_reverse_preserves_geometry():
    document = demo_document()
    plan = NavigationPlanner().plan_selected_path(
        document,
        Pose(450.0, 160.0, 0.0),
        "route",
        direction=PathDirection.AUTO,
    )
    assert plan.path_direction is PathDirection.REVERSE
    motions = [record for record in plan.toolpath.records if isinstance(record, Motion)]
    assert motions[-1].geometry.end == Point2D(0.0, 0.0)
    assert motions[1].geometry.start.distance_to(Point2D(400.0, 150.0)) < 1.0e-6


def test_return_home_always_targets_world_origin():
    document = demo_document()
    plan = NavigationPlanner().plan_return_home(document, Pose(40.0, -20.0, 35.0))
    assert plan.mode is NavigationMode.RETURN_HOME
    assert plan.target_world == Point2D(0.0, 0.0)
    assert plan.target_yaw_deg == 12.0
    assert plan.end_point == Point2D(0.0, 0.0)


def test_click_to_go_routes_around_polygon_obstacle():
    obstacle = MapObstacle(
        (Point2D(40.0, -20.0), Point2D(60.0, -20.0), Point2D(60.0, 20.0), Point2D(40.0, 20.0)),
        obstacle_id="wall",
    )
    document = MapDocument.new().with_obstacle(obstacle)
    plan = NavigationPlanner().plan_click_to_go(
        document,
        Pose(0.0, 0.0, 0.0),
        Point2D(100.0, 0.0),
    )
    motions = [record for record in plan.toolpath.records if isinstance(record, Motion)]
    assert len(motions) >= 3
    assert motions[0].geometry.start_point() == Point2D(0.0, 0.0)
    assert motions[-1].geometry.end_point() == Point2D(100.0, 0.0)
    assert any(abs(motion.geometry.end_point().y_mm) >= 20.0 for motion in motions[:-1])


def test_click_to_go_moves_target_inside_obstacle_to_nearest_safe_point():
    obstacle = MapObstacle(
        (Point2D(40.0, -20.0), Point2D(60.0, -20.0), Point2D(60.0, 20.0), Point2D(40.0, 20.0)),
        obstacle_id="wall",
    )
    document = MapDocument.new().with_obstacle(obstacle)
    plan = NavigationPlanner().plan_click_to_go(document, Pose(0.0, 0.0, 0.0), Point2D(50.0, 0.0))
    assert plan.requested_target_world == Point2D(50.0, 0.0)
    assert plan.target_world == Point2D(62.0, 0.0)
    assert plan.end_point == Point2D(62.0, 0.0)


def test_selected_path_that_crosses_obstacle_is_split_into_pen_up_detour():
    obstacle = MapObstacle(
        (Point2D(40.0, -20.0), Point2D(60.0, -20.0), Point2D(60.0, 20.0), Point2D(40.0, 20.0)),
        obstacle_id="wall",
    )
    path = MapPath(
        (Point2D(0.0, 0.0), Point2D(100.0, 0.0)),
        path_id="through-wall",
        path_kind=PATH_KIND_POLYLINE,
    )
    document = MapDocument.new().with_obstacle(obstacle).with_path(path)
    plan = NavigationPlanner().plan_selected_path(
        document,
        Pose(0.0, 0.0, 0.0),
        "through-wall",
        pen_mode=PenMode.DRAW,
        direction=PathDirection.FORWARD,
    )
    assert plan.route_legs[0].detour_count >= 1
    assert any(isinstance(record, PenDown) for record in plan.toolpath.records)
    assert plan.toolpath.travel_length_mm() > 0.0
    states = []
    for record in plan.toolpath.records:
        if isinstance(record, PenDown):
            states.append("down")
        elif isinstance(record, PenUp):
            states.append("up")
    assert states.count("down") >= 2


def test_obstacle_fallback_preserves_clear_polyline_segments_without_dense_output():
    path = MapPath(
        (
            Point2D(0.0, 0.0),
            Point2D(1000.0, 0.0),
            Point2D(1000.0, 1000.0),
            Point2D(2000.0, 1000.0),
        ),
        path_id="sparse-route",
        path_kind=PATH_KIND_POLYLINE,
    )
    obstacle = MapObstacle(
        (
            Point2D(900.0, 400.0),
            Point2D(1100.0, 400.0),
            Point2D(1100.0, 600.0),
            Point2D(900.0, 600.0),
        ),
        obstacle_id="middle-wall",
    )
    document = MapDocument.new().with_path(path).with_obstacle(obstacle)

    plan = NavigationPlanner().plan_selected_path(
        document,
        Pose(0.0, 0.0, 0.0),
        "sparse-route",
        pen_mode=PenMode.DRAW,
        direction=PathDirection.FORWARD,
    )

    motions = [record for record in plan.toolpath.records if isinstance(record, Motion)]
    assert len(motions) == 7
    assert motions[0].geometry == Line(Point2D(0.0, 0.0), Point2D(1000.0, 0.0))
    assert motions[-1].geometry == Line(Point2D(1000.0, 1000.0), Point2D(2000.0, 1000.0))
    assert plan.route_legs[0].detour_count >= 1
    pen_down = False
    for record in plan.toolpath.records:
        if isinstance(record, PenDown):
            pen_down = True
        elif isinstance(record, PenUp):
            pen_down = False
        elif isinstance(record, Motion):
            expected_speed = PlannerConfig().draw_speed_mm_s if pen_down else PlannerConfig().travel_speed_mm_s
            assert record.speed_mm_s == expected_speed


def test_selected_path_endpoint_inside_obstacle_uses_effective_endpoint():
    obstacle = MapObstacle(
        (Point2D(40.0, -20.0), Point2D(60.0, -20.0), Point2D(60.0, 20.0), Point2D(40.0, 20.0)),
        obstacle_id="wall",
    )
    path = MapPath((Point2D(0.0, 0.0), Point2D(50.0, 0.0)), path_id="ends-inside", path_kind=PATH_KIND_POLYLINE)
    document = MapDocument.new().with_obstacle(obstacle).with_path(path)
    plan = NavigationPlanner().plan_selected_path(document, Pose(0.0, 0.0, 0.0), "ends-inside", pen_mode=PenMode.DRAW, direction=PathDirection.FORWARD)
    leg = plan.route_legs[0]
    assert leg.requested_end == Point2D(50.0, 0.0)
    assert leg.effective_end == Point2D(62.0, 0.0)
    assert plan.target_world == Point2D(62.0, 0.0)


def test_path_sequence_preserves_click_order_and_compiles_one_job():
    document = MapDocument.new()
    document = document.with_path(MapPath((Point2D(0.0, 0.0), Point2D(40.0, 0.0)), path_id="a"))
    document = document.with_path(MapPath((Point2D(40.0, 0.0), Point2D(40.0, 40.0)), path_id="b"))
    document = document.with_path(MapPath((Point2D(40.0, 40.0), Point2D(0.0, 40.0)), path_id="c"))
    plan = NavigationPlanner().plan_path_sequence(
        document,
        Pose(0.0, 0.0, 0.0),
        ("b", "c", "a"),
        pen_mode=PenMode.DRAW,
        direction=PathDirection.FORWARD,
    )
    assert plan.path_ids == ("b", "c", "a")
    assert [leg.path_id for leg in plan.route_legs] == ["b", "c", "a"]
    assert plan.path_directions == (PathDirection.FORWARD,) * 3
    assert plan.toolpath.motion_count >= 3


def test_plan_detects_map_mutation_and_unknown_path():
    document = demo_document()
    planner = NavigationPlanner(PlannerConfig(travel_speed_mm_s=80.0))
    plan = planner.plan_click_to_go(document, Pose(0.0, 0.0, 0.0), Point2D(200.0, 100.0))
    assert document_revision(document) == plan.document_revision
    changed = document.with_home(MapHome(101.0, 50.0))
    with pytest.raises(NavigationPlanningError, match="stale"):
        plan.ensure_current(changed)
    with pytest.raises(NavigationPlanningError, match="unknown path"):
        planner.plan_selected_path(document, Pose(0.0, 0.0, 0.0), "missing")


def test_zero_length_click_is_event_only_but_still_exports_start_point():
    document = demo_document()
    plan = NavigationPlanner().plan_click_to_go(document, Pose(0.0, 0.0, 0.0), Point2D(100.0, 50.0))
    assert plan.toolpath.motion_count == 0
    decoded = decode_trj2(plan.trj2_bytes())
    assert decoded.header.start_x_mm == 0.0
    assert decoded.records  # PEN_UP is retained in the canonical file
