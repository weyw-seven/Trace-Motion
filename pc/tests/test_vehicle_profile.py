from pc_trajectory.demo.map_model import MapDocument, MapObstacle
from pc_trajectory.demo.map_planner import NavigationPlanner, PlannerConfig, VehicleProfile
from pc_trajectory.demo.protocol import Pose
from pc_trajectory.geometry import Point2D
from pc_trajectory.toolpath import Motion


def test_vehicle_profile_reports_conservative_clearance_radius():
    profile = VehicleProfile(
        enabled=True,
        front_mm=30.0,
        rear_mm=20.0,
        left_mm=16.0,
        right_mm=12.0,
        safety_margin_mm=3.0,
        localization_margin_mm=2.0,
    )
    assert profile.effective_radius_mm == (30.0**2 + 16.0**2) ** 0.5 + 5.0
    assert profile.effective_diameter_mm == 2.0 * profile.effective_radius_mm


def test_disabled_vehicle_profile_keeps_existing_point_robot_route():
    obstacle = MapObstacle(
        (Point2D(40.0, -20.0), Point2D(60.0, -20.0), Point2D(60.0, 20.0), Point2D(40.0, 20.0)),
        obstacle_id="wall",
    )
    document = MapDocument.new().with_obstacle(obstacle)
    plan = NavigationPlanner(PlannerConfig(vehicle_profile=VehicleProfile(enabled=False))).plan_click_to_go(
        document, Pose(0.0, 0.0, 0.0), Point2D(100.0, 0.0)
    )
    assert plan.end_point == Point2D(100.0, 0.0)
    assert plan.toolpath.motion_count >= 3


def test_enabled_vehicle_profile_inflates_obstacle_for_detour():
    obstacle = MapObstacle(
        (Point2D(200.0, -20.0), Point2D(220.0, -20.0), Point2D(220.0, 20.0), Point2D(200.0, 20.0)),
        obstacle_id="wall",
    )
    document = MapDocument.new().with_obstacle(obstacle)
    config = PlannerConfig(
        vehicle_profile=VehicleProfile(
            enabled=True,
            front_mm=20.0,
            rear_mm=20.0,
            left_mm=15.0,
            right_mm=15.0,
            safety_margin_mm=2.0,
        )
    )
    plan = NavigationPlanner(config).plan_click_to_go(document, Pose(0.0, 0.0, 0.0), Point2D(400.0, 0.0))
    motions = [record for record in plan.toolpath.records if isinstance(record, Motion)]
    assert len(motions) >= 3
    assert any(abs(record.geometry.end_point().y_mm) >= 46.0 for record in motions[:-1])

