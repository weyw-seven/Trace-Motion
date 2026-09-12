"""Generate a visual acceptance bundle for the vehicle footprint planner."""

from __future__ import annotations

import json
from pathlib import Path

from pc_trajectory.demo.map_model import MapDocument, MapHome, MapObstacle
from pc_trajectory.demo.map_planner import NavigationPlanner, PlannerConfig, VehicleProfile, _world_obstacles
from pc_trajectory.demo.protocol import Pose
from pc_trajectory.geometry import Point2D
from pc_trajectory.toolpath import Motion


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "test_results" / "virtual_map_demo" / "vehicle_clearance"


def main() -> None:
    import matplotlib.pyplot as plt

    OUTPUT.mkdir(parents=True, exist_ok=True)
    document = MapDocument(
        name="Vehicle clearance acceptance",
        width_mm=700.0,
        height_mm=500.0,
        home=MapHome(350.0, 250.0, 0.0),
        obstacles=(
            MapObstacle(
                (Point2D(530.0, 195.0), Point2D(610.0, 195.0), Point2D(610.0, 305.0), Point2D(530.0, 305.0)),
                obstacle_id="wall",
                name="Raw obstacle",
            ),
        ),
    )
    profile = VehicleProfile(
        enabled=True,
        front_mm=35.0,
        rear_mm=30.0,
        left_mm=22.0,
        right_mm=22.0,
        safety_margin_mm=5.0,
        localization_margin_mm=3.0,
    )
    config = PlannerConfig(vehicle_profile=profile)
    start = Pose(40.0, 0.0, 0.0)
    plan = NavigationPlanner(config).plan_click_to_go(document, start, Point2D(750.0, 250.0))
    raw = document.obstacles[0].to_world(document.home)
    inflated = _world_obstacles(document, config)[0]

    fig, ax = plt.subplots(figsize=(10, 6), constrained_layout=True)
    ax.set_title("Vehicle footprint clearance · raw vs inflated obstacle")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("WORLD X (mm), +X forward")
    ax.set_ylabel("WORLD Y (mm), +Y left")
    ax.fill([p.x_mm for p in raw], [p.y_mm for p in raw], color="#fca5a5", alpha=0.55, edgecolor="#dc2626", linewidth=2, label="raw obstacle")
    ax.fill([p.x_mm for p in inflated], [p.y_mm for p in inflated], facecolor="none", edgecolor="#7c3aed", linestyle="--", linewidth=2, label="vehicle-inflated obstacle")
    ax.scatter([start.x_mm], [start.y_mm], color="#0f766e", s=60, label="start")
    for record in plan.toolpath.records:
        if not isinstance(record, Motion):
            continue
        ax.plot(
            [record.geometry.start_point().x_mm, record.geometry.end_point().x_mm],
            [record.geometry.start_point().y_mm, record.geometry.end_point().y_mm],
            color="#f59e0b", linestyle="--", linewidth=2.4,
        )
    ax.scatter([plan.end_point.x_mm], [plan.end_point.y_mm], color="#2563eb", s=60, label="target")
    ax.text(10, -145, f"Effective clearance radius = {profile.effective_radius_mm:.1f} mm", color="#4c1d95")
    ax.legend(loc="best")
    dashboard = OUTPUT / "vehicle_clearance_dashboard.png"
    fig.savefig(dashboard, dpi=160, bbox_inches="tight")
    plt.close(fig)

    report = {
        "effective_radius_mm": profile.effective_radius_mm,
        "effective_diameter_mm": profile.effective_diameter_mm,
        "motion_count": plan.toolpath.motion_count,
        "target_world": {"x_mm": plan.end_point.x_mm, "y_mm": plan.end_point.y_mm},
        "dashboard": dashboard.name,
    }
    (OUTPUT / "acceptance.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Generated {OUTPUT}")


if __name__ == "__main__":
    main()
