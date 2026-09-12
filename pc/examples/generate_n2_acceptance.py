"""Generate N2 navigation planning/simulation acceptance artifacts."""

from __future__ import annotations

import json
from pathlib import Path

from pc_trajectory.demo.map_model import (
    PATH_KIND_POLYLINE,
    MapDocument,
    MapHome,
    MapLandmark,
    MapObstacle,
    MapPath,
)
from pc_trajectory.demo.map_planner import NavigationPlanner, PenMode
from pc_trajectory.demo.navigation_job import NavigationJob
from pc_trajectory.demo.navigation_simulator import SimulatedNavigationRunner
from pc_trajectory.demo.protocol import Pose
from pc_trajectory.geometry import Point2D
from pc_trajectory.preview import sample_geometry, save_toolpath_preview
from pc_trajectory.toolpath import Motion, PenDown, PenState, PenUp
from pc_trajectory.toolpath_trj2_export import write_toolpath_trj2


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "test_results" / "virtual_map_demo" / "n2_simulation"


def build_document() -> MapDocument:
    return MapDocument(
        name="N2 virtual navigation acceptance",
        width_mm=1000.0,
        height_mm=700.0,
        home=MapHome(200.0, 150.0, 0.0, 7),
        paths=(
            MapPath(
                (
                    Point2D(200.0, 150.0),
                    Point2D(350.0, 150.0),
                    Point2D(500.0, 230.0),
                    Point2D(650.0, 250.0),
                ),
                path_id="demo-route",
                name="Demo route",
                path_kind=PATH_KIND_POLYLINE,
            ),
        ),
        landmarks=(
            MapLandmark("station", "Station", 760.0, 500.0),
            MapLandmark("dock", "Dock", 650.0, 250.0),
        ),
        obstacles=(
            MapObstacle(
                (
                    Point2D(420.0, 270.0),
                    Point2D(540.0, 270.0),
                    Point2D(540.0, 410.0),
                    Point2D(420.0, 410.0),
                ),
                obstacle_id="demo-wall",
                name="Demo obstacle",
            ),
        ),
    )


def _write_dashboard(document: MapDocument, plans_and_traces: list[tuple[str, object, tuple[Pose, ...]]]) -> Path:
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(16, 5), constrained_layout=True)
    for ax, (label, plan, trace) in zip(axes, plans_and_traces):
        ax.set_title(label)
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("WORLD X (mm)")
        ax.set_ylabel("WORLD Y (mm)")
        ax.scatter([0.0], [0.0], marker="+", s=120, color="#dc2626", label="Home / WORLD origin")
        for obstacle in document.obstacles:
            points = obstacle.to_world(document.home)
            ax.fill(
                [point.x_mm for point in points],
                [point.y_mm for point in points],
                color="#fca5a5",
                alpha=0.35,
                edgecolor="#dc2626",
                linewidth=1.5,
                label=obstacle.name,
            )

        state = PenState.UP
        for record in plan.toolpath.records:
            if isinstance(record, PenUp):
                state = PenState.UP
                continue
            if isinstance(record, PenDown):
                state = PenState.DOWN
                continue
            if not isinstance(record, Motion):
                continue
            points = sample_geometry(record.geometry, sample_step_mm=5.0)
            ax.plot(
                [point.x_mm for point in points],
                [point.y_mm for point in points],
                color="#2563eb" if state is PenState.DOWN else "#f59e0b",
                linestyle="-" if state is PenState.DOWN else "--",
                linewidth=2.2,
            )
        if len(trace) >= 2:
            ax.plot([pose.x_mm for pose in trace], [pose.y_mm for pose in trace], color="#0f766e", linewidth=1.4, label="simulated pose")
        if plan.end_point is not None:
            ax.scatter([plan.end_point.x_mm], [plan.end_point.y_mm], marker="x", s=80, color="#16a34a", label="target")
        ax.legend(fontsize=8, loc="best")
    output = OUTPUT / "navigation_dashboard.png"
    fig.savefig(output, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return output


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    document = build_document()
    document.save(OUTPUT / "demo_map.vmap.json")
    planner = NavigationPlanner()
    start = Pose(0.0, 0.0, 0.0)

    click = planner.plan_click_to_go(document, start, Point2D(760.0, 500.0))
    draw_start = Pose(-120.0, -70.0, 0.0)
    draw = planner.plan_selected_path(document, draw_start, "demo-route", pen_mode=PenMode.DRAW)
    home_start = Pose(560.0, 350.0, 0.0)
    home = planner.plan_return_home(document, home_start)
    cases = (
        ("click_to_go", click, start),
        ("selected_path_draw", draw, draw_start),
        ("return_home", home, home_start),
    )

    reports = {}
    dashboard_data = []
    for name, plan, pose in cases:
        write_toolpath_trj2(plan.toolpath, OUTPUT / f"{name}.traj", start_yaw_deg=pose.yaw_deg, start_point=Point2D(pose.x_mm, pose.y_mm))
        save_toolpath_preview(
            plan.toolpath,
            OUTPUT / f"{name}_preview.png",
            show_events=True,
            show_pen_state=True,
            show_direction=True,
            show_motion_labels=True,
        )
        runner = SimulatedNavigationRunner(pose)
        job = NavigationJob.create(plan, runner=runner)
        job.start()
        job.run_to_completion(dt_s=0.02)
        job.write_report(OUTPUT / f"{name}_report.json")
        job.write_replay(OUTPUT / f"{name}_replay.jsonl")
        reports[name] = job.report()
        dashboard_data.append((name.replace("_", " ").title(), plan, runner.trace))

    dashboard = _write_dashboard(document, dashboard_data)
    aggregate = {
        "stage": "N2 navigation planning and simulation",
        "artifacts": sorted(path.name for path in OUTPUT.iterdir()),
        "jobs": reports,
        "dashboard": dashboard.name,
    }
    (OUTPUT / "job_report.json").write_text(json.dumps(aggregate, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (OUTPUT / "test_report.txt").write_text(
        "N2 acceptance\n"
        "- polygon obstacle: click-to-go and return-home detours use visibility-graph routing\n"
        "- click_to_go: planned and simulated to target\n"
        "- selected_path_draw: PenUp travel, PenDown route, PenUp finish\n"
        "- return_home: planned and simulated to WORLD origin\n"
        f"- dashboard: {dashboard.name}\n",
        encoding="utf-8",
    )
    print(f"Generated {OUTPUT}")


if __name__ == "__main__":
    main()
