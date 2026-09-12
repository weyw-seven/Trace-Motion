"""Generate a visual acceptance bundle for N2.6 obstacle avoidance/sequences."""

from __future__ import annotations

import json
from pathlib import Path

from pc_trajectory.demo.map_model import PATH_KIND_POLYLINE, MapDocument, MapHome, MapObstacle, MapPath
from pc_trajectory.demo.map_planner import NavigationPlanner, PenMode, PathDirection
from pc_trajectory.demo.navigation_job import NavigationJob
from pc_trajectory.demo.navigation_simulator import SimulatedNavigationRunner
from pc_trajectory.demo.protocol import Pose
from pc_trajectory.geometry import Point2D
from pc_trajectory.preview import sample_geometry, save_toolpath_preview
from pc_trajectory.toolpath import Motion, PenDown, PenState, PenUp
from pc_trajectory.toolpath_trj2_export import write_toolpath_trj2


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "test_results" / "virtual_map_demo" / "n2_obstacle_sequence"


def build_document() -> MapDocument:
    return MapDocument(
        name="N2.6 obstacle and sequence acceptance",
        width_mm=600.0,
        height_mm=400.0,
        home=MapHome(300.0, 200.0, 0.0),
        paths=(
            MapPath((Point2D(0.0, 0.0), Point2D(180.0, 0.0)), path_id="cross", name="Cross obstacle", path_kind=PATH_KIND_POLYLINE),
            MapPath((Point2D(180.0, 0.0), Point2D(180.0, 100.0)), path_id="north", name="North leg", path_kind=PATH_KIND_POLYLINE),
            MapPath((Point2D(180.0, 100.0), Point2D(-80.0, 100.0)), path_id="return", name="Return leg", path_kind=PATH_KIND_POLYLINE),
        ),
        obstacles=(
            MapObstacle(
                (Point2D(55.0, -35.0), Point2D(115.0, -35.0), Point2D(115.0, 35.0), Point2D(55.0, 35.0)),
                obstacle_id="wall",
                name="Central obstacle",
            ),
        ),
    )


def write_dashboard(document: MapDocument, plan, trace: tuple[Pose, ...]) -> Path:
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 7), constrained_layout=True)
    ax.set_title("N2.6 automatic obstacle avoidance + ordered path sequence")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("WORLD X (mm), +X forward")
    ax.set_ylabel("WORLD Y (mm), +Y left")
    ax.scatter([0], [0], marker="+", s=140, color="#dc2626", label="WORLD origin")
    for obstacle in document.obstacles:
        points = obstacle.to_world(document.home)
        ax.fill([p.x_mm for p in points], [p.y_mm for p in points], color="#fca5a5", alpha=0.4, edgecolor="#dc2626", label=obstacle.name)

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
        points = sample_geometry(record.geometry, sample_step_mm=4.0)
        ax.plot(
            [p.x_mm for p in points], [p.y_mm for p in points],
            color="#2563eb" if state is PenState.DOWN else "#f59e0b",
            linestyle="-" if state is PenState.DOWN else "--",
            linewidth=2.3,
        )
    if len(trace) >= 2:
        ax.plot([p.x_mm for p in trace], [p.y_mm for p in trace], color="#0f766e", linewidth=1.3, label="simulated pose")
    for leg in plan.route_legs:
        ax.text(leg.effective_end.x_mm, leg.effective_end.y_mm, str(leg.sequence_index), color="#92400e", fontsize=11, weight="bold")
    ax.legend(loc="best")
    output = OUTPUT / "obstacle_sequence_dashboard.png"
    fig.savefig(output, dpi=160, bbox_inches="tight")
    plt.close(fig)
    return output


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    document = build_document()
    document.save(OUTPUT / "obstacle_sequence_map.vmap.json")
    planner = NavigationPlanner()
    start = Pose(-70.0, -70.0, 0.0)
    plan = planner.plan_path_sequence(
        document,
        start,
        ("cross", "north", "return"),
        pen_mode=PenMode.DRAW,
        direction=PathDirection.FORWARD,
    )
    write_toolpath_trj2(plan.toolpath, OUTPUT / "obstacle_sequence.traj", start_yaw_deg=start.yaw_deg, start_point=Point2D(start.x_mm, start.y_mm))
    save_toolpath_preview(plan.toolpath, OUTPUT / "obstacle_sequence_preview.png", show_events=True, show_pen_state=True, show_direction=True, show_motion_labels=True)
    runner = SimulatedNavigationRunner(start)
    job = NavigationJob.create(plan, runner=runner)
    job.start()
    job.run_to_completion(dt_s=0.02)
    job.write_report(OUTPUT / "obstacle_sequence_report.json")
    job.write_replay(OUTPUT / "obstacle_sequence_replay.jsonl")
    dashboard = write_dashboard(document, plan, runner.trace)
    aggregate = {
        "stage": "N2.6 obstacle avoidance and ordered path sequence",
        "path_order": list(plan.path_ids),
        "effective_target": {"x_mm": plan.target_world.x_mm, "y_mm": plan.target_world.y_mm},
        "detour_count": sum(leg.detour_count for leg in plan.route_legs),
        "dashboard": dashboard.name,
        "artifacts": sorted(path.name for path in OUTPUT.iterdir()),
    }
    (OUTPUT / "acceptance.json").write_text(json.dumps(aggregate, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (OUTPUT / "acceptance.txt").write_text(
        "N2.6 acceptance\n"
        "- crossing path is automatically split into PenDown drawing and PenUp detour segments\n"
        "- path sequence is compiled in order: cross -> north -> return\n"
        "- dashboard shows obstacle, numbered legs, planned route and simulator trace\n",
        encoding="utf-8",
    )
    print(f"Generated {OUTPUT}")


if __name__ == "__main__":
    main()
