"""Generate and strictly review the current cheap_map navigation TRJ2."""

from __future__ import annotations

import json
from pathlib import Path
import zlib

from pc_trajectory.demo.map_model import PATH_KIND_POLYLINE, MapDocument
from pc_trajectory.demo.map_planner import (
    NavigationPlanner,
    PathDirection,
    PenMode,
    PlannerConfig,
    VehicleProfile,
    _segment_clear,
    _world_obstacles,
)
from pc_trajectory.demo.protocol import Pose
from pc_trajectory.demo.toolpath_review import review_trj2
from pc_trajectory.geometry import Point2D
from pc_trajectory.preview import sample_geometry, save_toolpath_preview
from pc_trajectory.toolpath import Motion, PenDown, PenState, PenUp
from pc_trajectory.traj2_format import TRJ2_HEADER_SIZE, TRJ2_RECORD_SIZE, unpack_header_raw
from pc_trajectory.traj2_reader import decode_trj2, format_trj2
from pc_trajectory.traj2_validate import validate_trj2_file
from pc_trajectory.traj2_writer import encode_trj2


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "test_results" / "virtual_map_demo" / "n2_simulation" / "cheap_map.vmap.json"
OUTPUT = ROOT / "test_results" / "virtual_map_demo" / "cheap_map_traj_review"


def main() -> None:
    document = MapDocument.load(SOURCE)
    if len(document.paths) != 1 or document.paths[0].path_kind != PATH_KIND_POLYLINE:
        raise RuntimeError("cheap_map review expects one exact polyline path")

    config = PlannerConfig(vehicle_profile=VehicleProfile(enabled=True))
    start = Pose(0.0, 0.0, 0.0)
    plan = NavigationPlanner(config).plan_selected_path(
        document,
        start,
        document.paths[0].path_id,
        pen_mode=PenMode.DRAW,
        direction=PathDirection.FORWARD,
    )
    payload = plan.trj2_bytes()
    decoded = decode_trj2(payload)
    validate_trj2_file(decoded)
    roundtrip_exact = encode_trj2(decoded) == payload
    raw_header = unpack_header_raw(payload[:TRJ2_HEADER_SIZE])
    review = review_trj2(payload)

    obstacles = _world_obstacles(document, config)
    collision_free = True
    pen_state = PenState.UP
    pen_sequence: list[str] = []
    drawing_motion_count = 0
    travel_motion_count = 0
    motion_speeds_match_pen_state = True
    for record in plan.toolpath.records:
        if isinstance(record, PenUp):
            pen_state = PenState.UP
            pen_sequence.append("PEN_UP")
        elif isinstance(record, PenDown):
            pen_state = PenState.DOWN
            pen_sequence.append("PEN_DOWN")
        elif isinstance(record, Motion):
            if pen_state is PenState.DOWN:
                drawing_motion_count += 1
                motion_speeds_match_pen_state = (
                    motion_speeds_match_pen_state
                    and record.speed_mm_s == config.draw_speed_mm_s
                )
            else:
                travel_motion_count += 1
                motion_speeds_match_pen_state = (
                    motion_speeds_match_pen_state
                    and record.speed_mm_s == config.travel_speed_mm_s
                )
            points = sample_geometry(record.geometry, sample_step_mm=2.0)
            collision_free = collision_free and all(
                _segment_clear(first, second, obstacles)
                for first, second in zip(points, points[1:])
            )

    expected_size = TRJ2_HEADER_SIZE + decoded.record_count * TRJ2_RECORD_SIZE
    checks = {
        "strict_decode_and_semantic_validation": True,
        "canonical_magic": raw_header.magic == b"TRJ2",
        "canonical_version": raw_header.version == 2,
        "canonical_header_size": raw_header.header_size == TRJ2_HEADER_SIZE,
        "canonical_record_size": raw_header.record_size == TRJ2_RECORD_SIZE,
        "exact_payload_size": len(payload) == expected_size,
        "encode_decode_roundtrip_exact": roundtrip_exact,
        "motion_continuity_validated_by_toolpath": True,
        "collision_free_against_inflated_obstacle": collision_free,
        "starts_and_ends_pen_up": pen_sequence[:1] == ["PEN_UP"] and pen_sequence[-1:] == ["PEN_UP"],
        "contains_draw_sections": "PEN_DOWN" in pen_sequence,
        "motion_speeds_match_pen_state": motion_speeds_match_pen_state,
        "line_count_is_bounded": review.line_count <= 20,
    }
    report = {
        "source_map": str(SOURCE.relative_to(ROOT)),
        "assumptions": {
            "start_pose": {"x_mm": start.x_mm, "y_mm": start.y_mm, "yaw_deg": start.yaw_deg},
            "pen_mode": PenMode.DRAW.value,
            "direction": PathDirection.FORWARD.value,
            "path_kind": document.paths[0].path_kind,
            "vehicle_profile": {
                "enabled": config.vehicle_profile.enabled,
                "front_mm": config.vehicle_profile.front_mm,
                "rear_mm": config.vehicle_profile.rear_mm,
                "left_mm": config.vehicle_profile.left_mm,
                "right_mm": config.vehicle_profile.right_mm,
                "safety_margin_mm": config.vehicle_profile.safety_margin_mm,
                "effective_radius_mm": config.vehicle_profile.effective_radius_mm,
            },
        },
        "header": {
            "magic": raw_header.magic.decode("ascii"),
            "version": raw_header.version,
            "header_size": raw_header.header_size,
            "record_count": raw_header.record_count,
            "record_size": raw_header.record_size,
            "start_x_mm": raw_header.start_x_mm,
            "start_y_mm": raw_header.start_y_mm,
            "start_yaw_deg": raw_header.start_yaw_deg,
        },
        "payload": {
            "size_bytes": len(payload),
            "expected_size_bytes": expected_size,
            "crc32": f"{zlib.crc32(payload) & 0xFFFFFFFF:08X}",
        },
        "records": {
            "total": review.record_count,
            "line": review.line_count,
            "circle": review.circle_count,
            "cubic": review.cubic_count,
            "pen_up": review.pen_up_count,
            "pen_down": review.pen_down_count,
            "wait": review.wait_count,
            "drawing_motion": drawing_motion_count,
            "travel_motion": travel_motion_count,
            "pen_sequence": pen_sequence,
        },
        "route": {
            "requested_start_world": [plan.route_legs[0].requested_start.x_mm, plan.route_legs[0].requested_start.y_mm],
            "effective_start_world": [plan.route_legs[0].effective_start.x_mm, plan.route_legs[0].effective_start.y_mm],
            "requested_end_world": [plan.route_legs[0].requested_end.x_mm, plan.route_legs[0].requested_end.y_mm],
            "effective_end_world": [plan.route_legs[0].effective_end.x_mm, plan.route_legs[0].effective_end.y_mm],
            "detour_count": plan.route_legs[0].detour_count,
            "end_world": [plan.end_point.x_mm, plan.end_point.y_mm] if plan.end_point else None,
        },
        "checks": checks,
        "pass": all(checks.values()),
    }

    OUTPUT.mkdir(parents=True, exist_ok=True)
    (OUTPUT / "cheap_map_current.traj").write_bytes(payload)
    (OUTPUT / "cheap_map_current.txt").write_text(format_trj2(decoded) + "\n", encoding="utf-8")
    (OUTPUT / "cheap_map_current_review.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    save_toolpath_preview(
        plan.toolpath,
        OUTPUT / "cheap_map_current_preview.png",
        show_events=True,
        show_pen_state=True,
        show_direction=True,
        show_motion_labels=True,
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
