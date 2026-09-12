from pathlib import Path

from pc_trajectory.demo.hardware_preflight import preflight_m6_trajectory
from pc_trajectory.demo.map_model import MapDocument
from pc_trajectory.demo.map_planner import NavigationPlanner, PathDirection, PenMode, PlannerConfig
from pc_trajectory.demo.protocol import Pose
from pc_trajectory.demo.toolpath_review import review_trj2
from pc_trajectory.traj2_format import (
    Trj2CircleRecord,
    Trj2File,
    Trj2Header,
    Trj2LineRecord,
    Trj2PenDownRecord,
    Trj2PenUpRecord,
)
from pc_trajectory.traj2_writer import encode_trj2


M6_HELLO = {
    "build_profile": "motion-circle-pen",
    "hardware_enabled": True,
    "motion_enabled": True,
    "circle_enabled": True,
    "pen_enabled": True,
}


def _payload(*records):
    return encode_trj2(Trj2File(Trj2Header(0.0, 0.0, 0.0), records))


def test_m6_preflight_accepts_normal_line_circle_pen_job() -> None:
    payload = _payload(
        Trj2PenUpRecord(),
        Trj2LineRecord(200.0, 0.0, 80.0, 150.0),
        Trj2PenDownRecord(),
        Trj2CircleRecord(200.0, 150.0, 150.0, -90.0, 90.0, 80.0, 150.0),
        Trj2PenUpRecord(),
    )
    result = preflight_m6_trajectory(payload, hello=M6_HELLO)
    assert result.ok
    assert result.record_count == 5
    assert result.total_distance_mm > 400.0
    assert result.estimated_total_s > result.estimated_motion_s


def test_m6_preflight_reports_profile_and_record_specific_errors() -> None:
    payload = _payload(Trj2LineRecord(100.0, 0.0, 20.0, 0.0))
    result = preflight_m6_trajectory(payload, hello={"build_profile": "safe"})
    assert not result.ok
    assert any("build_profile" in item for item in result.errors)
    assert any("record 1 speed" in item for item in result.errors)
    assert any("at least 150" in item for item in result.warnings)


def test_m6_preflight_requires_ready_hello() -> None:
    payload = _payload(Trj2LineRecord(200.0, 0.0, 80.0, 0.0))
    result = preflight_m6_trajectory(payload)
    assert not result.ok
    assert "wait for READY" in result.errors[0]


def test_m6_preflight_accepts_screenshot_sized_job_under_new_limits() -> None:
    """Fourteen ordinary lines / about 3 m must no longer be an upload blocker."""

    records = tuple(
        Trj2LineRecord((index + 1) * 220.0, 0.0, 250.0, 1000.0)
        for index in range(14)
    )
    result = preflight_m6_trajectory(_payload(*records), hello=M6_HELLO)
    assert result.ok
    assert result.record_count == 14
    assert result.total_distance_mm == 3080.0


def test_m6_preflight_accepts_broader_circle_envelope() -> None:
    result = preflight_m6_trajectory(
        _payload(Trj2CircleRecord(0.0, 40.0, 40.0, -90.0, 360.0, 120.0, 500.0)),
        hello=M6_HELLO,
    )
    assert result.ok
    assert any("below the 100 mm" in item for item in result.warnings)


def test_m7_ui_acceptance_map_exports_a_valid_circle_job() -> None:
    root = Path(__file__).resolve().parents[1]
    document = MapDocument.load(root / "examples" / "m7_ui_draw_acceptance.vmap.json")
    config = PlannerConfig(**document.settings["planner"])
    plan = NavigationPlanner(config).plan_path_sequence(
        document,
        Pose(0.0, 0.0, 0.0),
        tuple(document.settings["navigation"]["selected_path_ids"]),
        pen_mode=PenMode.DRAW,
        direction=PathDirection.FORWARD,
    )
    payload = plan.trj2_bytes()
    review = review_trj2(payload)
    result = preflight_m6_trajectory(payload, hello=M6_HELLO)
    assert result.ok
    assert review.record_count == 10
    assert review.circle_count == 1
