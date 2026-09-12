import math

import pytest

import pc_trajectory.trj1_export as bridge
from pc_trajectory.geometry import (
    Arc,
    CubicBezier,
    Line,
    Point2D,
)
from pc_trajectory.toolpath import (
    Motion,
    PenDown,
    PenUp,
    Toolpath,
    Wait,
)
from pc_trajectory.traj_format import (
    CircleSegment,
    LineSegment,
    Trajectory,
)
from pc_trajectory.traj_writer import encode_traj
from pc_trajectory.trj1_export import (
    Trj1EmptyMotionSequenceError,
    Trj1ExportError,
    Trj1MotionContinuityError,
    Trj1UnsupportedGeometryError,
    Trj1UnsupportedRecordError,
    encode_motions_trj1,
    encode_toolpath_trj1,
    motion_to_trj1_segment,
    motions_to_trj1_trajectory,
    toolpath_to_trj1_trajectory,
    write_motions_trj1,
    write_toolpath_trj1,
)


ABS = 1.0e-9

# Independent, frozen Phase-A / ESP32-validated Golden bytes.
# This is deliberately NOT produced by struct.pack inside this test.
GOLDEN_TRJ1_BYTES = bytes.fromhex(
    "54524a3101002000030000002c000000"
    "00000000000000000000000000000000"
    "0100000000009643000000000000fa43"
    "00000000000000000000000000000000"
    "000000000000000000000000"
    "0200000000005c43000000000000fa43"
    "00007a4300007a430000b4c200003443"
    "000000000000000000000000"
    "01000000000096430000000000000000"
    "0000fa43000000000000000000000000"
    "000000000000000000000000"
)


def assert_close(actual, expected, tol=ABS):
    assert math.isclose(actual, expected, rel_tol=0.0, abs_tol=tol)


def golden_motions():
    return (
        Motion(
            Line(
                Point2D(0.0, 0.0),
                Point2D(500.0, 0.0),
            ),
            speed_mm_s=300.0,
            acceleration_mm_s2=0.0,
        ),
        Motion(
            Arc(
                center=Point2D(500.0, 250.0),
                radius_mm=250.0,
                start_angle_deg=-90.0,
                sweep_deg=180.0,
            ),
            speed_mm_s=220.0,
            acceleration_mm_s2=0.0,
        ),
        Motion(
            Line(
                Point2D(500.0, 500.0),
                Point2D(0.0, 500.0),
            ),
            speed_mm_s=300.0,
            acceleration_mm_s2=0.0,
        ),
    )


def test_frozen_golden_fixture_is_164_bytes():
    assert len(GOLDEN_TRJ1_BYTES) == 164


def test_line_motion_lowers_to_phase_a_line_segment():
    motion = Motion(
        Line(
            Point2D(10.0, 20.0),
            Point2D(30.0, 40.0),
        ),
        speed_mm_s=123.0,
        acceleration_mm_s2=456.0,
    )

    segment = motion_to_trj1_segment(motion)

    assert isinstance(segment, LineSegment)
    assert_close(segment.end_x_mm, 30.0)
    assert_close(segment.end_y_mm, 40.0)
    assert_close(segment.speed_mm_s, 123.0)
    assert_close(segment.acceleration_mm_s2, 456.0)


def test_line_lowering_does_not_copy_explicit_start_into_trj1_segment():
    motion = Motion(
        Line(
            Point2D(10.0, 20.0),
            Point2D(30.0, 40.0),
        ),
        speed_mm_s=100.0,
    )

    segment = motion_to_trj1_segment(motion)

    assert not hasattr(segment, "start_x_mm")
    assert not hasattr(segment, "start_y_mm")


def test_arc_motion_lowers_to_phase_a_circle_segment_without_reparameterizing():
    arc = Arc(
        center=Point2D(500.0, 250.0),
        radius_mm=250.0,
        start_angle_deg=-90.0,
        sweep_deg=180.0,
    )
    motion = Motion(
        arc,
        speed_mm_s=220.0,
        acceleration_mm_s2=12.0,
    )

    segment = motion_to_trj1_segment(motion)

    assert isinstance(segment, CircleSegment)
    assert_close(segment.center_x_mm, 500.0)
    assert_close(segment.center_y_mm, 250.0)
    assert_close(segment.radius_mm, 250.0)
    assert_close(segment.start_angle_deg, -90.0)
    assert_close(segment.sweep_deg, 180.0)
    assert_close(segment.speed_mm_s, 220.0)
    assert_close(segment.acceleration_mm_s2, 12.0)


def test_negative_arc_sweep_is_preserved():
    motion = Motion(
        Arc(
            center=Point2D(0.0, 0.0),
            radius_mm=10.0,
            start_angle_deg=90.0,
            sweep_deg=-135.0,
        ),
        speed_mm_s=50.0,
    )

    segment = motion_to_trj1_segment(motion)

    assert isinstance(segment, CircleSegment)
    assert_close(segment.sweep_deg, -135.0)


def test_cubic_bezier_is_explicitly_rejected_by_trj1_target():
    curve = CubicBezier(
        start=Point2D(0.0, 0.0),
        control1=Point2D(20.0, 0.0),
        control2=Point2D(80.0, 100.0),
        end=Point2D(100.0, 100.0),
    )

    with pytest.raises(Trj1UnsupportedGeometryError) as exc_info:
        motion_to_trj1_segment(
            Motion(curve, speed_mm_s=100.0),
            motion_index=7,
        )

    error = exc_info.value
    assert error.motion_index == 7
    assert error.geometry_type == "CubicBezier"


def test_motion_to_segment_rejects_non_motion():
    with pytest.raises(Trj1ExportError):
        motion_to_trj1_segment("not motion")  # type: ignore[arg-type]


def test_motions_export_rejects_empty_sequence():
    with pytest.raises(Trj1EmptyMotionSequenceError):
        motions_to_trj1_trajectory(())


def test_motions_export_rejects_non_motion_member():
    motions = (
        golden_motions()[0],
        "not motion",
    )

    with pytest.raises(Trj1ExportError):
        motions_to_trj1_trajectory(motions)  # type: ignore[arg-type]


def test_header_start_xy_is_derived_from_first_explicit_motion_start():
    motions = (
        Motion(
            Line(
                Point2D(123.5, -44.25),
                Point2D(200.0, 0.0),
            ),
            speed_mm_s=100.0,
        ),
    )

    trajectory = motions_to_trj1_trajectory(
        motions,
        start_yaw_deg=17.0,
    )

    assert isinstance(trajectory, Trajectory)
    assert_close(trajectory.header.start_x_mm, 123.5)
    assert_close(trajectory.header.start_y_mm, -44.25)
    assert_close(trajectory.header.start_yaw_deg, 17.0)


def test_start_yaw_is_not_inferred_from_first_line_tangent():
    # First line points +Y (90 deg tangent), but chassis yaw target remains
    # the caller-specified 0 deg.
    motions = (
        Motion(
            Line(
                Point2D(0.0, 0.0),
                Point2D(0.0, 100.0),
            ),
            speed_mm_s=100.0,
        ),
    )

    trajectory = motions_to_trj1_trajectory(
        motions,
        start_yaw_deg=0.0,
    )

    assert_close(trajectory.header.start_yaw_deg, 0.0)


@pytest.mark.parametrize("yaw", [math.nan, math.inf, -math.inf])
def test_nonfinite_start_yaw_is_rejected(yaw):
    with pytest.raises(Trj1ExportError):
        motions_to_trj1_trajectory(
            golden_motions(),
            start_yaw_deg=yaw,
        )


@pytest.mark.parametrize("tol", [-0.1, math.nan, math.inf, -math.inf])
def test_invalid_continuity_tolerance_is_rejected(tol):
    with pytest.raises(Trj1ExportError):
        motions_to_trj1_trajectory(
            golden_motions(),
            continuity_tolerance_mm=tol,
        )


def test_explicit_line_start_discontinuity_is_caught_before_implicit_start_lowering():
    motions = (
        Motion(
            Line(
                Point2D(0.0, 0.0),
                Point2D(100.0, 0.0),
            ),
            speed_mm_s=100.0,
        ),
        Motion(
            Line(
                Point2D(100.02, 0.0),
                Point2D(200.0, 0.0),
            ),
            speed_mm_s=100.0,
        ),
    )

    with pytest.raises(Trj1MotionContinuityError) as exc_info:
        motions_to_trj1_trajectory(motions)

    error = exc_info.value
    assert error.previous_motion_index == 0
    assert error.next_motion_index == 1
    assert_close(error.position_error_mm, 0.02)
    assert_close(error.tolerance_mm, 0.01)


def test_small_explicit_start_error_within_pc_tolerance_can_lower():
    motions = (
        Motion(
            Line(
                Point2D(0.0, 0.0),
                Point2D(100.0, 0.0),
            ),
            speed_mm_s=100.0,
        ),
        Motion(
            Line(
                Point2D(100.005, 0.0),
                Point2D(200.0, 0.0),
            ),
            speed_mm_s=100.0,
        ),
    )

    trajectory = motions_to_trj1_trajectory(motions)

    assert len(trajectory.segments) == 2


def test_custom_stricter_tolerance_can_reject_small_explicit_start_error():
    motions = (
        Motion(
            Line(
                Point2D(0.0, 0.0),
                Point2D(100.0, 0.0),
            ),
            speed_mm_s=100.0,
        ),
        Motion(
            Line(
                Point2D(100.005, 0.0),
                Point2D(200.0, 0.0),
            ),
            speed_mm_s=100.0,
        ),
    )

    with pytest.raises(Trj1MotionContinuityError):
        motions_to_trj1_trajectory(
            motions,
            continuity_tolerance_mm=0.001,
        )


def test_golden_motion_sequence_lowers_to_expected_phase_a_objects():
    trajectory = motions_to_trj1_trajectory(
        golden_motions(),
        start_yaw_deg=0.0,
    )

    assert_close(trajectory.header.start_x_mm, 0.0)
    assert_close(trajectory.header.start_y_mm, 0.0)
    assert_close(trajectory.header.start_yaw_deg, 0.0)
    assert len(trajectory.segments) == 3

    assert isinstance(trajectory.segments[0], LineSegment)
    assert isinstance(trajectory.segments[1], CircleSegment)
    assert isinstance(trajectory.segments[2], LineSegment)


def test_new_phase_b_ir_reproduces_exact_esp32_validated_164_byte_golden():
    actual = encode_motions_trj1(
        golden_motions(),
        start_yaw_deg=0.0,
    )

    assert len(actual) == 164
    assert actual == GOLDEN_TRJ1_BYTES


def test_bridge_encoding_equals_direct_phase_a_encoding():
    trajectory = motions_to_trj1_trajectory(
        golden_motions(),
        start_yaw_deg=0.0,
    )

    assert encode_motions_trj1(
        golden_motions(),
        start_yaw_deg=0.0,
    ) == encode_traj(trajectory)


def test_encode_motions_delegates_binary_work_to_phase_a_writer(monkeypatch):
    sentinel = b"phase-a-writer-called"
    captured = {}

    def fake_encode_traj(trajectory):
        captured["trajectory"] = trajectory
        return sentinel

    monkeypatch.setattr(bridge, "encode_traj", fake_encode_traj)

    result = bridge.encode_motions_trj1(golden_motions())

    assert result == sentinel
    assert isinstance(captured["trajectory"], Trajectory)


def test_event_free_toolpath_can_lower_to_trj1():
    toolpath = Toolpath(golden_motions())

    trajectory = toolpath_to_trj1_trajectory(
        toolpath,
        start_yaw_deg=0.0,
    )

    assert len(trajectory.segments) == 3
    assert encode_toolpath_trj1(toolpath) == GOLDEN_TRJ1_BYTES


@pytest.mark.parametrize(
    "record",
    [PenDown(), PenUp(), Wait(0.25)],
)
def test_toolpath_events_are_rejected_not_silently_discarded(record):
    first = golden_motions()[0]

    # Put the unsupported event at record index 1.
    toolpath = Toolpath(
        (
            first,
            record,
        )
    )

    with pytest.raises(Trj1UnsupportedRecordError) as exc_info:
        toolpath_to_trj1_trajectory(toolpath)

    error = exc_info.value
    assert error.record_index == 1
    assert error.record_type == type(record).__name__


def test_normal_b4_compiled_pen_toolpath_is_incompatible_with_trj1_by_design():
    # Semantics equivalent to one Stroke:
    # PEN_DOWN -> draw -> PEN_UP
    motion = Motion(
        Line(
            Point2D(0.0, 0.0),
            Point2D(100.0, 0.0),
        ),
        speed_mm_s=300.0,
    )
    toolpath = Toolpath(
        (
            PenDown(),
            motion,
            PenUp(),
        )
    )

    with pytest.raises(Trj1UnsupportedRecordError) as exc_info:
        encode_toolpath_trj1(toolpath)

    assert exc_info.value.record_index == 0
    assert exc_info.value.record_type == "PenDown"


def test_toolpath_with_bezier_motion_is_rejected_even_without_events():
    curve = CubicBezier(
        start=Point2D(0.0, 0.0),
        control1=Point2D(30.0, 0.0),
        control2=Point2D(70.0, 100.0),
        end=Point2D(100.0, 100.0),
    )
    toolpath = Toolpath(
        (
            Motion(curve, speed_mm_s=100.0),
        )
    )

    with pytest.raises(Trj1UnsupportedGeometryError):
        encode_toolpath_trj1(toolpath)


def test_toolpath_export_rejects_empty_toolpath():
    with pytest.raises(Trj1EmptyMotionSequenceError):
        toolpath_to_trj1_trajectory(Toolpath())


def test_toolpath_export_rejects_non_toolpath_input():
    with pytest.raises(Trj1ExportError):
        toolpath_to_trj1_trajectory("not toolpath")  # type: ignore[arg-type]


def test_toolpath_uses_its_own_continuity_tolerance_by_default():
    motion0 = Motion(
        Line(
            Point2D(0.0, 0.0),
            Point2D(100.0, 0.0),
        ),
        speed_mm_s=100.0,
    )
    motion1 = Motion(
        Line(
            Point2D(100.5, 0.0),
            Point2D(200.0, 0.0),
        ),
        speed_mm_s=100.0,
    )

    toolpath = Toolpath(
        (motion0, motion1),
        continuity_tolerance_mm=1.0,
    )

    trajectory = toolpath_to_trj1_trajectory(toolpath)

    assert len(trajectory.segments) == 2


def test_toolpath_export_can_override_with_stricter_tolerance():
    motion0 = Motion(
        Line(
            Point2D(0.0, 0.0),
            Point2D(100.0, 0.0),
        ),
        speed_mm_s=100.0,
    )
    motion1 = Motion(
        Line(
            Point2D(100.5, 0.0),
            Point2D(200.0, 0.0),
        ),
        speed_mm_s=100.0,
    )
    toolpath = Toolpath(
        (motion0, motion1),
        continuity_tolerance_mm=1.0,
    )

    with pytest.raises(Trj1MotionContinuityError):
        toolpath_to_trj1_trajectory(
            toolpath,
            continuity_tolerance_mm=0.01,
        )


def test_write_motions_trj1_writes_exact_golden_file(tmp_path):
    output = tmp_path / "golden_from_phase_b.traj"

    returned = write_motions_trj1(
        golden_motions(),
        output,
        start_yaw_deg=0.0,
    )

    assert returned == output
    assert output.read_bytes() == GOLDEN_TRJ1_BYTES
    assert output.stat().st_size == 164


def test_write_event_free_toolpath_trj1_writes_exact_golden_file(tmp_path):
    output = tmp_path / "golden_toolpath.traj"
    toolpath = Toolpath(golden_motions())

    returned = write_toolpath_trj1(
        toolpath,
        output,
        start_yaw_deg=0.0,
    )

    assert returned == output
    assert output.read_bytes() == GOLDEN_TRJ1_BYTES


def test_bridge_does_not_add_trj_binary_layout_constants():
    # Architectural regression: struct layout remains owned by Phase A.
    forbidden_names = {
        "HEADER_FMT",
        "RECORD_FMT",
        "HEADER_SIZE",
        "RECORD_SIZE",
    }

    assert forbidden_names.isdisjoint(set(vars(bridge)))


def test_golden_motion_speeds_preserve_original_phase_a_300_220_300():
    trajectory = motions_to_trj1_trajectory(golden_motions())

    assert [segment.speed_mm_s for segment in trajectory.segments] == [
        300.0,
        220.0,
        300.0,
    ]


def test_motion_acceleration_is_preserved_exactly_into_phase_a_segments():
    motions = (
        Motion(
            Line(
                Point2D(0.0, 0.0),
                Point2D(100.0, 0.0),
            ),
            speed_mm_s=100.0,
            acceleration_mm_s2=321.0,
        ),
        Motion(
            Arc(
                center=Point2D(100.0, 50.0),
                radius_mm=50.0,
                start_angle_deg=-90.0,
                sweep_deg=90.0,
            ),
            speed_mm_s=80.0,
            acceleration_mm_s2=654.0,
        ),
    )

    trajectory = motions_to_trj1_trajectory(motions)

    assert_close(trajectory.segments[0].acceleration_mm_s2, 321.0)
    assert_close(trajectory.segments[1].acceleration_mm_s2, 654.0)
