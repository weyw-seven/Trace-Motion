import dataclasses
import math

import pytest

from pc_trajectory.drawing import PC_CONTINUITY_TOLERANCE_MM
from pc_trajectory.geometry import Arc, CubicBezier, Line, Point2D
from pc_trajectory.toolpath import (
    INITIAL_PEN_STATE,
    Motion,
    MotionContinuityError,
    PenDown,
    PenState,
    PenUp,
    Toolpath,
    ToolpathError,
    Wait,
)


ABS = 1.0e-9


def assert_close(actual, expected, tol=ABS):
    assert math.isclose(actual, expected, rel_tol=0.0, abs_tol=tol)


def assert_point_close(actual: Point2D, expected: Point2D, tol=ABS):
    assert_close(actual.x_mm, expected.x_mm, tol)
    assert_close(actual.y_mm, expected.y_mm, tol)


def test_initial_pen_state_is_locked_up():
    assert INITIAL_PEN_STATE is PenState.UP


def test_motion_accepts_line_and_execution_constraints():
    line = Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0))
    motion = Motion(
        geometry=line,
        speed_mm_s=300.0,
        acceleration_mm_s2=0.0,
    )

    assert motion.geometry is line
    assert_close(motion.speed_mm_s, 300.0)
    assert_close(motion.acceleration_mm_s2, 0.0)
    assert_point_close(motion.start_point(), Point2D(0.0, 0.0))
    assert_point_close(motion.end_point(), Point2D(100.0, 0.0))
    assert_close(motion.length_mm(), 100.0)


@pytest.mark.parametrize("speed", [0.0, -1.0, math.nan, math.inf, -math.inf])
def test_motion_rejects_invalid_speed(speed):
    line = Line(Point2D(0.0, 0.0), Point2D(1.0, 0.0))

    with pytest.raises(ToolpathError):
        Motion(line, speed_mm_s=speed)


@pytest.mark.parametrize("acceleration", [-0.001, math.nan, math.inf, -math.inf])
def test_motion_rejects_invalid_acceleration(acceleration):
    line = Line(Point2D(0.0, 0.0), Point2D(1.0, 0.0))

    with pytest.raises(ToolpathError):
        Motion(
            line,
            speed_mm_s=1.0,
            acceleration_mm_s2=acceleration,
        )


def test_motion_accepts_positive_acceleration():
    line = Line(Point2D(0.0, 0.0), Point2D(1.0, 0.0))
    motion = Motion(
        line,
        speed_mm_s=10.0,
        acceleration_mm_s2=1200.0,
    )

    assert_close(motion.acceleration_mm_s2, 1200.0)


def test_motion_rejects_unsupported_geometry_type():
    with pytest.raises(ToolpathError):
        Motion("not geometry", speed_mm_s=10.0)  # type: ignore[arg-type]


def test_cubic_bezier_is_valid_motion_geometry():
    curve = CubicBezier(
        start=Point2D(0.0, 0.0),
        control1=Point2D(0.0, 10.0),
        control2=Point2D(10.0, 10.0),
        end=Point2D(10.0, 0.0),
    )

    motion = Motion(curve, speed_mm_s=50.0)

    assert motion.geometry is curve
    assert motion.length_mm() > 0.0


def test_motion_has_no_travel_or_drawing_role_flag():
    fields = {field.name for field in dataclasses.fields(Motion)}

    assert fields == {
        "geometry",
        "speed_mm_s",
        "acceleration_mm_s2",
    }
    assert "is_travel" not in fields
    assert "is_drawing" not in fields


def test_pen_events_are_payload_free_logical_events():
    assert dataclasses.fields(PenUp) == ()
    assert dataclasses.fields(PenDown) == ()


def test_wait_accepts_positive_finite_duration():
    wait = Wait(0.25)
    assert_close(wait.duration_s, 0.25)


@pytest.mark.parametrize("duration", [0.0, -1.0, math.nan, math.inf, -math.inf])
def test_wait_rejects_invalid_duration(duration):
    with pytest.raises(ToolpathError):
        Wait(duration)


def test_empty_toolpath_is_valid():
    toolpath = Toolpath()

    assert toolpath.is_empty()
    assert not toolpath.has_motion()
    assert toolpath.record_count == 0
    assert toolpath.motion_count == 0
    assert toolpath.event_count == 0
    assert toolpath.wait_count == 0
    assert_close(toolpath.total_motion_length_mm(), 0.0)
    assert_close(toolpath.drawing_length_mm(), 0.0)
    assert_close(toolpath.travel_length_mm(), 0.0)
    assert_close(toolpath.total_wait_duration_s(), 0.0)
    assert toolpath.final_pen_state() is PenState.UP

    with pytest.raises(ToolpathError):
        toolpath.start_point()
    with pytest.raises(ToolpathError):
        toolpath.end_point()


def test_toolpath_normalizes_records_to_tuple():
    records = [PenDown(), PenUp()]
    toolpath = Toolpath(records)

    assert isinstance(toolpath.records, tuple)
    assert toolpath.record_count == 2


def test_toolpath_rejects_unsupported_record_type():
    with pytest.raises(ToolpathError):
        Toolpath(("not a record",))  # type: ignore[arg-type]


@pytest.mark.parametrize("tolerance", [-1.0, math.nan, math.inf, -math.inf])
def test_toolpath_rejects_invalid_continuity_tolerance(tolerance):
    with pytest.raises(ToolpathError):
        Toolpath((), continuity_tolerance_mm=tolerance)


def test_event_only_toolpath_can_change_pen_state_without_motion_position():
    toolpath = Toolpath(
        (
            PenDown(),
            Wait(0.1),
            PenUp(),
        )
    )

    assert not toolpath.has_motion()
    assert toolpath.record_count == 3
    assert toolpath.motion_count == 0
    assert toolpath.event_count == 3
    assert toolpath.wait_count == 1
    assert_close(toolpath.total_wait_duration_s(), 0.1)
    assert toolpath.final_pen_state() is PenState.UP

    with pytest.raises(ToolpathError):
        toolpath.start_point()


def test_first_motion_under_initial_pen_up_is_travel():
    motion = Motion(
        Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),
        speed_mm_s=500.0,
    )
    toolpath = Toolpath((motion,))

    assert_close(toolpath.total_motion_length_mm(), 100.0)
    assert_close(toolpath.drawing_length_mm(), 0.0)
    assert_close(toolpath.travel_length_mm(), 100.0)


def test_pen_down_then_motion_is_drawing():
    motion = Motion(
        Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),
        speed_mm_s=300.0,
    )
    toolpath = Toolpath((PenDown(), motion, PenUp()))

    assert_close(toolpath.drawing_length_mm(), 100.0)
    assert_close(toolpath.travel_length_mm(), 0.0)
    assert toolpath.final_pen_state() is PenState.UP


def test_wait_changes_neither_pen_state_nor_motion_classification():
    motion0 = Motion(
        Line(Point2D(0.0, 0.0), Point2D(50.0, 0.0)),
        speed_mm_s=300.0,
    )
    motion1 = Motion(
        Line(Point2D(50.0, 0.0), Point2D(100.0, 0.0)),
        speed_mm_s=300.0,
    )

    toolpath = Toolpath(
        (
            PenDown(),
            motion0,
            Wait(0.2),
            motion1,
            PenUp(),
        )
    )

    assert_close(toolpath.drawing_length_mm(), 100.0)
    assert_close(toolpath.travel_length_mm(), 0.0)
    assert_close(toolpath.total_wait_duration_s(), 0.2)


def test_repeated_pen_events_are_structurally_valid_and_state_is_idempotent():
    motion = Motion(
        Line(Point2D(0.0, 0.0), Point2D(20.0, 0.0)),
        speed_mm_s=100.0,
    )

    toolpath = Toolpath(
        (
            PenDown(),
            PenDown(),
            motion,
            PenUp(),
            PenUp(),
        )
    )

    assert_close(toolpath.drawing_length_mm(), 20.0)
    assert toolpath.final_pen_state() is PenState.UP


def test_events_do_not_hide_motion_discontinuity():
    motion0 = Motion(
        Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),
        speed_mm_s=300.0,
    )
    motion1 = Motion(
        Line(Point2D(200.0, 0.0), Point2D(300.0, 0.0)),
        speed_mm_s=300.0,
    )

    with pytest.raises(MotionContinuityError) as exc_info:
        Toolpath(
            (
                PenDown(),
                motion0,
                PenUp(),
                Wait(0.1),
                PenDown(),
                motion1,
            )
        )

    error = exc_info.value
    assert error.previous_record_index == 1
    assert error.next_record_index == 5
    assert_close(error.position_error_mm, 100.0)
    assert_close(error.tolerance_mm, PC_CONTINUITY_TOLERANCE_MM)


def test_real_travel_motion_makes_two_drawing_regions_continuous():
    draw0 = Motion(
        Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),
        speed_mm_s=300.0,
    )
    travel = Motion(
        Line(Point2D(100.0, 0.0), Point2D(200.0, 0.0)),
        speed_mm_s=500.0,
    )
    draw1 = Motion(
        Line(Point2D(200.0, 0.0), Point2D(300.0, 0.0)),
        speed_mm_s=300.0,
    )

    toolpath = Toolpath(
        (
            PenDown(),
            draw0,
            PenUp(),
            travel,
            PenDown(),
            draw1,
            PenUp(),
        )
    )

    toolpath.validate_position_continuity()
    assert_close(toolpath.drawing_length_mm(), 200.0)
    assert_close(toolpath.travel_length_mm(), 100.0)
    assert_close(toolpath.total_motion_length_mm(), 300.0)


def test_toolpath_accepts_small_motion_position_error_within_pc_tolerance():
    motion0 = Motion(
        Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),
        speed_mm_s=300.0,
    )
    motion1 = Motion(
        Line(Point2D(100.005, 0.0), Point2D(200.0, 0.0)),
        speed_mm_s=300.0,
    )

    toolpath = Toolpath((motion0, motion1))
    toolpath.validate_position_continuity()


def test_toolpath_rejects_motion_position_error_above_pc_tolerance():
    motion0 = Motion(
        Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),
        speed_mm_s=300.0,
    )
    motion1 = Motion(
        Line(Point2D(100.02, 0.0), Point2D(200.0, 0.0)),
        speed_mm_s=300.0,
    )

    with pytest.raises(MotionContinuityError):
        Toolpath((motion0, motion1))


def test_toolpath_can_be_revalidated_with_custom_tolerance():
    motion0 = Motion(
        Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),
        speed_mm_s=300.0,
    )
    motion1 = Motion(
        Line(Point2D(100.5, 0.0), Point2D(200.0, 0.0)),
        speed_mm_s=300.0,
    )
    toolpath = Toolpath(
        (motion0, motion1),
        continuity_tolerance_mm=1.0,
    )

    with pytest.raises(MotionContinuityError):
        toolpath.validate_position_continuity(tolerance_mm=0.01)


def test_start_and_end_points_ignore_leading_and_trailing_events():
    motion = Motion(
        Line(Point2D(10.0, 20.0), Point2D(30.0, 40.0)),
        speed_mm_s=100.0,
    )
    toolpath = Toolpath(
        (
            PenDown(),
            Wait(0.1),
            motion,
            PenUp(),
            Wait(0.2),
        )
    )

    assert_point_close(toolpath.start_point(), Point2D(10.0, 20.0))
    assert_point_close(toolpath.end_point(), Point2D(30.0, 40.0))


def test_mixed_drawing_and_travel_lengths_are_derived_from_pen_state():
    draw0 = Motion(
        Line(Point2D(0.0, 0.0), Point2D(40.0, 0.0)),
        speed_mm_s=200.0,
    )
    travel0 = Motion(
        Line(Point2D(40.0, 0.0), Point2D(50.0, 0.0)),
        speed_mm_s=500.0,
    )
    draw1 = Motion(
        Line(Point2D(50.0, 0.0), Point2D(80.0, 0.0)),
        speed_mm_s=200.0,
    )
    travel1 = Motion(
        Line(Point2D(80.0, 0.0), Point2D(100.0, 0.0)),
        speed_mm_s=500.0,
    )

    toolpath = Toolpath(
        (
            PenDown(),
            draw0,
            PenUp(),
            travel0,
            PenDown(),
            draw1,
            PenUp(),
            travel1,
        )
    )

    assert_close(toolpath.drawing_length_mm(), 70.0)
    assert_close(toolpath.travel_length_mm(), 30.0)
    assert_close(toolpath.total_motion_length_mm(), 100.0)


def test_same_position_stroke_boundary_can_pen_cycle_without_travel_motion():
    draw0 = Motion(
        Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),
        speed_mm_s=300.0,
    )
    draw1 = Motion(
        Line(Point2D(100.0, 0.0), Point2D(200.0, 0.0)),
        speed_mm_s=300.0,
    )

    toolpath = Toolpath(
        (
            PenDown(),
            draw0,
            PenUp(),
            PenDown(),
            draw1,
            PenUp(),
        )
    )

    assert toolpath.motion_count == 2
    assert_close(toolpath.drawing_length_mm(), 200.0)
    assert_close(toolpath.travel_length_mm(), 0.0)


def test_golden_single_stroke_toolpath():
    line0 = Motion(
        Line(Point2D(0.0, 0.0), Point2D(500.0, 0.0)),
        speed_mm_s=300.0,
    )
    arc = Motion(
        Arc(
            center=Point2D(500.0, 250.0),
            radius_mm=250.0,
            start_angle_deg=-90.0,
            sweep_deg=180.0,
        ),
        speed_mm_s=220.0,
    )
    line2 = Motion(
        Line(Point2D(500.0, 500.0), Point2D(0.0, 500.0)),
        speed_mm_s=300.0,
    )

    toolpath = Toolpath(
        (
            PenDown(),
            line0,
            arc,
            line2,
            PenUp(),
        )
    )

    assert toolpath.record_count == 5
    assert toolpath.motion_count == 3
    assert toolpath.event_count == 2
    assert_close(
        toolpath.drawing_length_mm(),
        1000.0 + 250.0 * math.pi,
    )
    assert_close(toolpath.travel_length_mm(), 0.0)
    assert_close(
        toolpath.total_motion_length_mm(),
        1000.0 + 250.0 * math.pi,
    )
    assert_point_close(toolpath.start_point(), Point2D(0.0, 0.0))
    assert_point_close(toolpath.end_point(), Point2D(0.0, 500.0))
    assert toolpath.final_pen_state() is PenState.UP


def test_two_stroke_toolpath_matches_trj2_design_semantics():
    draw0 = Motion(
        Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),
        speed_mm_s=300.0,
    )
    travel = Motion(
        Line(Point2D(100.0, 0.0), Point2D(200.0, 0.0)),
        speed_mm_s=500.0,
    )
    draw1 = Motion(
        Line(Point2D(200.0, 0.0), Point2D(300.0, 0.0)),
        speed_mm_s=300.0,
    )

    toolpath = Toolpath(
        (
            PenDown(),
            draw0,
            PenUp(),
            travel,
            PenDown(),
            draw1,
            PenUp(),
        )
    )

    assert toolpath.record_count == 7
    assert toolpath.motion_count == 3
    assert toolpath.event_count == 4
    assert_close(toolpath.drawing_length_mm(), 200.0)
    assert_close(toolpath.travel_length_mm(), 100.0)
    assert_close(toolpath.total_motion_length_mm(), 300.0)
    assert toolpath.final_pen_state() is PenState.UP


def test_toolpath_module_does_not_turn_motion_into_binary_record():
    # Architectural regression: execution IR remains richer than / separate
    # from TRJ binary representation.
    fields = {field.name for field in dataclasses.fields(Motion)}
    assert "type" not in fields
    assert "flags" not in fields
    assert "data" not in fields
    assert "record_size" not in fields
