import math

import pytest

from pc_trajectory.drawing import Drawing, Stroke, StrokeBuilder
from pc_trajectory.geometry import Arc, BoundingBox, CubicBezier, Line, Point2D, Vec2
from pc_trajectory.path_analysis import (
    AnalysisError,
    DiagnosticCode,
    DiagnosticSeverity,
    FIRMWARE_CONTINUITY_TOLERANCE_MM,
    FIRMWARE_LENGTH_EPSILON_MM,
    FIRMWARE_TANGENT_CONTINUITY_DEG,
    SHORT_GEOMETRY_WARNING_MM,
    analyze_drawing,
    analyze_geometry,
    analyze_junction,
    analyze_stroke,
)


ABS = 1.0e-9


def assert_close(actual, expected, tol=ABS):
    assert math.isclose(actual, expected, rel_tol=0.0, abs_tol=tol)


def assert_vec_close(actual: Vec2, expected: Vec2, tol=ABS):
    assert_close(actual.x, expected.x, tol)
    assert_close(actual.y, expected.y, tol)


def line_from_angle(
    start: Point2D,
    angle_deg: float,
    length_mm: float = 100.0,
) -> Line:
    theta = math.radians(angle_deg)
    return Line(
        start,
        Point2D(
            start.x_mm + length_mm * math.cos(theta),
            start.y_mm + length_mm * math.sin(theta),
        ),
    )


def golden_stroke() -> Stroke:
    builder = StrokeBuilder(Point2D(0.0, 0.0))
    builder.line_to(500.0, 0.0)
    builder.arc(
        center=Point2D(500.0, 250.0),
        sweep_deg=180.0,
    )
    builder.line_to(0.0, 500.0)
    return builder.build()


def test_policy_constants_are_locked():
    assert_close(FIRMWARE_CONTINUITY_TOLERANCE_MM, 2.0)
    assert_close(FIRMWARE_TANGENT_CONTINUITY_DEG, 5.0)
    assert_close(FIRMWARE_LENGTH_EPSILON_MM, 1.0e-3)
    assert_close(SHORT_GEOMETRY_WARNING_MM, 2.0)


def test_junction_zero_degree_is_continuous():
    previous = Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0))
    next_geometry = Line(Point2D(100.0, 0.0), Point2D(200.0, 0.0))

    analysis = analyze_junction(previous, next_geometry)

    assert_close(analysis.position_error_mm, 0.0)
    assert analysis.pc_position_continuous
    assert analysis.firmware_position_continuous
    assert_vec_close(analysis.outgoing_tangent, Vec2(1.0, 0.0))
    assert_vec_close(analysis.incoming_tangent, Vec2(1.0, 0.0))
    assert_close(analysis.tangent_dot, 1.0)
    assert_close(analysis.tangent_angle_deg, 0.0)
    assert analysis.tangent_continuous
    assert not analysis.stop_required


@pytest.mark.parametrize(
    "angle_deg,expected_continuous",
    [
        (0.0, True),
        (4.0, True),
        (5.0, True),
        (5.001, False),
        (6.0, False),
        (90.0, False),
        (180.0, False),
    ],
)
def test_tangent_threshold_matches_firmware_dot_policy(
    angle_deg,
    expected_continuous,
):
    previous = Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0))
    next_geometry = line_from_angle(Point2D(100.0, 0.0), angle_deg)

    analysis = analyze_junction(previous, next_geometry)

    assert analysis.tangent_continuous is expected_continuous
    assert analysis.stop_required is (not expected_continuous)
    assert_close(analysis.tangent_angle_deg, angle_deg, tol=1.0e-9)


def test_ninety_degree_corner_requires_stop():
    previous = Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0))
    next_geometry = Line(Point2D(100.0, 0.0), Point2D(100.0, 100.0))

    analysis = analyze_junction(previous, next_geometry)

    assert_close(analysis.tangent_angle_deg, 90.0)
    assert not analysis.tangent_continuous
    assert analysis.stop_required


def test_reverse_direction_180_degree_requires_stop():
    previous = Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0))
    next_geometry = Line(Point2D(100.0, 0.0), Point2D(0.0, 0.0))

    analysis = analyze_junction(previous, next_geometry)

    assert_close(analysis.tangent_dot, -1.0)
    assert_close(analysis.tangent_angle_deg, 180.0)
    assert not analysis.tangent_continuous
    assert analysis.stop_required


def test_line_to_arc_tangent_continuity():
    line = Line(Point2D(0.0, 0.0), Point2D(500.0, 0.0))
    arc = Arc(
        center=Point2D(500.0, 250.0),
        radius_mm=250.0,
        start_angle_deg=-90.0,
        sweep_deg=180.0,
    )

    analysis = analyze_junction(line, arc)

    assert_close(analysis.tangent_angle_deg, 0.0)
    assert analysis.tangent_continuous
    assert not analysis.stop_required


def test_arc_to_line_tangent_continuity():
    arc = Arc(
        center=Point2D(500.0, 250.0),
        radius_mm=250.0,
        start_angle_deg=-90.0,
        sweep_deg=180.0,
    )
    line = Line(Point2D(500.0, 500.0), Point2D(0.0, 500.0))

    analysis = analyze_junction(arc, line)

    assert_close(analysis.tangent_angle_deg, 0.0)
    assert analysis.tangent_continuous


def test_arc_to_arc_tangent_continuity():
    arc0 = Arc(
        center=Point2D(0.0, 10.0),
        radius_mm=10.0,
        start_angle_deg=-90.0,
        sweep_deg=90.0,
    )
    # arc0 ends at (10,10) with upward tangent.
    arc1 = Arc(
        center=Point2D(0.0, 10.0),
        radius_mm=10.0,
        start_angle_deg=0.0,
        sweep_deg=90.0,
    )

    analysis = analyze_junction(arc0, arc1)

    assert_close(analysis.position_error_mm, 0.0)
    assert_close(analysis.tangent_angle_deg, 0.0)
    assert analysis.tangent_continuous


def test_line_to_bezier_tangent_continuity():
    line = Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0))
    curve = CubicBezier(
        start=Point2D(100.0, 0.0),
        control1=Point2D(150.0, 0.0),
        control2=Point2D(200.0, 50.0),
        end=Point2D(200.0, 100.0),
    )

    analysis = analyze_junction(line, curve)

    assert_close(analysis.tangent_angle_deg, 0.0)
    assert analysis.tangent_continuous


def test_bezier_to_line_tangent_continuity():
    curve = CubicBezier(
        start=Point2D(0.0, 0.0),
        control1=Point2D(0.0, 50.0),
        control2=Point2D(50.0, 100.0),
        end=Point2D(100.0, 100.0),
    )
    line = Line(Point2D(100.0, 100.0), Point2D(150.0, 100.0))

    analysis = analyze_junction(curve, line)

    assert_close(analysis.tangent_angle_deg, 0.0)
    assert analysis.tangent_continuous


def test_small_position_error_is_pc_and_firmware_continuous():
    previous = Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0))
    next_geometry = Line(Point2D(100.005, 0.0), Point2D(200.0, 0.0))

    analysis = analyze_junction(previous, next_geometry)

    assert_close(analysis.position_error_mm, 0.005)
    assert analysis.pc_position_continuous
    assert analysis.firmware_position_continuous


def test_position_error_can_fail_pc_quality_but_pass_firmware_acceptance():
    previous = Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0))
    next_geometry = Line(Point2D(100.5, 0.0), Point2D(200.0, 0.0))

    analysis = analyze_junction(previous, next_geometry)

    assert not analysis.pc_position_continuous
    assert analysis.firmware_position_continuous
    assert analysis.tangent_continuous
    assert not analysis.stop_required


def test_position_error_beyond_firmware_tolerance_is_execution_error_not_stop_case():
    previous = Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0))
    next_geometry = Line(Point2D(103.0, 0.0), Point2D(200.0, 0.0))

    analysis = analyze_junction(previous, next_geometry)

    assert not analysis.pc_position_continuous
    assert not analysis.firmware_position_continuous
    assert analysis.tangent_continuous
    assert not analysis.stop_required


@pytest.mark.parametrize(
    "name,value",
    [
        ("pc", -1.0),
        ("firmware", -1.0),
        ("tangent", -1.0),
        ("tangent", 181.0),
        ("pc", math.nan),
    ],
)
def test_analyze_junction_rejects_invalid_thresholds(name, value):
    previous = Line(Point2D(0.0, 0.0), Point2D(1.0, 0.0))
    next_geometry = Line(Point2D(1.0, 0.0), Point2D(2.0, 0.0))

    kwargs = {}
    if name == "pc":
        kwargs["pc_continuity_tolerance_mm"] = value
    elif name == "firmware":
        kwargs["firmware_continuity_tolerance_mm"] = value
    else:
        kwargs["tangent_tolerance_deg"] = value

    with pytest.raises(AnalysisError):
        analyze_junction(previous, next_geometry, **kwargs)


def test_geometry_analysis_normal_length_is_clean():
    line = Line(Point2D(0.0, 0.0), Point2D(10.0, 0.0))

    analysis = analyze_geometry(line)

    assert_close(analysis.length_mm, 10.0)
    assert analysis.firmware_length_valid
    assert not analysis.short_geometry
    assert analysis.bounding_box == BoundingBox(0.0, 0.0, 10.0, 0.0)


def test_geometry_just_below_short_warning_threshold_is_warning_candidate():
    line = Line(Point2D(0.0, 0.0), Point2D(1.999, 0.0))

    analysis = analyze_geometry(line)

    assert analysis.firmware_length_valid
    assert analysis.short_geometry


def test_geometry_exactly_at_short_warning_threshold_is_not_short():
    line = Line(Point2D(0.0, 0.0), Point2D(2.0, 0.0))

    analysis = analyze_geometry(line)

    assert not analysis.short_geometry


def test_geometry_at_firmware_length_epsilon_is_invalid_for_executor():
    line = Line(
        Point2D(0.0, 0.0),
        Point2D(FIRMWARE_LENGTH_EPSILON_MM, 0.0),
    )

    analysis = analyze_geometry(line)

    assert not analysis.firmware_length_valid
    assert not analysis.short_geometry


def test_geometry_above_firmware_epsilon_but_below_short_threshold_is_warning_only():
    line = Line(Point2D(0.0, 0.0), Point2D(0.01, 0.0))

    analysis = analyze_geometry(line)

    assert analysis.firmware_length_valid
    assert analysis.short_geometry


def test_single_geometry_stroke_has_no_junctions():
    stroke = Stroke(
        (Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),)
    )

    analysis = analyze_stroke(stroke)

    assert analysis.geometry_count == 1
    assert analysis.junction_count == 0
    assert analysis.junctions == ()
    assert analysis.tangent_continuous_count == 0
    assert analysis.stop_required_count == 0
    assert analysis.worst_junction_angle_deg is None
    assert analysis.error_count == 0
    assert analysis.warning_count == 0
    assert analysis.passes_execution_policy


def test_golden_stroke_analysis():
    analysis = analyze_stroke(golden_stroke())

    assert analysis.geometry_count == 3
    assert analysis.junction_count == 2
    assert_close(analysis.total_length_mm, 1000.0 + 250.0 * math.pi)
    assert analysis.bounding_box == BoundingBox(0.0, 0.0, 750.0, 500.0)

    assert analysis.tangent_continuous_count == 2
    assert analysis.stop_required_count == 0
    assert analysis.short_geometry_count == 0
    assert analysis.firmware_too_short_count == 0
    assert_close(analysis.worst_junction_angle_deg, 0.0)

    assert analysis.error_count == 0
    assert analysis.warning_count == 0
    assert analysis.passes_execution_policy


def test_ninety_degree_stroke_emits_stop_required_warning():
    stroke = Stroke(
        (
            Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),
            Line(Point2D(100.0, 0.0), Point2D(100.0, 100.0)),
        )
    )

    analysis = analyze_stroke(stroke)

    assert analysis.stop_required_count == 1
    assert_close(analysis.worst_junction_angle_deg, 90.0)
    assert analysis.error_count == 0
    assert analysis.warning_count == 1

    diagnostic = analysis.diagnostics[0]
    assert diagnostic.severity is DiagnosticSeverity.WARNING
    assert diagnostic.code is DiagnosticCode.STOP_REQUIRED
    assert diagnostic.junction_index == 0


def test_short_geometry_emits_warning_not_error():
    stroke = Stroke(
        (Line(Point2D(0.0, 0.0), Point2D(1.0, 0.0)),)
    )

    analysis = analyze_stroke(stroke)

    assert analysis.short_geometry_count == 1
    assert analysis.firmware_too_short_count == 0
    assert analysis.error_count == 0
    assert analysis.warning_count == 1
    assert analysis.passes_execution_policy
    assert analysis.diagnostics[0].code is DiagnosticCode.SHORT_GEOMETRY


def test_firmware_too_short_geometry_emits_error():
    stroke = Stroke(
        (
            Line(
                Point2D(0.0, 0.0),
                Point2D(FIRMWARE_LENGTH_EPSILON_MM, 0.0),
            ),
        )
    )

    analysis = analyze_stroke(stroke)

    assert analysis.short_geometry_count == 0
    assert analysis.firmware_too_short_count == 1
    assert analysis.error_count == 1
    assert analysis.warning_count == 0
    assert not analysis.passes_execution_policy
    assert (
        analysis.diagnostics[0].code
        is DiagnosticCode.FIRMWARE_GEOMETRY_TOO_SHORT
    )


def test_loose_stroke_continuity_can_surface_pc_quality_warning():
    stroke = Stroke(
        (
            Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),
            Line(Point2D(100.5, 0.0), Point2D(200.0, 0.0)),
        ),
        continuity_tolerance_mm=1.0,
    )

    analysis = analyze_stroke(stroke)

    assert analysis.error_count == 0
    assert analysis.warning_count == 1
    assert analysis.passes_execution_policy
    assert (
        analysis.diagnostics[0].code
        is DiagnosticCode.PC_POSITION_TOLERANCE_EXCEEDED
    )


def test_very_loose_stroke_continuity_surfaces_firmware_discontinuity_error():
    stroke = Stroke(
        (
            Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),
            Line(Point2D(103.0, 0.0), Point2D(200.0, 0.0)),
        ),
        continuity_tolerance_mm=5.0,
    )

    analysis = analyze_stroke(stroke)

    assert analysis.error_count == 1
    assert not analysis.passes_execution_policy
    assert (
        analysis.diagnostics[0].code
        is DiagnosticCode.FIRMWARE_POSITION_DISCONTINUITY
    )


def test_drawing_analysis_empty_drawing_is_valid():
    analysis = analyze_drawing(Drawing())

    assert analysis.stroke_count == 0
    assert analysis.geometry_count == 0
    assert analysis.total_junction_count == 0
    assert_close(analysis.total_drawing_length_mm, 0.0)
    assert analysis.bounding_box is None
    assert analysis.worst_junction_angle_deg is None
    assert analysis.error_count == 0
    assert analysis.warning_count == 0
    assert analysis.passes_execution_policy


def test_drawing_analysis_aggregates_multiple_strokes_without_cross_stroke_junction():
    smooth = golden_stroke()
    corner = Stroke(
        (
            Line(Point2D(1000.0, 0.0), Point2D(1100.0, 0.0)),
            Line(Point2D(1100.0, 0.0), Point2D(1100.0, 100.0)),
        )
    )
    drawing = Drawing((smooth, corner))

    analysis = analyze_drawing(drawing)

    assert analysis.stroke_count == 2
    assert analysis.geometry_count == 5
    # 2 junctions in Golden + 1 inside corner Stroke; there is NO junction
    # between the two independent Strokes.
    assert analysis.total_junction_count == 3
    assert analysis.total_tangent_continuous_count == 2
    assert analysis.total_stop_required_count == 1
    assert_close(analysis.worst_junction_angle_deg, 90.0)
    assert analysis.warning_count == 1
    assert analysis.error_count == 0


def test_drawing_diagnostics_are_annotated_with_stroke_index():
    stroke0 = Stroke(
        (Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),)
    )
    stroke1 = Stroke(
        (
            Line(Point2D(200.0, 0.0), Point2D(201.0, 0.0)),
        )
    )
    analysis = analyze_drawing(Drawing((stroke0, stroke1)))

    assert analysis.warning_count == 1
    diagnostic = analysis.diagnostics[0]
    assert diagnostic.code is DiagnosticCode.SHORT_GEOMETRY
    assert diagnostic.stroke_index == 1
    assert diagnostic.geometry_index == 0


def test_drawing_analysis_preserves_drawing_only_length():
    stroke0 = Stroke(
        (Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),)
    )
    stroke1 = Stroke(
        (Line(Point2D(1000.0, 0.0), Point2D(1050.0, 0.0)),)
    )

    analysis = analyze_drawing(Drawing((stroke0, stroke1)))

    # The 900 mm inter-stroke gap is future Travel, not drawing length.
    assert_close(analysis.total_drawing_length_mm, 150.0)


def test_custom_tangent_tolerance_can_be_used_without_changing_geometry():
    previous = Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0))
    next_geometry = line_from_angle(Point2D(100.0, 0.0), 8.0)

    default = analyze_junction(previous, next_geometry)
    custom = analyze_junction(
        previous,
        next_geometry,
        tangent_tolerance_deg=10.0,
    )

    assert not default.tangent_continuous
    assert custom.tangent_continuous


def test_analysis_does_not_predict_exact_junction_speed():
    analysis = analyze_stroke(
        Stroke(
            (
                Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),
                Line(Point2D(100.0, 0.0), Point2D(200.0, 0.0)),
            )
        )
    )

    # Architectural regression: B3 exposes classification, not a duplicated
    # executor motion profile / predicted numerical junction speed.
    assert not hasattr(analysis.junctions[0], "junction_speed_mm_s")
    assert not hasattr(analysis.junctions[0], "duration_s")
