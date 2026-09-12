import math

import pytest

from pc_trajectory.drawing import (
    Drawing,
    DrawingError,
    PC_CONTINUITY_TOLERANCE_MM,
    Stroke,
    StrokeBuilder,
    StrokeContinuityError,
)
from pc_trajectory.geometry import (
    Arc,
    BoundingBox,
    CubicBezier,
    Line,
    Point2D,
)


ABS = 1.0e-9


def assert_close(actual, expected, tol=ABS):
    assert math.isclose(actual, expected, rel_tol=0.0, abs_tol=tol)


def assert_point_close(actual: Point2D, expected: Point2D, tol=ABS):
    assert_close(actual.x_mm, expected.x_mm, tol)
    assert_close(actual.y_mm, expected.y_mm, tol)


def golden_stroke() -> Stroke:
    line0 = Line(Point2D(0.0, 0.0), Point2D(500.0, 0.0))
    arc = Arc(
        center=Point2D(500.0, 250.0),
        radius_mm=250.0,
        start_angle_deg=-90.0,
        sweep_deg=180.0,
    )
    line2 = Line(Point2D(500.0, 500.0), Point2D(0.0, 500.0))
    return Stroke((line0, arc, line2))


def test_single_line_stroke_basic_properties():
    stroke = Stroke(
        (
            Line(
                Point2D(10.0, 20.0),
                Point2D(110.0, 20.0),
            ),
        )
    )

    assert stroke.geometry_count == 1
    assert_point_close(stroke.start_point(), Point2D(10.0, 20.0))
    assert_point_close(stroke.end_point(), Point2D(110.0, 20.0))
    assert_close(stroke.length_mm(), 100.0)
    assert stroke.bounding_box() == BoundingBox(10.0, 20.0, 110.0, 20.0)


def test_stroke_normalizes_geometry_sequence_to_tuple():
    geometries = [
        Line(Point2D(0.0, 0.0), Point2D(1.0, 0.0)),
        Line(Point2D(1.0, 0.0), Point2D(2.0, 0.0)),
    ]
    stroke = Stroke(geometries)

    assert isinstance(stroke.geometries, tuple)
    assert stroke.geometry_count == 2


def test_empty_stroke_is_rejected():
    with pytest.raises(DrawingError):
        Stroke(())


def test_stroke_rejects_unsupported_geometry_type():
    with pytest.raises(DrawingError):
        Stroke(("not geometry",))


def test_exactly_continuous_line_arc_line_stroke_passes():
    stroke = golden_stroke()
    stroke.validate_continuity()

    assert stroke.geometry_count == 3


def test_stroke_accepts_small_position_error_within_pc_tolerance():
    line0 = Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0))
    line1 = Line(
        Point2D(100.0 + 0.005, 0.0),
        Point2D(200.0, 0.0),
    )

    stroke = Stroke((line0, line1))
    stroke.validate_continuity()

    assert PC_CONTINUITY_TOLERANCE_MM == 0.01


def test_stroke_rejects_position_error_larger_than_pc_tolerance():
    line0 = Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0))
    line1 = Line(
        Point2D(100.0 + 0.02, 0.0),
        Point2D(200.0, 0.0),
    )

    with pytest.raises(StrokeContinuityError) as exc_info:
        Stroke((line0, line1))

    error = exc_info.value
    assert error.previous_index == 0
    assert error.next_index == 1
    assert_close(error.error_mm, 0.02)
    assert_close(error.tolerance_mm, 0.01)


def test_stroke_can_be_revalidated_with_stricter_tolerance():
    line0 = Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0))
    line1 = Line(
        Point2D(100.005, 0.0),
        Point2D(200.0, 0.0),
    )
    stroke = Stroke((line0, line1))

    with pytest.raises(StrokeContinuityError):
        stroke.validate_continuity(tolerance_mm=0.001)


@pytest.mark.parametrize("tolerance", [-1.0, math.nan, math.inf])
def test_invalid_continuity_tolerance_is_rejected(tolerance):
    line = Line(Point2D(0.0, 0.0), Point2D(1.0, 0.0))
    with pytest.raises(DrawingError):
        Stroke((line,), continuity_tolerance_mm=tolerance)


def test_mixed_line_arc_bezier_stroke_length_and_bounds():
    line = Line(Point2D(0.0, 0.0), Point2D(10.0, 0.0))
    arc = Arc(
        center=Point2D(10.0, 10.0),
        radius_mm=10.0,
        start_angle_deg=-90.0,
        sweep_deg=90.0,
    )
    bezier = CubicBezier(
        start=Point2D(20.0, 10.0),
        control1=Point2D(25.0, 10.0),
        control2=Point2D(25.0, 20.0),
        end=Point2D(20.0, 20.0),
    )

    stroke = Stroke((line, arc, bezier))

    assert stroke.geometry_count == 3
    assert_close(
        stroke.length_mm(),
        10.0 + 5.0 * math.pi + bezier.length_mm(),
        tol=1.0e-8,
    )

    box = stroke.bounding_box()
    assert_close(box.min_x_mm, 0.0)
    assert_close(box.min_y_mm, 0.0)
    assert box.max_x_mm > 20.0
    assert_close(box.max_y_mm, 20.0)


def test_empty_drawing_is_allowed():
    drawing = Drawing()

    assert drawing.is_empty()
    assert drawing.stroke_count == 0
    assert drawing.geometry_count == 0
    assert_close(drawing.total_drawing_length_mm(), 0.0)

    with pytest.raises(DrawingError):
        drawing.bounding_box()
    with pytest.raises(DrawingError):
        drawing.start_point()
    with pytest.raises(DrawingError):
        drawing.end_point()


def test_drawing_normalizes_strokes_to_tuple():
    stroke = Stroke(
        (Line(Point2D(0.0, 0.0), Point2D(1.0, 0.0)),)
    )
    drawing = Drawing([stroke])

    assert isinstance(drawing.strokes, tuple)
    assert drawing.stroke_count == 1


def test_drawing_rejects_non_stroke_items():
    with pytest.raises(DrawingError):
        Drawing(("not a stroke",))


def test_two_stroke_drawing_statistics_do_not_include_future_travel():
    stroke0 = Stroke(
        (Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),)
    )
    stroke1 = Stroke(
        (Line(Point2D(300.0, 50.0), Point2D(350.0, 50.0)),)
    )
    drawing = Drawing((stroke0, stroke1))

    assert not drawing.is_empty()
    assert drawing.stroke_count == 2
    assert drawing.geometry_count == 2

    # Drawing length intentionally excludes the 200+ mm pen-up gap.
    assert_close(drawing.total_drawing_length_mm(), 150.0)
    assert drawing.bounding_box() == BoundingBox(0.0, 0.0, 350.0, 50.0)
    assert_point_close(drawing.start_point(), Point2D(0.0, 0.0))
    assert_point_close(drawing.end_point(), Point2D(350.0, 50.0))


def test_stroke_builder_line_to_updates_current_point():
    builder = StrokeBuilder(Point2D(0.0, 0.0))

    returned = builder.line_to(10.0, 20.0)

    assert returned is builder
    assert builder.geometry_count == 1
    assert_point_close(builder.current_point, Point2D(10.0, 20.0))

    stroke = builder.build()
    assert stroke.geometry_count == 1
    assert_point_close(stroke.start_point(), Point2D(0.0, 0.0))
    assert_point_close(stroke.end_point(), Point2D(10.0, 20.0))


def test_stroke_builder_arc_derives_radius_and_start_angle_from_current_point():
    builder = StrokeBuilder(Point2D(500.0, 0.0))

    builder.arc(
        center=Point2D(500.0, 250.0),
        sweep_deg=180.0,
    )

    stroke = builder.build()
    arc = stroke.geometries[0]

    assert isinstance(arc, Arc)
    assert_close(arc.radius_mm, 250.0)
    assert_close(arc.start_angle_deg, -90.0)
    assert_close(arc.sweep_deg, 180.0)
    assert_point_close(arc.start_point(), Point2D(500.0, 0.0))
    assert_point_close(arc.end_point(), Point2D(500.0, 500.0))
    assert_point_close(builder.current_point, Point2D(500.0, 500.0))


def test_stroke_builder_rejects_arc_center_equal_to_current_point():
    builder = StrokeBuilder(Point2D(1.0, 2.0))

    with pytest.raises(DrawingError):
        builder.arc(
            center=Point2D(1.0, 2.0),
            sweep_deg=90.0,
        )


def test_stroke_builder_cubic_to_updates_current_point():
    builder = StrokeBuilder(Point2D(0.0, 0.0))

    builder.cubic_to(
        control1=Point2D(0.0, 10.0),
        control2=Point2D(10.0, 10.0),
        end=Point2D(10.0, 0.0),
    )

    stroke = builder.build()
    curve = stroke.geometries[0]

    assert isinstance(curve, CubicBezier)
    assert_point_close(curve.start_point(), Point2D(0.0, 0.0))
    assert_point_close(curve.end_point(), Point2D(10.0, 0.0))
    assert_point_close(builder.current_point, Point2D(10.0, 0.0))


def test_stroke_builder_add_geometry_accepts_matching_start():
    builder = StrokeBuilder(Point2D(0.0, 0.0))
    line = Line(Point2D(0.0, 0.0), Point2D(5.0, 0.0))

    builder.add_geometry(line)

    assert builder.geometry_count == 1
    assert_point_close(builder.current_point, Point2D(5.0, 0.0))


def test_stroke_builder_add_geometry_rejects_discontinuous_start():
    builder = StrokeBuilder(Point2D(0.0, 0.0))
    line = Line(Point2D(0.02, 0.0), Point2D(5.0, 0.0))

    with pytest.raises(StrokeContinuityError):
        builder.add_geometry(line)


def test_stroke_builder_does_not_silently_snap_small_start_error():
    builder = StrokeBuilder(Point2D(0.0, 0.0))
    line = Line(Point2D(0.005, 0.0), Point2D(5.0, 0.0))

    builder.add_geometry(line)
    stroke = builder.build()

    # The primitive remains exactly what the caller supplied.
    assert_point_close(stroke.geometries[0].start_point(), Point2D(0.005, 0.0))
    assert_point_close(stroke.start_point(), Point2D(0.005, 0.0))


def test_stroke_builder_rejects_empty_build():
    builder = StrokeBuilder(Point2D(0.0, 0.0))

    with pytest.raises(DrawingError):
        builder.build()


def test_golden_stroke_properties():
    stroke = golden_stroke()

    assert stroke.geometry_count == 3
    assert_point_close(stroke.start_point(), Point2D(0.0, 0.0))
    assert_point_close(stroke.end_point(), Point2D(0.0, 500.0))
    assert_close(stroke.length_mm(), 1000.0 + 250.0 * math.pi)
    assert stroke.bounding_box() == BoundingBox(0.0, 0.0, 750.0, 500.0)


def test_golden_stroke_can_be_built_with_stroke_builder():
    builder = StrokeBuilder(Point2D(0.0, 0.0))

    builder.line_to(500.0, 0.0)
    builder.arc(
        center=Point2D(500.0, 250.0),
        sweep_deg=180.0,
    )
    builder.line_to(0.0, 500.0)

    stroke = builder.build()

    assert stroke.geometry_count == 3
    assert_close(stroke.length_mm(), 1000.0 + 250.0 * math.pi)
    assert stroke.bounding_box() == BoundingBox(0.0, 0.0, 750.0, 500.0)

    assert isinstance(stroke.geometries[0], Line)
    assert isinstance(stroke.geometries[1], Arc)
    assert isinstance(stroke.geometries[2], Line)

    arc = stroke.geometries[1]
    assert_close(arc.radius_mm, 250.0)
    assert_close(arc.start_angle_deg, -90.0)
    assert_close(arc.sweep_deg, 180.0)


def test_golden_drawing_wraps_golden_stroke_without_changing_geometry():
    stroke = golden_stroke()
    drawing = Drawing((stroke,))

    assert drawing.stroke_count == 1
    assert drawing.geometry_count == 3
    assert_close(drawing.total_drawing_length_mm(), 1000.0 + 250.0 * math.pi)
    assert drawing.bounding_box() == BoundingBox(0.0, 0.0, 750.0, 500.0)
