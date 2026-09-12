import math

import pytest

from pc_trajectory.geometry import (
    Arc,
    BoundingBox,
    CubicBezier,
    GeometryError,
    Line,
    Point2D,
    Vec2,
)


ABS = 1.0e-9


def assert_close(actual, expected, tol=ABS):
    assert math.isclose(actual, expected, rel_tol=0.0, abs_tol=tol)


def assert_point_close(actual: Point2D, expected: Point2D, tol=ABS):
    assert_close(actual.x_mm, expected.x_mm, tol)
    assert_close(actual.y_mm, expected.y_mm, tol)


def assert_vec_close(actual: Vec2, expected: Vec2, tol=ABS):
    assert_close(actual.x, expected.x, tol)
    assert_close(actual.y, expected.y, tol)


def test_point_rejects_non_finite_coordinates():
    with pytest.raises(GeometryError):
        Point2D(math.nan, 0.0)
    with pytest.raises(GeometryError):
        Point2D(0.0, math.inf)


def test_vec_normalization_dot_cross_and_angle():
    a = Vec2(3.0, 4.0)
    n = a.normalized()

    assert_close(a.norm(), 5.0)
    assert_vec_close(n, Vec2(0.6, 0.8))
    assert_close(Vec2(1.0, 0.0).dot(Vec2(0.0, 1.0)), 0.0)
    assert_close(Vec2(1.0, 0.0).cross_z(Vec2(0.0, 1.0)), 1.0)
    assert_close(Vec2(1.0, 0.0).angle_to_deg(Vec2(0.0, 1.0)), 90.0)


def test_zero_vector_cannot_be_normalized():
    with pytest.raises(GeometryError):
        Vec2(0.0, 0.0).normalized()


def test_bounding_box_union_and_dimensions():
    a = BoundingBox(0.0, 1.0, 2.0, 5.0)
    b = BoundingBox(-3.0, 2.0, 1.0, 8.0)
    merged = a.union(b)

    assert merged == BoundingBox(-3.0, 1.0, 2.0, 8.0)
    assert_close(merged.width_mm, 5.0)
    assert_close(merged.height_mm, 7.0)


def test_line_geometry_3_4_5():
    line = Line(Point2D(10.0, 20.0), Point2D(13.0, 24.0))

    assert_close(line.length_mm(), 5.0)
    assert_point_close(line.start_point(), Point2D(10.0, 20.0))
    assert_point_close(line.end_point(), Point2D(13.0, 24.0))
    assert_point_close(line.point_at(0.5), Point2D(11.5, 22.0))
    assert_vec_close(line.start_tangent(), Vec2(0.6, 0.8))
    assert_vec_close(line.end_tangent(), Vec2(0.6, 0.8))
    assert_close(line.signed_curvature_inv_mm(), 0.0)
    assert line.bounding_box() == BoundingBox(10.0, 20.0, 13.0, 24.0)


def test_line_rejects_zero_length():
    with pytest.raises(GeometryError):
        Line(Point2D(1.0, 2.0), Point2D(1.0, 2.0))


@pytest.mark.parametrize("t", [-0.001, 1.001, math.inf, math.nan])
def test_line_rejects_invalid_parameter(t):
    line = Line(Point2D(0.0, 0.0), Point2D(1.0, 0.0))
    with pytest.raises(GeometryError):
        line.point_at(t)


def test_arc_ccw_quarter_circle_matches_executor_semantics():
    arc = Arc(
        center=Point2D(0.0, 0.0),
        radius_mm=10.0,
        start_angle_deg=0.0,
        sweep_deg=90.0,
    )

    assert_point_close(arc.start_point(), Point2D(10.0, 0.0))
    assert_point_close(arc.end_point(), Point2D(0.0, 10.0))
    assert_point_close(
        arc.point_at(0.5),
        Point2D(10.0 / math.sqrt(2.0), 10.0 / math.sqrt(2.0)),
    )
    assert_close(arc.length_mm(), 5.0 * math.pi)
    assert_vec_close(arc.start_tangent(), Vec2(0.0, 1.0))
    assert_vec_close(arc.end_tangent(), Vec2(-1.0, 0.0))
    assert_close(arc.signed_curvature_inv_mm(), +0.1)
    assert arc.bounding_box() == BoundingBox(0.0, 0.0, 10.0, 10.0)


def test_arc_cw_quarter_circle_has_negative_curvature():
    arc = Arc(
        center=Point2D(0.0, 0.0),
        radius_mm=10.0,
        start_angle_deg=90.0,
        sweep_deg=-90.0,
    )

    assert_point_close(arc.start_point(), Point2D(0.0, 10.0))
    assert_point_close(arc.end_point(), Point2D(10.0, 0.0))
    assert_vec_close(arc.start_tangent(), Vec2(1.0, 0.0))
    assert_vec_close(arc.end_tangent(), Vec2(0.0, -1.0))
    assert_close(arc.signed_curvature_inv_mm(), -0.1)


def test_arc_bounding_box_handles_angle_wrap():
    arc = Arc(
        center=Point2D(0.0, 0.0),
        radius_mm=10.0,
        start_angle_deg=350.0,
        sweep_deg=20.0,
    )
    box = arc.bounding_box()

    assert_close(box.max_x_mm, 10.0)
    assert_close(box.min_x_mm, 10.0 * math.cos(math.radians(10.0)))
    assert_close(box.min_y_mm, -10.0 * math.sin(math.radians(10.0)))
    assert_close(box.max_y_mm, +10.0 * math.sin(math.radians(10.0)))


def test_full_circle_bounding_box_and_length():
    arc = Arc(
        center=Point2D(20.0, -5.0),
        radius_mm=7.0,
        start_angle_deg=33.0,
        sweep_deg=360.0,
    )

    assert_close(arc.length_mm(), 14.0 * math.pi)
    assert arc.bounding_box() == BoundingBox(13.0, -12.0, 27.0, 2.0)
    assert_point_close(arc.start_point(), arc.end_point())


@pytest.mark.parametrize(
    "radius,sweep",
    [
        (0.0, 90.0),
        (-1.0, 90.0),
        (1.0, 0.0),
        (math.inf, 90.0),
        (1.0, math.nan),
    ],
)
def test_arc_rejects_invalid_geometry(radius, sweep):
    with pytest.raises(GeometryError):
        Arc(
            center=Point2D(0.0, 0.0),
            radius_mm=radius,
            start_angle_deg=0.0,
            sweep_deg=sweep,
        )


def test_cubic_bezier_straight_line_case():
    curve = CubicBezier(
        start=Point2D(0.0, 0.0),
        control1=Point2D(1.0, 0.0),
        control2=Point2D(2.0, 0.0),
        end=Point2D(3.0, 0.0),
    )

    assert_point_close(curve.point_at(0.0), Point2D(0.0, 0.0))
    assert_point_close(curve.point_at(0.5), Point2D(1.5, 0.0))
    assert_point_close(curve.point_at(1.0), Point2D(3.0, 0.0))
    assert_vec_close(curve.start_tangent(), Vec2(1.0, 0.0))
    assert_vec_close(curve.end_tangent(), Vec2(1.0, 0.0))
    assert_close(curve.length_mm(), 3.0, tol=1.0e-8)
    assert curve.bounding_box() == BoundingBox(0.0, 0.0, 3.0, 0.0)


def test_cubic_bezier_curved_case_point_tangents_bounds_and_length():
    curve = CubicBezier(
        start=Point2D(0.0, 0.0),
        control1=Point2D(0.0, 1.0),
        control2=Point2D(1.0, 1.0),
        end=Point2D(1.0, 0.0),
    )

    assert_point_close(curve.point_at(0.5), Point2D(0.5, 0.75))
    assert_vec_close(curve.start_tangent(), Vec2(0.0, 1.0))
    assert_vec_close(curve.end_tangent(), Vec2(0.0, -1.0))

    box = curve.bounding_box()
    assert_close(box.min_x_mm, 0.0)
    assert_close(box.max_x_mm, 1.0)
    assert_close(box.min_y_mm, 0.0)
    assert_close(box.max_y_mm, 0.75)

    # This symmetric cubic has arc length exactly 2.
    assert_close(curve.length_mm(tolerance_mm=1.0e-8), 2.0, tol=2.0e-8)


def test_cubic_bezier_endpoint_tangent_fallback_handles_repeated_control_point():
    curve = CubicBezier(
        start=Point2D(0.0, 0.0),
        control1=Point2D(0.0, 0.0),
        control2=Point2D(1.0, 0.0),
        end=Point2D(2.0, 0.0),
    )

    assert_vec_close(curve.start_tangent(), Vec2(1.0, 0.0))
    assert_vec_close(curve.end_tangent(), Vec2(1.0, 0.0))


def test_cubic_bezier_rejects_curve_collapsed_to_single_point():
    p = Point2D(2.0, 3.0)
    with pytest.raises(GeometryError):
        CubicBezier(p, p, p, p)


def test_cubic_bezier_curvature_sign_is_available_for_future_analysis():
    curve = CubicBezier(
        start=Point2D(0.0, 0.0),
        control1=Point2D(0.0, 1.0),
        control2=Point2D(1.0, 1.0),
        end=Point2D(1.0, 0.0),
    )

    # At u=0.5 this curve bends clockwise, hence negative signed curvature.
    assert curve.signed_curvature_inv_mm(0.5) < 0.0


def test_golden_line_arc_line_geometry():
    """
    Geometry regression for the already validated TRJ1 Golden trajectory:

        LINE   (0,0) -> (500,0)
        ARC    center=(500,250), r=250, start=-90, sweep=+180
        LINE   (500,500) -> (0,500)

    This test intentionally uses the Phase B internal geometry model, not the
    TRJ1 binary model.
    """
    line0 = Line(Point2D(0.0, 0.0), Point2D(500.0, 0.0))
    arc = Arc(
        center=Point2D(500.0, 250.0),
        radius_mm=250.0,
        start_angle_deg=-90.0,
        sweep_deg=180.0,
    )
    line2 = Line(Point2D(500.0, 500.0), Point2D(0.0, 500.0))

    assert_close(line0.length_mm(), 500.0)
    assert_vec_close(line0.end_tangent(), Vec2(1.0, 0.0))

    assert_point_close(arc.start_point(), Point2D(500.0, 0.0))
    assert_point_close(arc.end_point(), Point2D(500.0, 500.0))
    assert_close(arc.length_mm(), 250.0 * math.pi)
    assert_vec_close(arc.start_tangent(), Vec2(1.0, 0.0))
    assert_vec_close(arc.end_tangent(), Vec2(-1.0, 0.0))
    assert_close(arc.signed_curvature_inv_mm(), 1.0 / 250.0)

    assert_close(line2.length_mm(), 500.0)
    assert_vec_close(line2.start_tangent(), Vec2(-1.0, 0.0))

    # Both Golden junctions are exactly tangent-continuous geometrically.
    assert_close(line0.end_tangent().angle_to_deg(arc.start_tangent()), 0.0)
    assert_close(arc.end_tangent().angle_to_deg(line2.start_tangent()), 0.0)

    total_length = line0.length_mm() + arc.length_mm() + line2.length_mm()
    assert_close(total_length, 1000.0 + 250.0 * math.pi)

    bounds = line0.bounding_box().union(arc.bounding_box()).union(line2.bounding_box())
    assert bounds == BoundingBox(0.0, 0.0, 750.0, 500.0)
