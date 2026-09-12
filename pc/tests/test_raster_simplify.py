import pytest

from pc_trajectory.geometry import Point2D
from pc_trajectory.raster.simplify import simplify_polyline, smooth_polyline


def test_simplify_keeps_endpoints_and_removes_collinear_points():
    points = tuple(Point2D(x, 0) for x in range(6))
    simplified = simplify_polyline(points, tolerance_mm=0.01)
    assert simplified == (Point2D(0, 0), Point2D(5, 0))


def test_simplify_preserves_a_closed_loop():
    points = (Point2D(0, 0), Point2D(4, 0), Point2D(4, 4), Point2D(0, 4), Point2D(0, 0))
    simplified = simplify_polyline(points, tolerance_mm=0.1, closed=True)
    assert simplified[0] == simplified[-1]
    assert len(simplified) >= 4


def test_simplify_rejects_invalid_points():
    with pytest.raises(ValueError):
        simplify_polyline((Point2D(0, 0),), tolerance_mm=0.1)


def test_smoothing_preserves_open_endpoints_and_changes_pixel_jitter():
    points = (
        Point2D(0, 0), Point2D(1, 1), Point2D(2, 0),
        Point2D(3, 1), Point2D(4, 0),
    )
    smoothed = smooth_polyline(points, iterations=2, strength=0.5)
    assert smoothed[0] == points[0]
    assert smoothed[-1] == points[-1]
    assert any(point.y_mm != original.y_mm for point, original in zip(smoothed[1:-1], points[1:-1]))


def test_smoothing_preserves_closed_seam():
    points = (Point2D(0, 0), Point2D(4, 0), Point2D(4, 4), Point2D(0, 4), Point2D(0, 0))
    smoothed = smooth_polyline(points, iterations=1, strength=0.5, closed=True)
    assert smoothed[0] == smoothed[-1]
