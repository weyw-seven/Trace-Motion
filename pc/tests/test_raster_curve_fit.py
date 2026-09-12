import math

from pc_trajectory.geometry import Arc, Line, Point2D
from pc_trajectory.raster.curve_fit import fit_polyline_geometry


def _arc_points(radius: float, start_deg: float, sweep_deg: float, count: int):
    return tuple(
        Point2D(
            radius * math.cos(math.radians(start_deg + sweep_deg * index / (count - 1))),
            radius * math.sin(math.radians(start_deg + sweep_deg * index / (count - 1))),
        )
        for index in range(count)
    )


def test_quarter_circle_fits_one_executable_arc():
    geometry = fit_polyline_geometry(
        _arc_points(20.0, -30.0, 110.0, 41),
        tolerance_mm=0.01,
    )
    assert len(geometry) == 1
    assert isinstance(geometry[0], Arc)
    assert abs(geometry[0].radius_mm - 20.0) < 1.0e-9
    assert abs(geometry[0].sweep_deg - 110.0) < 1.0e-9


def test_straight_dense_polyline_fits_one_line():
    points = tuple(Point2D(index * 0.25, index * 0.5) for index in range(41))
    geometry = fit_polyline_geometry(points, tolerance_mm=0.01)
    assert geometry == (Line(points[0], points[-1]),)


def test_closed_circle_fits_continuous_circle_arcs():
    points = _arc_points(15.0, 0.0, 360.0, 73)
    geometry = fit_polyline_geometry(points, tolerance_mm=0.01, closed=True)
    assert geometry
    assert all(isinstance(item, Arc) for item in geometry)
    assert geometry[0].start_point().distance_to(geometry[-1].end_point()) < 1.0e-9
    for previous, following in zip(geometry, geometry[1:]):
        assert previous.end_point().distance_to(following.start_point()) < 1.0e-9


def test_corner_is_not_replaced_by_a_false_circle():
    points = tuple(Point2D(x, 0) for x in range(11)) + tuple(Point2D(10, y) for y in range(1, 11))
    geometry = fit_polyline_geometry(points, tolerance_mm=0.05)
    assert len(geometry) == 2
    assert all(isinstance(item, Line) for item in geometry)
