"""Fit smoothed WORLD polylines with executable LINE and CIRCLE geometry."""

from __future__ import annotations

import math

from ..geometry import Arc, Geometry, Line, Point2D
from .config import RasterError


def _distance_to_segment(point: Point2D, start: Point2D, end: Point2D) -> float:
    dx = end.x_mm - start.x_mm
    dy = end.y_mm - start.y_mm
    denominator = dx * dx + dy * dy
    if denominator <= 1.0e-24:
        return point.distance_to(start)
    t = ((point.x_mm - start.x_mm) * dx + (point.y_mm - start.y_mm) * dy) / denominator
    t = max(0.0, min(1.0, t))
    return math.hypot(
        point.x_mm - (start.x_mm + t * dx),
        point.y_mm - (start.y_mm + t * dy),
    )


def _line_error(points: tuple[Point2D, ...], first: int, last: int) -> tuple[float, int]:
    start, end = points[first], points[last]
    worst_error = -1.0
    worst_index = (first + last) // 2
    for index in range(first + 1, last):
        error = _distance_to_segment(points[index], start, end)
        if error > worst_error:
            worst_error = error
            worst_index = index
    return max(0.0, worst_error), worst_index


def _circle_through(a: Point2D, b: Point2D, c: Point2D) -> tuple[Point2D, float] | None:
    determinant = 2.0 * (
        a.x_mm * (b.y_mm - c.y_mm)
        + b.x_mm * (c.y_mm - a.y_mm)
        + c.x_mm * (a.y_mm - b.y_mm)
    )
    scale = max(a.distance_to(b), b.distance_to(c), a.distance_to(c), 1.0)
    if abs(determinant) <= 1.0e-12 * scale * scale:
        return None
    aa = a.x_mm * a.x_mm + a.y_mm * a.y_mm
    bb = b.x_mm * b.x_mm + b.y_mm * b.y_mm
    cc = c.x_mm * c.x_mm + c.y_mm * c.y_mm
    center = Point2D(
        (aa * (b.y_mm - c.y_mm) + bb * (c.y_mm - a.y_mm) + cc * (a.y_mm - b.y_mm)) / determinant,
        (aa * (c.x_mm - b.x_mm) + bb * (a.x_mm - c.x_mm) + cc * (b.x_mm - a.x_mm)) / determinant,
    )
    return center, center.distance_to(a)


def _unwrap_angles(points: tuple[Point2D, ...], center: Point2D) -> tuple[float, ...]:
    raw = tuple(math.atan2(point.y_mm - center.y_mm, point.x_mm - center.x_mm) for point in points)
    unwrapped = [raw[0]]
    for previous, current in zip(raw, raw[1:]):
        delta = (current - previous + math.pi) % (2.0 * math.pi) - math.pi
        unwrapped.append(unwrapped[-1] + delta)
    return tuple(unwrapped)


def _try_arc(
    points: tuple[Point2D, ...],
    first: int,
    last: int,
    *,
    tolerance_mm: float,
    min_arc_points: int,
    min_arc_sweep_deg: float,
    min_arc_radius_mm: float,
) -> Arc | None:
    if last - first + 1 < min_arc_points:
        return None
    _, pivot = _line_error(points, first, last)
    if pivot <= first or pivot >= last:
        return None
    fitted = _circle_through(points[first], points[pivot], points[last])
    if fitted is None:
        return None
    center, radius = fitted
    if not math.isfinite(radius) or radius < min_arc_radius_mm:
        return None
    segment = points[first:last + 1]
    radial_errors = tuple(abs(center.distance_to(point) - radius) for point in segment)
    if max(radial_errors, default=0.0) > tolerance_mm:
        return None
    angles = _unwrap_angles(segment, center)
    total = angles[-1] - angles[0]
    sweep_deg = math.degrees(total)
    if abs(sweep_deg) < min_arc_sweep_deg or abs(sweep_deg) > 300.0:
        return None
    direction = 1.0 if total > 0.0 else -1.0
    angular_slack = max(1.0e-6, math.atan2(tolerance_mm, radius) * 2.0)
    if any(direction * (following - previous) < -angular_slack
           for previous, following in zip(angles, angles[1:])):
        return None
    start_angle_deg = math.degrees(math.atan2(
        points[first].y_mm - center.y_mm,
        points[first].x_mm - center.x_mm,
    ))
    return Arc(center, radius, start_angle_deg, sweep_deg)


def _fit_open(
    points: tuple[Point2D, ...],
    *,
    tolerance_mm: float,
    min_arc_points: int,
    min_arc_sweep_deg: float,
    min_arc_radius_mm: float,
) -> tuple[Geometry, ...]:
    pending = [(0, len(points) - 1)]
    output: list[Geometry] = []
    while pending:
        first, last = pending.pop()
        if last == first + 1:
            output.append(Line(points[first], points[last]))
            continue
        line_error, split = _line_error(points, first, last)
        if line_error <= tolerance_mm:
            output.append(Line(points[first], points[last]))
            continue
        arc = _try_arc(
            points,
            first,
            last,
            tolerance_mm=tolerance_mm,
            min_arc_points=min_arc_points,
            min_arc_sweep_deg=min_arc_sweep_deg,
            min_arc_radius_mm=min_arc_radius_mm,
        )
        if arc is not None:
            output.append(arc)
            continue
        if split <= first or split >= last:
            split = (first + last) // 2
        # LIFO: process the left interval first and preserve path order.
        pending.append((split, last))
        pending.append((first, split))
    return tuple(output)


def fit_polyline_geometry(
    points,
    *,
    tolerance_mm: float = 0.35,
    closed: bool = False,
    min_arc_points: int = 6,
    min_arc_sweep_deg: float = 12.0,
    min_arc_radius_mm: float = 1.0,
) -> tuple[Geometry, ...]:
    """Approximate a dense polyline with ordered LINE/CIRCLE primitives.

    Every primitive begins and ends on an original polyline point.  This keeps
    geometry and encoded TRJ2 continuity exact while bounding the transverse
    fitting error by ``tolerance_mm``.
    """
    try:
        source = tuple(points)
    except TypeError as exc:
        raise RasterError("points must be an iterable of Point2D") from exc
    if any(not isinstance(point, Point2D) for point in source):
        raise RasterError("points must contain Point2D values")
    compact: list[Point2D] = []
    for point in source:
        if not compact or point != compact[-1]:
            compact.append(point)
    if closed and len(compact) > 1 and compact[0] == compact[-1]:
        compact.pop()
    if len(compact) < (3 if closed else 2):
        raise RasterError("polyline has too few distinct points")
    if isinstance(tolerance_mm, bool):
        raise RasterError("tolerance_mm must be positive and finite")
    try:
        tolerance = float(tolerance_mm)
        min_sweep = float(min_arc_sweep_deg)
        min_radius = float(min_arc_radius_mm)
    except (TypeError, ValueError, OverflowError) as exc:
        raise RasterError("curve fitting parameters must be finite numbers") from exc
    if not math.isfinite(tolerance) or tolerance <= 0.0:
        raise RasterError("tolerance_mm must be positive and finite")
    if type(min_arc_points) is not int or min_arc_points < 3:
        raise RasterError("min_arc_points must be an integer >= 3")
    if not math.isfinite(min_sweep) or not 0.0 < min_sweep <= 180.0:
        raise RasterError("min_arc_sweep_deg must be in (0, 180]")
    if not math.isfinite(min_radius) or min_radius <= 0.0:
        raise RasterError("min_arc_radius_mm must be positive and finite")
    kwargs = dict(
        tolerance_mm=tolerance,
        min_arc_points=min_arc_points,
        min_arc_sweep_deg=min_sweep,
        min_arc_radius_mm=min_radius,
    )
    if not closed:
        return _fit_open(tuple(compact), **kwargs)
    seam = compact[0]
    cut = max(range(1, len(compact)), key=lambda index: (seam.distance_to(compact[index]), -index))
    first_half = tuple(compact[:cut + 1])
    second_half = tuple(compact[cut:] + [compact[0]])
    return _fit_open(first_half, **kwargs) + _fit_open(second_half, **kwargs)


__all__ = ["fit_polyline_geometry"]
