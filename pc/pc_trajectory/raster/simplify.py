"""Deterministic Ramer-Douglas-Peucker simplification for WORLD polylines."""

from __future__ import annotations

import math

from ..geometry import Point2D
from .config import RasterError


def _distance_to_segment(point: Point2D, start: Point2D, end: Point2D) -> float:
    dx = end.x_mm - start.x_mm
    dy = end.y_mm - start.y_mm
    length_sq = dx * dx + dy * dy
    if length_sq <= 1.0e-24:
        return point.distance_to(start)
    t = ((point.x_mm - start.x_mm) * dx + (point.y_mm - start.y_mm) * dy) / length_sq
    t = max(0.0, min(1.0, t))
    projection = Point2D(start.x_mm + t * dx, start.y_mm + t * dy)
    return point.distance_to(projection)


def _rdp(points: tuple[Point2D, ...], tolerance_mm: float) -> tuple[Point2D, ...]:
    if len(points) <= 2:
        return points
    keep = {0, len(points) - 1}
    stack = [(0, len(points) - 1)]
    while stack:
        first, last = stack.pop()
        start, end = points[first], points[last]
        farthest_index = -1
        farthest_distance = tolerance_mm
        for index in range(first + 1, last):
            distance = _distance_to_segment(points[index], start, end)
            if distance > farthest_distance:
                farthest_index = index
                farthest_distance = distance
        if farthest_index >= 0:
            keep.add(farthest_index)
            stack.append((first, farthest_index))
            stack.append((farthest_index, last))
    return tuple(points[index] for index in sorted(keep))


def smooth_polyline(
    points,
    *,
    iterations: int = 2,
    strength: float = 0.35,
    closed: bool = False,
) -> tuple[Point2D, ...]:
    """Apply endpoint-preserving Laplacian smoothing to a WORLD polyline.

    Each pass moves an interior point toward the midpoint of its two
    neighbours.  It removes one-pixel stair-step jitter without inventing
    points outside the local segment.  Open-path endpoints and closed-path
    closure are fixed exactly, which keeps junction connections continuous.
    """
    if type(iterations) is not int or iterations < 0:
        raise RasterError("iterations must be a non-negative integer")
    if isinstance(strength, bool):
        raise RasterError("strength must be in [0, 1]")
    try:
        amount = float(strength)
    except (TypeError, ValueError, OverflowError) as exc:
        raise RasterError("strength must be in [0, 1]") from exc
    if not math.isfinite(amount) or not 0.0 <= amount <= 1.0:
        raise RasterError("strength must be in [0, 1]")
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
    repeated = closed and len(compact) > 1 and compact[0] == compact[-1]
    if repeated:
        compact.pop()
    if len(compact) < (3 if closed else 2):
        raise RasterError("polyline has too few distinct points")
    if iterations == 0 or amount == 0.0:
        return tuple(compact + ([compact[0]] if closed else []))

    current = compact
    for _ in range(iterations):
        updated: list[Point2D] = []
        count = len(current)
        for index, point in enumerate(current):
            if not closed and index in (0, count - 1):
                updated.append(point)
                continue
            previous = current[(index - 1) % count]
            following = current[(index + 1) % count]
            midpoint = Point2D(
                (previous.x_mm + following.x_mm) * 0.5,
                (previous.y_mm + following.y_mm) * 0.5,
            )
            updated.append(Point2D(
                point.x_mm + amount * (midpoint.x_mm - point.x_mm),
                point.y_mm + amount * (midpoint.y_mm - point.y_mm),
            ))
        current = updated
    return tuple(current + ([current[0]] if closed else []))


def simplify_polyline(
    points,
    *,
    tolerance_mm: float = 0.2,
    closed: bool = False,
) -> tuple[Point2D, ...]:
    """Simplify a polyline while retaining its endpoints.

    For a closed path the input may contain a repeated first point.  The
    returned closed path also contains that repeated point.  A zero tolerance
    is accepted and only removes consecutive duplicate points.
    """
    if isinstance(tolerance_mm, bool):
        raise RasterError("tolerance_mm must be non-negative and finite")
    try:
        tolerance = float(tolerance_mm)
    except (TypeError, ValueError, OverflowError) as exc:
        raise RasterError("tolerance_mm must be non-negative and finite") from exc
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise RasterError("tolerance_mm must be non-negative and finite")
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
    if not closed:
        return _rdp(tuple(compact), tolerance)

    # Preserve the seam and simplify the cyclic path by using the farthest
    # point from the first point as a stable open-polyline cut.
    seam = compact[0]
    cut = max(range(1, len(compact)), key=lambda index: (seam.distance_to(compact[index]), -index))
    rotated = tuple(compact[cut:] + compact[:cut] + [compact[cut]])
    simplified = _rdp(rotated, tolerance)
    if simplified[0] != simplified[-1]:
        simplified = simplified + (simplified[0],)
    if len(simplified) < 4:
        # A triangle is the minimum useful closed polyline.
        simplified = tuple(rotated)
    return simplified


__all__ = ["simplify_polyline", "smooth_polyline"]
