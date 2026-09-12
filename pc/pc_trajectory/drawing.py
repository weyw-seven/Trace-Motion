"""
Phase B / B2 drawing and stroke model.

This module sits above the pure mathematical geometry layer and below
path analysis / toolpath compilation.

It answers:
    - Which geometry primitives belong to the same continuous drawing stroke?
    - Which strokes belong to the same drawing?
    - What are the drawing-only lengths and bounds?
    - How can a caller construct a continuous stroke conveniently?

It intentionally does NOT contain:
    - PEN_UP / PEN_DOWN / WAIT
    - travel planning
    - stroke ordering optimization
    - firmware 2 mm continuity acceptance
    - firmware 5 degree tangent classification
    - STOP_REQUIRED diagnostics
    - TRJ1 / TRJ2 binary serialization
    - OpenCV / SVG / CAD parsing
    - preview/UI code

Architectural rule:
    Geometry describes "what the shape is".
    Drawing/Stroke describes "which shapes form continuous drawn strokes".
    Toolpath (later) describes "how the robot executes those strokes".
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence

from .geometry import (
    Arc,
    BoundingBox,
    CubicBezier,
    Geometry,
    GeometryError,
    Line,
    Point2D,
)


# This is a PC internal quality/continuity rule, not the ESP32's 2 mm
# executor acceptance tolerance.
PC_CONTINUITY_TOLERANCE_MM = 0.01


class DrawingError(ValueError):
    """Base error for invalid Drawing/Stroke state."""


class StrokeContinuityError(DrawingError):
    """Raised when adjacent geometry inside one Stroke is not position-continuous."""

    def __init__(
        self,
        previous_index: int,
        next_index: int,
        error_mm: float,
        tolerance_mm: float,
    ) -> None:
        self.previous_index = previous_index
        self.next_index = next_index
        self.error_mm = error_mm
        self.tolerance_mm = tolerance_mm

        super().__init__(
            "Stroke geometry is discontinuous between "
            f"indices {previous_index} and {next_index}: "
            f"position error {error_mm:.9f} mm exceeds "
            f"tolerance {tolerance_mm:.9f} mm"
        )


def _validate_tolerance_mm(tolerance_mm: float) -> float:
    tolerance_mm = float(tolerance_mm)
    if not math.isfinite(tolerance_mm):
        raise DrawingError("continuity_tolerance_mm must be finite")
    if tolerance_mm < 0.0:
        raise DrawingError("continuity_tolerance_mm must be >= 0")
    return tolerance_mm


def _is_supported_geometry(value: object) -> bool:
    return isinstance(value, (Line, Arc, CubicBezier))


@dataclass(frozen=True)
class Stroke:
    """
    One continuous drawn stroke.

    All geometry primitives inside a Stroke must be position-continuous.
    A Stroke must contain at least one geometry primitive.

    Stroke boundaries are semantic:
        geometry A -> geometry B inside one Stroke
            means the pen remains down between them.

        Stroke A -> Stroke B inside one Drawing
            means a future Toolpath Compiler may insert PEN_UP + travel + PEN_DOWN.
    """

    geometries: tuple[Geometry, ...]
    continuity_tolerance_mm: float = PC_CONTINUITY_TOLERANCE_MM

    def __post_init__(self) -> None:
        geometries = tuple(self.geometries)
        tolerance = _validate_tolerance_mm(self.continuity_tolerance_mm)

        if not geometries:
            raise DrawingError("Stroke must contain at least one geometry")

        for index, geometry in enumerate(geometries):
            if not _is_supported_geometry(geometry):
                raise DrawingError(
                    f"Stroke geometry at index {index} has unsupported type "
                    f"{type(geometry).__name__}"
                )

        object.__setattr__(self, "geometries", geometries)
        object.__setattr__(self, "continuity_tolerance_mm", tolerance)

        self.validate_continuity()

    @property
    def geometry_count(self) -> int:
        return len(self.geometries)

    def start_point(self) -> Point2D:
        return self.geometries[0].start_point()

    def end_point(self) -> Point2D:
        return self.geometries[-1].end_point()

    def length_mm(self) -> float:
        return sum(geometry.length_mm() for geometry in self.geometries)

    def bounding_box(self) -> BoundingBox:
        box = self.geometries[0].bounding_box()
        for geometry in self.geometries[1:]:
            box = box.union(geometry.bounding_box())
        return box

    def validate_continuity(
        self,
        *,
        tolerance_mm: float | None = None,
    ) -> None:
        """
        Validate position continuity between all adjacent geometry primitives.

        This deliberately checks POSITION continuity only.
        Tangent continuity belongs to path_analysis.py (B3).
        """
        tolerance = (
            self.continuity_tolerance_mm
            if tolerance_mm is None
            else _validate_tolerance_mm(tolerance_mm)
        )

        for previous_index in range(len(self.geometries) - 1):
            next_index = previous_index + 1
            previous_end = self.geometries[previous_index].end_point()
            next_start = self.geometries[next_index].start_point()
            error_mm = previous_end.distance_to(next_start)

            if error_mm > tolerance:
                raise StrokeContinuityError(
                    previous_index=previous_index,
                    next_index=next_index,
                    error_mm=error_mm,
                    tolerance_mm=tolerance,
                )


@dataclass(frozen=True)
class Drawing:
    """
    A collection of independent drawing strokes in their current logical order.

    An empty Drawing is allowed during construction/import pipelines.
    An empty Stroke is not allowed.

    Drawing length is DRAWING geometry length only. It does not include future
    pen-up travel between strokes.
    """

    strokes: tuple[Stroke, ...] = ()

    def __post_init__(self) -> None:
        strokes = tuple(self.strokes)

        for index, stroke in enumerate(strokes):
            if not isinstance(stroke, Stroke):
                raise DrawingError(
                    f"Drawing stroke at index {index} has unsupported type "
                    f"{type(stroke).__name__}"
                )

        object.__setattr__(self, "strokes", strokes)

    @property
    def stroke_count(self) -> int:
        return len(self.strokes)

    @property
    def geometry_count(self) -> int:
        return sum(stroke.geometry_count for stroke in self.strokes)

    def is_empty(self) -> bool:
        return not self.strokes

    def total_drawing_length_mm(self) -> float:
        return sum(stroke.length_mm() for stroke in self.strokes)

    def bounding_box(self) -> BoundingBox:
        if self.is_empty():
            raise DrawingError("Empty Drawing has no bounding box")

        box = self.strokes[0].bounding_box()
        for stroke in self.strokes[1:]:
            box = box.union(stroke.bounding_box())
        return box

    def start_point(self) -> Point2D:
        if self.is_empty():
            raise DrawingError("Empty Drawing has no start point")
        return self.strokes[0].start_point()

    def end_point(self) -> Point2D:
        if self.is_empty():
            raise DrawingError("Empty Drawing has no end point")
        return self.strokes[-1].end_point()


class StrokeBuilder:
    """
    Convenience builder for one continuous Stroke.

    The builder owns a current point and creates explicit PC geometry.

    Example:

        builder = StrokeBuilder(Point2D(0, 0))
        builder.line_to(500, 0)
        builder.arc(center=Point2D(500, 250), sweep_deg=180)
        builder.line_to(0, 500)
        stroke = builder.build()

    No binary serialization occurs here.
    """

    def __init__(
        self,
        start: Point2D,
        *,
        continuity_tolerance_mm: float = PC_CONTINUITY_TOLERANCE_MM,
    ) -> None:
        if not isinstance(start, Point2D):
            raise DrawingError("StrokeBuilder start must be a Point2D")

        self._start = start
        self._current_point = start
        self._geometries: list[Geometry] = []
        self._continuity_tolerance_mm = _validate_tolerance_mm(
            continuity_tolerance_mm
        )

    @property
    def start_point(self) -> Point2D:
        return self._start

    @property
    def current_point(self) -> Point2D:
        return self._current_point

    @property
    def geometry_count(self) -> int:
        return len(self._geometries)

    def add_geometry(self, geometry: Geometry) -> "StrokeBuilder":
        """
        Add an explicit geometry primitive.

        Its start must match the builder's current point within the PC
        continuity tolerance. The geometry is NOT silently shifted/snapped.
        """
        if not _is_supported_geometry(geometry):
            raise DrawingError(
                f"Unsupported geometry type {type(geometry).__name__}"
            )

        error_mm = self._current_point.distance_to(geometry.start_point())
        if error_mm > self._continuity_tolerance_mm:
            raise StrokeContinuityError(
                previous_index=max(-1, len(self._geometries) - 1),
                next_index=len(self._geometries),
                error_mm=error_mm,
                tolerance_mm=self._continuity_tolerance_mm,
            )

        self._geometries.append(geometry)
        self._current_point = geometry.end_point()
        return self

    def line_to(
        self,
        x_mm: float,
        y_mm: float,
    ) -> "StrokeBuilder":
        end = Point2D(x_mm, y_mm)
        line = Line(self._current_point, end)
        self._geometries.append(line)
        self._current_point = end
        return self

    def arc(
        self,
        *,
        center: Point2D,
        sweep_deg: float,
    ) -> "StrokeBuilder":
        """
        Add an Arc that starts exactly at the current point.

        radius and start_angle are derived from:
            current_point - center

        This avoids hand-entered CIRCLE start discontinuities.
        """
        if not isinstance(center, Point2D):
            raise DrawingError("arc center must be a Point2D")

        dx = self._current_point.x_mm - center.x_mm
        dy = self._current_point.y_mm - center.y_mm

        radius_mm = math.hypot(dx, dy)
        if radius_mm <= 0.0:
            raise DrawingError(
                "Cannot create Arc: current point must differ from center"
            )

        start_angle_deg = math.degrees(math.atan2(dy, dx))

        arc = Arc(
            center=center,
            radius_mm=radius_mm,
            start_angle_deg=start_angle_deg,
            sweep_deg=sweep_deg,
        )

        # The derived start is mathematically the same current point; retain
        # the Arc's computed endpoint as the canonical new current point.
        self._geometries.append(arc)
        self._current_point = arc.end_point()
        return self

    def cubic_to(
        self,
        *,
        control1: Point2D,
        control2: Point2D,
        end: Point2D,
    ) -> "StrokeBuilder":
        if not isinstance(control1, Point2D):
            raise DrawingError("control1 must be a Point2D")
        if not isinstance(control2, Point2D):
            raise DrawingError("control2 must be a Point2D")
        if not isinstance(end, Point2D):
            raise DrawingError("end must be a Point2D")

        curve = CubicBezier(
            start=self._current_point,
            control1=control1,
            control2=control2,
            end=end,
        )
        self._geometries.append(curve)
        self._current_point = end
        return self

    def build(self) -> Stroke:
        if not self._geometries:
            raise DrawingError("Cannot build an empty Stroke")

        return Stroke(
            tuple(self._geometries),
            continuity_tolerance_mm=self._continuity_tolerance_mm,
        )


__all__ = [
    "Drawing",
    "DrawingError",
    "PC_CONTINUITY_TOLERANCE_MM",
    "Stroke",
    "StrokeBuilder",
    "StrokeContinuityError",
]
