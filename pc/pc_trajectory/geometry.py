"""
Phase B geometry core for the PC trajectory compiler.

This module is intentionally independent from:
- TRJ1/TRJ2 binary serialization
- ESP32 decoder/executor APIs
- OpenCV / matplotlib
- Drawing / Stroke / Toolpath models

It represents mathematical 2-D geometry in WORLD millimetres.

Coordinate convention used by the project:
    +X : WORLD +X
    +Y : WORLD +Y
    positive angle / positive arc sweep : counter-clockwise (CCW)

Important architectural rule:
    PC internal geometry != .traj binary records.

For example, a PC Line stores both start and end points even though a TRJ1/TRJ2
LINE record stores only its endpoint. Exporters are responsible for lowering
the internal geometry into a target trajectory format.

Firmware policy thresholds such as:
- 2 mm continuity tolerance
- 5 degree tangent tolerance
- 1e-3 mm executor length epsilon

do NOT belong in this module. They are execution/diagnostic policy and should
be applied later by path_analysis/export validation. This core only rejects
non-finite and mathematically degenerate geometry.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Union


NUMERIC_EPSILON = 1.0e-12
DEFAULT_BEZIER_LENGTH_TOLERANCE_MM = 1.0e-6
DEFAULT_BEZIER_LENGTH_MAX_DEPTH = 24


class GeometryError(ValueError):
    """Raised when geometry is invalid or a requested geometric quantity is undefined."""


def _require_finite(name: str, value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise GeometryError(f"{name} must be finite, got {value!r}")
    return value


def _validate_unit_parameter(name: str, value: float) -> float:
    value = _require_finite(name, value)
    if value < 0.0 or value > 1.0:
        raise GeometryError(f"{name} must be in [0, 1], got {value!r}")
    return value


@dataclass(frozen=True)
class Vec2:
    """Dimensionless/directional 2-D vector."""

    x: float
    y: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "x", _require_finite("Vec2.x", self.x))
        object.__setattr__(self, "y", _require_finite("Vec2.y", self.y))

    def norm(self) -> float:
        return math.hypot(self.x, self.y)

    def normalized(self) -> "Vec2":
        magnitude = self.norm()
        if magnitude <= NUMERIC_EPSILON:
            raise GeometryError("Cannot normalize a near-zero vector")
        return Vec2(self.x / magnitude, self.y / magnitude)

    def dot(self, other: "Vec2") -> float:
        return self.x * other.x + self.y * other.y

    def cross_z(self, other: "Vec2") -> float:
        """Return the scalar Z component of the 2-D cross product."""
        return self.x * other.y - self.y * other.x

    def scaled(self, scalar: float) -> "Vec2":
        scalar = _require_finite("scalar", scalar)
        return Vec2(self.x * scalar, self.y * scalar)

    def angle_to_deg(self, other: "Vec2") -> float:
        """Return the unsigned angle to another vector in degrees, in [0, 180]."""
        a = self.normalized()
        b = other.normalized()
        dot = max(-1.0, min(1.0, a.dot(b)))
        return math.degrees(math.acos(dot))

    def __neg__(self) -> "Vec2":
        return Vec2(-self.x, -self.y)

    def __add__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> "Vec2":
        return self.scaled(scalar)

    def __rmul__(self, scalar: float) -> "Vec2":
        return self.scaled(scalar)


@dataclass(frozen=True)
class Point2D:
    """A 2-D point in WORLD millimetres."""

    x_mm: float
    y_mm: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "x_mm", _require_finite("Point2D.x_mm", self.x_mm))
        object.__setattr__(self, "y_mm", _require_finite("Point2D.y_mm", self.y_mm))

    def vector_to(self, other: "Point2D") -> Vec2:
        return Vec2(other.x_mm - self.x_mm, other.y_mm - self.y_mm)

    def distance_to(self, other: "Point2D") -> float:
        return math.hypot(other.x_mm - self.x_mm, other.y_mm - self.y_mm)

    def translated(self, vector: Vec2) -> "Point2D":
        return Point2D(self.x_mm + vector.x, self.y_mm + vector.y)


@dataclass(frozen=True)
class BoundingBox:
    """Axis-aligned WORLD-frame bounding box in millimetres."""

    min_x_mm: float
    min_y_mm: float
    max_x_mm: float
    max_y_mm: float

    def __post_init__(self) -> None:
        min_x = _require_finite("BoundingBox.min_x_mm", self.min_x_mm)
        min_y = _require_finite("BoundingBox.min_y_mm", self.min_y_mm)
        max_x = _require_finite("BoundingBox.max_x_mm", self.max_x_mm)
        max_y = _require_finite("BoundingBox.max_y_mm", self.max_y_mm)

        if min_x > max_x:
            raise GeometryError("BoundingBox min_x_mm cannot exceed max_x_mm")
        if min_y > max_y:
            raise GeometryError("BoundingBox min_y_mm cannot exceed max_y_mm")

        object.__setattr__(self, "min_x_mm", min_x)
        object.__setattr__(self, "min_y_mm", min_y)
        object.__setattr__(self, "max_x_mm", max_x)
        object.__setattr__(self, "max_y_mm", max_y)

    @classmethod
    def from_points(cls, points: Iterable[Point2D]) -> "BoundingBox":
        pts = tuple(points)
        if not pts:
            raise GeometryError("BoundingBox.from_points() requires at least one point")
        return cls(
            min(p.x_mm for p in pts),
            min(p.y_mm for p in pts),
            max(p.x_mm for p in pts),
            max(p.y_mm for p in pts),
        )

    @property
    def width_mm(self) -> float:
        return self.max_x_mm - self.min_x_mm

    @property
    def height_mm(self) -> float:
        return self.max_y_mm - self.min_y_mm

    def union(self, other: "BoundingBox") -> "BoundingBox":
        return BoundingBox(
            min(self.min_x_mm, other.min_x_mm),
            min(self.min_y_mm, other.min_y_mm),
            max(self.max_x_mm, other.max_x_mm),
            max(self.max_y_mm, other.max_y_mm),
        )

    def contains(self, point: Point2D, *, tolerance_mm: float = 0.0) -> bool:
        tolerance_mm = _require_finite("tolerance_mm", tolerance_mm)
        if tolerance_mm < 0.0:
            raise GeometryError("tolerance_mm must be >= 0")
        return (
            self.min_x_mm - tolerance_mm <= point.x_mm <= self.max_x_mm + tolerance_mm
            and self.min_y_mm - tolerance_mm <= point.y_mm <= self.max_y_mm + tolerance_mm
        )


@dataclass(frozen=True)
class Line:
    """Finite straight line segment in WORLD millimetres."""

    start: Point2D
    end: Point2D

    def __post_init__(self) -> None:
        if self.start.distance_to(self.end) <= NUMERIC_EPSILON:
            raise GeometryError("Line must have non-zero length")

    def start_point(self) -> Point2D:
        return self.start

    def end_point(self) -> Point2D:
        return self.end

    def length_mm(self) -> float:
        return self.start.distance_to(self.end)

    def point_at(self, t: float) -> Point2D:
        t = _validate_unit_parameter("t", t)
        return Point2D(
            self.start.x_mm + (self.end.x_mm - self.start.x_mm) * t,
            self.start.y_mm + (self.end.y_mm - self.start.y_mm) * t,
        )

    def tangent_at(self, t: float) -> Vec2:
        _validate_unit_parameter("t", t)
        return self.start.vector_to(self.end).normalized()

    def start_tangent(self) -> Vec2:
        return self.tangent_at(0.0)

    def end_tangent(self) -> Vec2:
        return self.tangent_at(1.0)

    def signed_curvature_inv_mm(self, t: float = 0.0) -> float:
        _validate_unit_parameter("t", t)
        return 0.0

    def bounding_box(self) -> BoundingBox:
        return BoundingBox.from_points((self.start, self.end))


@dataclass(frozen=True)
class Arc:
    """
    Circular arc in WORLD millimetres/degrees.

    Positive sweep is CCW; negative sweep is CW.
    The mathematical definition matches the current ESP32 LINE/CIRCLE executor:

        x(theta) = cx + r*cos(theta)
        y(theta) = cy + r*sin(theta)
    """

    center: Point2D
    radius_mm: float
    start_angle_deg: float
    sweep_deg: float

    def __post_init__(self) -> None:
        radius = _require_finite("Arc.radius_mm", self.radius_mm)
        start_angle = _require_finite("Arc.start_angle_deg", self.start_angle_deg)
        sweep = _require_finite("Arc.sweep_deg", self.sweep_deg)

        if radius <= 0.0:
            raise GeometryError("Arc.radius_mm must be > 0")
        if abs(sweep) <= NUMERIC_EPSILON:
            raise GeometryError("Arc.sweep_deg must be non-zero")

        object.__setattr__(self, "radius_mm", radius)
        object.__setattr__(self, "start_angle_deg", start_angle)
        object.__setattr__(self, "sweep_deg", sweep)

        if self.length_mm() <= NUMERIC_EPSILON:
            raise GeometryError("Arc must have non-zero length")

    @property
    def direction(self) -> float:
        return 1.0 if self.sweep_deg > 0.0 else -1.0

    @property
    def end_angle_deg(self) -> float:
        return self.start_angle_deg + self.sweep_deg

    def _point_at_angle_deg(self, angle_deg: float) -> Point2D:
        theta = math.radians(angle_deg)
        return Point2D(
            self.center.x_mm + self.radius_mm * math.cos(theta),
            self.center.y_mm + self.radius_mm * math.sin(theta),
        )

    def start_point(self) -> Point2D:
        return self._point_at_angle_deg(self.start_angle_deg)

    def end_point(self) -> Point2D:
        return self._point_at_angle_deg(self.end_angle_deg)

    def length_mm(self) -> float:
        return self.radius_mm * abs(math.radians(self.sweep_deg))

    def point_at(self, t: float) -> Point2D:
        t = _validate_unit_parameter("t", t)
        return self._point_at_angle_deg(self.start_angle_deg + self.sweep_deg * t)

    def tangent_at(self, t: float) -> Vec2:
        t = _validate_unit_parameter("t", t)
        theta = math.radians(self.start_angle_deg + self.sweep_deg * t)
        tangent = Vec2(
            self.direction * (-math.sin(theta)),
            self.direction * math.cos(theta),
        )
        return tangent.normalized()

    def start_tangent(self) -> Vec2:
        return self.tangent_at(0.0)

    def end_tangent(self) -> Vec2:
        return self.tangent_at(1.0)

    def signed_curvature_inv_mm(self, t: float = 0.0) -> float:
        _validate_unit_parameter("t", t)
        return self.direction / self.radius_mm

    def _contains_angle_deg(self, candidate_deg: float) -> bool:
        """
        Return whether a cardinal angle lies on this signed arc.

        Sweeps with magnitude >= 360 degrees cover every cardinal direction.
        Multi-turn arcs are therefore handled naturally for bounding boxes.
        """
        if abs(self.sweep_deg) >= 360.0 - NUMERIC_EPSILON:
            return True

        if self.sweep_deg > 0.0:
            travel = (candidate_deg - self.start_angle_deg) % 360.0
            return travel <= self.sweep_deg + NUMERIC_EPSILON

        travel = (self.start_angle_deg - candidate_deg) % 360.0
        return travel <= (-self.sweep_deg) + NUMERIC_EPSILON

    def bounding_box(self) -> BoundingBox:
        points = [self.start_point(), self.end_point()]

        # Circle extrema can only occur at the four cardinal angles.
        # Use exact Cartesian extrema here instead of cos/sin(90/180/270 deg),
        # so floating-point trig residue does not pollute the bounding box.
        cardinal_points = {
            0.0: Point2D(self.center.x_mm + self.radius_mm, self.center.y_mm),
            90.0: Point2D(self.center.x_mm, self.center.y_mm + self.radius_mm),
            180.0: Point2D(self.center.x_mm - self.radius_mm, self.center.y_mm),
            270.0: Point2D(self.center.x_mm, self.center.y_mm - self.radius_mm),
        }
        for angle_deg, point in cardinal_points.items():
            if self._contains_angle_deg(angle_deg):
                points.append(point)

        return BoundingBox.from_points(points)


def _midpoint(a: Point2D, b: Point2D) -> Point2D:
    return Point2D(
        0.5 * (a.x_mm + b.x_mm),
        0.5 * (a.y_mm + b.y_mm),
    )


def _bezier_coordinate_extrema_parameters(
    p0: float,
    p1: float,
    p2: float,
    p3: float,
) -> tuple[float, ...]:
    """Return interior u values where a cubic Bézier coordinate has zero derivative."""
    # P(u) = a*u^3 + b*u^2 + c*u + d
    a = -p0 + 3.0 * p1 - 3.0 * p2 + p3
    b = 3.0 * p0 - 6.0 * p1 + 3.0 * p2
    c = -3.0 * p0 + 3.0 * p1

    # P'(u) = 3a*u^2 + 2b*u + c
    qa = 3.0 * a
    qb = 2.0 * b
    qc = c

    roots: list[float] = []

    if abs(qa) <= NUMERIC_EPSILON:
        if abs(qb) <= NUMERIC_EPSILON:
            return ()
        u = -qc / qb
        if 0.0 < u < 1.0:
            roots.append(u)
        return tuple(roots)

    discriminant = qb * qb - 4.0 * qa * qc
    if discriminant < -NUMERIC_EPSILON:
        return ()
    discriminant = max(0.0, discriminant)
    sqrt_d = math.sqrt(discriminant)

    u1 = (-qb - sqrt_d) / (2.0 * qa)
    u2 = (-qb + sqrt_d) / (2.0 * qa)

    for u in (u1, u2):
        if 0.0 < u < 1.0 and not any(abs(u - existing) <= NUMERIC_EPSILON for existing in roots):
            roots.append(u)

    return tuple(roots)


@dataclass(frozen=True)
class CubicBezier:
    """
    Cubic Bézier primitive in WORLD millimetres.

    P(u) =
        (1-u)^3 P0
        + 3(1-u)^2 u P1
        + 3(1-u)u^2 P2
        + u^3 P3

    where:
        P0 = start
        P1 = control1
        P2 = control2
        P3 = end

    u is a geometric curve parameter, NOT time and NOT normalized arc length.
    """

    start: Point2D
    control1: Point2D
    control2: Point2D
    end: Point2D

    def __post_init__(self) -> None:
        control_polygon_length = (
            self.start.distance_to(self.control1)
            + self.control1.distance_to(self.control2)
            + self.control2.distance_to(self.end)
        )
        if control_polygon_length <= NUMERIC_EPSILON:
            raise GeometryError("CubicBezier must not collapse to a single point")

    def start_point(self) -> Point2D:
        return self.start

    def end_point(self) -> Point2D:
        return self.end

    def point_at(self, u: float) -> Point2D:
        u = _validate_unit_parameter("u", u)
        v = 1.0 - u

        w0 = v * v * v
        w1 = 3.0 * v * v * u
        w2 = 3.0 * v * u * u
        w3 = u * u * u

        return Point2D(
            w0 * self.start.x_mm
            + w1 * self.control1.x_mm
            + w2 * self.control2.x_mm
            + w3 * self.end.x_mm,
            w0 * self.start.y_mm
            + w1 * self.control1.y_mm
            + w2 * self.control2.y_mm
            + w3 * self.end.y_mm,
        )

    def derivative_at(self, u: float) -> Vec2:
        u = _validate_unit_parameter("u", u)
        v = 1.0 - u

        dx = (
            3.0 * v * v * (self.control1.x_mm - self.start.x_mm)
            + 6.0 * v * u * (self.control2.x_mm - self.control1.x_mm)
            + 3.0 * u * u * (self.end.x_mm - self.control2.x_mm)
        )
        dy = (
            3.0 * v * v * (self.control1.y_mm - self.start.y_mm)
            + 6.0 * v * u * (self.control2.y_mm - self.control1.y_mm)
            + 3.0 * u * u * (self.end.y_mm - self.control2.y_mm)
        )
        return Vec2(dx, dy)

    def second_derivative_at(self, u: float) -> Vec2:
        u = _validate_unit_parameter("u", u)
        v = 1.0 - u

        ddx = 6.0 * (
            v * (self.control2.x_mm - 2.0 * self.control1.x_mm + self.start.x_mm)
            + u * (self.end.x_mm - 2.0 * self.control2.x_mm + self.control1.x_mm)
        )
        ddy = 6.0 * (
            v * (self.control2.y_mm - 2.0 * self.control1.y_mm + self.start.y_mm)
            + u * (self.end.y_mm - 2.0 * self.control2.y_mm + self.control1.y_mm)
        )
        return Vec2(ddx, ddy)

    def start_tangent(self) -> Vec2:
        # The first non-coincident control/end point determines the limiting
        # forward direction when P1 == P0.
        for candidate in (self.control1, self.control2, self.end):
            direction = self.start.vector_to(candidate)
            if direction.norm() > NUMERIC_EPSILON:
                return direction.normalized()
        raise GeometryError("CubicBezier start tangent is undefined")

    def end_tangent(self) -> Vec2:
        # Forward direction at u=1 points toward the endpoint.
        for candidate in (self.control2, self.control1, self.start):
            direction = candidate.vector_to(self.end)
            if direction.norm() > NUMERIC_EPSILON:
                return direction.normalized()
        raise GeometryError("CubicBezier end tangent is undefined")

    def tangent_at(self, u: float) -> Vec2:
        u = _validate_unit_parameter("u", u)

        if u <= NUMERIC_EPSILON:
            return self.start_tangent()
        if u >= 1.0 - NUMERIC_EPSILON:
            return self.end_tangent()

        derivative = self.derivative_at(u)
        if derivative.norm() <= NUMERIC_EPSILON:
            raise GeometryError(
                f"CubicBezier tangent is undefined at u={u!r} because the first derivative vanishes"
            )
        return derivative.normalized()

    def signed_curvature_inv_mm(self, u: float) -> float:
        u = _validate_unit_parameter("u", u)
        first = self.derivative_at(u)
        second = self.second_derivative_at(u)
        speed = first.norm()

        if speed <= NUMERIC_EPSILON:
            raise GeometryError(
                f"CubicBezier curvature is undefined at u={u!r} because the first derivative vanishes"
            )

        return first.cross_z(second) / (speed * speed * speed)

    def _split_half(self) -> tuple["CubicBezier", "CubicBezier"]:
        p01 = _midpoint(self.start, self.control1)
        p12 = _midpoint(self.control1, self.control2)
        p23 = _midpoint(self.control2, self.end)
        p012 = _midpoint(p01, p12)
        p123 = _midpoint(p12, p23)
        p0123 = _midpoint(p012, p123)

        return (
            CubicBezier(self.start, p01, p012, p0123),
            CubicBezier(p0123, p123, p23, self.end),
        )

    def length_mm(
        self,
        *,
        tolerance_mm: float = DEFAULT_BEZIER_LENGTH_TOLERANCE_MM,
        max_depth: int = DEFAULT_BEZIER_LENGTH_MAX_DEPTH,
    ) -> float:
        """
        Approximate arc length with adaptive de Casteljau subdivision.

        The stopping estimate compares the control-polygon length with the
        endpoint chord. This is for PC geometry/analysis. It is NOT an ESP32
        native Bézier arc-length parameterization.
        """
        tolerance_mm = _require_finite("tolerance_mm", tolerance_mm)
        if tolerance_mm <= 0.0:
            raise GeometryError("tolerance_mm must be > 0")
        if not isinstance(max_depth, int) or max_depth < 0:
            raise GeometryError("max_depth must be a non-negative integer")

        def recurse(curve: "CubicBezier", tolerance: float, depth: int) -> float:
            chord = curve.start.distance_to(curve.end)
            polygon = (
                curve.start.distance_to(curve.control1)
                + curve.control1.distance_to(curve.control2)
                + curve.control2.distance_to(curve.end)
            )

            if depth >= max_depth or (polygon - chord) <= tolerance:
                # A common stable estimate bounded by chord and control polygon.
                return 0.5 * (polygon + chord)

            left, right = curve._split_half()
            half_tol = 0.5 * tolerance
            return recurse(left, half_tol, depth + 1) + recurse(right, half_tol, depth + 1)

        return recurse(self, tolerance_mm, 0)

    def bounding_box(self) -> BoundingBox:
        parameters = {0.0, 1.0}

        parameters.update(
            _bezier_coordinate_extrema_parameters(
                self.start.x_mm,
                self.control1.x_mm,
                self.control2.x_mm,
                self.end.x_mm,
            )
        )
        parameters.update(
            _bezier_coordinate_extrema_parameters(
                self.start.y_mm,
                self.control1.y_mm,
                self.control2.y_mm,
                self.end.y_mm,
            )
        )

        return BoundingBox.from_points(self.point_at(u) for u in sorted(parameters))


Geometry = Union[Line, Arc, CubicBezier]


__all__ = [
    "Arc",
    "BoundingBox",
    "CubicBezier",
    "DEFAULT_BEZIER_LENGTH_MAX_DEPTH",
    "DEFAULT_BEZIER_LENGTH_TOLERANCE_MM",
    "Geometry",
    "GeometryError",
    "Line",
    "NUMERIC_EPSILON",
    "Point2D",
    "Vec2",
]
