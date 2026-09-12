"""Virtual-map navigation planning for the N2 desktop demo.

The planner is deliberately small and deterministic.  It turns a WORLD pose
and a hand-edited :class:`MapDocument` into the existing Toolpath execution IR
and then into canonical TRJ2 bytes.  No firmware behaviour is changed here.

Coordinate contract
-------------------
``MapDocument`` stores paths in a map-local frame.  ``MapHome`` is the map
point which the robot calls WORLD ``(0, 0)``.  Therefore every map point and
geometry is translated by ``-home.point_map`` before it is put in a
Toolpath.  The robot axes remain ``+X`` forward, ``+Y`` left and positive yaw
counter-clockwise.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import hashlib
import json
import math
from typing import Iterable

from ..geometry import Arc, CubicBezier, Geometry, Line, Point2D
from ..toolpath import Motion, PenDown, PenUp, Toolpath
from ..toolpath_trj2_export import encode_toolpath_trj2
from .map_model import MapDocument, MapObstacle, MapPath
from .protocol import Pose


class NavigationPlanningError(ValueError):
    """Raised when a navigation request cannot be converted to a Toolpath."""


class NavigationMode(str, Enum):
    CLICK_TO_GO = "CLICK_TO_GO"
    SELECTED_PATH = "SELECTED_PATH"
    RETURN_HOME = "RETURN_HOME"


class PenMode(str, Enum):
    MOVE_ONLY = "MOVE_ONLY"
    DRAW = "DRAW"


class PathDirection(str, Enum):
    AUTO = "AUTO"
    FORWARD = "FORWARD"
    REVERSE = "REVERSE"


@dataclass(frozen=True)
class VehicleProfile:
    """Vehicle footprint measured from the odometry reference point.

    ``front`` and ``rear`` are extents along WORLD +X/-X, while ``left`` and
    ``right`` are extents along WORLD +Y/-Y.  The planner uses the enclosing
    circle so the clearance remains valid while the robot rotates.
    """

    enabled: bool = False
    front_mm: float = 70.0
    rear_mm: float = 70.0
    left_mm: float = 60.0
    right_mm: float = 60.0
    safety_margin_mm: float = 5.0
    localization_margin_mm: float = 0.0

    def __post_init__(self) -> None:
        if not isinstance(self.enabled, bool):
            raise NavigationPlanningError("vehicle footprint enabled must be a boolean")
        for name in (
            "front_mm",
            "rear_mm",
            "left_mm",
            "right_mm",
            "safety_margin_mm",
            "localization_margin_mm",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise NavigationPlanningError(f"{name} must be >= 0")
            object.__setattr__(self, name, value)
        if self.front_mm + self.rear_mm <= 0.0 or self.left_mm + self.right_mm <= 0.0:
            raise NavigationPlanningError("vehicle footprint must have a positive width and length")

    @property
    def effective_radius_mm(self) -> float:
        """Conservative radius used to inflate map obstacles."""

        if not self.enabled:
            return 0.0
        half_length = max(self.front_mm, self.rear_mm)
        half_width = max(self.left_mm, self.right_mm)
        return math.hypot(half_length, half_width) + self.safety_margin_mm + self.localization_margin_mm

    @property
    def effective_diameter_mm(self) -> float:
        return 2.0 * self.effective_radius_mm


@dataclass(frozen=True)
class PlannerConfig:
    """Default motion settings used by the N2 planner."""

    # Normal pen-up travel defaults to the validated full-speed M6 setting.
    travel_speed_mm_s: float = 100.0
    draw_speed_mm_s: float = 65.0
    # The real N3 motion profile requires an explicit positive acceleration.
    acceleration_mm_s2: float = 300.0
    geometry_tolerance_mm: float = 2.0
    enable_arc_fitting: bool = True
    min_arc_points: int = 6
    min_arc_sweep_deg: float = 12.0
    min_arc_radius_mm: float = 30.0
    smoothing_iterations: int = 2
    smoothing_strength: float = 0.35
    simplify_tolerance_mm: float = 0.5
    continuity_tolerance_mm: float = 1.0e-6
    obstacle_clearance_mm: float = 2.0
    obstacle_sample_step_mm: float = 5.0
    vehicle_profile: VehicleProfile = field(default_factory=VehicleProfile)

    def __post_init__(self) -> None:
        for name in (
            "travel_speed_mm_s",
            "draw_speed_mm_s",
            "geometry_tolerance_mm",
            "simplify_tolerance_mm",
            "continuity_tolerance_mm",
            "obstacle_clearance_mm",
            "obstacle_sample_step_mm",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise NavigationPlanningError(f"{name} must be > 0")
            object.__setattr__(self, name, value)
        acceleration = float(self.acceleration_mm_s2)
        if not math.isfinite(acceleration) or acceleration < 0.0:
            raise NavigationPlanningError("acceleration_mm_s2 must be >= 0")
        object.__setattr__(self, "acceleration_mm_s2", acceleration)
        if not isinstance(self.enable_arc_fitting, bool):
            raise NavigationPlanningError("enable_arc_fitting must be a boolean")
        if not isinstance(self.min_arc_points, int) or self.min_arc_points < 3:
            raise NavigationPlanningError("min_arc_points must be an integer >= 3")
        sweep = float(self.min_arc_sweep_deg)
        if not math.isfinite(sweep) or not 0.0 < sweep <= 180.0:
            raise NavigationPlanningError("min_arc_sweep_deg must be in (0, 180]")
        object.__setattr__(self, "min_arc_sweep_deg", sweep)
        radius = float(self.min_arc_radius_mm)
        if not math.isfinite(radius) or radius <= 0.0:
            raise NavigationPlanningError("min_arc_radius_mm must be > 0")
        object.__setattr__(self, "min_arc_radius_mm", radius)
        if not isinstance(self.smoothing_iterations, int) or self.smoothing_iterations < 0:
            raise NavigationPlanningError("smoothing_iterations must be >= 0")
        strength = float(self.smoothing_strength)
        if not math.isfinite(strength) or not 0.0 <= strength <= 1.0:
            raise NavigationPlanningError("smoothing_strength must be in [0, 1]")
        object.__setattr__(self, "smoothing_strength", strength)
        if not isinstance(self.vehicle_profile, VehicleProfile):
            raise NavigationPlanningError("vehicle_profile must be a VehicleProfile")


def document_revision(document: MapDocument) -> str:
    """Return a stable digest used to detect stale plans in the UI."""

    # ``settings`` is UI metadata.  Saving a map updates it, but that must not
    # make an already compiled geometric plan stale when the map geometry did
    # not change.  Planner/vehicle changes already invalidate ``_plan`` in the
    # UI before a new plan is compiled.
    document_data = document.to_dict()
    document_data.pop("settings", None)
    payload = json.dumps(document_data, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _translate_point(point: Point2D, document: MapDocument) -> Point2D:
    return document.world_point(point)


def translate_geometry(geometry: Geometry, document: MapDocument) -> Geometry:
    """Translate one map-local geometry into the robot WORLD frame."""

    def point(value: Point2D) -> Point2D:
        return _translate_point(value, document)

    if isinstance(geometry, Line):
        return Line(point(geometry.start), point(geometry.end))
    if isinstance(geometry, Arc):
        return Arc(point(geometry.center), geometry.radius_mm, geometry.start_angle_deg, geometry.sweep_deg)
    if isinstance(geometry, CubicBezier):
        return CubicBezier(
            point(geometry.start),
            point(geometry.control1),
            point(geometry.control2),
            point(geometry.end),
        )
    raise NavigationPlanningError(f"unsupported geometry type: {type(geometry).__name__}")


def reverse_geometry(geometry: Geometry) -> Geometry:
    """Reverse a geometry while preserving its exact mathematical shape."""

    if isinstance(geometry, Line):
        return Line(geometry.end, geometry.start)
    if isinstance(geometry, Arc):
        return Arc(
            geometry.center,
            geometry.radius_mm,
            geometry.end_angle_deg,
            -geometry.sweep_deg,
        )
    if isinstance(geometry, CubicBezier):
        return CubicBezier(
            geometry.end,
            geometry.control2,
            geometry.control1,
            geometry.start,
        )
    raise NavigationPlanningError(f"unsupported geometry type: {type(geometry).__name__}")


def _orientation(a: Point2D, b: Point2D, c: Point2D) -> float:
    return (b.x_mm - a.x_mm) * (c.y_mm - a.y_mm) - (b.y_mm - a.y_mm) * (c.x_mm - a.x_mm)


def _point_on_segment(point: Point2D, start: Point2D, end: Point2D) -> bool:
    epsilon = 1.0e-8
    if abs(_orientation(start, end, point)) > epsilon:
        return False
    return (
        min(start.x_mm, end.x_mm) - epsilon <= point.x_mm <= max(start.x_mm, end.x_mm) + epsilon
        and min(start.y_mm, end.y_mm) - epsilon <= point.y_mm <= max(start.y_mm, end.y_mm) + epsilon
    )


def _point_inside_or_on(point: Point2D, polygon: tuple[Point2D, ...]) -> bool:
    """Ray-cast point test; boundary points count as occupied."""

    for start, end in zip(polygon, polygon[1:] + polygon[:1]):
        if _point_on_segment(point, start, end):
            return True
    inside = False
    previous = polygon[-1]
    for current in polygon:
        crosses = (current.y_mm > point.y_mm) != (previous.y_mm > point.y_mm)
        if crosses:
            x_at_y = (
                (previous.x_mm - current.x_mm)
                * (point.y_mm - current.y_mm)
                / (previous.y_mm - current.y_mm)
                + current.x_mm
            )
            if point.x_mm < x_at_y:
                inside = not inside
        previous = current
    return inside


def _closest_point_on_segment(point: Point2D, start: Point2D, end: Point2D) -> Point2D:
    dx = end.x_mm - start.x_mm
    dy = end.y_mm - start.y_mm
    length_sq = dx * dx + dy * dy
    if length_sq <= 1.0e-18:
        return start
    t = ((point.x_mm - start.x_mm) * dx + (point.y_mm - start.y_mm) * dy) / length_sq
    t = max(0.0, min(1.0, t))
    return Point2D(start.x_mm + t * dx, start.y_mm + t * dy)


def _polygon_signed_area(polygon: tuple[Point2D, ...]) -> float:
    return 0.5 * sum(
        start.x_mm * end.y_mm - end.x_mm * start.y_mm
        for start, end in zip(polygon, polygon[1:] + polygon[:1])
    )


def _nearest_safe_point(
    point: Point2D,
    obstacles: tuple[tuple[Point2D, ...], ...],
    clearance_mm: float,
) -> Point2D:
    """Move an endpoint to the nearest point outside all obstacle polygons."""

    if not obstacles or not any(_point_inside_or_on(point, polygon) for polygon in obstacles):
        return point

    candidate = point
    for _ in range(12):
        blocked = [polygon for polygon in obstacles if _point_inside_or_on(candidate, polygon)]
        if not blocked:
            return candidate
        polygon = min(
            blocked,
            key=lambda item: min(
                candidate.distance_to(_closest_point_on_segment(candidate, start, end))
                for start, end in zip(item, item[1:] + item[:1])
            ),
        )
        edges = tuple(zip(polygon, polygon[1:] + polygon[:1]))
        edge_start, edge_end = min(
            edges,
            key=lambda edge: candidate.distance_to(_closest_point_on_segment(candidate, edge[0], edge[1])),
        )
        boundary = _closest_point_on_segment(candidate, edge_start, edge_end)
        edge_dx = edge_end.x_mm - edge_start.x_mm
        edge_dy = edge_end.y_mm - edge_start.y_mm
        edge_length = math.hypot(edge_dx, edge_dy)
        if edge_length <= 1.0e-12:
            raise NavigationPlanningError("obstacle contains a degenerate edge")
        if _polygon_signed_area(polygon) >= 0.0:
            normals = ((edge_dy / edge_length, -edge_dx / edge_length), (-edge_dy / edge_length, edge_dx / edge_length))
        else:
            normals = ((-edge_dy / edge_length, edge_dx / edge_length), (edge_dy / edge_length, -edge_dx / edge_length))
        step = clearance_mm * (2.0 ** min(_, 4))
        candidates = tuple(
            Point2D(boundary.x_mm + normal[0] * step, boundary.y_mm + normal[1] * step)
            for normal in normals
        )
        outside = [item for item in candidates if not any(_point_inside_or_on(item, other) for other in obstacles)]
        if outside:
            return min(outside, key=lambda item: point.distance_to(item))
        candidate = min(candidates, key=lambda item: point.distance_to(item))
    raise NavigationPlanningError("cannot find a safe point outside the obstacle set")


def _proper_segment_crossing(a: Point2D, b: Point2D, c: Point2D, d: Point2D) -> bool:
    epsilon = 1.0e-8
    first = _orientation(a, b, c)
    second = _orientation(a, b, d)
    third = _orientation(c, d, a)
    fourth = _orientation(c, d, b)
    return (
        ((first > epsilon and second < -epsilon) or (first < -epsilon and second > epsilon))
        and ((third > epsilon and fourth < -epsilon) or (third < -epsilon and fourth > epsilon))
    )


def _segment_clear(start: Point2D, end: Point2D, obstacles: tuple[tuple[Point2D, ...], ...]) -> bool:
    """Check a segment against polygon interiors and proper edge crossings."""

    for polygon in obstacles:
        if _point_inside_or_on(start, polygon) and not any(start == vertex for vertex in polygon):
            return False
        if _point_inside_or_on(end, polygon) and not any(end == vertex for vertex in polygon):
            return False
        for edge_start, edge_end in zip(polygon, polygon[1:] + polygon[:1]):
            if _proper_segment_crossing(start, end, edge_start, edge_end):
                return False
        midpoint = Point2D(
            (start.x_mm + end.x_mm) * 0.5,
            (start.y_mm + end.y_mm) * 0.5,
        )
        if _point_inside_or_on(midpoint, polygon) and not any(midpoint == vertex for vertex in polygon):
            # A segment along a polygon edge is allowed; a midpoint strictly
            # inside the polygon is a collision.
            if not any(_point_on_segment(midpoint, edge_start, edge_end) for edge_start, edge_end in zip(polygon, polygon[1:] + polygon[:1])):
                return False
    return True


def _shortest_detour(
    start: Point2D,
    goal: Point2D,
    obstacles: tuple[tuple[Point2D, ...], ...],
) -> tuple[Point2D, ...]:
    """Find a shortest visibility-graph route around polygon obstacles."""

    if not obstacles or _segment_clear(start, goal, obstacles):
        return (start, goal)
    for polygon in obstacles:
        if _point_inside_or_on(goal, polygon):
            raise NavigationPlanningError("navigation target lies inside an obstacle")
        if _point_inside_or_on(start, polygon):
            raise NavigationPlanningError("current robot pose lies inside an obstacle")

    nodes: list[Point2D] = [start, goal]
    for polygon in obstacles:
        nodes.extend(polygon)
    adjacency: list[list[tuple[int, float]]] = [[] for _ in nodes]
    for first in range(len(nodes)):
        for second in range(first + 1, len(nodes)):
            if _segment_clear(nodes[first], nodes[second], obstacles):
                distance = nodes[first].distance_to(nodes[second])
                adjacency[first].append((second, distance))
                adjacency[second].append((first, distance))

    distances = [float("inf")] * len(nodes)
    previous: list[int | None] = [None] * len(nodes)
    visited: set[int] = set()
    distances[0] = 0.0
    while len(visited) < len(nodes):
        candidates = [index for index in range(len(nodes)) if index not in visited]
        if not candidates:
            break
        current = min(candidates, key=lambda index: distances[index])
        if not math.isfinite(distances[current]):
            break
        visited.add(current)
        if current == 1:
            break
        for neighbor, edge_length in adjacency[current]:
            candidate = distances[current] + edge_length
            if candidate < distances[neighbor]:
                distances[neighbor] = candidate
                previous[neighbor] = current

    if not math.isfinite(distances[1]):
        raise NavigationPlanningError("no collision-free route exists for the selected target")
    route: list[Point2D] = []
    cursor: int | None = 1
    while cursor is not None:
        route.append(nodes[cursor])
        cursor = previous[cursor]
    route.reverse()
    return tuple(route)


def _cross(a: Point2D, b: Point2D, c: Point2D) -> float:
    return (b.x_mm - a.x_mm) * (c.y_mm - a.y_mm) - (b.y_mm - a.y_mm) * (c.x_mm - a.x_mm)


def _convex_hull(points: tuple[Point2D, ...]) -> tuple[Point2D, ...]:
    """Return a CCW monotonic-chain hull for conservative footprint inflation."""

    unique = sorted({(point.x_mm, point.y_mm) for point in points})
    if len(unique) <= 2:
        return tuple(Point2D(x, y) for x, y in unique)
    values = [Point2D(x, y) for x, y in unique]
    lower: list[Point2D] = []
    for point in values:
        while len(lower) >= 2 and _cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper: list[Point2D] = []
    for point in reversed(values):
        while len(upper) >= 2 and _cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    return tuple(lower[:-1] + upper[:-1])


def _offset_convex_polygon(polygon: tuple[Point2D, ...], radius_mm: float) -> tuple[Point2D, ...]:
    """Inflate a convex polygon by a radius using offset edge intersections.

    Obstacles are intentionally converted to their convex hull first.  This is
    conservative for concave hand-drawn obstacles and keeps the existing
    visibility-graph planner dependency-free.
    """

    hull = _convex_hull(polygon)
    if radius_mm <= 0.0 or len(hull) < 3:
        return hull
    if _polygon_signed_area(hull) < 0.0:
        hull = tuple(reversed(hull))
    result: list[Point2D] = []
    count = len(hull)

    def offset_line(start: Point2D, end: Point2D) -> tuple[Point2D, Point2D]:
        dx = end.x_mm - start.x_mm
        dy = end.y_mm - start.y_mm
        length = math.hypot(dx, dy)
        if length <= 1.0e-12:
            return start, end
        # Hull is CCW; the right-hand normal is outside.
        nx, ny = dy / length, -dx / length
        return (
            Point2D(start.x_mm + nx * radius_mm, start.y_mm + ny * radius_mm),
            Point2D(end.x_mm + nx * radius_mm, end.y_mm + ny * radius_mm),
        )

    def intersection(first: tuple[Point2D, Point2D], second: tuple[Point2D, Point2D]) -> Point2D:
        a, b = first
        c, d = second
        r_x, r_y = b.x_mm - a.x_mm, b.y_mm - a.y_mm
        s_x, s_y = d.x_mm - c.x_mm, d.y_mm - c.y_mm
        denominator = r_x * s_y - r_y * s_x
        if abs(denominator) <= 1.0e-12:
            # Parallel edges only occur for a degenerate/near-flat corner.
            return Point2D((b.x_mm + c.x_mm) * 0.5, (b.y_mm + c.y_mm) * 0.5)
        t = ((c.x_mm - a.x_mm) * s_y - (c.y_mm - a.y_mm) * s_x) / denominator
        return Point2D(a.x_mm + t * r_x, a.y_mm + t * r_y)

    for index, vertex in enumerate(hull):
        previous = hull[(index - 1) % count]
        following = hull[(index + 1) % count]
        result.append(intersection(offset_line(previous, vertex), offset_line(vertex, following)))
    return tuple(result)


def _world_obstacles(
    document: MapDocument,
    config: PlannerConfig | None = None,
) -> tuple[tuple[Point2D, ...], ...]:
    profile = config.vehicle_profile if config is not None else VehicleProfile()
    obstacles = tuple(obstacle.to_world(document.home) for obstacle in document.obstacles if obstacle.enabled)
    if profile.effective_radius_mm <= 0.0:
        return obstacles
    return tuple(_offset_convex_polygon(obstacle, profile.effective_radius_mm) for obstacle in obstacles)


def _append_avoiding_travel(
    records: list[object],
    start: Point2D,
    goal: Point2D,
    *,
    document: MapDocument,
    speed_mm_s: float,
    acceleration_mm_s2: float,
    tolerance_mm: float,
    clearance_mm: float,
    config: PlannerConfig | None = None,
) -> Point2D:
    obstacles = _world_obstacles(document, config)
    safe_start = _nearest_safe_point(start, obstacles, clearance_mm)
    safe_goal = _nearest_safe_point(goal, obstacles, clearance_mm)
    current = start
    if current.distance_to(safe_start) > tolerance_mm:
        current = _append_motion(
            records,
            current,
            safe_start,
            speed_mm_s=speed_mm_s,
            acceleration_mm_s2=acceleration_mm_s2,
            tolerance_mm=tolerance_mm,
        )
    route = _shortest_detour(safe_start, safe_goal, obstacles)
    current = safe_start
    for next_point in route[1:]:
        current = _append_motion(
            records,
            current,
            next_point,
            speed_mm_s=speed_mm_s,
            acceleration_mm_s2=acceleration_mm_s2,
            tolerance_mm=tolerance_mm,
        )
    return current


@dataclass(frozen=True)
class _SampledEdge:
    geometry_index: int
    start_t: float
    end_t: float
    start: Point2D
    end: Point2D
    clear: bool


def _sample_geometry_edges(
    geometry: Geometry,
    geometry_index: int,
    obstacles: tuple[tuple[Point2D, ...], ...],
    step_mm: float,
) -> tuple[_SampledEdge, ...]:
    """Sample one primitive for collision tests without changing its output shape."""

    count = max(2, min(4096, int(math.ceil(geometry.length_mm() / step_mm)) + 1))
    parameters = tuple(index / (count - 1) for index in range(count))
    points = tuple(geometry.point_at(parameter) for parameter in parameters)
    return tuple(
        _SampledEdge(
            geometry_index,
            start_t,
            end_t,
            start,
            end,
            _segment_clear(start, end, obstacles),
        )
        for start_t, end_t, start, end in zip(
            parameters,
            parameters[1:],
            points,
            points[1:],
        )
    )


def _lerp_point(first: Point2D, second: Point2D, t: float) -> Point2D:
    return Point2D(
        first.x_mm + (second.x_mm - first.x_mm) * t,
        first.y_mm + (second.y_mm - first.y_mm) * t,
    )


def _split_cubic(curve: CubicBezier, t: float) -> tuple[CubicBezier, CubicBezier]:
    """Split a cubic Bézier exactly at ``t`` using de Casteljau."""

    p01 = _lerp_point(curve.start, curve.control1, t)
    p12 = _lerp_point(curve.control1, curve.control2, t)
    p23 = _lerp_point(curve.control2, curve.end, t)
    p012 = _lerp_point(p01, p12, t)
    p123 = _lerp_point(p12, p23, t)
    point = _lerp_point(p012, p123, t)
    return (
        CubicBezier(curve.start, p01, p012, point),
        CubicBezier(point, p123, p23, curve.end),
    )


def _slice_geometry(geometry: Geometry, start_t: float, end_t: float) -> Geometry:
    """Return the exact subsection of a primitive between two parameters."""

    if start_t <= 0.0 and end_t >= 1.0:
        return geometry
    if isinstance(geometry, Line):
        return Line(geometry.point_at(start_t), geometry.point_at(end_t))
    if isinstance(geometry, Arc):
        return Arc(
            geometry.center,
            geometry.radius_mm,
            geometry.start_angle_deg + geometry.sweep_deg * start_t,
            geometry.sweep_deg * (end_t - start_t),
        )
    if isinstance(geometry, CubicBezier):
        if start_t <= 0.0:
            return _split_cubic(geometry, end_t)[0]
        remainder = _split_cubic(geometry, start_t)[1]
        relative_end = (end_t - start_t) / (1.0 - start_t)
        return _split_cubic(remainder, relative_end)[0]
    raise NavigationPlanningError(f"unsupported geometry type: {type(geometry).__name__}")


def _append_line_piece(
    pieces: list[tuple[Geometry, bool]],
    start: Point2D,
    end: Point2D,
    *,
    draw_allowed: bool,
    tolerance_mm: float,
) -> None:
    if start.distance_to(end) > tolerance_mm:
        pieces.append((Line(start, end), draw_allowed))


def _safe_path_pieces(
    geometries: tuple[Geometry, ...],
    obstacles: tuple[tuple[Point2D, ...], ...],
    *,
    config: PlannerConfig,
) -> tuple[tuple[tuple[Geometry, bool], ...], Point2D, Point2D, int]:
    """Return drawing pieces with PenUp detours where an obstacle is crossed.

    The original fitted primitives are retained when collision-free.  When an
    obstacle affects a path, the path is sampled and only the obstructed spans
    are replaced by visibility-graph travel segments.  This keeps the normal
    smooth/polyline behaviour while making collision handling automatic.
    """

    if not geometries:
        raise NavigationPlanningError("path produced no fitted geometry")
    original_start = geometries[0].start_point()
    original_end = geometries[-1].end_point()
    if not obstacles:
        return tuple((geometry, True) for geometry in geometries), original_start, original_end, 0

    safe_start = _nearest_safe_point(original_start, obstacles, config.obstacle_clearance_mm)
    safe_end = _nearest_safe_point(original_end, obstacles, config.obstacle_clearance_mm)
    sampled_edges = tuple(
        edge
        for geometry_index, geometry in enumerate(geometries)
        for edge in _sample_geometry_edges(
            geometry,
            geometry_index,
            obstacles,
            config.obstacle_sample_step_mm,
        )
    )
    if all(edge.clear for edge in sampled_edges) and safe_start == original_start and safe_end == original_end:
        return tuple((geometry, True) for geometry in geometries), original_start, original_end, 0

    pieces: list[tuple[Geometry, bool]] = []
    detour_count = 0
    edge_index = 0
    while edge_index < len(sampled_edges):
        edge = sampled_edges[edge_index]
        if edge.clear:
            geometry_index = edge.geometry_index
            start_t = edge.start_t
            end_t = edge.end_t
            edge_index += 1
            while (
                edge_index < len(sampled_edges)
                and sampled_edges[edge_index].clear
                and sampled_edges[edge_index].geometry_index == geometry_index
            ):
                end_t = sampled_edges[edge_index].end_t
                edge_index += 1
            pieces.append((_slice_geometry(geometries[geometry_index], start_t, end_t), True))
            continue

        blocked_start = edge_index
        while edge_index < len(sampled_edges) and not sampled_edges[edge_index].clear:
            edge_index += 1
        blocked_end = edge_index - 1
        route_start = sampled_edges[blocked_start].start
        route_end = sampled_edges[blocked_end].end
        if blocked_start == 0 and safe_start != original_start:
            route_start = safe_start
        if blocked_end == len(sampled_edges) - 1 and safe_end != original_end:
            route_end = safe_end
        route = _shortest_detour(route_start, route_end, obstacles)
        detour_count += max(1, len(route) - 2)
        for first, second in zip(route, route[1:]):
            _append_line_piece(
                pieces,
                first,
                second,
                draw_allowed=False,
                tolerance_mm=config.continuity_tolerance_mm,
            )
    return tuple(pieces), safe_start, safe_end, detour_count


def _path_geometry(path: MapPath, document: MapDocument, config: PlannerConfig) -> tuple[Geometry, ...]:
    if not config.enable_arc_fitting and path.path_kind == "freehand":
        points = path.processed_points(
            smoothing_iterations=config.smoothing_iterations,
            smoothing_strength=config.smoothing_strength,
            simplify_tolerance_mm=config.simplify_tolerance_mm,
        )
        return tuple(
            translate_geometry(Line(start, end), document)
            for start, end in zip(points, points[1:])
            if start.distance_to(end) > 1.0e-12
        )
    fitted = path.fitted_geometry(
        tolerance_mm=config.geometry_tolerance_mm,
        min_arc_points=config.min_arc_points,
        min_arc_sweep_deg=config.min_arc_sweep_deg,
        min_arc_radius_mm=config.min_arc_radius_mm,
        smoothing_iterations=config.smoothing_iterations,
        smoothing_strength=config.smoothing_strength,
        simplify_tolerance_mm=config.simplify_tolerance_mm,
    )
    return tuple(translate_geometry(geometry, document) for geometry in fitted)


def _append_motion(
    records: list[object],
    start: Point2D,
    end: Point2D,
    *,
    speed_mm_s: float,
    acceleration_mm_s2: float,
    tolerance_mm: float,
) -> Point2D:
    if start.distance_to(end) <= tolerance_mm:
        return end
    records.append(Motion(Line(start, end), speed_mm_s, acceleration_mm_s2))
    return end


def _toolpath(records: Iterable[object], config: PlannerConfig) -> Toolpath:
    return Toolpath(tuple(records), continuity_tolerance_mm=config.continuity_tolerance_mm)


@dataclass(frozen=True)
class RouteLeg:
    """Review metadata for one selected path in a compiled route."""

    path_id: str
    sequence_index: int
    direction: PathDirection
    requested_start: Point2D
    effective_start: Point2D
    requested_end: Point2D
    effective_end: Point2D
    detour_count: int = 0


@dataclass(frozen=True)
class NavigationPlan:
    """A compiled, reviewable navigation job."""

    mode: NavigationMode
    pen_mode: PenMode
    start_pose: Pose
    toolpath: Toolpath
    document_revision: str
    target_world: Point2D | None = None
    path_id: str | None = None
    path_direction: PathDirection | None = None
    requested_target_world: Point2D | None = None
    path_ids: tuple[str, ...] = ()
    path_directions: tuple[PathDirection, ...] = ()
    route_legs: tuple[RouteLeg, ...] = ()
    target_yaw_deg: float | None = None

    @property
    def start_point(self) -> Point2D:
        return Point2D(self.start_pose.x_mm, self.start_pose.y_mm)

    @property
    def end_point(self) -> Point2D | None:
        if not self.toolpath.has_motion():
            return self.start_point
        return self.toolpath.end_point()

    def trj2_bytes(self) -> bytes:
        return encode_toolpath_trj2(
            self.toolpath,
            start_yaw_deg=self.start_pose.yaw_deg,
            start_point=self.start_point,
        )

    def ensure_current(self, document: MapDocument) -> None:
        if document_revision(document) != self.document_revision:
            raise NavigationPlanningError("navigation plan is stale because the map changed")


class NavigationPlanner:
    """Build click, selected-path, ordered-sequence, and return-home plans."""

    def __init__(self, config: PlannerConfig | None = None) -> None:
        self.config = config or PlannerConfig()

    def plan_click_to_go(
        self,
        document: MapDocument,
        current_pose: Pose,
        target_map: Point2D,
    ) -> NavigationPlan:
        requested_target_world = _translate_point(target_map, document)
        records: list[object] = [PenUp()]
        target_world = _append_avoiding_travel(
            records,
            Point2D(current_pose.x_mm, current_pose.y_mm),
            requested_target_world,
            document=document,
            speed_mm_s=self.config.travel_speed_mm_s,
            acceleration_mm_s2=self.config.acceleration_mm_s2,
            tolerance_mm=self.config.continuity_tolerance_mm,
            clearance_mm=self.config.obstacle_clearance_mm,
            config=self.config,
        )
        return NavigationPlan(
            mode=NavigationMode.CLICK_TO_GO,
            pen_mode=PenMode.MOVE_ONLY,
            start_pose=current_pose,
            toolpath=_toolpath(records, self.config),
            document_revision=document_revision(document),
            target_world=target_world,
            requested_target_world=requested_target_world,
        )

    def plan_return_home(
        self,
        document: MapDocument,
        current_pose: Pose,
    ) -> NavigationPlan:
        records: list[object] = [PenUp()]
        requested_target_world = Point2D(0.0, 0.0)
        target_world = _append_avoiding_travel(
            records,
            Point2D(current_pose.x_mm, current_pose.y_mm),
            requested_target_world,
            document=document,
            speed_mm_s=self.config.travel_speed_mm_s,
            acceleration_mm_s2=self.config.acceleration_mm_s2,
            tolerance_mm=self.config.continuity_tolerance_mm,
            clearance_mm=self.config.obstacle_clearance_mm,
            config=self.config,
        )
        return NavigationPlan(
            mode=NavigationMode.RETURN_HOME,
            pen_mode=PenMode.MOVE_ONLY,
            start_pose=current_pose,
            toolpath=_toolpath(records, self.config),
            document_revision=document_revision(document),
            target_world=target_world,
            requested_target_world=requested_target_world,
            target_yaw_deg=document.home.yaw_deg,
        )

    def plan_selected_path(
        self,
        document: MapDocument,
        current_pose: Pose,
        path_id: str,
        *,
        pen_mode: PenMode = PenMode.MOVE_ONLY,
        direction: PathDirection = PathDirection.AUTO,
    ) -> NavigationPlan:
        return self.plan_path_sequence(
            document,
            current_pose,
            (path_id,),
            pen_mode=pen_mode,
            direction=direction,
        )

    def plan_path_sequence(
        self,
        document: MapDocument,
        current_pose: Pose,
        path_ids: Iterable[str],
        *,
        pen_mode: PenMode = PenMode.MOVE_ONLY,
        direction: PathDirection = PathDirection.AUTO,
    ) -> NavigationPlan:
        """Compile several paths in the requested order into one Toolpath."""

        if not isinstance(pen_mode, PenMode):
            pen_mode = PenMode(pen_mode)
        if not isinstance(direction, PathDirection):
            direction = PathDirection(direction)
        ordered_ids = tuple(path_ids)
        if not ordered_ids:
            raise NavigationPlanningError("select at least one path before planning")

        records: list[object] = [PenUp()]
        current = Point2D(current_pose.x_mm, current_pose.y_mm)
        obstacles = _world_obstacles(document, self.config)
        selected_directions: list[PathDirection] = []
        route_legs: list[RouteLeg] = []
        for sequence_index, path_id in enumerate(ordered_ids, start=1):
            path = next((item for item in document.paths if item.path_id == path_id), None)
            if path is None:
                raise NavigationPlanningError(f"unknown path ID: {path_id}")
            geometries = _path_geometry(path, document, self.config)
            if not geometries:
                raise NavigationPlanningError(f"path {path_id!r} produced no fitted geometry")

            requested_start = geometries[0].start_point()
            requested_end = geometries[-1].end_point()
            selected_direction = direction
            if direction is PathDirection.AUTO:
                selected_direction = (
                    PathDirection.FORWARD
                    if current.distance_to(requested_start) <= current.distance_to(requested_end)
                    else PathDirection.REVERSE
                )
            if selected_direction is PathDirection.REVERSE:
                geometries = tuple(reverse_geometry(item) for item in reversed(geometries))
                requested_start, requested_end = requested_end, requested_start

            pieces, effective_start, effective_end, detour_count = _safe_path_pieces(
                geometries,
                obstacles,
                config=self.config,
            )
            _append_avoiding_travel(
                records,
                current,
                effective_start,
                document=document,
                speed_mm_s=self.config.travel_speed_mm_s,
                acceleration_mm_s2=self.config.acceleration_mm_s2,
                tolerance_mm=self.config.continuity_tolerance_mm,
                clearance_mm=self.config.obstacle_clearance_mm,
                config=self.config,
            )
            state_down = False
            for piece, draw_allowed in pieces:
                should_draw = pen_mode is PenMode.DRAW and draw_allowed
                if should_draw and not state_down:
                    records.append(PenDown())
                    state_down = True
                elif not should_draw and state_down:
                    records.append(PenUp())
                    state_down = False
                speed_mm_s = (
                    self.config.draw_speed_mm_s
                    if should_draw
                    else self.config.travel_speed_mm_s
                )
                records.append(Motion(piece, speed_mm_s, self.config.acceleration_mm_s2))
            if state_down:
                records.append(PenUp())
            elif not records or not isinstance(records[-1], PenUp):
                records.append(PenUp())
            current = effective_end
            selected_directions.append(selected_direction)
            route_legs.append(
                RouteLeg(
                    path_id=path_id,
                    sequence_index=sequence_index,
                    direction=selected_direction,
                    requested_start=requested_start,
                    effective_start=effective_start,
                    requested_end=requested_end,
                    effective_end=effective_end,
                    detour_count=detour_count,
                )
            )

        return NavigationPlan(
            mode=NavigationMode.SELECTED_PATH,
            pen_mode=pen_mode,
            start_pose=current_pose,
            toolpath=_toolpath(records, self.config),
            document_revision=document_revision(document),
            target_world=current,
            requested_target_world=route_legs[-1].requested_end,
            path_id=ordered_ids[0] if len(ordered_ids) == 1 else None,
            path_direction=selected_directions[0] if len(selected_directions) == 1 else None,
            path_ids=ordered_ids,
            path_directions=tuple(selected_directions),
            route_legs=tuple(route_legs),
        )


def plan_click_to_go(document: MapDocument, current_pose: Pose, target_map: Point2D, *, config: PlannerConfig | None = None) -> NavigationPlan:
    return NavigationPlanner(config).plan_click_to_go(document, current_pose, target_map)


def plan_return_home(document: MapDocument, current_pose: Pose, *, config: PlannerConfig | None = None) -> NavigationPlan:
    return NavigationPlanner(config).plan_return_home(document, current_pose)


def plan_selected_path(
    document: MapDocument,
    current_pose: Pose,
    path_id: str,
    *,
    pen_mode: PenMode = PenMode.MOVE_ONLY,
    direction: PathDirection = PathDirection.AUTO,
    config: PlannerConfig | None = None,
) -> NavigationPlan:
    return NavigationPlanner(config).plan_selected_path(
        document,
        current_pose,
        path_id,
        pen_mode=pen_mode,
        direction=direction,
    )


def plan_path_sequence(
    document: MapDocument,
    current_pose: Pose,
    path_ids: Iterable[str],
    *,
    pen_mode: PenMode = PenMode.MOVE_ONLY,
    direction: PathDirection = PathDirection.AUTO,
    config: PlannerConfig | None = None,
) -> NavigationPlan:
    return NavigationPlanner(config).plan_path_sequence(
        document,
        current_pose,
        path_ids,
        pen_mode=pen_mode,
        direction=direction,
    )


__all__ = [
    "NavigationMode",
    "NavigationPlan",
    "NavigationPlanner",
    "NavigationPlanningError",
    "RouteLeg",
    "PathDirection",
    "PenMode",
    "PlannerConfig",
    "VehicleProfile",
    "document_revision",
    "plan_click_to_go",
    "plan_return_home",
    "plan_selected_path",
    "plan_path_sequence",
    "reverse_geometry",
    "translate_geometry",
]
