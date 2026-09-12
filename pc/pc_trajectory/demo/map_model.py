"""Semantic model and JSON persistence for the virtual map editor.

Map paths are stored in a stable map-local millimetre frame.  The Home point
is stored separately and is used only when a path is converted to the robot's
WORLD frame.  This keeps a hand-drawn background and its paths visually fixed
when the user drags Home.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
from typing import Any, Iterable
from uuid import uuid4

from ..geometry import Geometry, Line, Point2D
from ..raster.curve_fit import fit_polyline_geometry
from ..raster.simplify import simplify_polyline, smooth_polyline


MAP_FORMAT_VERSION = 1
PATH_KIND_FREEHAND = "freehand"
PATH_KIND_POLYLINE = "polyline"
PATH_KINDS = (PATH_KIND_FREEHAND, PATH_KIND_POLYLINE)


class MapModelError(ValueError):
    """Raised when a virtual map is invalid or cannot be persisted."""


def _finite(name: str, value: Any) -> float:
    if isinstance(value, bool):
        raise MapModelError(f"{name} must be finite")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise MapModelError(f"{name} must be finite") from exc
    if not math.isfinite(number):
        raise MapModelError(f"{name} must be finite")
    return number


def _positive(name: str, value: Any) -> float:
    number = _finite(name, value)
    if number <= 0.0:
        raise MapModelError(f"{name} must be positive")
    return number


def _point_from_json(value: Any, name: str) -> Point2D:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise MapModelError(f"{name} must be [x_mm, y_mm]")
    return Point2D(_finite(f"{name}[0]", value[0]), _finite(f"{name}[1]", value[1]))


def _point_to_json(point: Point2D) -> list[float]:
    return [point.x_mm, point.y_mm]


def _orientation(a: Point2D, b: Point2D, c: Point2D) -> float:
    return (b.x_mm - a.x_mm) * (c.y_mm - a.y_mm) - (b.y_mm - a.y_mm) * (c.x_mm - a.x_mm)


def _segments_intersect(a: Point2D, b: Point2D, c: Point2D, d: Point2D) -> bool:
    """Return whether two closed segments intersect, including touching."""

    epsilon = 1.0e-9

    def on_segment(p: Point2D, q: Point2D, r: Point2D) -> bool:
        return (
            min(p.x_mm, r.x_mm) - epsilon <= q.x_mm <= max(p.x_mm, r.x_mm) + epsilon
            and min(p.y_mm, r.y_mm) - epsilon <= q.y_mm <= max(p.y_mm, r.y_mm) + epsilon
        )

    ab_c = _orientation(a, b, c)
    ab_d = _orientation(a, b, d)
    cd_a = _orientation(c, d, a)
    cd_b = _orientation(c, d, b)
    if ((ab_c > epsilon and ab_d < -epsilon) or (ab_c < -epsilon and ab_d > epsilon)) and (
        (cd_a > epsilon and cd_b < -epsilon) or (cd_a < -epsilon and cd_b > epsilon)
    ):
        return True
    return (
        (abs(ab_c) <= epsilon and on_segment(a, c, b))
        or (abs(ab_d) <= epsilon and on_segment(a, d, b))
        or (abs(cd_a) <= epsilon and on_segment(c, a, d))
        or (abs(cd_b) <= epsilon and on_segment(c, b, d))
    )


@dataclass(frozen=True)
class MapHome:
    map_x_mm: float = 0.0
    map_y_mm: float = 0.0
    yaw_deg: float = 0.0
    tag_id: int = 0

    def __post_init__(self) -> None:
        object.__setattr__(self, "map_x_mm", _finite("home.map_x_mm", self.map_x_mm))
        object.__setattr__(self, "map_y_mm", _finite("home.map_y_mm", self.map_y_mm))
        object.__setattr__(self, "yaw_deg", _finite("home.yaw_deg", self.yaw_deg))
        if isinstance(self.tag_id, bool) or not isinstance(self.tag_id, int) or self.tag_id < 0:
            raise MapModelError("home.tag_id must be a non-negative integer")

    @property
    def point_map(self) -> Point2D:
        return Point2D(self.map_x_mm, self.map_y_mm)

    def to_dict(self) -> dict[str, Any]:
        return {
            "map_x_mm": self.map_x_mm,
            "map_y_mm": self.map_y_mm,
            "yaw_deg": self.yaw_deg,
            "tag_id": self.tag_id,
        }

    @classmethod
    def from_dict(cls, value: Any) -> "MapHome":
        if not isinstance(value, dict):
            raise MapModelError("home must be an object")
        return cls(
            map_x_mm=value.get("map_x_mm", value.get("x_mm", 0.0)),
            map_y_mm=value.get("map_y_mm", value.get("y_mm", 0.0)),
            yaw_deg=value.get("yaw_deg", 0.0),
            tag_id=value.get("tag_id", 0),
        )


@dataclass(frozen=True)
class MapBackground:
    """Optional future photo/image layer; N1 only stores its metadata."""

    source_type: str = "image"
    relative_path: str = ""
    opacity: float = 0.65
    locked: bool = True
    map_bounds_mm: tuple[float, float, float, float] | None = None
    calibration_path: str | None = None
    enabled: bool = True

    def __post_init__(self) -> None:
        if not isinstance(self.source_type, str) or not self.source_type.strip():
            raise MapModelError("background.source_type must be a non-empty string")
        if not isinstance(self.relative_path, str):
            raise MapModelError("background.relative_path must be a string")
        opacity = _finite("background.opacity", self.opacity)
        if not 0.0 <= opacity <= 1.0:
            raise MapModelError("background.opacity must be in [0, 1]")
        if not isinstance(self.locked, bool) or not isinstance(self.enabled, bool):
            raise MapModelError("background.locked and enabled must be booleans")
        object.__setattr__(self, "opacity", opacity)
        if self.map_bounds_mm is not None:
            if len(self.map_bounds_mm) != 4:
                raise MapModelError("background.map_bounds_mm must have four values")
            bounds = tuple(_finite(f"background.map_bounds_mm[{i}]", value) for i, value in enumerate(self.map_bounds_mm))
            if bounds[0] >= bounds[2] or bounds[1] >= bounds[3]:
                raise MapModelError("background bounds must have positive width and height")
            object.__setattr__(self, "map_bounds_mm", bounds)
        if self.calibration_path is not None and not isinstance(self.calibration_path, str):
            raise MapModelError("background.calibration_path must be a string or null")

    def to_dict(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "source_type": self.source_type,
            "relative_path": self.relative_path,
            "opacity": self.opacity,
            "locked": self.locked,
            "map_bounds_mm": list(self.map_bounds_mm) if self.map_bounds_mm is not None else None,
            "calibration_path": self.calibration_path,
        }

    @classmethod
    def from_dict(cls, value: Any) -> "MapBackground":
        if not isinstance(value, dict):
            raise MapModelError("background must be an object")
        bounds = value.get("map_bounds_mm")
        return cls(
            enabled=value.get("enabled", True),
            source_type=value.get("source_type", "image"),
            relative_path=value.get("relative_path", ""),
            opacity=value.get("opacity", 0.65),
            locked=value.get("locked", True),
            map_bounds_mm=tuple(bounds) if bounds is not None else None,
            calibration_path=value.get("calibration_path"),
        )


@dataclass(frozen=True)
class MapPath:
    points_map_mm: tuple[Point2D, ...]
    path_id: str = field(default_factory=lambda: f"path-{uuid4().hex[:8]}")
    name: str = "Path"
    closed: bool = False
    drawable: bool = True
    path_kind: str = PATH_KIND_FREEHAND

    def __post_init__(self) -> None:
        points = tuple(self.points_map_mm)
        if len(points) < 2:
            raise MapModelError("a path requires at least two points")
        if any(not isinstance(point, Point2D) for point in points):
            raise MapModelError("path points must be Point2D values")
        if not isinstance(self.path_id, str) or not self.path_id.strip():
            raise MapModelError("path_id must be a non-empty string")
        if not isinstance(self.name, str) or not self.name.strip():
            raise MapModelError("path name must be a non-empty string")
        if not isinstance(self.closed, bool) or not isinstance(self.drawable, bool):
            raise MapModelError("path closed and drawable must be booleans")
        if self.path_kind not in PATH_KINDS:
            raise MapModelError(
                f"path_kind must be one of {PATH_KINDS}, got {self.path_kind!r}"
            )
        object.__setattr__(self, "points_map_mm", points)

    @property
    def start_map(self) -> Point2D:
        return self.points_map_mm[0]

    @property
    def end_map(self) -> Point2D:
        return self.points_map_mm[-1]

    def to_world(self, home: MapHome) -> tuple[Point2D, ...]:
        return tuple(
            Point2D(point.x_mm - home.map_x_mm, point.y_mm - home.map_y_mm)
            for point in self.points_map_mm
        )

    def fitted_geometry(
        self,
        *,
        tolerance_mm: float = 2.0,
        min_arc_points: int = 6,
        min_arc_sweep_deg: float = 12.0,
        min_arc_radius_mm: float = 1.0,
        smoothing_iterations: int = 2,
        smoothing_strength: float = 0.35,
        simplify_tolerance_mm: float = 0.5,
    ) -> tuple[Geometry, ...]:
        """Fit the hand-drawn Map polyline to ordered LINE/CIRCLE geometry."""

        if self.path_kind == PATH_KIND_POLYLINE:
            points = list(self.points_map_mm)
            if self.closed and points[-1] != points[0]:
                points.append(points[0])
            return tuple(
                Line(start, end)
                for start, end in zip(points, points[1:])
                if start.distance_to(end) > 1.0e-12
            )

        return fit_polyline_geometry(
            self.processed_points(
                smoothing_iterations=smoothing_iterations,
                smoothing_strength=smoothing_strength,
                simplify_tolerance_mm=simplify_tolerance_mm,
            ),
            tolerance_mm=tolerance_mm,
            closed=self.closed,
            min_arc_points=min_arc_points,
            min_arc_sweep_deg=min_arc_sweep_deg,
            min_arc_radius_mm=min_arc_radius_mm,
        )

    def processed_points(
        self,
        *,
        smoothing_iterations: int = 2,
        smoothing_strength: float = 0.35,
        simplify_tolerance_mm: float = 0.5,
    ) -> tuple[Point2D, ...]:
        """Return endpoint-preserving, mm-space smoothed and simplified points."""

        if self.path_kind == PATH_KIND_POLYLINE:
            points = list(self.points_map_mm)
            if self.closed and points[-1] != points[0]:
                points.append(points[0])
            return tuple(points)

        smoothed = smooth_polyline(
            self.points_map_mm,
            iterations=smoothing_iterations,
            strength=smoothing_strength,
            closed=self.closed,
        )
        return simplify_polyline(
            smoothed,
            tolerance_mm=simplify_tolerance_mm,
            closed=self.closed,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.path_id,
            "name": self.name,
            "closed": self.closed,
            "drawable": self.drawable,
            "path_kind": self.path_kind,
            "points_map_mm": [_point_to_json(point) for point in self.points_map_mm],
        }

    @classmethod
    def from_dict(cls, value: Any) -> "MapPath":
        if not isinstance(value, dict):
            raise MapModelError("each path must be an object")
        raw_points = value.get("points_map_mm", value.get("points_mm"))
        if not isinstance(raw_points, list):
            raise MapModelError("path.points_map_mm must be a list")
        return cls(
            points_map_mm=tuple(_point_from_json(point, "path point") for point in raw_points),
            path_id=value.get("id", ""),
            name=value.get("name", "Path"),
            closed=value.get("closed", False),
            drawable=value.get("drawable", True),
            # Maps created before path_kind existed are treated as freehand;
            # this keeps their established smoothing behaviour unchanged.
            path_kind=value.get("path_kind", value.get("kind", PATH_KIND_FREEHAND)),
        )


@dataclass(frozen=True)
class MapObstacle:
    """A closed forbidden polygon stored in Map-local millimetres."""

    points_map_mm: tuple[Point2D, ...]
    obstacle_id: str = field(default_factory=lambda: f"obstacle-{uuid4().hex[:8]}")
    name: str = "Obstacle"
    enabled: bool = True

    def __post_init__(self) -> None:
        points = []
        for point in self.points_map_mm:
            if not points or point != points[-1]:
                points.append(point)
        if len(points) > 1 and points[0] == points[-1]:
            points.pop()
        if len(points) < 3:
            raise MapModelError("an obstacle requires at least three distinct points")
        if any(not isinstance(point, Point2D) for point in points):
            raise MapModelError("obstacle points must be Point2D values")
        if points[-1] == points[0]:
            raise MapModelError("obstacle cannot collapse to a repeated point")
        if not isinstance(self.obstacle_id, str) or not self.obstacle_id.strip():
            raise MapModelError("obstacle_id must be a non-empty string")
        if not isinstance(self.name, str) or not self.name.strip():
            raise MapModelError("obstacle name must be a non-empty string")
        if not isinstance(self.enabled, bool):
            raise MapModelError("obstacle enabled must be a boolean")

        edge_count = len(points)
        for first in range(edge_count):
            a, b = points[first], points[(first + 1) % edge_count]
            if a.distance_to(b) <= 1.0e-12:
                raise MapModelError("obstacle edges must have non-zero length")
            for second in range(first + 1, edge_count):
                c, d = points[second], points[(second + 1) % edge_count]
                if second in (first, (first + 1) % edge_count) or first == (second + 1) % edge_count:
                    continue
                if _segments_intersect(a, b, c, d):
                    raise MapModelError("obstacle polygon edges must not self-intersect")
        object.__setattr__(self, "points_map_mm", tuple(points))

    def to_world(self, home: MapHome) -> tuple[Point2D, ...]:
        return tuple(
            Point2D(point.x_mm - home.map_x_mm, point.y_mm - home.map_y_mm)
            for point in self.points_map_mm
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.obstacle_id,
            "name": self.name,
            "enabled": self.enabled,
            "points_map_mm": [_point_to_json(point) for point in self.points_map_mm],
        }

    @classmethod
    def from_dict(cls, value: Any) -> "MapObstacle":
        if not isinstance(value, dict):
            raise MapModelError("each obstacle must be an object")
        raw_points = value.get("points_map_mm", value.get("points_mm"))
        if not isinstance(raw_points, list):
            raise MapModelError("obstacle.points_map_mm must be a list")
        return cls(
            points_map_mm=tuple(_point_from_json(point, "obstacle point") for point in raw_points),
            obstacle_id=value.get("id", ""),
            name=value.get("name", "Obstacle"),
            enabled=value.get("enabled", True),
        )


@dataclass(frozen=True)
class MapLandmark:
    landmark_id: str
    name: str
    map_x_mm: float
    map_y_mm: float

    def __post_init__(self) -> None:
        if not isinstance(self.landmark_id, str) or not self.landmark_id.strip():
            raise MapModelError("landmark_id must be a non-empty string")
        if not isinstance(self.name, str) or not self.name.strip():
            raise MapModelError("landmark name must be a non-empty string")
        object.__setattr__(self, "map_x_mm", _finite("landmark.map_x_mm", self.map_x_mm))
        object.__setattr__(self, "map_y_mm", _finite("landmark.map_y_mm", self.map_y_mm))

    @property
    def point_map(self) -> Point2D:
        return Point2D(self.map_x_mm, self.map_y_mm)

    def to_world(self, home: MapHome) -> Point2D:
        return Point2D(self.map_x_mm - home.map_x_mm, self.map_y_mm - home.map_y_mm)

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.landmark_id,
            "name": self.name,
            "map_x_mm": self.map_x_mm,
            "map_y_mm": self.map_y_mm,
        }

    @classmethod
    def from_dict(cls, value: Any) -> "MapLandmark":
        if not isinstance(value, dict):
            raise MapModelError("each landmark must be an object")
        return cls(
            landmark_id=value.get("id", ""),
            name=value.get("name", "Landmark"),
            map_x_mm=value.get("map_x_mm", value.get("x_mm", 0.0)),
            map_y_mm=value.get("map_y_mm", value.get("y_mm", 0.0)),
        )


@dataclass(frozen=True)
class MapViewState:
    pan_u_px: float = 0.0
    pan_v_px: float = 0.0
    zoom: float = 1.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "pan_u_px", _finite("view.pan_u_px", self.pan_u_px))
        object.__setattr__(self, "pan_v_px", _finite("view.pan_v_px", self.pan_v_px))
        object.__setattr__(self, "zoom", _positive("view.zoom", self.zoom))

    def to_dict(self) -> dict[str, float]:
        return {"pan_u_px": self.pan_u_px, "pan_v_px": self.pan_v_px, "zoom": self.zoom}

    @classmethod
    def from_dict(cls, value: Any) -> "MapViewState":
        if not isinstance(value, dict):
            raise MapModelError("view must be an object")
        return cls(value.get("pan_u_px", 0.0), value.get("pan_v_px", 0.0), value.get("zoom", 1.0))


@dataclass(frozen=True)
class MapDocument:
    name: str
    width_mm: float
    height_mm: float
    home: MapHome = field(default_factory=MapHome)
    paths: tuple[MapPath, ...] = ()
    landmarks: tuple[MapLandmark, ...] = ()
    obstacles: tuple[MapObstacle, ...] = ()
    background: MapBackground | None = None
    view: MapViewState = field(default_factory=MapViewState)
    version: int = MAP_FORMAT_VERSION
    # UI/runtime preferences associated with this map.  The map model keeps
    # this deliberately generic so it can load files without importing the
    # navigation UI (and therefore without introducing a circular import).
    settings: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise MapModelError("map name must be a non-empty string")
        object.__setattr__(self, "width_mm", _positive("workspace.width_mm", self.width_mm))
        object.__setattr__(self, "height_mm", _positive("workspace.height_mm", self.height_mm))
        if self.version != MAP_FORMAT_VERSION:
            raise MapModelError(f"unsupported map version: {self.version}")
        paths = tuple(self.paths)
        landmarks = tuple(self.landmarks)
        obstacles = tuple(self.obstacles)
        if any(not isinstance(path, MapPath) for path in paths):
            raise MapModelError("paths must contain MapPath values")
        if any(not isinstance(item, MapLandmark) for item in landmarks):
            raise MapModelError("landmarks must contain MapLandmark values")
        if any(not isinstance(item, MapObstacle) for item in obstacles):
            raise MapModelError("obstacles must contain MapObstacle values")
        path_ids = [path.path_id for path in paths]
        landmark_ids = [item.landmark_id for item in landmarks]
        obstacle_ids = [item.obstacle_id for item in obstacles]
        if len(path_ids) != len(set(path_ids)):
            raise MapModelError("path IDs must be unique")
        if len(landmark_ids) != len(set(landmark_ids)):
            raise MapModelError("landmark IDs must be unique")
        if len(obstacle_ids) != len(set(obstacle_ids)):
            raise MapModelError("obstacle IDs must be unique")
        if self.background is not None and not isinstance(self.background, MapBackground):
            raise MapModelError("background must be MapBackground or None")
        if not isinstance(self.settings, dict):
            raise MapModelError("settings must be an object")
        try:
            # Validate and copy the metadata through JSON.  This rejects
            # Tk/geometry objects, NaN/Infinity and other values that could
            # not survive a map-file round trip.
            normalized_settings = json.loads(
                json.dumps(self.settings, ensure_ascii=False, allow_nan=False)
            )
        except (TypeError, ValueError) as exc:
            raise MapModelError("settings must contain JSON-compatible values") from exc
        if not isinstance(normalized_settings, dict):
            raise MapModelError("settings must be an object")
        object.__setattr__(self, "settings", normalized_settings)
        object.__setattr__(self, "paths", paths)
        object.__setattr__(self, "landmarks", landmarks)
        object.__setattr__(self, "obstacles", obstacles)

    @classmethod
    def new(cls, name: str = "untitled", width_mm: float = 1200.0, height_mm: float = 800.0) -> "MapDocument":
        return cls(name=name, width_mm=width_mm, height_mm=height_mm)

    def world_point(self, point_map: Point2D) -> Point2D:
        return Point2D(point_map.x_mm - self.home.map_x_mm, point_map.y_mm - self.home.map_y_mm)

    def map_point(self, point_world: Point2D) -> Point2D:
        return Point2D(point_world.x_mm + self.home.map_x_mm, point_world.y_mm + self.home.map_y_mm)

    def path_world_points(self, path_id: str) -> tuple[Point2D, ...]:
        for path in self.paths:
            if path.path_id == path_id:
                return path.to_world(self.home)
        raise MapModelError(f"unknown path ID: {path_id}")

    def with_path(self, path: MapPath) -> "MapDocument":
        replaced = tuple(path if item.path_id == path.path_id else item for item in self.paths)
        if all(item.path_id != path.path_id for item in self.paths):
            replaced += (path,)
        return self._replace(paths=replaced)

    def without_path(self, path_id: str) -> "MapDocument":
        return self._replace(paths=tuple(item for item in self.paths if item.path_id != path_id))

    def with_landmark(self, landmark: MapLandmark) -> "MapDocument":
        replaced = tuple(
            landmark if item.landmark_id == landmark.landmark_id else item
            for item in self.landmarks
        )
        if all(item.landmark_id != landmark.landmark_id for item in self.landmarks):
            replaced += (landmark,)
        return self._replace(landmarks=replaced)

    def without_landmark(self, landmark_id: str) -> "MapDocument":
        return self._replace(
            landmarks=tuple(
                item for item in self.landmarks if item.landmark_id != landmark_id
            )
        )

    def with_obstacle(self, obstacle: MapObstacle) -> "MapDocument":
        replaced = tuple(
            obstacle if item.obstacle_id == obstacle.obstacle_id else item
            for item in self.obstacles
        )
        if all(item.obstacle_id != obstacle.obstacle_id for item in self.obstacles):
            replaced += (obstacle,)
        return self._replace(obstacles=replaced)

    def without_obstacle(self, obstacle_id: str) -> "MapDocument":
        return self._replace(
            obstacles=tuple(
                item for item in self.obstacles if item.obstacle_id != obstacle_id
            )
        )

    def with_home(self, home: MapHome) -> "MapDocument":
        return self._replace(home=home)

    def with_background(self, background: MapBackground | None) -> "MapDocument":
        return self._replace(background=background)

    def _replace(self, **changes: Any) -> "MapDocument":
        values = {
            "name": self.name,
            "width_mm": self.width_mm,
            "height_mm": self.height_mm,
            "home": self.home,
            "paths": self.paths,
            "landmarks": self.landmarks,
            "obstacles": self.obstacles,
            "background": self.background,
            "view": self.view,
            "settings": self.settings,
            "version": self.version,
        }
        values.update(changes)
        return MapDocument(**values)

    def to_dict(self) -> dict[str, Any]:
        return {
            "version": self.version,
            "name": self.name,
            "workspace": {"width_mm": self.width_mm, "height_mm": self.height_mm},
            "view": self.view.to_dict(),
            "home": self.home.to_dict(),
            "background": self.background.to_dict() if self.background is not None else None,
            "settings": self.settings,
            "paths": [path.to_dict() for path in self.paths],
            "landmarks": [item.to_dict() for item in self.landmarks],
            "obstacles": [item.to_dict() for item in self.obstacles],
        }

    @classmethod
    def from_dict(cls, value: Any) -> "MapDocument":
        if not isinstance(value, dict):
            raise MapModelError("map document must be an object")
        workspace = value.get("workspace")
        if not isinstance(workspace, dict):
            raise MapModelError("workspace must be an object")
        raw_paths = value.get("paths", [])
        raw_landmarks = value.get("landmarks", [])
        raw_obstacles = value.get("obstacles", [])
        raw_settings = value.get("settings", {})
        if not isinstance(raw_paths, list) or not isinstance(raw_landmarks, list) or not isinstance(raw_obstacles, list):
            raise MapModelError("paths, landmarks and obstacles must be lists")
        if not isinstance(raw_settings, dict):
            raise MapModelError("settings must be an object")
        raw_background = value.get("background")
        return cls(
            version=value.get("version", MAP_FORMAT_VERSION),
            name=value.get("name", "untitled"),
            width_mm=workspace.get("width_mm"),
            height_mm=workspace.get("height_mm"),
            view=MapViewState.from_dict(value.get("view", {})),
            home=MapHome.from_dict(value.get("home", {})),
            background=MapBackground.from_dict(raw_background) if raw_background is not None else None,
            paths=tuple(MapPath.from_dict(item) for item in raw_paths),
            landmarks=tuple(MapLandmark.from_dict(item) for item in raw_landmarks),
            obstacles=tuple(MapObstacle.from_dict(item) for item in raw_obstacles),
            settings=raw_settings,
        )

    def save(self, path: str | Path) -> Path:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(
            json.dumps(self.to_dict(), ensure_ascii=False, indent=2, sort_keys=False) + "\n",
            encoding="utf-8",
        )
        return destination

    @classmethod
    def load(cls, path: str | Path) -> "MapDocument":
        source = Path(path)
        try:
            value = json.loads(source.read_text(encoding="utf-8"))
        except OSError as exc:
            raise MapModelError(f"cannot read map file: {source}") from exc
        except json.JSONDecodeError as exc:
            raise MapModelError(f"invalid map JSON: {exc.msg}") from exc
        return cls.from_dict(value)


__all__ = [
    "MAP_FORMAT_VERSION",
    "MapBackground",
    "MapDocument",
    "MapHome",
    "MapLandmark",
    "MapModelError",
    "MapPath",
    "MapViewState",
    "MapObstacle",
    "PATH_KIND_FREEHAND",
    "PATH_KIND_POLYLINE",
    "PATH_KINDS",
]
