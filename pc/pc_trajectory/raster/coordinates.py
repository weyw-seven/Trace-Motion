"""Uniform pixel-center -> WORLD transform. No implicit first-stroke selection.

The caller supplies the anchor pixel after stroke selection (R2). That anchor
maps to WORLD (0,0) by default. Bounds/points use the oriented full-image frame,
not crop-relative coordinates. Cropping never silently changes the origin.
"""

from dataclasses import dataclass
import math

from ..geometry import BoundingBox, Point2D
from .config import RasterError, SizeConfig, positive_finite


@dataclass(frozen=True)
class PixelPoint:
    x: float
    y: float

    def __post_init__(self) -> None:
        for name in ("x", "y"):
            value = getattr(self, name)
            if isinstance(value, bool):
                raise RasterError("pixel coordinates must be finite numbers")
            try:
                value = float(value)
            except (TypeError, ValueError, OverflowError) as exc:
                raise RasterError("pixel coordinates must be finite numbers") from exc
            if not math.isfinite(value):
                raise RasterError("pixel coordinates must be finite numbers")
            object.__setattr__(self, name, value)


@dataclass(frozen=True)
class PixelWorldTransform:
    mm_per_pixel: float
    anchor_pixel: PixelPoint
    anchor_world: Point2D = Point2D(0, 0)

    def __post_init__(self) -> None:
        object.__setattr__(self, "mm_per_pixel", positive_finite("mm_per_pixel", self.mm_per_pixel))
        if not isinstance(self.anchor_pixel, PixelPoint) or not isinstance(self.anchor_world, Point2D):
            raise RasterError("anchor_pixel/anchor_world must be PixelPoint/Point2D")

    def to_world(self, point: PixelPoint) -> Point2D:
        if not isinstance(point, PixelPoint):
            raise RasterError("point must be PixelPoint")
        return Point2D(
            self.anchor_world.x_mm + (point.x - self.anchor_pixel.x) * self.mm_per_pixel,
            self.anchor_world.y_mm - (point.y - self.anchor_pixel.y) * self.mm_per_pixel,
        )

    def to_pixel(self, point: Point2D) -> PixelPoint:
        if not isinstance(point, Point2D):
            raise RasterError("point must be Point2D")
        return PixelPoint(
            self.anchor_pixel.x + (point.x_mm - self.anchor_world.x_mm) / self.mm_per_pixel,
            self.anchor_pixel.y - (point.y_mm - self.anchor_world.y_mm) / self.mm_per_pixel,
        )

    def bounds_in_world(self, bounds: tuple[int, int, int, int]) -> BoundingBox:
        left, top, right, bottom = _validate_bounds(bounds)
        lo = self.to_world(PixelPoint(left, bottom - 1))
        hi = self.to_world(PixelPoint(right - 1, top))
        return BoundingBox(lo.x_mm, lo.y_mm, hi.x_mm, hi.y_mm)

    def as_dict(self) -> dict:
        return {
            "mm_per_pixel": self.mm_per_pixel,
            "anchor_pixel": {"x": self.anchor_pixel.x, "y": self.anchor_pixel.y},
            "anchor_world_mm": {"x": self.anchor_world.x_mm, "y": self.anchor_world.y_mm},
            "pixel_frame": "exif_oriented_full_image_pixel_centers",
            "world_y_direction": "opposite_pixel_y",
        }


def _validate_bounds(bounds) -> tuple[int, int, int, int]:
    if not isinstance(bounds, tuple) or len(bounds) != 4 or any(type(v) is not int for v in bounds):
        raise RasterError("bounds must be a nonempty half-open integer (left, top, right, bottom) tuple")
    left, top, right, bottom = bounds
    if left < 0 or top < 0 or right <= left or bottom <= top:
        raise RasterError("bounds must be a nonempty half-open image rectangle")
    return bounds


def fit_pixel_transform(
    bounds: tuple[int, int, int, int] | None,
    *,
    anchor_pixel: PixelPoint,
    size: SizeConfig | None = None,
    anchor_world: Point2D = Point2D(0, 0),
) -> PixelWorldTransform:
    """Fit cleaned foreground bounds, measured between outer pixel centers.

    A width-only constraint cannot size a vertical path; use max_extent_mm or
    height_mm. A point/empty mask has no scale and is rejected explicitly.
    """
    if bounds is None:
        raise RasterError("empty foreground has no physical extent")
    left, top, right, bottom = _validate_bounds(bounds)
    size = size if size is not None else SizeConfig()
    if not isinstance(size, SizeConfig):
        raise RasterError("size must be SizeConfig")
    dx, dy = right - left - 1, bottom - top - 1
    if max(dx, dy) == 0:
        raise RasterError("single-pixel foreground has no physical extent")
    if size.max_extent_mm is not None:
        scale = size.max_extent_mm / max(dx, dy)
    else:
        candidates = []
        if size.width_mm is not None and dx > 0:
            candidates.append(size.width_mm / dx)
        if size.height_mm is not None and dy > 0:
            candidates.append(size.height_mm / dy)
        if not candidates:
            raise RasterError("requested size axis has zero extent; choose the nonzero axis or max_extent_mm")
        scale = min(candidates)
    return PixelWorldTransform(scale, anchor_pixel, anchor_world)


def fit_pixel_transform_points(
    points,
    *,
    anchor_pixel: PixelPoint,
    size: SizeConfig | None = None,
    anchor_world: Point2D = Point2D(0, 0),
) -> PixelWorldTransform:
    """Fit a transform from the actual traced point extent.

    ``fit_pixel_transform`` intentionally uses image-mask bounds, which are
    useful for R1 diagnostics.  A trajectory should be sized from the final
    skeleton points instead: thick source ink must not consume physical
    drawing area.  This helper accepts any iterable of :class:`PixelPoint` and
    measures the inclusive point-to-point extent directly.
    """
    if not isinstance(anchor_pixel, PixelPoint):
        raise RasterError("anchor_pixel must be PixelPoint")
    try:
        values = tuple(points)
    except TypeError as exc:
        raise RasterError("points must be an iterable of PixelPoint") from exc
    if not values or any(not isinstance(point, PixelPoint) for point in values):
        raise RasterError("points must contain at least one PixelPoint")
    size = size if size is not None else SizeConfig()
    if not isinstance(size, SizeConfig):
        raise RasterError("size must be SizeConfig")
    width_px = max(point.x for point in values) - min(point.x for point in values)
    height_px = max(point.y for point in values) - min(point.y for point in values)
    if max(width_px, height_px) <= 0.0:
        raise RasterError("single-point foreground has no physical extent")
    if size.max_extent_mm is not None:
        scale = size.max_extent_mm / max(width_px, height_px)
    else:
        candidates = []
        if size.width_mm is not None and width_px > 0.0:
            candidates.append(size.width_mm / width_px)
        if size.height_mm is not None and height_px > 0.0:
            candidates.append(size.height_mm / height_px)
        if not candidates:
            raise RasterError(
                "requested size axis has zero extent; choose the nonzero axis or max_extent_mm"
            )
        scale = min(candidates)
    return PixelWorldTransform(scale, anchor_pixel, anchor_world)


__all__ = [
    "PixelPoint",
    "PixelWorldTransform",
    "fit_pixel_transform",
    "fit_pixel_transform_points",
]
