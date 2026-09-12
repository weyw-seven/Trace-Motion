"""Pixel <-> WORLD millimetre transforms for the virtual map editor."""

from __future__ import annotations

from dataclasses import dataclass
import math

from ..geometry import Point2D


class MapTransformError(ValueError):
    """Raised when a map view transform is invalid."""


@dataclass(frozen=True)
class MapTransform:
    """Uniform map transform with WORLD +Y upward and screen Y downward.

    ``origin_u_px`` and ``origin_v_px`` are the screen coordinates of WORLD
    (0, 0). ``mm_per_pixel`` is always positive.
    """

    origin_u_px: float
    origin_v_px: float
    mm_per_pixel: float

    def __post_init__(self) -> None:
        for name in ("origin_u_px", "origin_v_px", "mm_per_pixel"):
            value = float(getattr(self, name))
            if not math.isfinite(value):
                raise MapTransformError(f"{name} must be finite")
            object.__setattr__(self, name, value)
        if self.mm_per_pixel <= 0.0:
            raise MapTransformError("mm_per_pixel must be positive")

    def pixel_to_world(self, u_px: float, v_px: float) -> Point2D:
        u = float(u_px)
        v = float(v_px)
        if not math.isfinite(u) or not math.isfinite(v):
            raise MapTransformError("pixel coordinates must be finite")
        return Point2D(
            (u - self.origin_u_px) * self.mm_per_pixel,
            (self.origin_v_px - v) * self.mm_per_pixel,
        )

    def world_to_pixel(self, point: Point2D) -> tuple[float, float]:
        return (
            self.origin_u_px + point.x_mm / self.mm_per_pixel,
            self.origin_v_px - point.y_mm / self.mm_per_pixel,
        )


__all__ = ["MapTransform", "MapTransformError"]
