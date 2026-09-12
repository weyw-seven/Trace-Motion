"""Map-local millimetre to Tk canvas pixel transform."""

from __future__ import annotations

from dataclasses import dataclass
import math

from ..geometry import Point2D


class MapViewError(ValueError):
    """Raised when a map view transform is invalid."""


@dataclass(frozen=True)
class MapViewTransform:
    """Transform Map coordinates (+Y up) into screen coordinates (+Y down)."""

    origin_u_px: float
    origin_v_px: float
    pixels_per_mm: float

    def __post_init__(self) -> None:
        for name in ("origin_u_px", "origin_v_px", "pixels_per_mm"):
            value = float(getattr(self, name))
            if not math.isfinite(value):
                raise MapViewError(f"{name} must be finite")
            object.__setattr__(self, name, value)
        if self.pixels_per_mm <= 0.0:
            raise MapViewError("pixels_per_mm must be positive")

    def map_to_pixel(self, point: Point2D) -> tuple[float, float]:
        return (
            self.origin_u_px + point.x_mm * self.pixels_per_mm,
            self.origin_v_px - point.y_mm * self.pixels_per_mm,
        )

    def pixel_to_map(self, u_px: float, v_px: float) -> Point2D:
        u = float(u_px)
        v = float(v_px)
        if not math.isfinite(u) or not math.isfinite(v):
            raise MapViewError("pixel coordinates must be finite")
        return Point2D(
            (u - self.origin_u_px) / self.pixels_per_mm,
            (self.origin_v_px - v) / self.pixels_per_mm,
        )

    @classmethod
    def fit_workspace(
        cls,
        width_mm: float,
        height_mm: float,
        canvas_width_px: float,
        canvas_height_px: float,
        *,
        margin_px: float = 32.0,
    ) -> "MapViewTransform":
        width = float(width_mm)
        height = float(height_mm)
        canvas_width = float(canvas_width_px)
        canvas_height = float(canvas_height_px)
        margin = float(margin_px)
        if min(width, height, canvas_width, canvas_height) <= 0.0:
            raise MapViewError("workspace and canvas dimensions must be positive")
        available_width = max(1.0, canvas_width - 2.0 * margin)
        available_height = max(1.0, canvas_height - 2.0 * margin)
        scale = min(available_width / width, available_height / height)
        return cls(
            (canvas_width - width * scale) / 2.0,
            (canvas_height + height * scale) / 2.0,
            scale,
        )


__all__ = ["MapViewError", "MapViewTransform"]
