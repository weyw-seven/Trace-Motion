"""Validated settings for R1. No binary protocol or motor parameters live here."""

from dataclasses import dataclass
import math


class RasterError(ValueError):
    """Invalid raster input, configuration, or coordinate operation."""


def positive_finite(name: str, value: float) -> float:
    if isinstance(value, bool):
        raise RasterError(f"{name} must be a positive finite number")
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise RasterError(f"{name} must be a positive finite number") from exc
    if not math.isfinite(number) or number <= 0:
        raise RasterError(f"{name} must be a positive finite number")
    return number


@dataclass(frozen=True)
class PreprocessConfig:
    """Foreground is normalized grayscale <= threshold.

    invert=True selects light ink on a dark background. Transparency always
    means background. Components use 8-connectivity; min_component_pixels=1
    preserves every foreground pixel. No gap closing or spur trimming occurs.
    """

    threshold: int = 127
    invert: bool = False
    min_component_pixels: int = 1
    max_image_pixels: int = 4_000_000

    def __post_init__(self) -> None:
        if type(self.threshold) is not int or not 0 <= self.threshold <= 254:
            raise RasterError("threshold must be an integer in [0, 254]")
        if type(self.invert) is not bool:
            raise RasterError("invert must be bool")
        for name in ("min_component_pixels", "max_image_pixels"):
            value = getattr(self, name)
            if type(value) is not int or value < 1:
                raise RasterError(f"{name} must be a positive integer")


@dataclass(frozen=True)
class SizeConfig:
    """Fit foreground pixel-center bounds with one uniform mm/pixel scale.

    Default: longest side 100 mm. Supply max_extent_mm OR width_mm/height_mm.
    With both width and height, fit inside that box without stretching.
    """

    max_extent_mm: float | None = None
    width_mm: float | None = None
    height_mm: float | None = None

    def __post_init__(self) -> None:
        if all(getattr(self, n) is None for n in ("max_extent_mm", "width_mm", "height_mm")):
            object.__setattr__(self, "max_extent_mm", 100.0)
        if self.max_extent_mm is not None and (self.width_mm is not None or self.height_mm is not None):
            raise RasterError("use max_extent_mm OR width_mm/height_mm")
        for name in ("max_extent_mm", "width_mm", "height_mm"):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, positive_finite(name, value))
