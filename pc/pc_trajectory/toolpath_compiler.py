"""
Phase B / B4.2 Drawing -> Toolpath compiler.

This module bridges:

    Drawing / Stroke / Geometry
                ↓
         Toolpath execution IR

It performs deterministic execution lowering only.

Current baseline policy:
    - Preserve Drawing.strokes order exactly.
    - Preserve geometry order inside each Stroke exactly.
    - Initial logical pen state is already UP.
    - Before each Stroke: PEN_DOWN.
    - After each Stroke: PEN_UP.
    - Between different Stroke endpoints:
        insert one straight pen-up Travel Motion when needed.
    - If adjacent Stroke endpoint/start are already within configured PC
      position tolerance, omit the zero/tiny Travel Motion but preserve the
      PEN_UP -> PEN_DOWN Stroke boundary.
    - Drawing Motion uses drawing speed/acceleration config.
    - Travel Motion uses travel speed/acceleration config.
    - CubicBezier remains CubicBezier Motion. No curve lowering happens here.

This module intentionally does NOT:
    - reorder Strokes
    - reverse Strokes
    - solve TSP / nearest-neighbor travel optimization
    - insert automatic Wait events
    - modify geometry to smooth corners
    - assign speed from curvature/path analysis
    - lower CubicBezier to LINE/CIRCLE
    - serialize TRJ1/TRJ2
    - define binary type IDs
    - infer chassis start_yaw
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from .drawing import Drawing, PC_CONTINUITY_TOLERANCE_MM, Stroke
from .geometry import Line
from .toolpath import Motion, PenDown, PenUp, Toolpath, ToolpathRecord


class ToolpathCompilerError(ValueError):
    """Raised for invalid compiler configuration or input."""


def _finite(name: str, value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ToolpathCompilerError(f"{name} must be finite")
    return value


def _finite_positive(name: str, value: float) -> float:
    value = _finite(name, value)
    if value <= 0.0:
        raise ToolpathCompilerError(f"{name} must be > 0")
    return value


def _finite_nonnegative(name: str, value: float) -> float:
    value = _finite(name, value)
    if value < 0.0:
        raise ToolpathCompilerError(f"{name} must be >= 0")
    return value


@dataclass(frozen=True)
class ToolpathCompilerConfig:
    """
    Baseline execution constraints for Drawing -> Toolpath lowering.

    These are compiler defaults, not geometry properties.

    acceleration == 0 retains the project convention:
        no explicit acceleration constraint; executor may use its default.
    """

    drawing_speed_mm_s: float = 300.0
    drawing_acceleration_mm_s2: float = 0.0

    travel_speed_mm_s: float = 500.0
    travel_acceleration_mm_s2: float = 0.0

    position_tolerance_mm: float = PC_CONTINUITY_TOLERANCE_MM

    def __post_init__(self) -> None:
        drawing_speed = _finite_positive(
            "drawing_speed_mm_s",
            self.drawing_speed_mm_s,
        )
        drawing_acceleration = _finite_nonnegative(
            "drawing_acceleration_mm_s2",
            self.drawing_acceleration_mm_s2,
        )
        travel_speed = _finite_positive(
            "travel_speed_mm_s",
            self.travel_speed_mm_s,
        )
        travel_acceleration = _finite_nonnegative(
            "travel_acceleration_mm_s2",
            self.travel_acceleration_mm_s2,
        )
        tolerance = _finite_nonnegative(
            "position_tolerance_mm",
            self.position_tolerance_mm,
        )

        object.__setattr__(self, "drawing_speed_mm_s", drawing_speed)
        object.__setattr__(
            self,
            "drawing_acceleration_mm_s2",
            drawing_acceleration,
        )
        object.__setattr__(self, "travel_speed_mm_s", travel_speed)
        object.__setattr__(
            self,
            "travel_acceleration_mm_s2",
            travel_acceleration,
        )
        object.__setattr__(self, "position_tolerance_mm", tolerance)


class ToolpathCompiler:
    """
    Deterministic baseline Drawing -> Toolpath compiler.

    The compiler is intentionally stateless across compile() calls.
    """

    def __init__(
        self,
        config: ToolpathCompilerConfig | None = None,
    ) -> None:
        if config is None:
            config = ToolpathCompilerConfig()
        if not isinstance(config, ToolpathCompilerConfig):
            raise ToolpathCompilerError(
                "config must be a ToolpathCompilerConfig"
            )
        self._config = config

    @property
    def config(self) -> ToolpathCompilerConfig:
        return self._config

    def _drawing_motion(self, geometry) -> Motion:
        return Motion(
            geometry=geometry,
            speed_mm_s=self._config.drawing_speed_mm_s,
            acceleration_mm_s2=self._config.drawing_acceleration_mm_s2,
        )

    def _travel_motion(self, start, end) -> Motion:
        return Motion(
            geometry=Line(start, end),
            speed_mm_s=self._config.travel_speed_mm_s,
            acceleration_mm_s2=self._config.travel_acceleration_mm_s2,
        )

    def _append_stroke_motions(
        self,
        records: list[ToolpathRecord],
        stroke: Stroke,
    ) -> None:
        records.append(PenDown())

        for geometry in stroke.geometries:
            records.append(self._drawing_motion(geometry))

        records.append(PenUp())

    def compile(self, drawing: Drawing) -> Toolpath:
        if not isinstance(drawing, Drawing):
            raise ToolpathCompilerError("drawing must be a Drawing")

        if drawing.is_empty():
            return Toolpath(
                (),
                continuity_tolerance_mm=self._config.position_tolerance_mm,
            )

        records: list[ToolpathRecord] = []
        previous_stroke: Stroke | None = None

        for stroke in drawing.strokes:
            if previous_stroke is not None:
                previous_end = previous_stroke.end_point()
                next_start = stroke.start_point()
                gap_mm = previous_end.distance_to(next_start)

                # Preserve the Stroke semantic boundary in all cases.
                # The previous Stroke already emitted PenUp. If its endpoint
                # differs from the next start, insert a real pen-up Travel
                # Motion before PenDown of the next Stroke.
                if gap_mm > self._config.position_tolerance_mm:
                    records.append(
                        self._travel_motion(previous_end, next_start)
                    )

            self._append_stroke_motions(records, stroke)
            previous_stroke = stroke

        # Toolpath performs its own Motion-continuity validation here.
        return Toolpath(
            tuple(records),
            continuity_tolerance_mm=self._config.position_tolerance_mm,
        )


def compile_drawing(
    drawing: Drawing,
    config: ToolpathCompilerConfig | None = None,
) -> Toolpath:
    """Convenience functional API for deterministic Drawing -> Toolpath compile."""
    return ToolpathCompiler(config).compile(drawing)


__all__ = [
    "ToolpathCompiler",
    "ToolpathCompilerConfig",
    "ToolpathCompilerError",
    "compile_drawing",
]
