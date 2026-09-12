"""
Phase B / B4.1 Toolpath execution IR.

This module represents HOW the robot should execute already-created geometry.

Layering:

    geometry.py
        "What mathematical curve is this?"

    drawing.py
        "Which curves belong to the same drawn Stroke?"

    path_analysis.py
        "What is the geometry/execution quality?"

    toolpath.py
        "Which motion/event records should execute, in what state?"

This module intentionally does NOT:
    - serialize TRJ1/TRJ2 binary
    - define TRJ2 numeric record type IDs
    - order/reverse Strokes
    - compile Drawing -> Toolpath
    - lower CubicBezier to LINE/CIRCLE
    - predict junction speed / duration / braking profile
    - render preview/UI

Important semantic rules:
    - Initial logical pen state is UP.
    - PenUp / PenDown / Wait do not change logical XY.
    - Motion changes logical XY along its Geometry.
    - Whether a Motion is drawing or travel is derived ONLY from pen state.
      Motion therefore has no `is_travel` / `is_drawing` flag.
    - Consecutive Motion geometries must be position-continuous even when
      Event records occur between them. If the next Stroke starts elsewhere,
      a real travel Motion must be present.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Union

from .drawing import PC_CONTINUITY_TOLERANCE_MM
from .geometry import (
    Arc,
    CubicBezier,
    Geometry,
    Line,
    Point2D,
)


class ToolpathError(ValueError):
    """Base error for invalid Toolpath execution IR."""


class MotionContinuityError(ToolpathError):
    """Raised when two Motion records are not position-continuous."""

    def __init__(
        self,
        previous_record_index: int,
        next_record_index: int,
        position_error_mm: float,
        tolerance_mm: float,
    ) -> None:
        self.previous_record_index = previous_record_index
        self.next_record_index = next_record_index
        self.position_error_mm = position_error_mm
        self.tolerance_mm = tolerance_mm

        super().__init__(
            "Toolpath Motion discontinuity between record "
            f"{previous_record_index} and record {next_record_index}: "
            f"position error {position_error_mm:.9f} mm exceeds "
            f"tolerance {tolerance_mm:.9f} mm"
        )


class PenState(str, Enum):
    UP = "UP"
    DOWN = "DOWN"


INITIAL_PEN_STATE = PenState.UP


def _finite(name: str, value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ToolpathError(f"{name} must be finite")
    return value


def _finite_positive(name: str, value: float) -> float:
    value = _finite(name, value)
    if value <= 0.0:
        raise ToolpathError(f"{name} must be > 0")
    return value


def _finite_nonnegative(name: str, value: float) -> float:
    value = _finite(name, value)
    if value < 0.0:
        raise ToolpathError(f"{name} must be >= 0")
    return value


def _validate_tolerance_mm(value: float) -> float:
    value = _finite_nonnegative("continuity_tolerance_mm", value)
    return value


def _is_supported_geometry(value: object) -> bool:
    return isinstance(value, (Line, Arc, CubicBezier))


@dataclass(frozen=True)
class Motion:
    """
    Execute one geometry primitive with trajectory constraints.

    Geometry and execution parameters are deliberately separate:
        Geometry -> where to move.
        Motion   -> how that geometry is constrained for execution.

    `acceleration_mm_s2 == 0` preserves the current trajectory convention:
    no acceleration constraint is specified by this Motion; the executor may
    use its configured default.
    """

    geometry: Geometry
    speed_mm_s: float
    acceleration_mm_s2: float = 0.0

    def __post_init__(self) -> None:
        if not _is_supported_geometry(self.geometry):
            raise ToolpathError(
                f"Motion geometry has unsupported type "
                f"{type(self.geometry).__name__}"
            )

        speed = _finite_positive("speed_mm_s", self.speed_mm_s)
        acceleration = _finite_nonnegative(
            "acceleration_mm_s2",
            self.acceleration_mm_s2,
        )

        object.__setattr__(self, "speed_mm_s", speed)
        object.__setattr__(
            self,
            "acceleration_mm_s2",
            acceleration,
        )

    def start_point(self) -> Point2D:
        return self.geometry.start_point()

    def end_point(self) -> Point2D:
        return self.geometry.end_point()

    def length_mm(self) -> float:
        return self.geometry.length_mm()


@dataclass(frozen=True)
class PenUp:
    """Blocking logical pen-up event. Does not change XY."""


@dataclass(frozen=True)
class PenDown:
    """Blocking logical pen-down event. Does not change XY."""


@dataclass(frozen=True)
class Wait:
    """Blocking wait event. Does not change XY or pen state."""

    duration_s: float

    def __post_init__(self) -> None:
        duration = _finite_positive("duration_s", self.duration_s)
        object.__setattr__(self, "duration_s", duration)


ToolpathRecord = Union[Motion, PenUp, PenDown, Wait]


def _is_toolpath_record(value: object) -> bool:
    return isinstance(value, (Motion, PenUp, PenDown, Wait))


@dataclass(frozen=True)
class Toolpath:
    """
    Ordered execution IR.

    Empty Toolpath is valid.

    Toolpath itself does not own TRJ Header start pose / start_yaw. Its Motion
    geometries contain explicit XY starts/ends. Header pose belongs to a later
    export/execution-configuration layer.
    """

    records: tuple[ToolpathRecord, ...] = ()
    continuity_tolerance_mm: float = PC_CONTINUITY_TOLERANCE_MM

    def __post_init__(self) -> None:
        records = tuple(self.records)
        tolerance = _validate_tolerance_mm(self.continuity_tolerance_mm)

        for index, record in enumerate(records):
            if not _is_toolpath_record(record):
                raise ToolpathError(
                    f"Toolpath record at index {index} has unsupported type "
                    f"{type(record).__name__}"
                )

        object.__setattr__(self, "records", records)
        object.__setattr__(self, "continuity_tolerance_mm", tolerance)

        self.validate_position_continuity()

    @property
    def record_count(self) -> int:
        return len(self.records)

    @property
    def motion_count(self) -> int:
        return sum(isinstance(record, Motion) for record in self.records)

    @property
    def event_count(self) -> int:
        return self.record_count - self.motion_count

    @property
    def wait_count(self) -> int:
        return sum(isinstance(record, Wait) for record in self.records)

    def is_empty(self) -> bool:
        return not self.records

    def has_motion(self) -> bool:
        return any(isinstance(record, Motion) for record in self.records)

    def start_point(self) -> Point2D:
        for record in self.records:
            if isinstance(record, Motion):
                return record.start_point()
        raise ToolpathError("Toolpath has no Motion start point")

    def end_point(self) -> Point2D:
        for record in reversed(self.records):
            if isinstance(record, Motion):
                return record.end_point()
        raise ToolpathError("Toolpath has no Motion end point")

    def validate_position_continuity(
        self,
        *,
        tolerance_mm: float | None = None,
    ) -> None:
        """
        Validate continuity across Motion records.

        Event records are skipped because they do not change logical XY.
        Therefore a jump between two Motions cannot be hidden by PEN/WAIT
        events; a real travel Motion is required.
        """
        tolerance = (
            self.continuity_tolerance_mm
            if tolerance_mm is None
            else _validate_tolerance_mm(tolerance_mm)
        )

        previous_motion: Motion | None = None
        previous_record_index: int | None = None

        for record_index, record in enumerate(self.records):
            if not isinstance(record, Motion):
                continue

            if previous_motion is not None:
                error_mm = previous_motion.end_point().distance_to(
                    record.start_point()
                )
                if error_mm > tolerance:
                    raise MotionContinuityError(
                        previous_record_index=previous_record_index,  # type: ignore[arg-type]
                        next_record_index=record_index,
                        position_error_mm=error_mm,
                        tolerance_mm=tolerance,
                    )

            previous_motion = record
            previous_record_index = record_index

    def final_pen_state(self) -> PenState:
        state = INITIAL_PEN_STATE

        for record in self.records:
            if isinstance(record, PenUp):
                state = PenState.UP
            elif isinstance(record, PenDown):
                state = PenState.DOWN

        return state

    def total_wait_duration_s(self) -> float:
        return sum(
            record.duration_s
            for record in self.records
            if isinstance(record, Wait)
        )

    def total_motion_length_mm(self) -> float:
        return sum(
            record.length_mm()
            for record in self.records
            if isinstance(record, Motion)
        )

    def _motion_lengths_by_pen_state(self) -> tuple[float, float]:
        """
        Return (drawing_length_mm, travel_length_mm).

        State is derived from the record stream. There is no Motion role flag.
        """
        state = INITIAL_PEN_STATE
        drawing = 0.0
        travel = 0.0

        for record in self.records:
            if isinstance(record, PenUp):
                state = PenState.UP
            elif isinstance(record, PenDown):
                state = PenState.DOWN
            elif isinstance(record, Motion):
                if state is PenState.DOWN:
                    drawing += record.length_mm()
                else:
                    travel += record.length_mm()
            # Wait changes neither pen state nor XY.

        return drawing, travel

    def drawing_length_mm(self) -> float:
        drawing, _ = self._motion_lengths_by_pen_state()
        return drawing

    def travel_length_mm(self) -> float:
        _, travel = self._motion_lengths_by_pen_state()
        return travel


__all__ = [
    "INITIAL_PEN_STATE",
    "Motion",
    "MotionContinuityError",
    "PenDown",
    "PenState",
    "PenUp",
    "Toolpath",
    "ToolpathError",
    "ToolpathRecord",
    "Wait",
]
