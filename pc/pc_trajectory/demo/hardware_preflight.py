"""M6 motion-circle-pen trajectory preflight.

The ESP32 remains the authority that accepts or rejects a trajectory.  This
module mirrors the currently deployed M6 limits so the UI can point to the
offending record before opening the binary upload transaction.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping

from ..traj2_format import (
    Trj2CircleRecord,
    Trj2CubicBezierRecord,
    Trj2LineRecord,
    Trj2PenDownRecord,
    Trj2PenUpRecord,
    Trj2WaitRecord,
)
from ..traj2_reader import decode_trj2


@dataclass(frozen=True)
class HardwarePreflight:
    """A review of one exact TRJ2 upload for the current ESP32 profile."""

    record_count: int
    total_distance_mm: float
    estimated_motion_s: float
    estimated_total_s: float
    errors: tuple[str, ...]
    warnings: tuple[str, ...]

    @property
    def ok(self) -> bool:
        return not self.errors

    def summary(self) -> str:
        duration = f"~{self.estimated_total_s:.1f} s"
        return (
            f"{self.record_count} records · {self.total_distance_mm:.0f} mm · "
            f"estimated {duration}"
        )


_M6_PROFILE = "motion-circle-pen"
_M6_MAX_RECORDS = 1024
_M6_MAX_TOTAL_DISTANCE_MM = 30000.0
_M6_MAX_LINE_DISTANCE_MM = 800.0
_M6_MIN_SPEED_MM_S = 30.0
_M6_MAX_SPEED_MM_S = 250.0
_M6_MAX_ACCELERATION_MM_S2 = 1000.0
_M6_MIN_CIRCLE_RADIUS_MM = 30.0
_M6_MAX_CIRCLE_RADIUS_MM = 5000.0
_M6_MAX_CIRCLE_SWEEP_DEG = 360.0
_M6_MAX_ARC_LENGTH_MM = 3000.0
_M6_PEN_ACTION_BUDGET_S = 0.42


def _bool(hello: Mapping[str, Any] | None, key: str) -> bool:
    return bool(hello and hello.get(key) is True)


def preflight_m6_trajectory(
    payload: bytes,
    *,
    hello: Mapping[str, Any] | None = None,
) -> HardwarePreflight:
    """Check exact TRJ2 bytes against the guarded M6 drawing profile."""

    file = decode_trj2(bytes(payload))
    errors: list[str] = []
    warnings: list[str] = []
    profile = str((hello or {}).get("build_profile", "")).strip()

    if hello is None:
        errors.append("ESP32 profile has not been identified; wait for READY")
    elif profile != _M6_PROFILE:
        errors.append(
            f"ESP32 build_profile is {profile or 'unknown'}; M6 requires {_M6_PROFILE}"
        )
    else:
        for key, label in (
            ("hardware_enabled", "hardware"),
            ("motion_enabled", "motion"),
            ("circle_enabled", "circle"),
            ("pen_enabled", "pen"),
        ):
            if not _bool(hello, key):
                errors.append(f"ESP32 {_M6_PROFILE} profile reports {label} disabled")

    if file.record_count > _M6_MAX_RECORDS:
        errors.append(f"record count {file.record_count} exceeds M6 limit {_M6_MAX_RECORDS}")

    current_x = file.header.start_x_mm
    current_y = file.header.start_y_mm
    total_distance = 0.0
    motion_seconds = 0.0
    pen_actions = 0
    wait_seconds = 0.0

    for index, record in enumerate(file.records, start=1):
        if isinstance(record, Trj2LineRecord):
            distance = math.hypot(record.end_x_mm - current_x, record.end_y_mm - current_y)
            current_x, current_y = record.end_x_mm, record.end_y_mm
            if distance <= 0.0:
                errors.append(f"record {index} LINE has zero length")
            elif distance > _M6_MAX_LINE_DISTANCE_MM:
                errors.append(f"record {index} LINE {distance:.1f} mm exceeds {_M6_MAX_LINE_DISTANCE_MM:.0f} mm")
            elif distance < 150.0:
                warnings.append(f"record {index} LINE is {distance:.1f} mm; normal runs should be at least 150 mm")
            total_distance += distance
            if record.speed_mm_s > 0.0:
                motion_seconds += distance / record.speed_mm_s
            _validate_motion(index, record.speed_mm_s, record.acceleration_mm_s2, errors)
        elif isinstance(record, Trj2CircleRecord):
            radius = record.radius_mm
            sweep = abs(record.sweep_deg)
            arc_length = abs(math.radians(record.sweep_deg) * radius)
            if not radius > _M6_MIN_CIRCLE_RADIUS_MM:
                errors.append(f"record {index} CIRCLE radius must exceed {_M6_MIN_CIRCLE_RADIUS_MM:.0f} mm")
            elif radius < 100.0:
                warnings.append(f"record {index} CIRCLE radius {radius:.1f} mm is below the 100 mm normal-run recommendation")
            if radius > _M6_MAX_CIRCLE_RADIUS_MM:
                errors.append(f"record {index} CIRCLE radius {radius:.1f} mm exceeds {_M6_MAX_CIRCLE_RADIUS_MM:.0f} mm")
            if sweep > _M6_MAX_CIRCLE_SWEEP_DEG:
                errors.append(f"record {index} CIRCLE sweep {sweep:.1f}° exceeds {_M6_MAX_CIRCLE_SWEEP_DEG:.0f}°")
            if arc_length > _M6_MAX_ARC_LENGTH_MM:
                errors.append(f"record {index} CIRCLE arc {arc_length:.1f} mm exceeds {_M6_MAX_ARC_LENGTH_MM:.0f} mm")
            total_distance += arc_length
            if record.speed_mm_s > 0.0:
                motion_seconds += arc_length / record.speed_mm_s
            _validate_motion(index, record.speed_mm_s, record.acceleration_mm_s2, errors)
            end_angle = math.radians(record.start_angle_deg + record.sweep_deg)
            current_x = record.center_x_mm + radius * math.cos(end_angle)
            current_y = record.center_y_mm + radius * math.sin(end_angle)
        elif isinstance(record, (Trj2PenUpRecord, Trj2PenDownRecord)):
            pen_actions += 1
        elif isinstance(record, Trj2WaitRecord):
            wait_seconds += record.duration_s
        elif isinstance(record, Trj2CubicBezierRecord):
            errors.append(f"record {index} CUBIC_BEZIER is not supported by M6")
        else:
            errors.append(f"record {index} has unsupported type {type(record).__name__}")

    if total_distance > _M6_MAX_TOTAL_DISTANCE_MM:
        errors.append(f"total path {total_distance:.1f} mm exceeds {_M6_MAX_TOTAL_DISTANCE_MM:.0f} mm")

    estimated_total = motion_seconds + wait_seconds + pen_actions * _M6_PEN_ACTION_BUDGET_S
    if estimated_total > 1200.0:
        errors.append(f"estimated duration {estimated_total:.1f} s exceeds M6 1200 s run limit")
    return HardwarePreflight(
        record_count=file.record_count,
        total_distance_mm=total_distance,
        estimated_motion_s=motion_seconds,
        estimated_total_s=estimated_total,
        errors=tuple(errors),
        warnings=tuple(warnings),
    )


def _validate_motion(index: int, speed: float, acceleration: float, errors: list[str]) -> None:
    if not _M6_MIN_SPEED_MM_S <= speed <= _M6_MAX_SPEED_MM_S:
        errors.append(
            f"record {index} speed {speed:.1f} mm/s is outside "
            f"{_M6_MIN_SPEED_MM_S:.0f}..{_M6_MAX_SPEED_MM_S:.0f} mm/s"
        )
    if not 0.0 < acceleration <= _M6_MAX_ACCELERATION_MM_S2:
        errors.append(
            f"record {index} acceleration {acceleration:.1f} mm/s² is outside "
            f"(0, {_M6_MAX_ACCELERATION_MM_S2:.0f}] mm/s²"
        )


__all__ = ["HardwarePreflight", "preflight_m6_trajectory"]
