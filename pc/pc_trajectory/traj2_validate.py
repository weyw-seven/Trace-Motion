"""
TRJ2 semantic and canonical validation.

This module validates both:
    - high-level Trj2File model objects
    - strict raw header/record fields read from binary

Important distinction:
    These are PC-side TRJ2 canonical rules. There is no TRJ2 ESP32 firmware
    implementation yet, so numeric epsilons here must not be described as
    firmware behavior.

Position continuity target:
    0.01 mm

This is intentionally much stricter than the legacy TRJ1 firmware tolerance.
"""

from __future__ import annotations

import math

from .traj2_format import (
    TRJ2_HEADER_SIZE,
    TRJ2_MAGIC,
    TRJ2_RECORD_SIZE,
    TRJ2_VERSION,
    Trj2CircleRecord,
    Trj2CubicBezierRecord,
    Trj2File,
    Trj2Header,
    Trj2LineRecord,
    Trj2PenDownRecord,
    Trj2PenUpRecord,
    Trj2RawHeader,
    Trj2RawRecord,
    Trj2Record,
    Trj2RecordType,
    Trj2WaitRecord,
)


TRJ2_CONTINUITY_TOLERANCE_MM = 0.01
TRJ2_NUMERIC_EPSILON = 1.0e-12
TRJ2_SWEEP_EPSILON_DEG = 1.0e-6


class Trj2ValidationError(ValueError):
    """Base error for invalid TRJ2 model or canonical binary semantics."""


class Trj2ContinuityError(Trj2ValidationError):
    def __init__(
        self,
        *,
        record_index: int,
        position_error_mm: float,
        tolerance_mm: float,
    ) -> None:
        self.record_index = record_index
        self.position_error_mm = position_error_mm
        self.tolerance_mm = tolerance_mm

        super().__init__(
            f"TRJ2 record {record_index} start position error "
            f"{position_error_mm:.9f} mm exceeds tolerance "
            f"{tolerance_mm:.9f} mm"
        )


def _finite(name: str, value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise Trj2ValidationError(f"{name} must be finite")
    return value


def _finite_positive(name: str, value: float) -> float:
    value = _finite(name, value)
    if value <= 0.0:
        raise Trj2ValidationError(f"{name} must be > 0")
    return value


def _finite_nonnegative(name: str, value: float) -> float:
    value = _finite(name, value)
    if value < 0.0:
        raise Trj2ValidationError(f"{name} must be >= 0")
    return value


def _distance(
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> float:
    return math.hypot(ax - bx, ay - by)


def validate_header(header: Trj2Header) -> None:
    if not isinstance(header, Trj2Header):
        raise Trj2ValidationError("header must be Trj2Header")

    _finite("header.start_x_mm", header.start_x_mm)
    _finite("header.start_y_mm", header.start_y_mm)
    _finite("header.start_yaw_deg", header.start_yaw_deg)


def _validate_motion_common(
    *,
    prefix: str,
    speed_mm_s: float,
    acceleration_mm_s2: float,
) -> None:
    _finite_positive(f"{prefix}.speed_mm_s", speed_mm_s)
    _finite_nonnegative(
        f"{prefix}.acceleration_mm_s2",
        acceleration_mm_s2,
    )


def validate_record(record: Trj2Record) -> None:
    if isinstance(record, Trj2LineRecord):
        _validate_motion_common(
            prefix="LINE",
            speed_mm_s=record.speed_mm_s,
            acceleration_mm_s2=record.acceleration_mm_s2,
        )
        _finite("LINE.end_x_mm", record.end_x_mm)
        _finite("LINE.end_y_mm", record.end_y_mm)
        return

    if isinstance(record, Trj2CircleRecord):
        _validate_motion_common(
            prefix="CIRCLE",
            speed_mm_s=record.speed_mm_s,
            acceleration_mm_s2=record.acceleration_mm_s2,
        )
        _finite("CIRCLE.center_x_mm", record.center_x_mm)
        _finite("CIRCLE.center_y_mm", record.center_y_mm)
        _finite_positive("CIRCLE.radius_mm", record.radius_mm)
        _finite("CIRCLE.start_angle_deg", record.start_angle_deg)
        sweep = _finite("CIRCLE.sweep_deg", record.sweep_deg)
        if abs(sweep) <= TRJ2_SWEEP_EPSILON_DEG:
            raise Trj2ValidationError(
                "CIRCLE.sweep_deg magnitude must exceed "
                f"{TRJ2_SWEEP_EPSILON_DEG} deg"
            )
        return

    if isinstance(record, Trj2CubicBezierRecord):
        _validate_motion_common(
            prefix="CUBIC_BEZIER",
            speed_mm_s=record.speed_mm_s,
            acceleration_mm_s2=record.acceleration_mm_s2,
        )
        for name, value in (
            ("control1_x_mm", record.control1_x_mm),
            ("control1_y_mm", record.control1_y_mm),
            ("control2_x_mm", record.control2_x_mm),
            ("control2_y_mm", record.control2_y_mm),
            ("end_x_mm", record.end_x_mm),
            ("end_y_mm", record.end_y_mm),
        ):
            _finite(f"CUBIC_BEZIER.{name}", value)
        return

    if isinstance(record, (Trj2PenUpRecord, Trj2PenDownRecord)):
        return

    if isinstance(record, Trj2WaitRecord):
        _finite_positive("WAIT.duration_s", record.duration_s)
        return

    raise Trj2ValidationError(
        f"Unsupported TRJ2 record object {type(record).__name__}"
    )


def validate_trj2_file(
    file: Trj2File,
    *,
    continuity_tolerance_mm: float = TRJ2_CONTINUITY_TOLERANCE_MM,
) -> None:
    if not isinstance(file, Trj2File):
        raise Trj2ValidationError("file must be Trj2File")

    tolerance = _finite_nonnegative(
        "continuity_tolerance_mm",
        continuity_tolerance_mm,
    )

    validate_header(file.header)

    current_x = float(file.header.start_x_mm)
    current_y = float(file.header.start_y_mm)

    for index, record in enumerate(file.records):
        validate_record(record)

        if isinstance(record, Trj2LineRecord):
            length = _distance(
                current_x,
                current_y,
                record.end_x_mm,
                record.end_y_mm,
            )
            if length <= TRJ2_NUMERIC_EPSILON:
                raise Trj2ValidationError(
                    f"LINE record {index} has zero length"
                )

            current_x = float(record.end_x_mm)
            current_y = float(record.end_y_mm)
            continue

        if isinstance(record, Trj2CircleRecord):
            theta0 = math.radians(record.start_angle_deg)

            start_x = (
                record.center_x_mm
                + record.radius_mm * math.cos(theta0)
            )
            start_y = (
                record.center_y_mm
                + record.radius_mm * math.sin(theta0)
            )

            error = _distance(
                current_x,
                current_y,
                start_x,
                start_y,
            )

            if error > tolerance:
                raise Trj2ContinuityError(
                    record_index=index,
                    position_error_mm=error,
                    tolerance_mm=tolerance,
                )

            theta1 = math.radians(
                record.start_angle_deg + record.sweep_deg
            )

            current_x = (
                record.center_x_mm
                + record.radius_mm * math.cos(theta1)
            )
            current_y = (
                record.center_y_mm
                + record.radius_mm * math.sin(theta1)
            )
            continue

        if isinstance(record, Trj2CubicBezierRecord):
            # P0 is current logical XY. A closed Bezier P0 == P3 is legal.
            # Reject only a mathematically collapsed four-point curve.
            p1_error = _distance(
                current_x,
                current_y,
                record.control1_x_mm,
                record.control1_y_mm,
            )
            p2_error = _distance(
                current_x,
                current_y,
                record.control2_x_mm,
                record.control2_y_mm,
            )
            p3_error = _distance(
                current_x,
                current_y,
                record.end_x_mm,
                record.end_y_mm,
            )

            if (
                p1_error <= TRJ2_NUMERIC_EPSILON
                and p2_error <= TRJ2_NUMERIC_EPSILON
                and p3_error <= TRJ2_NUMERIC_EPSILON
            ):
                raise Trj2ValidationError(
                    f"CUBIC_BEZIER record {index} is fully collapsed"
                )

            current_x = float(record.end_x_mm)
            current_y = float(record.end_y_mm)
            continue

        # PEN_UP / PEN_DOWN / WAIT are zero-displacement Events.


def validate_raw_header(raw: Trj2RawHeader) -> None:
    if raw.magic != TRJ2_MAGIC:
        raise Trj2ValidationError(
            f"bad TRJ2 magic {raw.magic!r}"
        )

    if raw.version != TRJ2_VERSION:
        raise Trj2ValidationError(
            f"unsupported TRJ2 version {raw.version}"
        )

    if raw.header_size != TRJ2_HEADER_SIZE:
        raise Trj2ValidationError(
            f"canonical TRJ2 header_size must be "
            f"{TRJ2_HEADER_SIZE}, got {raw.header_size}"
        )

    if raw.record_size != TRJ2_RECORD_SIZE:
        raise Trj2ValidationError(
            f"canonical TRJ2 record_size must be "
            f"{TRJ2_RECORD_SIZE}, got {raw.record_size}"
        )

    if raw.flags != 0:
        raise Trj2ValidationError("TRJ2 header flags must be 0")

    if raw.reserved != 0:
        raise Trj2ValidationError("TRJ2 header reserved must be 0")

    _finite("header.start_x_mm", raw.start_x_mm)
    _finite("header.start_y_mm", raw.start_y_mm)
    _finite("header.start_yaw_deg", raw.start_yaw_deg)


def _require_zero(
    name: str,
    value: float,
) -> None:
    _finite(name, value)
    if value != 0.0:
        raise Trj2ValidationError(f"{name} must be 0")


def _require_payload_zero(
    raw: Trj2RawRecord,
    *,
    start_index: int,
    record_name: str,
) -> None:
    for index in range(start_index, 8):
        _require_zero(
            f"{record_name}.data[{index}]",
            raw.data[index],
        )


def validate_raw_record(
    raw: Trj2RawRecord,
    *,
    record_index: int,
) -> None:
    if raw.flags != 0:
        raise Trj2ValidationError(
            f"record {record_index} flags must be 0"
        )

    if raw.reserved != 0:
        raise Trj2ValidationError(
            f"record {record_index} reserved must be 0"
        )

    try:
        record_type = Trj2RecordType(raw.type_value)
    except ValueError as exc:
        raise Trj2ValidationError(
            f"record {record_index} has unknown type "
            f"0x{raw.type_value:02X}"
        ) from exc

    if record_type is Trj2RecordType.NONE:
        raise Trj2ValidationError(
            f"record {record_index}: NONE is not a canonical TRJ2 record"
        )

    for data_index, value in enumerate(raw.data):
        _finite(
            f"record[{record_index}].data[{data_index}]",
            value,
        )

    if record_type in (
        Trj2RecordType.LINE,
        Trj2RecordType.CIRCLE,
        Trj2RecordType.CUBIC_BEZIER,
    ):
        _finite_positive(
            f"record[{record_index}].speed_mm_s",
            raw.speed_mm_s,
        )
        _finite_nonnegative(
            f"record[{record_index}].acceleration_mm_s2",
            raw.acceleration_mm_s2,
        )

        if record_type is Trj2RecordType.LINE:
            _require_payload_zero(
                raw,
                start_index=2,
                record_name=f"LINE record {record_index}",
            )

        elif record_type is Trj2RecordType.CIRCLE:
            _require_payload_zero(
                raw,
                start_index=5,
                record_name=f"CIRCLE record {record_index}",
            )

        else:
            _require_payload_zero(
                raw,
                start_index=6,
                record_name=f"CUBIC_BEZIER record {record_index}",
            )

        return

    # Events are canonical zero-speed / zero-acceleration barriers.
    _require_zero(
        f"event record[{record_index}].speed_mm_s",
        raw.speed_mm_s,
    )
    _require_zero(
        f"event record[{record_index}].acceleration_mm_s2",
        raw.acceleration_mm_s2,
    )

    if record_type in (
        Trj2RecordType.PEN_UP,
        Trj2RecordType.PEN_DOWN,
    ):
        _require_payload_zero(
            raw,
            start_index=0,
            record_name=f"{record_type.name} record {record_index}",
        )
        return

    if record_type is Trj2RecordType.WAIT:
        _finite_positive(
            f"WAIT record[{record_index}].duration_s",
            raw.data[0],
        )
        _require_payload_zero(
            raw,
            start_index=1,
            record_name=f"WAIT record {record_index}",
        )
        return

    raise Trj2ValidationError(
        f"record {record_index}: unsupported type {record_type.name}"
    )


__all__ = [
    "TRJ2_CONTINUITY_TOLERANCE_MM",
    "TRJ2_NUMERIC_EPSILON",
    "TRJ2_SWEEP_EPSILON_DEG",
    "Trj2ContinuityError",
    "Trj2ValidationError",
    "validate_header",
    "validate_raw_header",
    "validate_raw_record",
    "validate_record",
    "validate_trj2_file",
]
