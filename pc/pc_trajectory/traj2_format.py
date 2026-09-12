"""
Canonical TRJ2 binary format definitions.

TRJ2 keeps the proven TRJ1 framing:

    32-byte header
    N x 44-byte records

but changes the record-count semantics and adds Motion/Event record types.

This module is the ONLY B6 module that owns struct format strings and the
low-level pack/unpack mapping. Higher layers must not duplicate binary layout.

Coordinate convention:
    Trajectory WORLD frame
    +X = robot forward when chassis yaw = 0 deg
    +Y = robot left when chassis yaw = 0 deg
    positive angle / sweep = CCW

Units:
    position / radius       mm
    speed                   mm/s
    acceleration            mm/s^2
    angle / sweep           degree
    WAIT duration           second
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct
from typing import Union


TRJ2_MAGIC = b"TRJ2"
TRJ2_VERSION = 2

TRJ2_HEADER_SIZE = 32
TRJ2_RECORD_SIZE = 44

TRJ2_HEADER_FMT = "<4sHHIHHfffI"
TRJ2_RECORD_FMT = "<BBHff8f"

assert struct.calcsize(TRJ2_HEADER_FMT) == TRJ2_HEADER_SIZE
assert struct.calcsize(TRJ2_RECORD_FMT) == TRJ2_RECORD_SIZE


class Trj2RecordType(IntEnum):
    NONE = 0x00

    LINE = 0x01
    CIRCLE = 0x02
    CUBIC_BEZIER = 0x03

    PEN_UP = 0x20
    PEN_DOWN = 0x21
    WAIT = 0x22


@dataclass(frozen=True)
class Trj2Header:
    start_x_mm: float
    start_y_mm: float
    start_yaw_deg: float


@dataclass(frozen=True)
class Trj2LineRecord:
    end_x_mm: float
    end_y_mm: float
    speed_mm_s: float
    acceleration_mm_s2: float = 0.0


@dataclass(frozen=True)
class Trj2CircleRecord:
    center_x_mm: float
    center_y_mm: float
    radius_mm: float
    start_angle_deg: float
    sweep_deg: float
    speed_mm_s: float
    acceleration_mm_s2: float = 0.0


@dataclass(frozen=True)
class Trj2CubicBezierRecord:
    """
    P0 is implicit: current logical XY before this record.

    Payload:
        data[0:2] = P1 / control1
        data[2:4] = P2 / control2
        data[4:6] = P3 / endpoint
    """

    control1_x_mm: float
    control1_y_mm: float

    control2_x_mm: float
    control2_y_mm: float

    end_x_mm: float
    end_y_mm: float

    speed_mm_s: float
    acceleration_mm_s2: float = 0.0


@dataclass(frozen=True)
class Trj2PenUpRecord:
    pass


@dataclass(frozen=True)
class Trj2PenDownRecord:
    pass


@dataclass(frozen=True)
class Trj2WaitRecord:
    duration_s: float


Trj2Record = Union[
    Trj2LineRecord,
    Trj2CircleRecord,
    Trj2CubicBezierRecord,
    Trj2PenUpRecord,
    Trj2PenDownRecord,
    Trj2WaitRecord,
]


@dataclass(frozen=True)
class Trj2File:
    header: Trj2Header
    records: tuple[Trj2Record, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "records", tuple(self.records))

    @property
    def record_count(self) -> int:
        return len(self.records)


@dataclass(frozen=True)
class Trj2RawHeader:
    magic: bytes
    version: int
    header_size: int
    record_count: int
    record_size: int
    flags: int
    start_x_mm: float
    start_y_mm: float
    start_yaw_deg: float
    reserved: int


@dataclass(frozen=True)
class Trj2RawRecord:
    type_value: int
    flags: int
    reserved: int
    speed_mm_s: float
    acceleration_mm_s2: float
    data: tuple[float, float, float, float, float, float, float, float]


def pack_header(
    header: Trj2Header,
    *,
    record_count: int,
) -> bytes:
    """Pack canonical TRJ2 header fields."""
    return struct.pack(
        TRJ2_HEADER_FMT,
        TRJ2_MAGIC,
        TRJ2_VERSION,
        TRJ2_HEADER_SIZE,
        record_count,
        TRJ2_RECORD_SIZE,
        0,
        header.start_x_mm,
        header.start_y_mm,
        header.start_yaw_deg,
        0,
    )


def unpack_header_raw(data: bytes) -> Trj2RawHeader:
    if len(data) != TRJ2_HEADER_SIZE:
        raise ValueError(
            f"TRJ2 header buffer must be exactly {TRJ2_HEADER_SIZE} bytes"
        )

    values = struct.unpack(TRJ2_HEADER_FMT, data)

    return Trj2RawHeader(
        magic=values[0],
        version=values[1],
        header_size=values[2],
        record_count=values[3],
        record_size=values[4],
        flags=values[5],
        start_x_mm=values[6],
        start_y_mm=values[7],
        start_yaw_deg=values[8],
        reserved=values[9],
    )


def header_from_raw(raw: Trj2RawHeader) -> Trj2Header:
    return Trj2Header(
        start_x_mm=raw.start_x_mm,
        start_y_mm=raw.start_y_mm,
        start_yaw_deg=raw.start_yaw_deg,
    )


def _record_payload(
    record: Trj2Record,
) -> tuple[
    Trj2RecordType,
    float,
    float,
    tuple[float, float, float, float, float, float, float, float],
]:
    if isinstance(record, Trj2LineRecord):
        return (
            Trj2RecordType.LINE,
            record.speed_mm_s,
            record.acceleration_mm_s2,
            (
                record.end_x_mm,
                record.end_y_mm,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ),
        )

    if isinstance(record, Trj2CircleRecord):
        return (
            Trj2RecordType.CIRCLE,
            record.speed_mm_s,
            record.acceleration_mm_s2,
            (
                record.center_x_mm,
                record.center_y_mm,
                record.radius_mm,
                record.start_angle_deg,
                record.sweep_deg,
                0.0,
                0.0,
                0.0,
            ),
        )

    if isinstance(record, Trj2CubicBezierRecord):
        return (
            Trj2RecordType.CUBIC_BEZIER,
            record.speed_mm_s,
            record.acceleration_mm_s2,
            (
                record.control1_x_mm,
                record.control1_y_mm,
                record.control2_x_mm,
                record.control2_y_mm,
                record.end_x_mm,
                record.end_y_mm,
                0.0,
                0.0,
            ),
        )

    if isinstance(record, Trj2PenUpRecord):
        return (
            Trj2RecordType.PEN_UP,
            0.0,
            0.0,
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )

    if isinstance(record, Trj2PenDownRecord):
        return (
            Trj2RecordType.PEN_DOWN,
            0.0,
            0.0,
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )

    if isinstance(record, Trj2WaitRecord):
        return (
            Trj2RecordType.WAIT,
            0.0,
            0.0,
            (
                record.duration_s,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ),
        )

    raise TypeError(
        f"Unsupported TRJ2 record type {type(record).__name__}"
    )


def pack_record(record: Trj2Record) -> bytes:
    record_type, speed, acceleration, payload = _record_payload(record)

    return struct.pack(
        TRJ2_RECORD_FMT,
        int(record_type),
        0,
        0,
        speed,
        acceleration,
        *payload,
    )


def unpack_record_raw(data: bytes) -> Trj2RawRecord:
    if len(data) != TRJ2_RECORD_SIZE:
        raise ValueError(
            f"TRJ2 record buffer must be exactly {TRJ2_RECORD_SIZE} bytes"
        )

    values = struct.unpack(TRJ2_RECORD_FMT, data)

    return Trj2RawRecord(
        type_value=values[0],
        flags=values[1],
        reserved=values[2],
        speed_mm_s=values[3],
        acceleration_mm_s2=values[4],
        data=tuple(values[5:13]),
    )


def record_from_raw(raw: Trj2RawRecord) -> Trj2Record:
    try:
        record_type = Trj2RecordType(raw.type_value)
    except ValueError as exc:
        raise ValueError(
            f"Unknown TRJ2 record type 0x{raw.type_value:02X}"
        ) from exc

    d = raw.data

    if record_type is Trj2RecordType.LINE:
        return Trj2LineRecord(
            end_x_mm=d[0],
            end_y_mm=d[1],
            speed_mm_s=raw.speed_mm_s,
            acceleration_mm_s2=raw.acceleration_mm_s2,
        )

    if record_type is Trj2RecordType.CIRCLE:
        return Trj2CircleRecord(
            center_x_mm=d[0],
            center_y_mm=d[1],
            radius_mm=d[2],
            start_angle_deg=d[3],
            sweep_deg=d[4],
            speed_mm_s=raw.speed_mm_s,
            acceleration_mm_s2=raw.acceleration_mm_s2,
        )

    if record_type is Trj2RecordType.CUBIC_BEZIER:
        return Trj2CubicBezierRecord(
            control1_x_mm=d[0],
            control1_y_mm=d[1],
            control2_x_mm=d[2],
            control2_y_mm=d[3],
            end_x_mm=d[4],
            end_y_mm=d[5],
            speed_mm_s=raw.speed_mm_s,
            acceleration_mm_s2=raw.acceleration_mm_s2,
        )

    if record_type is Trj2RecordType.PEN_UP:
        return Trj2PenUpRecord()

    if record_type is Trj2RecordType.PEN_DOWN:
        return Trj2PenDownRecord()

    if record_type is Trj2RecordType.WAIT:
        return Trj2WaitRecord(duration_s=d[0])

    # NONE is known numerically but is not a valid canonical file record.
    raise ValueError(
        f"TRJ2 record type {record_type.name} is not a canonical file record"
    )


__all__ = [
    "TRJ2_HEADER_FMT",
    "TRJ2_HEADER_SIZE",
    "TRJ2_MAGIC",
    "TRJ2_RECORD_FMT",
    "TRJ2_RECORD_SIZE",
    "TRJ2_VERSION",
    "Trj2CircleRecord",
    "Trj2CubicBezierRecord",
    "Trj2File",
    "Trj2Header",
    "Trj2LineRecord",
    "Trj2PenDownRecord",
    "Trj2PenUpRecord",
    "Trj2RawHeader",
    "Trj2RawRecord",
    "Trj2Record",
    "Trj2RecordType",
    "Trj2WaitRecord",
    "header_from_raw",
    "pack_header",
    "pack_record",
    "record_from_raw",
    "unpack_header_raw",
    "unpack_record_raw",
]
