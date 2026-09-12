from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct
from typing import TypeAlias


# ---------------------------------------------------------------------------
# Canonical TRJ1 V1 wire format
# ---------------------------------------------------------------------------

MAGIC = b"TRJ1"
VERSION = 1

HEADER_SIZE = 32
RECORD_SIZE = 44

HEADER_FMT = "<4sHHIHHfffI"
RECORD_FMT = "<BBHff8f"

HEADER_FLAGS_V1 = 0
HEADER_RESERVED_V1 = 0
RECORD_FLAGS_V1 = 0
RECORD_RESERVED_V1 = 0

SEGMENT_DATA_FLOATS = 8

# Firmware-side geometry thresholds mirrored for save-time rejection.
# These are not path-planning parameters.
DECODER_CIRCLE_SWEEP_EPSILON_DEG = 1.0e-6
EXECUTOR_LENGTH_EPSILON_MM = 1.0e-3

# PC writer target: much stricter than firmware's 2 mm executor tolerance.
PC_CONTINUITY_TOLERANCE_MM = 0.01

assert struct.calcsize(HEADER_FMT) == HEADER_SIZE
assert struct.calcsize(RECORD_FMT) == RECORD_SIZE


class SegmentType(IntEnum):
    NONE = 0
    LINE = 1
    CIRCLE = 2


class TrajectoryFormatError(ValueError):
    """Binary layout or canonical-TRJ1 violation."""


@dataclass(frozen=True, slots=True)
class TrajectoryHeader:
    """
    Semantic trajectory-wide start pose.

    Canonical wire metadata (magic/version/header_size/record_size/flags/
    reserved/segment_count) is intentionally NOT user-configurable.
    """

    start_x_mm: float = 0.0
    start_y_mm: float = 0.0
    start_yaw_deg: float = 0.0


@dataclass(frozen=True, slots=True)
class LineSegment:
    end_x_mm: float
    end_y_mm: float
    speed_mm_s: float
    acceleration_mm_s2: float = 0.0


@dataclass(frozen=True, slots=True)
class CircleSegment:
    center_x_mm: float
    center_y_mm: float
    radius_mm: float
    start_angle_deg: float
    sweep_deg: float
    speed_mm_s: float
    acceleration_mm_s2: float = 0.0


Segment: TypeAlias = LineSegment | CircleSegment


@dataclass(frozen=True, slots=True)
class Trajectory:
    header: TrajectoryHeader
    segments: tuple[Segment, ...]

    def __init__(
        self,
        header: TrajectoryHeader,
        segments,
    ) -> None:
        object.__setattr__(self, "header", header)
        object.__setattr__(self, "segments", tuple(segments))


def pack_header(header: TrajectoryHeader, segment_count: int) -> bytes:
    """
    Pack one canonical 32-byte TRJ1 V1 header.

    This low-level function assumes semantic validation has already happened.
    """
    if not isinstance(segment_count, int) or isinstance(segment_count, bool):
        raise TypeError("segment_count must be an int")
    if not 0 <= segment_count <= 0xFFFFFFFF:
        raise TrajectoryFormatError("segment_count does not fit uint32")

    try:
        raw = struct.pack(
            HEADER_FMT,
            MAGIC,
            VERSION,
            HEADER_SIZE,
            segment_count,
            RECORD_SIZE,
            HEADER_FLAGS_V1,
            float(header.start_x_mm),
            float(header.start_y_mm),
            float(header.start_yaw_deg),
            HEADER_RESERVED_V1,
        )
    except (OverflowError, struct.error) as exc:
        raise TrajectoryFormatError(f"cannot encode TRJ1 header: {exc}") from exc

    if len(raw) != HEADER_SIZE:
        raise AssertionError("internal error: packed header is not 32 bytes")
    return raw


def pack_segment(segment: Segment) -> bytes:
    """
    Pack one canonical 44-byte V1 record.

    V1 unused data slots, record flags and reserved fields are always zero.
    """
    if isinstance(segment, LineSegment):
        type_value = SegmentType.LINE
        data = (
            float(segment.end_x_mm),
            float(segment.end_y_mm),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
    elif isinstance(segment, CircleSegment):
        type_value = SegmentType.CIRCLE
        data = (
            float(segment.center_x_mm),
            float(segment.center_y_mm),
            float(segment.radius_mm),
            float(segment.start_angle_deg),
            float(segment.sweep_deg),
            0.0,
            0.0,
            0.0,
        )
    else:
        raise TypeError(f"unsupported segment object: {type(segment).__name__}")

    try:
        raw = struct.pack(
            RECORD_FMT,
            int(type_value),
            RECORD_FLAGS_V1,
            RECORD_RESERVED_V1,
            float(segment.speed_mm_s),
            float(segment.acceleration_mm_s2),
            *data,
        )
    except (OverflowError, struct.error) as exc:
        raise TrajectoryFormatError(f"cannot encode segment: {exc}") from exc

    if len(raw) != RECORD_SIZE:
        raise AssertionError("internal error: packed record is not 44 bytes")
    return raw


def unpack_header(raw: bytes) -> tuple[TrajectoryHeader, int]:
    """
    Decode and strictly verify one canonical 32-byte V1 header.

    Returns (semantic_header, segment_count).
    """
    if len(raw) != HEADER_SIZE:
        raise TrajectoryFormatError(
            f"header must be exactly {HEADER_SIZE} bytes, got {len(raw)}"
        )

    (
        magic,
        version,
        header_size,
        segment_count,
        record_size,
        flags,
        start_x,
        start_y,
        start_yaw,
        reserved,
    ) = struct.unpack(HEADER_FMT, raw)

    if magic != MAGIC:
        raise TrajectoryFormatError(f"bad magic: {magic!r}")
    if version != VERSION:
        raise TrajectoryFormatError(f"unsupported version: {version}")
    if header_size != HEADER_SIZE:
        raise TrajectoryFormatError(
            f"non-canonical header_size: {header_size}, expected {HEADER_SIZE}"
        )
    if record_size != RECORD_SIZE:
        raise TrajectoryFormatError(
            f"non-canonical record_size: {record_size}, expected {RECORD_SIZE}"
        )
    if flags != HEADER_FLAGS_V1:
        raise TrajectoryFormatError(f"non-canonical header flags: {flags}")
    if reserved != HEADER_RESERVED_V1:
        raise TrajectoryFormatError(f"non-zero header reserved: {reserved}")

    return (
        TrajectoryHeader(
            start_x_mm=start_x,
            start_y_mm=start_y,
            start_yaw_deg=start_yaw,
        ),
        segment_count,
    )


def unpack_segment(raw: bytes) -> Segment:
    """
    Decode and strictly verify one canonical 44-byte V1 record.

    Semantic/geometry validation is completed by traj_validate.
    """
    if len(raw) != RECORD_SIZE:
        raise TrajectoryFormatError(
            f"record must be exactly {RECORD_SIZE} bytes, got {len(raw)}"
        )

    values = struct.unpack(RECORD_FMT, raw)
    type_value, flags, reserved, speed, acceleration, *data = values

    if flags != RECORD_FLAGS_V1:
        raise TrajectoryFormatError(f"non-canonical record flags: {flags}")
    if reserved != RECORD_RESERVED_V1:
        raise TrajectoryFormatError(f"non-zero record reserved: {reserved}")

    try:
        segment_type = SegmentType(type_value)
    except ValueError as exc:
        raise TrajectoryFormatError(f"unsupported segment type: {type_value}") from exc

    if segment_type == SegmentType.LINE:
        if any(value != 0.0 for value in data[2:]):
            raise TrajectoryFormatError("LINE unused data[2..7] must be zero")
        return LineSegment(
            end_x_mm=data[0],
            end_y_mm=data[1],
            speed_mm_s=speed,
            acceleration_mm_s2=acceleration,
        )

    if segment_type == SegmentType.CIRCLE:
        if any(value != 0.0 for value in data[5:]):
            raise TrajectoryFormatError("CIRCLE unused data[5..7] must be zero")
        return CircleSegment(
            center_x_mm=data[0],
            center_y_mm=data[1],
            radius_mm=data[2],
            start_angle_deg=data[3],
            sweep_deg=data[4],
            speed_mm_s=speed,
            acceleration_mm_s2=acceleration,
        )

    # NONE exists in the enum but is intentionally unsupported as a V1 record.
    raise TrajectoryFormatError(f"unsupported segment type: {type_value}")
