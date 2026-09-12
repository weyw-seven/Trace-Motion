"""TRJ1 V1 PC-side protocol package (Phase A)."""

from .traj_format import (
    MAGIC,
    VERSION,
    HEADER_SIZE,
    RECORD_SIZE,
    HEADER_FMT,
    RECORD_FMT,
    SegmentType,
    TrajectoryHeader,
    LineSegment,
    CircleSegment,
    Trajectory,
    TrajectoryFormatError,
)
from .traj_validate import (
    ValidationError,
    validate_header,
    validate_segment,
    validate_trajectory,
)
from .traj_writer import encode_traj, write_traj
from .traj_reader import decode_traj, read_traj, format_trajectory

__all__ = [
    "MAGIC",
    "VERSION",
    "HEADER_SIZE",
    "RECORD_SIZE",
    "HEADER_FMT",
    "RECORD_FMT",
    "SegmentType",
    "TrajectoryHeader",
    "LineSegment",
    "CircleSegment",
    "Trajectory",
    "TrajectoryFormatError",
    "ValidationError",
    "validate_header",
    "validate_segment",
    "validate_trajectory",
    "encode_traj",
    "write_traj",
    "decode_traj",
    "read_traj",
    "format_trajectory",
]
