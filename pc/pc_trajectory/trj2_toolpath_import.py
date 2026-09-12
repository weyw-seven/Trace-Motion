"""
B8 — canonical TRJ2 -> Phase-B Toolpath semantic import bridge.

Binary parsing remains owned by B6. B8 only reconstructs semantic execution IR:

    bytes/file
      ↓ B6 reader
    Trj2File
      ↓ B8
    ImportedTrj2Toolpath
      ├─ Toolpath
      ├─ start_point
      └─ start_yaw_deg

The wrapper is necessary because the frozen Toolpath model intentionally does
not own trajectory-header pose.

Coordinate convention:
    +X = robot forward when chassis yaw = 0 deg
    +Y = robot left when chassis yaw = 0 deg
    positive angle / sweep = CCW
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .geometry import Arc, CubicBezier, Line, Point2D
from .toolpath import Motion, PenDown, PenUp, Toolpath, ToolpathRecord, Wait
from .traj2_format import (
    Trj2CircleRecord,
    Trj2CubicBezierRecord,
    Trj2File,
    Trj2LineRecord,
    Trj2PenDownRecord,
    Trj2PenUpRecord,
    Trj2Record,
    Trj2WaitRecord,
)
from .traj2_reader import decode_trj2, read_trj2
from .traj2_validate import (
    TRJ2_CONTINUITY_TOLERANCE_MM,
    Trj2ValidationError,
    validate_trj2_file,
)


class Trj2ToolpathImportError(ValueError):
    """Base error for TRJ2 -> Toolpath semantic reconstruction."""


class Trj2ImportUnsupportedRecordError(Trj2ToolpathImportError):
    def __init__(self, *, record_index: int, record_type: str) -> None:
        self.record_index = record_index
        self.record_type = record_type
        super().__init__(
            f"Unsupported TRJ2 semantic record {record_index}: {record_type}"
        )


class Trj2ImportContinuityError(Trj2ToolpathImportError):
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
            f"TRJ2 CIRCLE record {record_index} start position error "
            f"{position_error_mm:.9f} mm exceeds import tolerance "
            f"{tolerance_mm:.9f} mm"
        )


@dataclass(frozen=True)
class ImportedTrj2Toolpath:
    toolpath: Toolpath
    start_point: Point2D
    start_yaw_deg: float


def trj2_record_to_toolpath_record(
    record: Trj2Record,
    *,
    current_point: Point2D,
    record_index: int = 0,
    continuity_tolerance_mm: float = TRJ2_CONTINUITY_TOLERANCE_MM,
) -> tuple[ToolpathRecord, Point2D]:
    """
    Reconstruct one Toolpath record and return the next logical XY.

    Event records return the same logical point.
    """
    if not isinstance(current_point, Point2D):
        raise Trj2ToolpathImportError("current_point must be a Point2D")

    if isinstance(record, Trj2LineRecord):
        end = Point2D(record.end_x_mm, record.end_y_mm)
        motion = Motion(
            geometry=Line(start=current_point, end=end),
            speed_mm_s=record.speed_mm_s,
            acceleration_mm_s2=record.acceleration_mm_s2,
        )
        return motion, end

    if isinstance(record, Trj2CircleRecord):
        arc = Arc(
            center=Point2D(record.center_x_mm, record.center_y_mm),
            radius_mm=record.radius_mm,
            start_angle_deg=record.start_angle_deg,
            sweep_deg=record.sweep_deg,
        )
        error_mm = current_point.distance_to(arc.start_point())
        if error_mm > continuity_tolerance_mm:
            raise Trj2ImportContinuityError(
                record_index=record_index,
                position_error_mm=error_mm,
                tolerance_mm=continuity_tolerance_mm,
            )
        motion = Motion(
            geometry=arc,
            speed_mm_s=record.speed_mm_s,
            acceleration_mm_s2=record.acceleration_mm_s2,
        )
        return motion, arc.end_point()

    if isinstance(record, Trj2CubicBezierRecord):
        end = Point2D(record.end_x_mm, record.end_y_mm)
        curve = CubicBezier(
            start=current_point,
            control1=Point2D(
                record.control1_x_mm,
                record.control1_y_mm,
            ),
            control2=Point2D(
                record.control2_x_mm,
                record.control2_y_mm,
            ),
            end=end,
        )
        motion = Motion(
            geometry=curve,
            speed_mm_s=record.speed_mm_s,
            acceleration_mm_s2=record.acceleration_mm_s2,
        )
        return motion, end

    if isinstance(record, Trj2PenUpRecord):
        return PenUp(), current_point

    if isinstance(record, Trj2PenDownRecord):
        return PenDown(), current_point

    if isinstance(record, Trj2WaitRecord):
        return Wait(record.duration_s), current_point

    raise Trj2ImportUnsupportedRecordError(
        record_index=record_index,
        record_type=type(record).__name__,
    )


def trj2_file_to_toolpath(file: Trj2File) -> ImportedTrj2Toolpath:
    """Reconstruct Toolpath while preserving exact TRJ2 Header pose."""
    if not isinstance(file, Trj2File):
        raise Trj2ToolpathImportError("file must be a Trj2File")

    try:
        validate_trj2_file(file)
    except Trj2ValidationError as exc:
        raise Trj2ToolpathImportError(
            f"Invalid canonical Trj2File: {exc}"
        ) from exc

    header_start = Point2D(
        file.header.start_x_mm,
        file.header.start_y_mm,
    )
    current = header_start
    records: list[ToolpathRecord] = []

    for record_index, record in enumerate(file.records):
        tool_record, current = trj2_record_to_toolpath_record(
            record,
            current_point=current,
            record_index=record_index,
            continuity_tolerance_mm=TRJ2_CONTINUITY_TOLERANCE_MM,
        )
        records.append(tool_record)

    toolpath = Toolpath(
        records=tuple(records),
        continuity_tolerance_mm=TRJ2_CONTINUITY_TOLERANCE_MM,
    )

    return ImportedTrj2Toolpath(
        toolpath=toolpath,
        start_point=header_start,
        start_yaw_deg=file.header.start_yaw_deg,
    )


def decode_toolpath_trj2(
    data: bytes | bytearray | memoryview,
) -> ImportedTrj2Toolpath:
    """Delegate bytes parsing to B6, then perform B8 semantic import."""
    return trj2_file_to_toolpath(decode_trj2(data))


def read_toolpath_trj2(
    path: str | Path,
) -> ImportedTrj2Toolpath:
    """Delegate file parsing to B6, then perform B8 semantic import."""
    return trj2_file_to_toolpath(read_trj2(path))


__all__ = [
    "ImportedTrj2Toolpath",
    "Trj2ImportContinuityError",
    "Trj2ImportUnsupportedRecordError",
    "Trj2ToolpathImportError",
    "decode_toolpath_trj2",
    "read_toolpath_trj2",
    "trj2_file_to_toolpath",
    "trj2_record_to_toolpath_record",
]
