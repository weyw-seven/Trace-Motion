"""
B7 — Phase-B Toolpath -> canonical TRJ2 export bridge.

This module performs semantic lowering only:

    Toolpath
        ↓
    Trj2File
        ↓
    B6 traj2_writer
        ↓
    canonical TRJ2 bytes

It deliberately does NOT own binary layout. In particular, this module must
not define HEADER_FMT / RECORD_FMT / type-number constants or call struct.pack.

Mapping
-------
Motion(Line)          -> Trj2LineRecord
Motion(Arc)           -> Trj2CircleRecord
Motion(CubicBezier)   -> Trj2CubicBezierRecord

PenUp                  -> Trj2PenUpRecord
PenDown                -> Trj2PenDownRecord
Wait                   -> Trj2WaitRecord

Draw vs travel is NOT encoded as a distinct Motion type. The same LINE/CIRCLE/
BEZIER record is interpreted according to the current pen state.

Start-pose rules
----------------
- If Toolpath contains at least one Motion:
    Header start_x/start_y are derived from the FIRST Motion's explicit start.
    An optional `start_point` is only a consistency assertion and never
    replaces the Motion start.
- If Toolpath contains no Motion:
    caller must provide explicit `start_point`.
- start_yaw_deg is always caller supplied and is never inferred from tangent.

Continuity rules
----------------
Phase-B Geometry contains explicit Motion starts, while TRJ2 Motion records use
an implicit logical current point for LINE/Bezier P0 and continuity semantics
for CIRCLE. Therefore explicit Motion continuity is rechecked before lowering
so that no start-position information is silently lost.
"""

from __future__ import annotations

import math
from pathlib import Path

from .geometry import Arc, CubicBezier, Line, Point2D
from .toolpath import Motion, PenDown, PenUp, Toolpath, Wait
from .traj2_format import (
    Trj2CircleRecord,
    Trj2CubicBezierRecord,
    Trj2File,
    Trj2Header,
    Trj2LineRecord,
    Trj2PenDownRecord,
    Trj2PenUpRecord,
    Trj2Record,
    Trj2WaitRecord,
)
from .traj2_validate import validate_trj2_file
from .traj2_writer import encode_trj2, write_trj2


class ToolpathTrj2ExportError(ValueError):
    """Base error for Toolpath -> TRJ2 semantic lowering."""


class Trj2ExportStartPoseError(ToolpathTrj2ExportError):
    """Raised when TRJ2 header start XY cannot be resolved consistently."""


class Trj2ExportContinuityError(ToolpathTrj2ExportError):
    """Raised before explicit Motion start information would be lost."""

    def __init__(
        self,
        *,
        record_index: int,
        motion_index: int,
        position_error_mm: float,
        tolerance_mm: float,
    ) -> None:
        self.record_index = record_index
        self.motion_index = motion_index
        self.position_error_mm = position_error_mm
        self.tolerance_mm = tolerance_mm

        super().__init__(
            f"Toolpath record {record_index} / Motion {motion_index} "
            f"start position error {position_error_mm:.9f} mm exceeds "
            f"TRJ2 export tolerance {tolerance_mm:.9f} mm"
        )


class Trj2ExportUnsupportedRecordError(ToolpathTrj2ExportError):
    """Defensive error for a record type not supported by B7."""

    def __init__(
        self,
        *,
        record_index: int,
        record_type: str,
    ) -> None:
        self.record_index = record_index
        self.record_type = record_type

        super().__init__(
            f"Unsupported Toolpath record {record_index}: {record_type}"
        )


class Trj2ExportUnsupportedGeometryError(ToolpathTrj2ExportError):
    """Defensive error for a Motion geometry not supported by B7."""

    def __init__(
        self,
        *,
        motion_index: int,
        geometry_type: str,
    ) -> None:
        self.motion_index = motion_index
        self.geometry_type = geometry_type

        super().__init__(
            f"Unsupported Motion {motion_index} geometry: {geometry_type}"
        )


def _finite(name: str, value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ToolpathTrj2ExportError(f"{name} must be finite")
    return value


def _finite_nonnegative(name: str, value: float) -> float:
    value = _finite(name, value)
    if value < 0.0:
        raise ToolpathTrj2ExportError(f"{name} must be >= 0")
    return value


def _require_point(
    name: str,
    value: Point2D | None,
) -> Point2D:
    if not isinstance(value, Point2D):
        raise Trj2ExportStartPoseError(
            f"{name} must be a Point2D"
        )
    return value


def motion_to_trj2_record(
    motion: Motion,
    *,
    motion_index: int = 0,
) -> Trj2Record:
    """
    Lower one Motion into the B6 semantic TRJ2 record model.

    No binary packing occurs here.
    """
    if not isinstance(motion, Motion):
        raise ToolpathTrj2ExportError("motion must be a Motion")

    geometry = motion.geometry

    if isinstance(geometry, Line):
        end = geometry.end_point()
        return Trj2LineRecord(
            end_x_mm=end.x_mm,
            end_y_mm=end.y_mm,
            speed_mm_s=motion.speed_mm_s,
            acceleration_mm_s2=motion.acceleration_mm_s2,
        )

    if isinstance(geometry, Arc):
        return Trj2CircleRecord(
            center_x_mm=geometry.center.x_mm,
            center_y_mm=geometry.center.y_mm,
            radius_mm=geometry.radius_mm,
            start_angle_deg=geometry.start_angle_deg,
            sweep_deg=geometry.sweep_deg,
            speed_mm_s=motion.speed_mm_s,
            acceleration_mm_s2=motion.acceleration_mm_s2,
        )

    if isinstance(geometry, CubicBezier):
        return Trj2CubicBezierRecord(
            control1_x_mm=geometry.control1.x_mm,
            control1_y_mm=geometry.control1.y_mm,
            control2_x_mm=geometry.control2.x_mm,
            control2_y_mm=geometry.control2.y_mm,
            end_x_mm=geometry.end.x_mm,
            end_y_mm=geometry.end.y_mm,
            speed_mm_s=motion.speed_mm_s,
            acceleration_mm_s2=motion.acceleration_mm_s2,
        )

    raise Trj2ExportUnsupportedGeometryError(
        motion_index=motion_index,
        geometry_type=type(geometry).__name__,
    )


def toolpath_record_to_trj2_record(
    record,
    *,
    record_index: int = 0,
    motion_index: int = 0,
) -> Trj2Record:
    """
    Lower one Toolpath record while preserving its semantic type and order.
    """
    if isinstance(record, Motion):
        return motion_to_trj2_record(
            record,
            motion_index=motion_index,
        )

    if isinstance(record, PenUp):
        return Trj2PenUpRecord()

    if isinstance(record, PenDown):
        return Trj2PenDownRecord()

    if isinstance(record, Wait):
        return Trj2WaitRecord(
            duration_s=record.duration_s,
        )

    raise Trj2ExportUnsupportedRecordError(
        record_index=record_index,
        record_type=type(record).__name__,
    )


def _resolve_header_start(
    toolpath: Toolpath,
    *,
    start_point: Point2D | None,
    tolerance_mm: float,
) -> Point2D:
    """
    Resolve canonical Header start XY without altering Motion geometry.
    """
    first_motion: Motion | None = None

    for record in toolpath.records:
        if isinstance(record, Motion):
            first_motion = record
            break

    if first_motion is None:
        if start_point is None:
            raise Trj2ExportStartPoseError(
                "Toolpath has no Motion; explicit start_point is required "
                "to construct the TRJ2 header"
            )
        return _require_point("start_point", start_point)

    derived = first_motion.start_point()

    if start_point is not None:
        asserted = _require_point("start_point", start_point)
        error_mm = derived.distance_to(asserted)

        if error_mm > tolerance_mm:
            raise Trj2ExportStartPoseError(
                "Explicit start_point does not match first Motion start: "
                f"{error_mm:.9f} mm > {tolerance_mm:.9f} mm"
            )

    # Even when a caller assertion was supplied, preserve the Phase-B
    # geometry exactly by using the Motion's own explicit start.
    return derived


def toolpath_to_trj2_file(
    toolpath: Toolpath,
    *,
    start_yaw_deg: float = 0.0,
    start_point: Point2D | None = None,
    continuity_tolerance_mm: float | None = None,
) -> Trj2File:
    """
    Lower a Toolpath into the canonical B6 Trj2File model.

    The returned model is validated by B6 before being returned.
    """
    if not isinstance(toolpath, Toolpath):
        raise ToolpathTrj2ExportError(
            "toolpath must be a Toolpath"
        )

    yaw = _finite("start_yaw_deg", start_yaw_deg)

    tolerance = (
        toolpath.continuity_tolerance_mm
        if continuity_tolerance_mm is None
        else _finite_nonnegative(
            "continuity_tolerance_mm",
            continuity_tolerance_mm,
        )
    )

    # Toolpath itself validates its own tolerance at construction, but normalize
    # again here so this layer remains correct if implementation changes later.
    tolerance = _finite_nonnegative(
        "continuity_tolerance_mm",
        tolerance,
    )

    header_start = _resolve_header_start(
        toolpath,
        start_point=start_point,
        tolerance_mm=tolerance,
    )

    current = header_start
    output_records: list[Trj2Record] = []
    motion_index = 0

    for record_index, record in enumerate(toolpath.records):
        if isinstance(record, Motion):
            explicit_start = record.start_point()
            error_mm = current.distance_to(explicit_start)

            if error_mm > tolerance:
                raise Trj2ExportContinuityError(
                    record_index=record_index,
                    motion_index=motion_index,
                    position_error_mm=error_mm,
                    tolerance_mm=tolerance,
                )

            output_records.append(
                motion_to_trj2_record(
                    record,
                    motion_index=motion_index,
                )
            )
            current = record.end_point()
            motion_index += 1
            continue

        output_records.append(
            toolpath_record_to_trj2_record(
                record,
                record_index=record_index,
                motion_index=motion_index,
            )
        )
        # Events do not change current logical XY.

    file = Trj2File(
        header=Trj2Header(
            start_x_mm=header_start.x_mm,
            start_y_mm=header_start.y_mm,
            start_yaw_deg=yaw,
        ),
        records=tuple(output_records),
    )

    # Final protocol legality remains owned by B6.
    validate_trj2_file(
        file,
        continuity_tolerance_mm=tolerance,
    )

    return file


def encode_toolpath_trj2(
    toolpath: Toolpath,
    *,
    start_yaw_deg: float = 0.0,
    start_point: Point2D | None = None,
    continuity_tolerance_mm: float | None = None,
) -> bytes:
    """Encode Toolpath by delegating canonical binary serialization to B6."""
    file = toolpath_to_trj2_file(
        toolpath,
        start_yaw_deg=start_yaw_deg,
        start_point=start_point,
        continuity_tolerance_mm=continuity_tolerance_mm,
    )
    return encode_trj2(file)


def write_toolpath_trj2(
    toolpath: Toolpath,
    path: str | Path,
    *,
    start_yaw_deg: float = 0.0,
    start_point: Point2D | None = None,
    continuity_tolerance_mm: float | None = None,
) -> Path:
    """Write Toolpath as canonical TRJ2 by delegating file output to B6."""
    file = toolpath_to_trj2_file(
        toolpath,
        start_yaw_deg=start_yaw_deg,
        start_point=start_point,
        continuity_tolerance_mm=continuity_tolerance_mm,
    )
    output = Path(path)
    return write_trj2(file, output)


__all__ = [
    "ToolpathTrj2ExportError",
    "Trj2ExportContinuityError",
    "Trj2ExportStartPoseError",
    "Trj2ExportUnsupportedGeometryError",
    "Trj2ExportUnsupportedRecordError",
    "encode_toolpath_trj2",
    "motion_to_trj2_record",
    "toolpath_record_to_trj2_record",
    "toolpath_to_trj2_file",
    "write_toolpath_trj2",
]
