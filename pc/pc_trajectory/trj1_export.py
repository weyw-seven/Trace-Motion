"""
TRJ1 compatibility bridge for the Phase-B PC trajectory IR.

Purpose
-------
Reconnect the new internal Geometry / Motion / Toolpath model to the already
validated Phase-A TRJ1 backend without duplicating the TRJ1 binary serializer.

Data flow:

    Phase-B Geometry / Motion
            ↓
      TRJ1 compatibility bridge
            ↓
    Phase-A TrajectoryHeader / Segment objects
            ↓
      Phase-A traj_writer.encode_traj()
            ↓
        canonical TRJ1 bytes

Architectural rules
-------------------
1. This module does NOT know the TRJ1 struct layout and does NOT use struct.pack.
2. LINE implicit-start lowering happens only after explicit Phase-B Motion
   continuity has been checked.
3. Header start_x/start_y are derived from the first Motion's explicit start.
4. start_yaw_deg is caller-supplied trajectory-level fixed-heading metadata.
   It is NEVER inferred from path tangent.
5. TRJ1 cannot represent:
       PenUp
       PenDown
       Wait
       CubicBezier
   These are rejected explicitly. They are never silently discarded.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable

from .drawing import PC_CONTINUITY_TOLERANCE_MM
from .geometry import Arc, CubicBezier, Line
from .toolpath import Motion, PenDown, PenUp, Toolpath, Wait
from .traj_format import (
    CircleSegment,
    LineSegment,
    Trajectory,
    TrajectoryHeader,
)
from .traj_writer import encode_traj, write_traj


class Trj1ExportError(ValueError):
    """Base error for Phase-B -> TRJ1 compatibility export."""


class Trj1EmptyMotionSequenceError(Trj1ExportError):
    """TRJ1 header start XY cannot be derived because there is no Motion."""


class Trj1UnsupportedGeometryError(Trj1ExportError):
    """Raised when a Motion geometry cannot be represented by TRJ1."""

    def __init__(
        self,
        *,
        motion_index: int,
        geometry_type: str,
    ) -> None:
        self.motion_index = motion_index
        self.geometry_type = geometry_type

        super().__init__(
            f"TRJ1 cannot represent Motion {motion_index} geometry "
            f"{geometry_type}; TRJ1 supports only LINE and CIRCLE"
        )


class Trj1UnsupportedRecordError(Trj1ExportError):
    """Raised when Toolpath contains an Event that TRJ1 cannot represent."""

    def __init__(
        self,
        *,
        record_index: int,
        record_type: str,
    ) -> None:
        self.record_index = record_index
        self.record_type = record_type

        super().__init__(
            f"TRJ1 cannot represent Toolpath record {record_index} "
            f"of type {record_type}; event semantics must not be discarded"
        )


class Trj1MotionContinuityError(Trj1ExportError):
    """
    Raised before lowering explicit-start Geometry into implicit-start TRJ1.

    This check is essential because a TRJ1 LINE stores only its endpoint.
    Once lowered, a Phase-B Line.start mismatch would otherwise be lost.
    """

    def __init__(
        self,
        *,
        previous_motion_index: int,
        next_motion_index: int,
        position_error_mm: float,
        tolerance_mm: float,
    ) -> None:
        self.previous_motion_index = previous_motion_index
        self.next_motion_index = next_motion_index
        self.position_error_mm = position_error_mm
        self.tolerance_mm = tolerance_mm

        super().__init__(
            "TRJ1 lowering would discard a Motion start discontinuity "
            f"between Motion {previous_motion_index} and "
            f"Motion {next_motion_index}: "
            f"{position_error_mm:.9f} mm > {tolerance_mm:.9f} mm"
        )


def _finite(name: str, value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise Trj1ExportError(f"{name} must be finite")
    return value


def _finite_nonnegative(name: str, value: float) -> float:
    value = _finite(name, value)
    if value < 0.0:
        raise Trj1ExportError(f"{name} must be >= 0")
    return value


def _normalize_motions(
    motions: Iterable[Motion],
) -> tuple[Motion, ...]:
    try:
        normalized = tuple(motions)
    except TypeError as exc:
        raise Trj1ExportError("motions must be an iterable of Motion") from exc

    if not normalized:
        raise Trj1EmptyMotionSequenceError(
            "Cannot export empty Motion sequence to TRJ1 because "
            "header start_x/start_y cannot be derived"
        )

    for index, motion in enumerate(normalized):
        if not isinstance(motion, Motion):
            raise Trj1ExportError(
                f"motions[{index}] must be Motion, got "
                f"{type(motion).__name__}"
            )

    return normalized


def _validate_motion_continuity(
    motions: tuple[Motion, ...],
    *,
    tolerance_mm: float,
) -> None:
    for index in range(len(motions) - 1):
        previous = motions[index]
        following = motions[index + 1]

        error_mm = previous.end_point().distance_to(
            following.start_point()
        )
        if error_mm > tolerance_mm:
            raise Trj1MotionContinuityError(
                previous_motion_index=index,
                next_motion_index=index + 1,
                position_error_mm=error_mm,
                tolerance_mm=tolerance_mm,
            )


def motion_to_trj1_segment(
    motion: Motion,
    *,
    motion_index: int = 0,
):
    """
    Lower one Phase-B Motion into a Phase-A TRJ1 segment object.

    No binary serialization occurs here.
    """
    if not isinstance(motion, Motion):
        raise Trj1ExportError("motion must be a Motion")

    geometry = motion.geometry

    if isinstance(geometry, Line):
        return LineSegment(
            end_x_mm=geometry.end_point().x_mm,
            end_y_mm=geometry.end_point().y_mm,
            speed_mm_s=motion.speed_mm_s,
            acceleration_mm_s2=motion.acceleration_mm_s2,
        )

    if isinstance(geometry, Arc):
        return CircleSegment(
            center_x_mm=geometry.center.x_mm,
            center_y_mm=geometry.center.y_mm,
            radius_mm=geometry.radius_mm,
            start_angle_deg=geometry.start_angle_deg,
            sweep_deg=geometry.sweep_deg,
            speed_mm_s=motion.speed_mm_s,
            acceleration_mm_s2=motion.acceleration_mm_s2,
        )

    if isinstance(geometry, CubicBezier):
        raise Trj1UnsupportedGeometryError(
            motion_index=motion_index,
            geometry_type="CubicBezier",
        )

    raise Trj1UnsupportedGeometryError(
        motion_index=motion_index,
        geometry_type=type(geometry).__name__,
    )


def motions_to_trj1_trajectory(
    motions: Iterable[Motion],
    *,
    start_yaw_deg: float = 0.0,
    continuity_tolerance_mm: float = PC_CONTINUITY_TOLERANCE_MM,
) -> Trajectory:
    """
    Convert a non-empty sequence of Phase-B Motion objects into the frozen
    Phase-A Trajectory representation.

    Header start XY are derived from the first explicit Motion start point.
    """
    normalized = _normalize_motions(motions)

    yaw = _finite("start_yaw_deg", start_yaw_deg)
    tolerance = _finite_nonnegative(
        "continuity_tolerance_mm",
        continuity_tolerance_mm,
    )

    _validate_motion_continuity(
        normalized,
        tolerance_mm=tolerance,
    )

    start = normalized[0].start_point()

    segments = tuple(
        motion_to_trj1_segment(
            motion,
            motion_index=index,
        )
        for index, motion in enumerate(normalized)
    )

    return Trajectory(
        header=TrajectoryHeader(
            start_x_mm=start.x_mm,
            start_y_mm=start.y_mm,
            start_yaw_deg=yaw,
        ),
        segments=segments,
    )


def toolpath_to_trj1_trajectory(
    toolpath: Toolpath,
    *,
    start_yaw_deg: float = 0.0,
    continuity_tolerance_mm: float | None = None,
) -> Trajectory:
    """
    Convert an event-free Toolpath to TRJ1.

    Any PenUp / PenDown / Wait record is rejected because TRJ1 has no event
    representation. Rejecting is intentional: silently deleting events would
    change drawing semantics.
    """
    if not isinstance(toolpath, Toolpath):
        raise Trj1ExportError("toolpath must be a Toolpath")

    motions: list[Motion] = []

    for record_index, record in enumerate(toolpath.records):
        if isinstance(record, Motion):
            motions.append(record)
            continue

        if isinstance(record, (PenUp, PenDown, Wait)):
            raise Trj1UnsupportedRecordError(
                record_index=record_index,
                record_type=type(record).__name__,
            )

        # Defensive fallback; Toolpath itself normally prevents this.
        raise Trj1UnsupportedRecordError(
            record_index=record_index,
            record_type=type(record).__name__,
        )

    tolerance = (
        toolpath.continuity_tolerance_mm
        if continuity_tolerance_mm is None
        else continuity_tolerance_mm
    )

    return motions_to_trj1_trajectory(
        motions,
        start_yaw_deg=start_yaw_deg,
        continuity_tolerance_mm=tolerance,
    )


def encode_motions_trj1(
    motions: Iterable[Motion],
    *,
    start_yaw_deg: float = 0.0,
    continuity_tolerance_mm: float = PC_CONTINUITY_TOLERANCE_MM,
) -> bytes:
    """
    Encode Phase-B Motions by delegating binary serialization to Phase A.
    """
    trajectory = motions_to_trj1_trajectory(
        motions,
        start_yaw_deg=start_yaw_deg,
        continuity_tolerance_mm=continuity_tolerance_mm,
    )
    return encode_traj(trajectory)


def encode_toolpath_trj1(
    toolpath: Toolpath,
    *,
    start_yaw_deg: float = 0.0,
    continuity_tolerance_mm: float | None = None,
) -> bytes:
    """
    Encode an event-free Phase-B Toolpath by delegating to Phase A.
    """
    trajectory = toolpath_to_trj1_trajectory(
        toolpath,
        start_yaw_deg=start_yaw_deg,
        continuity_tolerance_mm=continuity_tolerance_mm,
    )
    return encode_traj(trajectory)


def write_motions_trj1(
    motions: Iterable[Motion],
    path: str | Path,
    *,
    start_yaw_deg: float = 0.0,
    continuity_tolerance_mm: float = PC_CONTINUITY_TOLERANCE_MM,
) -> Path:
    trajectory = motions_to_trj1_trajectory(
        motions,
        start_yaw_deg=start_yaw_deg,
        continuity_tolerance_mm=continuity_tolerance_mm,
    )
    output = Path(path)
    write_traj(trajectory, output)
    return output


def write_toolpath_trj1(
    toolpath: Toolpath,
    path: str | Path,
    *,
    start_yaw_deg: float = 0.0,
    continuity_tolerance_mm: float | None = None,
) -> Path:
    trajectory = toolpath_to_trj1_trajectory(
        toolpath,
        start_yaw_deg=start_yaw_deg,
        continuity_tolerance_mm=continuity_tolerance_mm,
    )
    output = Path(path)
    write_traj(trajectory, output)
    return output


__all__ = [
    "Trj1EmptyMotionSequenceError",
    "Trj1ExportError",
    "Trj1MotionContinuityError",
    "Trj1UnsupportedGeometryError",
    "Trj1UnsupportedRecordError",
    "encode_motions_trj1",
    "encode_toolpath_trj1",
    "motion_to_trj1_segment",
    "motions_to_trj1_trajectory",
    "toolpath_to_trj1_trajectory",
    "write_motions_trj1",
    "write_toolpath_trj1",
]
