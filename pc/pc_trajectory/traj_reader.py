from __future__ import annotations

from pathlib import Path

from .traj_format import (
    HEADER_SIZE,
    RECORD_SIZE,
    CircleSegment,
    LineSegment,
    Trajectory,
    TrajectoryFormatError,
    unpack_header,
    unpack_segment,
)
from .traj_validate import ValidationError, validate_trajectory


def decode_traj(data: bytes) -> Trajectory:
    """
    Strict canonical-V1 verifier + decoder.

    Deliberately stricter than the firmware reader:
      - exact 32-byte header
      - exact 44-byte records
      - no trailing bytes
      - V1 flags/reserved/unused slots must be zero
    """
    if not isinstance(data, (bytes, bytearray, memoryview)):
        raise TypeError("data must be bytes-like")

    data = bytes(data)

    if len(data) < HEADER_SIZE:
        raise TrajectoryFormatError(
            f"trajectory header is truncated: {len(data)} < {HEADER_SIZE}"
        )

    header, segment_count = unpack_header(data[:HEADER_SIZE])

    expected_size = HEADER_SIZE + segment_count * RECORD_SIZE
    if len(data) != expected_size:
        if len(data) < expected_size:
            raise TrajectoryFormatError(
                f"trajectory is truncated: size={len(data)}, "
                f"expected={expected_size}"
            )
        raise TrajectoryFormatError(
            f"trajectory has trailing bytes: size={len(data)}, "
            f"expected={expected_size}"
        )

    segments = []
    offset = HEADER_SIZE
    for index in range(segment_count):
        record = data[offset : offset + RECORD_SIZE]
        try:
            segments.append(unpack_segment(record))
        except TrajectoryFormatError as exc:
            raise TrajectoryFormatError(f"segment[{index}]: {exc}") from exc
        offset += RECORD_SIZE

    trajectory = Trajectory(header=header, segments=segments)

    try:
        validate_trajectory(trajectory)
    except ValidationError as exc:
        raise TrajectoryFormatError(f"invalid trajectory semantics: {exc}") from exc

    return trajectory


def read_traj(path: str | Path) -> Trajectory:
    return decode_traj(Path(path).read_bytes())


def format_trajectory(trajectory: Trajectory) -> str:
    lines = [
        "TRJ1 V1",
        (
            "start=("
            f"{trajectory.header.start_x_mm:.3f}, "
            f"{trajectory.header.start_y_mm:.3f}, "
            f"{trajectory.header.start_yaw_deg:.3f} deg)"
        ),
        f"segments={len(trajectory.segments)}",
        "",
    ]

    for index, segment in enumerate(trajectory.segments):
        if isinstance(segment, LineSegment):
            lines.extend(
                [
                    f"[{index}] LINE",
                    f"    end=({segment.end_x_mm:.3f}, {segment.end_y_mm:.3f})",
                    f"    speed={segment.speed_mm_s:.3f}",
                    f"    acceleration={segment.acceleration_mm_s2:.3f}",
                ]
            )
        elif isinstance(segment, CircleSegment):
            lines.extend(
                [
                    f"[{index}] CIRCLE",
                    (
                        "    center=("
                        f"{segment.center_x_mm:.3f}, "
                        f"{segment.center_y_mm:.3f})"
                    ),
                    f"    radius={segment.radius_mm:.3f}",
                    f"    start_angle={segment.start_angle_deg:.3f}",
                    f"    sweep={segment.sweep_deg:.3f}",
                    f"    speed={segment.speed_mm_s:.3f}",
                    f"    acceleration={segment.acceleration_mm_s2:.3f}",
                ]
            )
        else:
            raise TypeError(f"unsupported segment object: {type(segment).__name__}")
        lines.append("")

    return "\n".join(lines).rstrip()
