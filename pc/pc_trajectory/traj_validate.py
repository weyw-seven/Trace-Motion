from __future__ import annotations

import math

from .traj_format import (
    CircleSegment,
    DECODER_CIRCLE_SWEEP_EPSILON_DEG,
    EXECUTOR_LENGTH_EPSILON_MM,
    LineSegment,
    PC_CONTINUITY_TOLERANCE_MM,
    Trajectory,
    TrajectoryHeader,
)


class ValidationError(ValueError):
    """Semantic or executable-geometry validation failure."""


def _finite(name: str, value: float) -> float:
    try:
        value = float(value)
    except (TypeError, ValueError) as exc:
        raise ValidationError(f"{name} must be a finite number") from exc

    if not math.isfinite(value):
        raise ValidationError(f"{name} must be finite, got {value!r}")
    return value


def validate_header(header: TrajectoryHeader) -> None:
    if not isinstance(header, TrajectoryHeader):
        raise ValidationError("header must be a TrajectoryHeader")

    _finite("header.start_x_mm", header.start_x_mm)
    _finite("header.start_y_mm", header.start_y_mm)
    _finite("header.start_yaw_deg", header.start_yaw_deg)


def validate_segment(segment) -> None:
    if not isinstance(segment, (LineSegment, CircleSegment)):
        raise ValidationError(
            f"unsupported segment object: {type(segment).__name__}"
        )

    speed = _finite("segment.speed_mm_s", segment.speed_mm_s)
    acceleration = _finite(
        "segment.acceleration_mm_s2",
        segment.acceleration_mm_s2,
    )

    if speed <= 0.0:
        raise ValidationError("segment.speed_mm_s must be > 0")
    if acceleration < 0.0:
        raise ValidationError("segment.acceleration_mm_s2 must be >= 0")

    if isinstance(segment, LineSegment):
        _finite("line.end_x_mm", segment.end_x_mm)
        _finite("line.end_y_mm", segment.end_y_mm)
        return

    center_x = _finite("circle.center_x_mm", segment.center_x_mm)
    center_y = _finite("circle.center_y_mm", segment.center_y_mm)
    radius = _finite("circle.radius_mm", segment.radius_mm)
    start_angle = _finite("circle.start_angle_deg", segment.start_angle_deg)
    sweep = _finite("circle.sweep_deg", segment.sweep_deg)

    # Keep local names useful in a debugger and ensure all conversions happen.
    _ = center_x, center_y, start_angle

    if radius <= 0.0:
        raise ValidationError("circle.radius_mm must be > 0")

    if abs(sweep) < DECODER_CIRCLE_SWEEP_EPSILON_DEG:
        raise ValidationError(
            "circle.sweep_deg magnitude must be >= "
            f"{DECODER_CIRCLE_SWEEP_EPSILON_DEG:g} deg"
        )


def _circle_start_end(segment: CircleSegment) -> tuple[float, float, float, float]:
    start_rad = math.radians(float(segment.start_angle_deg))
    end_rad = math.radians(
        float(segment.start_angle_deg) + float(segment.sweep_deg)
    )
    radius = float(segment.radius_mm)
    center_x = float(segment.center_x_mm)
    center_y = float(segment.center_y_mm)

    start_x = center_x + radius * math.cos(start_rad)
    start_y = center_y + radius * math.sin(start_rad)
    end_x = center_x + radius * math.cos(end_rad)
    end_y = center_y + radius * math.sin(end_rad)

    return start_x, start_y, end_x, end_y


def validate_trajectory(
    trajectory: Trajectory,
    *,
    continuity_tolerance_mm: float = PC_CONTINUITY_TOLERANCE_MM,
) -> None:
    """
    Validate all Phase-A save-time invariants.

    This intentionally does NOT perform tangent/G1 analysis or motion-profile
    simulation. It does enforce the geometry conditions needed to avoid an
    immediate executor rejection: non-degenerate LINE/ARC geometry and CIRCLE
    start-position continuity.
    """
    if not isinstance(trajectory, Trajectory):
        raise ValidationError("trajectory must be a Trajectory")

    validate_header(trajectory.header)

    tolerance = _finite("continuity_tolerance_mm", continuity_tolerance_mm)
    if tolerance < 0.0:
        raise ValidationError("continuity_tolerance_mm must be >= 0")

    current_x = float(trajectory.header.start_x_mm)
    current_y = float(trajectory.header.start_y_mm)

    for index, segment in enumerate(trajectory.segments):
        try:
            validate_segment(segment)

            if isinstance(segment, LineSegment):
                end_x = float(segment.end_x_mm)
                end_y = float(segment.end_y_mm)
                length = math.hypot(end_x - current_x, end_y - current_y)

                if not (length > EXECUTOR_LENGTH_EPSILON_MM):
                    raise ValidationError(
                        "LINE length must be > "
                        f"{EXECUTOR_LENGTH_EPSILON_MM:g} mm"
                    )

                current_x = end_x
                current_y = end_y
                continue

            start_x, start_y, end_x, end_y = _circle_start_end(segment)

            continuity_error = math.hypot(
                start_x - current_x,
                start_y - current_y,
            )
            if continuity_error > tolerance:
                raise ValidationError(
                    "CIRCLE start is discontinuous: "
                    f"error={continuity_error:.9g} mm, "
                    f"tolerance={tolerance:.9g} mm"
                )

            arc_length = (
                float(segment.radius_mm)
                * abs(math.radians(float(segment.sweep_deg)))
            )
            if not math.isfinite(arc_length) or not (
                arc_length > EXECUTOR_LENGTH_EPSILON_MM
            ):
                raise ValidationError(
                    "CIRCLE arc length must be > "
                    f"{EXECUTOR_LENGTH_EPSILON_MM:g} mm"
                )

            current_x = end_x
            current_y = end_y

        except ValidationError as exc:
            raise ValidationError(f"segment[{index}]: {exc}") from exc
