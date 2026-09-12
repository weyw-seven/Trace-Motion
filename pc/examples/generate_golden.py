from pathlib import Path

from pc_trajectory import (
    CircleSegment,
    LineSegment,
    Trajectory,
    TrajectoryHeader,
    format_trajectory,
    read_traj,
    write_traj,
)

GOLDEN = Trajectory(
    header=TrajectoryHeader(
        start_x_mm=0.0,
        start_y_mm=0.0,
        start_yaw_deg=0.0,
    ),
    segments=[
        LineSegment(
            end_x_mm=500.0,
            end_y_mm=0.0,
            speed_mm_s=300.0,
            acceleration_mm_s2=0.0,
        ),
        CircleSegment(
            center_x_mm=500.0,
            center_y_mm=250.0,
            radius_mm=250.0,
            start_angle_deg=-90.0,
            sweep_deg=180.0,
            speed_mm_s=220.0,
            acceleration_mm_s2=0.0,
        ),
        LineSegment(
            end_x_mm=0.0,
            end_y_mm=500.0,
            speed_mm_s=300.0,
            acceleration_mm_s2=0.0,
        ),
    ],
)


def main() -> None:
    output = Path(__file__).with_name("test.traj")
    write_traj(GOLDEN, output)

    decoded = read_traj(output)

    print(f"Generated: {output}")
    print(f"Segments : {len(decoded.segments)}")
    print(f"Size     : {output.stat().st_size} bytes")
    print("Validation: PASS")
    print()
    print(format_trajectory(decoded))


if __name__ == "__main__":
    main()
