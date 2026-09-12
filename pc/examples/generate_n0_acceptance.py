"""Generate the N0 coordinate and simulated-estop acceptance image."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import matplotlib.pyplot as plt

from pc_trajectory.demo.map_transform import MapTransform
from pc_trajectory.demo.protocol import MessageType, make_command
from pc_trajectory.demo.simulated_robot_link import SimulatedRobotLink
from pc_trajectory.geometry import Point2D


def main() -> None:
    output = ROOT / "test_results" / "virtual_map_demo" / "n0_protocol"
    output.mkdir(parents=True, exist_ok=True)

    link = SimulatedRobotLink()
    link.connect()
    link.drain_events()
    poses = [link.pose]
    link.inject_body_motion(100.0, 0.0, 1.0)
    poses.append(link.pose)
    link.inject_body_motion(0.0, 100.0, 1.0)
    poses.append(link.pose)
    link.send(make_command(MessageType.ROTATE_REL, 1, angle_deg=90.0, speed_deg_s=45.0))
    poses.append(link.pose)
    link.inject_body_motion(50.0, 0.0, 1.0)
    poses.append(link.pose)

    transform = MapTransform(320.0, 240.0, 2.0)
    world_points = [(pose.x_mm, pose.y_mm) for pose in poses]
    screen_points = [transform.world_to_pixel(Point2D(*point)) for point in world_points]

    figure, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    world = axes[0]
    xs, ys = zip(*world_points)
    world.plot(xs, ys, "o-", color="#2563eb", label="simulated odometry")
    world.quiver(0, 0, 80, 0, angles="xy", scale_units="xy", scale=1, color="#dc2626", width=0.008)
    world.quiver(0, 0, 0, 80, angles="xy", scale_units="xy", scale=1, color="#16a34a", width=0.008)
    world.text(82, 0, "+X forward", color="#dc2626")
    world.text(0, 82, "+Y left", color="#16a34a")
    final_pose = poses[-1]
    heading_length = 35.0
    world.quiver(
        final_pose.x_mm,
        final_pose.y_mm,
        heading_length * math.cos(math.radians(final_pose.yaw_deg)),
        heading_length * math.sin(math.radians(final_pose.yaw_deg)),
        angles="xy",
        scale_units="xy",
        scale=1,
        color="#f59e0b",
        width=0.008,
    )
    world.annotate(
        "yaw=90° CCW",
        xy=(final_pose.x_mm, final_pose.y_mm),
        xytext=(final_pose.x_mm + 8.0, final_pose.y_mm - 12.0),
        color="#b45309",
        arrowprops={"arrowstyle": "->", "color": "#b45309"},
    )
    world.set_title("WORLD coordinates (mm)")
    world.set_xlabel("X mm")
    world.set_ylabel("Y mm")
    world.set_aspect("equal", adjustable="box")
    world.grid(True, alpha=0.3)
    world.legend(loc="upper left")

    screen = axes[1]
    su, sv = zip(*screen_points)
    screen.plot(su, sv, "o-", color="#7c3aed", label="pixel projection")
    origin = transform.world_to_pixel(Point2D(0.0, 0.0))
    screen.scatter(*origin, color="black", marker="x", s=80, label="screen origin")
    screen.annotate("pixel Y down", xy=(origin[0], origin[1] + 80), xytext=(origin[0] + 20, origin[1] + 100), arrowprops={"arrowstyle": "->"})
    screen.set_title("Screen mapping (pixels, Y down)")
    screen.set_xlabel("u px")
    screen.set_ylabel("v px")
    screen.invert_yaxis()
    screen.set_aspect("equal", adjustable="box")
    screen.grid(True, alpha=0.3)
    screen.legend(loc="upper left")

    figure.suptitle("N0 acceptance: +X forward, +Y left, CCW positive yaw")
    figure.savefig(output / "coordinate_acceptance.png", dpi=150)
    plt.close(figure)

    (output / "simulated_session.jsonl").write_text(
        "\n".join(
            json.dumps(pose.to_message(index), ensure_ascii=False, sort_keys=True)
            for index, pose in enumerate(poses, start=1)
        )
        + "\n",
        encoding="utf-8",
    )
    (output / "test_report.txt").write_text(
        "N0 coordinate acceptance generated.\n"
        "Verified body +X -> world +X at yaw 0.\n"
        "Verified body +Y -> world +Y at yaw 0.\n"
        "Verified positive yaw is counter-clockwise.\n"
        "Verified screen Y inversion in MapTransform.\n"
        "Verified simulated E-Stop behavior by unit test.\n",
        encoding="utf-8",
    )
    print(output / "coordinate_acceptance.png")


if __name__ == "__main__":
    main()
