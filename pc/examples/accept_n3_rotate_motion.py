"""Acceptance test for the guarded real-hardware ROTATE_REL profile.

Run this with the wheels lifted clear of the ground.  The profile enables only
bounded relative rotation; trajectory LINE/CIRCLE execution and the physical
Pen remain disabled.
"""

from __future__ import annotations

import argparse
import time

from pc_trajectory.demo.robot_link import Pose, RobotLinkError, SerialRobotLink


def wait_for_status(
    link: SerialRobotLink,
    predicate,
    *,
    timeout_s: float,
    description: str,
) -> dict:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        status = dict(link.last_status or {})
        if predicate(status):
            return status
        time.sleep(0.05)
    raise RuntimeError(f"timeout waiting for {description}: {link.last_status}")


def wait_for_pose(link: SerialRobotLink, expected: Pose, timeout_s: float = 3.0) -> Pose:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        pose = link.last_pose
        if (
            pose is not None
            and abs(pose.x_mm - expected.x_mm) < 1.0
            and abs(pose.y_mm - expected.y_mm) < 1.0
            and abs(pose.yaw_deg - expected.yaw_deg) < 1.0
        ):
            return pose
        time.sleep(0.05)
    raise RuntimeError(f"RESET_POSE did not settle: {link.last_pose}")


def wait_for_pose_condition(link: SerialRobotLink, predicate, description: str, timeout_s: float = 3.0) -> Pose:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        pose = link.last_pose
        if pose is not None and predicate(pose):
            return pose
        time.sleep(0.05)
    raise RuntimeError(f"timeout waiting for {description}: {link.last_pose}")


def wait_rotation_finished(link: SerialRobotLink, description: str) -> dict:
    return wait_for_status(
        link,
        lambda item: item.get("motion_active") is False
        and item.get("runner") == "FINISHED",
        timeout_s=8.0,
        description=description,
    )


def expect_error(action, code: str) -> None:
    try:
        action()
    except RobotLinkError as exc:
        if code not in str(exc):
            raise RuntimeError(f"expected {code}, got {exc}") from exc
        return
    raise RuntimeError(f"command unexpectedly succeeded; expected {code}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--motion-traj", type=argparse.FileType("rb"), help="optional LINE/CIRCLE TRJ2 used to verify motion is still disabled")
    args = parser.parse_args()

    link = SerialRobotLink(
        port=args.port,
        baudrate=115200,
        ack_timeout_s=3.0,
        heartbeat_interval_s=1.0,
    )
    try:
        link.connect()
        hello = link.wait_ready(timeout_s=5.0)
        expected = {
            "build_profile": "motion-rotate",
            "execution_mode": "HARDWARE_ROTATE",
            "hardware_enabled": True,
            "motion_enabled": False,
            "rotate_enabled": True,
            "pen_enabled": False,
        }
        for name, value in expected.items():
            if hello.get(name) != value:
                raise RuntimeError(f"unexpected motion-rotate HELLO: {name}={hello.get(name)!r}")
        print("PASS READY motion-rotate profile")

        status = wait_for_status(
            link,
            lambda item: item.get("motor_ready") is True
            and item.get("odom_ready") is True
            and item.get("motion_ready") is True
            and item.get("hardware_initializing") is False
            and item.get("estop") is False,
            timeout_s=20.0,
            description="motor, odometry, and rotate controller ready",
        )
        print(f"PASS MOTOR+ODOM+ROTATE READY status={status}")
        expect_error(lambda: link.rotate_relative(31.0, 20.0), "ROTATE_LIMIT")
        print("PASS ROTATE_REL safety limit rejects 31 deg")

        link.reset_pose(Pose(0.0, 0.0, 0.0))
        start_pose = wait_for_pose(link, Pose(0.0, 0.0, 0.0))

        link.rotate_relative(10.0, 20.0)
        status = wait_rotation_finished(link, "+10 degree rotation")
        pose = wait_for_pose_condition(
            link,
            lambda item: item.timestamp_ms > start_pose.timestamp_ms and item.yaw_deg >= 7.0,
            "+10 degree CCW pose update",
        )
        print(f"PASS +10 deg is CCW pose={pose}")

        link.reset_pose(Pose(0.0, 0.0, 0.0))
        start_pose = wait_for_pose(link, Pose(0.0, 0.0, 0.0))
        link.rotate_relative(-10.0, 20.0)
        status = wait_rotation_finished(link, "-10 degree rotation")
        pose = wait_for_pose_condition(
            link,
            lambda item: item.timestamp_ms > start_pose.timestamp_ms and item.yaw_deg <= -7.0,
            "-10 degree CW pose update",
        )
        print(f"PASS -10 deg is CW pose={pose}")

        link.reset_pose(Pose(0.0, 0.0, 0.0))
        wait_for_pose(link, Pose(0.0, 0.0, 0.0))
        link.rotate_relative(25.0, 20.0)
        time.sleep(0.25)
        link.stop()
        status = wait_for_status(
            link,
            lambda item: item.get("motion_active") is False
            and item.get("runner") in {"STOPPED", "IDLE"},
            timeout_s=3.0,
            description="STOP after rotation",
        )
        print(f"PASS STOP -> {status.get('runner')}")

        link.reset_pose(Pose(0.0, 0.0, 0.0))
        wait_for_pose(link, Pose(0.0, 0.0, 0.0))
        link.rotate_relative(25.0, 20.0)
        time.sleep(0.25)
        link.estop()
        status = wait_for_status(
            link,
            lambda item: item.get("estop") is True,
            timeout_s=3.0,
            description="ESTOP during rotation",
        )
        expect_error(lambda: link.rotate_relative(10.0, 20.0), "ESTOP_ACTIVE")
        print(f"PASS ESTOP blocks rotation status={status}")

        link.clear_estop()
        status = wait_for_status(
            link,
            lambda item: item.get("estop") is False
            and item.get("motion_safety_latched") is False,
            timeout_s=3.0,
            description="CLEAR_ESTOP",
        )
        print(f"PASS CLEAR_ESTOP -> {status.get('runner')}")

        if args.motion_traj is not None:
            payload = args.motion_traj.read()
            receipt = link.upload(payload, job_id="rotate-profile-motion-reject")
            expect_error(
                lambda: link.run(job_id=receipt.job_id, crc32=receipt.crc32),
                "MOTION_DISABLED",
            )
            print("PASS LINE/CIRCLE trajectory remains MOTION_DISABLED")

        print("N3 ROTATE-MOTION ACCEPTANCE PASSED")
        return 0
    finally:
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
