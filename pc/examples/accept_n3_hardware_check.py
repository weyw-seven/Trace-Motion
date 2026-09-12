"""Acceptance test for the real-hardware, motion-disabled N3 profile.

This profile initializes the motor/IMU/odometry stack and reports real pose,
but must not execute ROTATE_REL or motion trajectories.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import time

from pc_trajectory.demo.robot_link import Pose, RobotLinkError, SerialRobotLink


def wait_for_status(
    link: SerialRobotLink,
    predicate,
    *,
    timeout_s: float,
    description: str,
    fail_on_hardware_error: bool = False,
) -> dict:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        status = dict(link.last_status or {})
        if predicate(status):
            return status
        if (
            fail_on_hardware_error
            and status.get("hardware_initializing") is False
            and status.get("error")
        ):
            raise RuntimeError(f"hardware initialization failed: {status['error']}")
        time.sleep(0.05)
    raise RuntimeError(f"timeout waiting for {description}: {link.last_status}")


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
    parser.add_argument("--motion-traj", required=True, type=Path)
    args = parser.parse_args()

    payload = args.motion_traj.read_bytes()
    if not payload:
        raise SystemExit(f"trajectory file is empty: {args.motion_traj}")

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
            "build_profile": "hardware-check",
            "execution_mode": "HARDWARE_CHECK",
            "hardware_enabled": True,
            "motion_enabled": False,
            "pen_enabled": False,
        }
        for name, value in expected.items():
            if hello.get(name) != value:
                raise RuntimeError(f"unexpected hardware-check HELLO: {name}={hello.get(name)!r}")
        print("PASS READY hardware-check profile")

        status = wait_for_status(
            link,
            lambda item: item.get("motor_ready") is True
            and item.get("odom_ready") is True
            and item.get("hardware_initializing") is False,
            timeout_s=20.0,
            description="motor and odometry ready",
            fail_on_hardware_error=True,
        )
        if status.get("estop") is True:
            raise RuntimeError(f"hardware-check unexpectedly starts ESTOPPED: {status}")
        print("PASS MOTOR+ODOM READY with motion disabled")

        link.reset_pose(Pose(12.0, -8.0, 7.0))
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            pose = link.last_pose
            if pose is not None and abs(pose.x_mm - 12.0) < 1.0 and abs(pose.y_mm + 8.0) < 1.0 and abs(pose.yaw_deg - 7.0) < 1.0:
                break
            time.sleep(0.05)
        else:
            raise RuntimeError(f"RESET_POSE did not update real odometry pose: {link.last_pose}")
        print(f"PASS RESET_POSE -> {link.last_pose}")

        expect_error(lambda: link.rotate_relative(10.0, 20.0), "MOTION_DISABLED")
        print("PASS ROTATE_REL rejected with MOTION_DISABLED")

        receipt = link.upload(payload, job_id="hardware-check-motion")
        expect_error(
            lambda: link.run(job_id=receipt.job_id, crc32=receipt.crc32),
            "MOTION_DISABLED",
        )
        print("PASS motion trajectory rejected with MOTION_DISABLED")

        link.estop()
        wait_for_status(link, lambda item: item.get("estop") is True, timeout_s=3.0, description="ESTOP")
        print("PASS ESTOP -> ESTOPPED")
        link.clear_estop()
        wait_for_status(link, lambda item: item.get("estop") is False, timeout_s=3.0, description="CLEAR_ESTOP")
        print("PASS CLEAR_ESTOP -> not latched")

        print("N3 HARDWARE-CHECK ACCEPTANCE PASSED")
        return 0
    finally:
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
