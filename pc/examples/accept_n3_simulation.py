"""Acceptance test for the hardware-free N3 LINE/CIRCLE simulator profile.

The profile runs the same TRJ2 geometry and speed planner as the future motion
path, but never initializes motor, odometry, or pen hardware.  This makes the
full command/state/error contract testable before the vehicle is available.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import time

from pc_trajectory.demo.robot_link import RobotLinkError, SerialRobotLink
from pc_trajectory.demo.robot_link import Pose


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


def expect_error(action, code: str) -> None:
    try:
        action()
    except RobotLinkError as exc:
        if code not in str(exc):
            raise RuntimeError(f"expected {code}, got {exc}") from exc
        return
    raise RuntimeError(f"command unexpectedly succeeded; expected {code}")


def assert_simulation_status(status: dict, *, runner: str | None = None) -> None:
    expected = {
        "execution_mode": "SIMULATED",
        "simulation_enabled": True,
        "hardware_enabled": False,
        "motion_enabled": False,
        "pen_enabled": False,
        "motor_ready": False,
        "odom_ready": False,
    }
    for name, value in expected.items():
        if status.get(name) != value:
            raise RuntimeError(f"simulation safety invariant failed: {name}={status.get(name)!r}; {status}")
    if runner is not None and status.get("runner") != runner:
        raise RuntimeError(f"expected runner={runner}, got {status}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="ESP32 COM port, for example COM9")
    parser.add_argument("--mixed-traj", required=True, type=Path)
    parser.add_argument("--wait-traj", required=True, type=Path)
    parser.add_argument("--line-traj", required=True, type=Path)
    parser.add_argument("--cubic-traj", required=True, type=Path)
    args = parser.parse_args()

    paths = (args.mixed_traj, args.wait_traj, args.line_traj, args.cubic_traj)
    for path in paths:
        if not path.read_bytes():
            raise SystemExit(f"trajectory file is empty: {path}")

    link = SerialRobotLink(
        port=args.port,
        baudrate=115200,
        ack_timeout_s=3.0,
        heartbeat_interval_s=1.0,
    )
    try:
        link.connect()
        hello = link.wait_ready(timeout_s=5.0)
        expected_hello = {
            "build_profile": "simulation",
            "execution_mode": "SIMULATED",
            "simulation_enabled": True,
            "hardware_enabled": False,
            "motion_enabled": False,
            "pen_enabled": False,
        }
        for name, value in expected_hello.items():
            if hello.get(name) != value:
                raise RuntimeError(f"not a simulation firmware build: {name}={hello.get(name)!r}")
        print("PASS READY hardware-free simulation profile")

        mixed = link.upload(args.mixed_traj.read_bytes(), job_id="sim-mixed")
        link.run(job_id=mixed.job_id, crc32=mixed.crc32)
        status = wait_for_status(
            link,
            lambda item: item.get("runner") == "FINISHED",
            timeout_s=15.0,
            description="mixed trajectory FINISHED",
        )
        assert_simulation_status(status, runner="FINISHED")
        if status.get("record_count") != 7:
            raise RuntimeError(f"unexpected mixed record_count: {status}")
        pose_deadline = time.monotonic() + 1.0
        pose = link.last_pose
        while (
            time.monotonic() < pose_deadline
            and (
                pose is None
                or abs(pose.x_mm - 0.0) > 2.0
                or abs(pose.y_mm - 100.0) > 2.0
            )
        ):
            time.sleep(0.05)
            pose = link.last_pose
        if pose is None or abs(pose.x_mm - 0.0) > 2.0 or abs(pose.y_mm - 100.0) > 2.0:
            raise RuntimeError(f"mixed trajectory final pose is not near (0, 100): {pose}")
        if status.get("pen") != "UP":
            raise RuntimeError(f"mixed trajectory did not finish pen-up: {status}")
        print(f"PASS LINE+CIRCLE+PEN+WAIT FINISHED pose={pose}")

        wait = link.upload(args.wait_traj.read_bytes(), job_id="sim-stop")
        link.run(job_id=wait.job_id, crc32=wait.crc32)
        wait_for_status(
            link,
            lambda item: item.get("runner") in {"RUNNING", "WAITING"},
            timeout_s=2.0,
            description="simulation RUNNING/WAITING before STOP",
        )
        link.stop()
        status = wait_for_status(
            link,
            lambda item: item.get("runner") == "STOPPED" and item.get("stop_requested") is False,
            timeout_s=3.0,
            description="STOPPED",
        )
        assert_simulation_status(status, runner="STOPPED")
        if status.get("pen") != "UP":
            raise RuntimeError(f"STOP did not leave pen logical state UP: {status}")
        print("PASS STOP -> STOPPED")

        wait = link.upload(args.wait_traj.read_bytes(), job_id="sim-estop")
        link.run(job_id=wait.job_id, crc32=wait.crc32)
        wait_for_status(
            link,
            lambda item: item.get("runner") in {"RUNNING", "WAITING"},
            timeout_s=2.0,
            description="simulation RUNNING/WAITING before ESTOP",
        )
        link.estop()
        status = wait_for_status(
            link,
            lambda item: item.get("runner") == "ESTOPPED"
            and item.get("estop") is True
            and item.get("stop_requested") is False,
            timeout_s=3.0,
            description="ESTOPPED",
        )
        assert_simulation_status(status, runner="ESTOPPED")
        print("PASS ESTOP -> ESTOPPED")
        link.clear_estop()
        status = wait_for_status(
            link,
            lambda item: item.get("runner") == "IDLE" and item.get("estop") is False,
            timeout_s=3.0,
            description="IDLE after CLEAR_ESTOP",
        )
        assert_simulation_status(status, runner="IDLE")
        print("PASS CLEAR_ESTOP -> IDLE")

        line = link.upload(args.line_traj.read_bytes(), job_id="sim-busy")
        link.run(job_id=line.job_id, crc32=line.crc32)
        wait_for_status(
            link,
            lambda item: item.get("runner") == "RUNNING",
            timeout_s=2.0,
            description="RUNNING before busy upload",
        )
        expect_error(
            lambda: link.upload(args.mixed_traj.read_bytes(), job_id="sim-busy-rejected"),
            "RUNNER_BUSY",
        )
        print("PASS upload while running rejected with RUNNER_BUSY")
        link.stop()
        wait_for_status(
            link,
            lambda item: item.get("runner") == "STOPPED",
            timeout_s=3.0,
            description="STOPPED after busy check",
        )

        cubic = link.upload(args.cubic_traj.read_bytes(), job_id="sim-cubic-rejected")
        expect_error(
            lambda: link.run(job_id=cubic.job_id, crc32=cubic.crc32),
            "UNSUPPORTED_RECORD",
        )
        print("PASS CUBIC rejected with UNSUPPORTED_RECORD")

        reset = Pose(120.0, -40.0, 15.0)
        link.reset_pose(reset)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            pose = link.last_pose
            if pose is not None and abs(pose.x_mm - reset.x_mm) < 0.5 and abs(pose.y_mm - reset.y_mm) < 0.5 and abs(pose.yaw_deg - reset.yaw_deg) < 0.5:
                break
            time.sleep(0.05)
        else:
            raise RuntimeError(f"RESET_POSE did not update simulated pose: {link.last_pose}")
        print(f"PASS RESET_POSE -> {link.last_pose}")

        print("N3 SIMULATION ACCEPTANCE PASSED")
        return 0
    finally:
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
