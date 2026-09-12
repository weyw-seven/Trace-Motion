"""Acceptance test for the guarded real-hardware multi-LINE profile.

The normal runs are on the floor, with the pen up and a clear 0.8 m path.
STOP/ESTOP cases also command the chassis, so keep the emergency stop reachable.
"""

from __future__ import annotations

import argparse
import time

from pc_trajectory.demo.robot_link import Pose, RobotLinkError, SerialRobotLink


def wait_for_status(link: SerialRobotLink, predicate, *, timeout_s: float, description: str) -> dict:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        status = dict(link.last_status or {})
        if predicate(status):
            return status
        time.sleep(0.05)
    raise RuntimeError(f"timeout waiting for {description}: {link.last_status}")


def motion_diagnostics(status: dict) -> dict:
    keys = (
        "runner", "runner_error", "error", "tracker", "tracker_phase",
        "record_index", "record_count", "tracking_error_mm",
        "tracking_yaw_error_deg", "settle_elapsed_ms",
        "wheel_a_target_mm_s", "wheel_a_actual_mm_s", "wheel_a_pwm",
        "wheel_a_stall_suspected", "wheel_a_stall_elapsed_ms",
        "wheel_b_target_mm_s", "wheel_b_actual_mm_s", "wheel_b_pwm",
        "wheel_b_stall_suspected", "wheel_b_stall_elapsed_ms",
        "wheel_d_target_mm_s", "wheel_d_actual_mm_s", "wheel_d_pwm",
        "wheel_d_stall_suspected", "wheel_d_stall_elapsed_ms",
    )
    return {key: status.get(key) for key in keys if key in status}


def wait_for_motion_finished(link: SerialRobotLink, *, timeout_s: float, description: str) -> dict:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        status = dict(link.last_status or {})
        if status.get("runner") == "ERROR":
            raise RuntimeError(f"{description} failed: {motion_diagnostics(status)}")
        if status.get("runner") == "FINISHED" and status.get("motion_active") is False:
            return status
        time.sleep(0.05)
    raise RuntimeError(f"timeout waiting for {description}: {motion_diagnostics(dict(link.last_status or {}))}")


def wait_for_motion_record(link: SerialRobotLink, record_index: int, *, job_id: str, timeout_s: float, description: str) -> dict:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        status = dict(link.last_status or {})
        if status.get("current_job") != job_id:
            time.sleep(0.05)
            continue
        if status.get("runner") == "ERROR":
            raise RuntimeError(f"{description} failed: {motion_diagnostics(status)}")
        # STATUS is sampled: the requested record can finish between reports.
        # Completed progress for this job also proves that it was reached.
        reached = (
            status.get("record_index", -1) >= record_index
            and status.get("record_count", 0) > record_index
        )
        if reached and (
            status.get("runner") == "RUNNING"
            or (status.get("runner") == "FINISHED" and status.get("motion_active") is False)
        ):
            return status
        if status.get("runner") in {"FINISHED", "STOPPED", "ESTOPPED"}:
            raise RuntimeError(f"{description} ended before verified progress: {motion_diagnostics(status)}")
        time.sleep(0.05)
    raise RuntimeError(f"timeout waiting for {description}: {motion_diagnostics(dict(link.last_status or {}))}")


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


def expect_error(action, code: str) -> None:
    try:
        action()
    except RobotLinkError as exc:
        if code not in str(exc):
            raise RuntimeError(f"expected {code}, got {exc}") from exc
        return
    raise RuntimeError(f"command unexpectedly succeeded; expected {code}")


def wait_for_line_pose(link: SerialRobotLink, expected_x_mm: float, timeout_s: float = 10.0) -> Pose:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        pose = link.last_pose
        if (
            pose is not None
            and abs(pose.x_mm - expected_x_mm) <= 20.0
            and abs(pose.y_mm) <= 20.0
            and abs(pose.yaw_deg) <= 5.0
        ):
            return pose
        time.sleep(0.05)
    raise RuntimeError(f"LINE endpoint outside first-run tolerance: pose={link.last_pose}")


def run_trajectory(link: SerialRobotLink, payload: bytes, job_id: str) -> dict:
    link.reset_pose(Pose(0.0, 0.0, 0.0))
    wait_for_pose(link, Pose(0.0, 0.0, 0.0))
    receipt = link.upload(payload, job_id=job_id)
    link.run(job_id=receipt.job_id, crc32=receipt.crc32)
    return receipt


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--line-traj", type=argparse.FileType("rb"), required=True)
    parser.add_argument("--long-traj", type=argparse.FileType("rb"))
    parser.add_argument("--circle-traj", type=argparse.FileType("rb"))
    parser.add_argument("--multi-traj", type=argparse.FileType("rb"))
    parser.add_argument("--low-speed-traj", type=argparse.FileType("rb"))
    parser.add_argument("--skip-interruptions", action="store_true")
    args = parser.parse_args()

    line_payload = args.line_traj.read()
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
            "build_profile": "motion-line",
            "execution_mode": "HARDWARE_LINE",
            "hardware_enabled": True,
            "motion_enabled": True,
            "rotate_enabled": False,
            "pen_enabled": False,
            "simulation_enabled": False,
        }
        for name, value in expected.items():
            if hello.get(name) != value:
                raise RuntimeError(f"unexpected motion-line HELLO: {name}={hello.get(name)!r}")
        print("PASS READY motion-line profile")

        status = wait_for_status(
            link,
            lambda item: item.get("motor_ready") is True
            and item.get("odom_ready") is True
            and item.get("hardware_initializing") is False
            and item.get("estop") is False,
            timeout_s=20.0,
            description="motor and odometry ready",
        )
        print(f"PASS MOTOR+ODOM READY status={status}")

        receipt = run_trajectory(link, line_payload, "motion-line-200mm")
        status = wait_for_motion_finished(link, timeout_s=35.0, description="200 mm LINE")
        pose = wait_for_line_pose(link, 200.0)
        print(f"PASS 200 mm LINE FINISHED pose={pose} status={status}")

        if args.multi_traj is not None:
            multi_receipt = run_trajectory(link, args.multi_traj.read(), "motion-line-320mm-multi")
            transition_status = wait_for_motion_record(
                link,
                1,
                job_id=multi_receipt.job_id,
                timeout_s=25.0,
                description="multi-LINE second record",
            )
            print(f"PASS multi-LINE entered second record status={transition_status}")
            status = wait_for_motion_finished(link, timeout_s=35.0, description="320 mm multi-LINE")
            pose = wait_for_line_pose(link, 320.0)
            print(f"PASS 320 mm multi-LINE FINISHED job={multi_receipt.job_id} pose={pose} status={status}")

        if not args.skip_interruptions:
            run_trajectory(link, line_payload, "motion-line-stop")
            time.sleep(0.35)
            link.stop()
            status = wait_for_status(
                link,
                lambda item: item.get("runner") in {"STOPPED", "IDLE"}
                and item.get("motion_active") is False,
                timeout_s=5.0,
                description="STOP during LINE",
            )
            print(f"PASS STOP -> {status.get('runner')}")

            estop_receipt = run_trajectory(link, line_payload, "motion-line-estop")
            time.sleep(0.35)
            link.estop()
            status = wait_for_status(
                link,
                lambda item: item.get("estop") is True
                and item.get("motion_active") is False,
                timeout_s=5.0,
                description="ESTOP during LINE",
            )
            expect_error(
                lambda: link.run(job_id=estop_receipt.job_id, crc32=estop_receipt.crc32),
                "ESTOP_ACTIVE",
            )
            print(f"PASS ESTOP blocks RUN status={status}")
            link.clear_estop()
            wait_for_status(
                link,
                lambda item: item.get("estop") is False,
                timeout_s=5.0,
                description="CLEAR_ESTOP",
            )
            print("PASS CLEAR_ESTOP")

        rejection_cases = (
            (args.long_traj, "LINE_TOO_LONG", "overlong LINE"),
            (args.circle_traj, "UNSUPPORTED_RECORD", "CIRCLE"),
            (args.low_speed_traj, "SPEED_LIMIT", "25 mm/s LINE"),
        )
        for file_handle, code, label in rejection_cases:
            if file_handle is None:
                continue
            payload = file_handle.read()
            receipt = link.upload(payload, job_id=f"motion-line-reject-{label.replace(' ', '-').lower()}")
            expect_error(lambda r=receipt: link.run(job_id=r.job_id, crc32=r.crc32), code)
            print(f"PASS {label} rejected with {code}")

        print("N3 MOTION-LINE ACCEPTANCE PASSED")
        return 0
    finally:
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
