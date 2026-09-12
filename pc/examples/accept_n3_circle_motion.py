"""Acceptance test for the guarded real-hardware LINE+CIRCLE motion profile.

Run on the floor with the pen up, a clear 1 m x 1 m area, and an immediately
reachable emergency stop.  This test verifies CIRCLE geometry and safety; it
does not enable a physical pen or CUBIC_BEZIER.
"""

from __future__ import annotations

import argparse
import math
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


def diagnostics(status: dict) -> dict:
    keys = (
        "runner", "runner_error", "tracker_error", "error", "tracker", "tracker_phase",
        "record_index", "record_count", "record_type", "tracking_error_mm",
        "tracking_yaw_error_deg", "reference_x_mm", "reference_y_mm",
        "settle_elapsed_ms", "wheel_a_target_mm_s", "wheel_a_actual_mm_s",
        "wheel_a_pwm", "wheel_a_stall_suspected", "wheel_a_stall_elapsed_ms",
        "wheel_b_target_mm_s", "wheel_b_actual_mm_s", "wheel_b_pwm",
        "wheel_b_stall_suspected", "wheel_b_stall_elapsed_ms",
        "wheel_d_target_mm_s", "wheel_d_actual_mm_s", "wheel_d_pwm",
        "wheel_d_stall_suspected", "wheel_d_stall_elapsed_ms",
    )
    return {key: status.get(key) for key in keys if key in status}


def expect_error(action, code: str) -> None:
    try:
        action()
    except RobotLinkError as exc:
        if code not in str(exc):
            raise RuntimeError(f"expected {code}, got {exc}") from exc
        return
    raise RuntimeError(f"command unexpectedly succeeded; expected {code}")


def reset_pose(link: SerialRobotLink) -> None:
    link.reset_pose(Pose(0.0, 0.0, 0.0))
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        pose = link.last_pose
        if (
            pose is not None
            and abs(pose.x_mm) < 1.0
            and abs(pose.y_mm) < 1.0
            and abs(pose.yaw_deg) < 1.0
        ):
            return
        time.sleep(0.05)
    raise RuntimeError(f"RESET_POSE did not settle: {link.last_pose}")


def upload_and_run(link: SerialRobotLink, payload: bytes, job_id: str):
    reset_pose(link)
    receipt = link.upload(payload, job_id=job_id)
    link.run(job_id=receipt.job_id, crc32=receipt.crc32)
    return receipt


def observe_until_finished(link: SerialRobotLink, job_id: str, *, timeout_s: float, description: str):
    deadline = time.monotonic() + timeout_s
    poses: list[Pose] = []
    seen_records: set[int] = set()
    seen_types: set[str] = set()
    last_timestamp = -1.0
    finished_at: float | None = None
    run_started = False
    while time.monotonic() < deadline:
        status = dict(link.last_status or {})
        pose = link.last_pose
        if pose is not None and pose.timestamp_ms > last_timestamp:
            poses.append(pose)
            last_timestamp = pose.timestamp_ms
        if status.get("current_job") == job_id:
            runner_state = status.get("runner")
            if (
                runner_state in {"STARTING", "RUNNING", "WAITING"}
                or status.get("motion_active") is True
            ):
                run_started = True
                finished_at = None

            # UPLOAD_END changes current_job before RUN_TRAJECTORY's STARTING
            # status reaches the PC.  During that short window the runner can
            # still contain the previous job's FINISHED state.  Never accept
            # a terminal state until this job has visibly entered execution.
            if not run_started:
                time.sleep(0.05)
                continue

            if isinstance(status.get("record_index"), int):
                seen_records.add(status["record_index"])
            if isinstance(status.get("record_type"), str):
                seen_types.add(status["record_type"])
            if runner_state == "ERROR":
                raise RuntimeError(f"{description} failed: {diagnostics(status)}")
            if runner_state == "FINISHED" and status.get("motion_active") is False:
                # STATUS and POSE are emitted as separate asynchronous lines.
                # Give the final POSE frame time to arrive before the caller
                # reads link.last_pose; otherwise a valid run can be checked
                # against the reset pose from the beginning of the job.
                if finished_at is None:
                    finished_at = time.monotonic()
                elif time.monotonic() - finished_at >= 0.25:
                    return status, poses, seen_records, seen_types
            if runner_state in {"STOPPED", "ESTOPPED"}:
                raise RuntimeError(f"{description} interrupted: {diagnostics(status)}")
        time.sleep(0.05)
    raise RuntimeError(f"timeout waiting for {description}: {diagnostics(dict(link.last_status or {}))}")


def assert_endpoint(pose: Pose | None, x_mm: float, y_mm: float, label: str) -> None:
    if pose is None:
        raise RuntimeError(f"{label} has no final POSE")
    if abs(pose.x_mm - x_mm) > 20.0 or abs(pose.y_mm - y_mm) > 20.0:
        raise RuntimeError(f"{label} endpoint outside 20 mm tolerance: {pose}")
    if abs(pose.yaw_deg) > 5.0:
        raise RuntimeError(f"{label} yaw drift exceeds 5 deg: {pose}")


def assert_arc_samples(poses: list[Pose], center_x_mm: float, center_y_mm: float, radius_mm: float, *, expected_y_sign: int, label: str) -> None:
    if len(poses) < 3:
        raise RuntimeError(f"{label} produced too few POSE samples: {len(poses)}")
    radial_errors = [abs(math.hypot(pose.x_mm - center_x_mm, pose.y_mm - center_y_mm) - radius_mm) for pose in poses]
    max_error = max(radial_errors)
    rms_error = math.sqrt(sum(error * error for error in radial_errors) / len(radial_errors))
    extreme_y = max(pose.y_mm for pose in poses) if expected_y_sign > 0 else min(pose.y_mm for pose in poses)
    if expected_y_sign * extreme_y < 30.0:
        raise RuntimeError(f"{label} moved in the wrong CIRCLE direction: extreme_y={extreme_y:.1f}")
    if max_error > 25.0 or rms_error > 15.0:
        raise RuntimeError(f"{label} radial error too large: max={max_error:.1f} mm rms={rms_error:.1f} mm")
    print(f"PASS {label} radial=max {max_error:.1f} mm rms {rms_error:.1f} mm")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--line-traj", type=argparse.FileType("rb"), required=True)
    parser.add_argument("--ccw-traj", type=argparse.FileType("rb"), required=True)
    parser.add_argument("--cw-traj", type=argparse.FileType("rb"), required=True)
    parser.add_argument("--mixed-traj", type=argparse.FileType("rb"), required=True)
    parser.add_argument("--small-radius-traj", type=argparse.FileType("rb"))
    parser.add_argument("--low-speed-traj", type=argparse.FileType("rb"))
    parser.add_argument("--cubic-traj", type=argparse.FileType("rb"))
    parser.add_argument("--skip-interruptions", action="store_true")
    args = parser.parse_args()

    line_payload = args.line_traj.read()
    ccw_payload = args.ccw_traj.read()
    cw_payload = args.cw_traj.read()
    mixed_payload = args.mixed_traj.read()

    link = SerialRobotLink(port=args.port, baudrate=115200, ack_timeout_s=3.0, heartbeat_interval_s=1.0)
    try:
        link.connect()
        hello = link.wait_ready(timeout_s=5.0)
        expected = {
            "build_profile": "motion-circle",
            "execution_mode": "HARDWARE_PATH",
            "hardware_enabled": True,
            "motion_enabled": True,
            "circle_enabled": True,
            "rotate_enabled": False,
            "pen_enabled": False,
            "simulation_enabled": False,
        }
        for key, value in expected.items():
            if hello.get(key) != value:
                raise RuntimeError(f"unexpected motion-circle HELLO: {key}={hello.get(key)!r}")
        print("PASS READY motion-circle profile")

        status = wait_for_status(
            link,
            lambda item: item.get("motor_ready") is True and item.get("odom_ready") is True
            and item.get("hardware_initializing") is False and item.get("estop") is False,
            timeout_s=20.0,
            description="motor and odometry ready",
        )
        print(f"PASS MOTOR+ODOM READY status={status}")

        line = upload_and_run(link, line_payload, "motion-circle-line")
        status, _, seen_records, seen_types = observe_until_finished(link, line.job_id, timeout_s=75.0, description="200 mm LINE")
        assert_endpoint(link.last_pose, 200.0, 0.0, "200 mm LINE")
        if status.get("record_index") != 0 or status.get("record_count") != 1:
            raise RuntimeError(f"LINE record progress is wrong: {diagnostics(status)}")
        if status.get("record_type") != "LINE" and "LINE" not in seen_types and 0 not in seen_records:
            raise RuntimeError(f"LINE record type was not observed: records={seen_records} types={seen_types}")
        print(f"PASS 200 mm LINE FINISHED pose={link.last_pose} status={status}")

        ccw = upload_and_run(link, ccw_payload, "motion-circle-ccw")
        status, poses, _, _ = observe_until_finished(link, ccw.job_id, timeout_s=75.0, description="CCW CIRCLE")
        assert_endpoint(link.last_pose, 150.0, 150.0, "CCW CIRCLE")
        assert_arc_samples(poses, 0.0, 150.0, 150.0, expected_y_sign=1, label="CCW CIRCLE")
        print(f"PASS CCW CIRCLE FINISHED pose={link.last_pose} status={status}")

        cw = upload_and_run(link, cw_payload, "motion-circle-cw")
        status, poses, _, _ = observe_until_finished(link, cw.job_id, timeout_s=75.0, description="CW CIRCLE")
        assert_endpoint(link.last_pose, 150.0, -150.0, "CW CIRCLE")
        assert_arc_samples(poses, 0.0, -150.0, 150.0, expected_y_sign=-1, label="CW CIRCLE")
        print(f"PASS CW CIRCLE FINISHED pose={link.last_pose} status={status}")

        mixed = upload_and_run(link, mixed_payload, "motion-line-circle-line")
        status, _, seen_records, seen_types = observe_until_finished(link, mixed.job_id, timeout_s=75.0, description="LINE+CIRCLE+LINE")
        assert_endpoint(link.last_pose, 350.0, 350.0, "LINE+CIRCLE+LINE")
        if status.get("record_index") != 2 or status.get("record_count") != 3:
            raise RuntimeError(f"mixed path record progress is wrong: {diagnostics(status)}")
        # STATUS is sampled asynchronously.  Seeing the terminal third record
        # proves all ordered records, including CIRCLE index 1, completed even
        # when its short RUNNING window falls between STATUS messages.
        print(
            "PASS LINE+CIRCLE+LINE FINISHED "
            f"pose={link.last_pose} sampled_records={sorted(seen_records)} "
            f"sampled_types={sorted(seen_types)} status={status}"
        )

        if not args.skip_interruptions:
            stop = upload_and_run(link, ccw_payload, "motion-circle-stop")
            wait_for_status(link, lambda item: item.get("current_job") == stop.job_id and item.get("motion_active") is True, timeout_s=5.0, description="CIRCLE start before STOP")
            link.stop()
            status = wait_for_status(link, lambda item: item.get("runner") in {"STOPPED", "IDLE"} and item.get("motion_active") is False, timeout_s=5.0, description="STOP during CIRCLE")
            print(f"PASS STOP -> {status.get('runner')}")

            estop = upload_and_run(link, ccw_payload, "motion-circle-estop")
            wait_for_status(link, lambda item: item.get("current_job") == estop.job_id and item.get("motion_active") is True, timeout_s=5.0, description="CIRCLE start before ESTOP")
            link.estop()
            status = wait_for_status(link, lambda item: item.get("estop") is True and item.get("motion_active") is False, timeout_s=5.0, description="ESTOP during CIRCLE")
            expect_error(lambda: link.run(job_id=estop.job_id, crc32=estop.crc32), "ESTOP_ACTIVE")
            link.clear_estop()
            wait_for_status(link, lambda item: item.get("estop") is False, timeout_s=5.0, description="CLEAR_ESTOP")
            print(f"PASS ESTOP/CLEAR_ESTOP status={status}")

        for fixture, code, label in (
            (args.small_radius_traj, "CIRCLE_RADIUS_LIMIT", "small-radius CIRCLE"),
            (args.low_speed_traj, "SPEED_LIMIT", "25 mm/s CIRCLE"),
            (args.cubic_traj, "UNSUPPORTED_RECORD", "CUBIC_BEZIER"),
        ):
            if fixture is None:
                continue
            receipt = link.upload(fixture.read(), job_id=f"motion-circle-reject-{label.replace(' ', '-').lower()}")
            expect_error(lambda r=receipt: link.run(job_id=r.job_id, crc32=r.crc32), code)
            print(f"PASS {label} rejected with {code}")

        print("N3 MOTION-CIRCLE ACCEPTANCE PASSED")
        return 0
    finally:
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
