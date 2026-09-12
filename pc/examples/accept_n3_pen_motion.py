"""Acceptance test for the guarded real-hardware LINE/CIRCLE/physical-pen profile.

Run on the floor with an external 5 V servo supply, common ground, a clear
working area, and an immediately reachable emergency stop.  The fixture
geometry is intentionally conservative: motion is 80 mm/s, LINEs are 200 mm,
and the CIRCLE radius is 150 mm.
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
        if status.get("hardware_initializing") is False and status.get("error"):
            raise RuntimeError(f"{description} failed: {diagnostics(status)}")
        time.sleep(0.05)
    raise RuntimeError(f"timeout waiting for {description}: {diagnostics(dict(link.last_status or {}))}")


def diagnostics(status: dict) -> dict:
    keys = (
        "runner", "runner_error", "tracker_error", "error", "tracker", "tracker_phase",
        "record_index", "record_count", "record_type", "pen", "pen_state", "pen_target",
        "pen_busy", "pen_settling", "pen_error", "tracking_error_mm",
        "tracking_yaw_error_deg", "reference_x_mm", "reference_y_mm",
        "command_vx_body_mm_s", "command_vy_body_mm_s", "command_w_rad_s",
        "wheel_a_target_mm_s", "wheel_a_actual_mm_s", "wheel_a_pwm",
        "wheel_b_target_mm_s", "wheel_b_actual_mm_s", "wheel_b_pwm",
        "wheel_d_target_mm_s", "wheel_d_actual_mm_s", "wheel_d_pwm",
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
    previous_pose = link.last_pose
    link.reset_pose(Pose(0.0, 0.0, 0.0))
    # ACK precedes POSE. Cached valid/idle STATUS cannot confirm the reset.
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        pose = link.last_pose
        status = dict(link.last_status or {})
        if status.get("error") or status.get("estop") is True:
            raise RuntimeError(f"RESET_POSE failed: {diagnostics(status)}")
        if (
            pose is not None
            and pose is not previous_pose
            and (previous_pose is None or pose.timestamp_ms > previous_pose.timestamp_ms)
            and all(math.isfinite(value) and abs(value) <= 2.0
                    for value in (pose.x_mm, pose.y_mm, pose.yaw_deg))
            and status.get("pose_valid") is True
            and status.get("motion_active") is False
        ):
            return
        time.sleep(0.05)
    raise RuntimeError(
        f"RESET_POSE timed out waiting for fresh zero POSE: previous={previous_pose}, "
        f"latest={link.last_pose}, status={diagnostics(dict(link.last_status or {}))}"
    )


def upload_and_run(link: SerialRobotLink, payload: bytes, job_id: str):
    if not payload:
        raise RuntimeError(f"empty trajectory for {job_id}")
    reset_pose(link)
    receipt = link.upload(payload, job_id=job_id)
    link.run(job_id=receipt.job_id, crc32=receipt.crc32)
    # Discard any terminal STATUS from the preceding job after RUN has been
    # acknowledged.  A fresh job can fail before its brief STARTING state is
    # sampled, and that new ERROR must not be hidden by the run-start gate.
    link.last_status = None
    return receipt


def observe_until_finished(link: SerialRobotLink, job_id: str, *, timeout_s: float, description: str):
    deadline = time.monotonic() + timeout_s
    poses: list[Pose] = []
    seen_records: set[int] = set()
    seen_types: set[str] = set()
    seen_pen_states: set[str] = set()
    last_timestamp = -1.0
    finished_at: float | None = None
    while time.monotonic() < deadline:
        status = dict(link.last_status or {})
        pose = link.last_pose
        if pose is not None and pose.timestamp_ms > last_timestamp:
            poses.append(pose)
            last_timestamp = pose.timestamp_ms
        if status.get("current_job") == job_id:
            runner_state = status.get("runner")
            if runner_state == "ERROR":
                raise RuntimeError(f"{description} failed: {diagnostics(status)}")
            if runner_state in {"STARTING", "RUNNING", "WAITING"} or status.get("motion_active") is True:
                finished_at = None
            # upload_and_run clears last_status after RUN is acknowledged, and
            # current_job is unique for every case.  A matching FINISHED frame
            # is therefore fresh evidence even if periodic STATUS sampling
            # missed all short-lived STARTING/RUNNING states.
            if isinstance(status.get("record_index"), int):
                seen_records.add(status["record_index"])
            if isinstance(status.get("record_type"), str):
                seen_types.add(status["record_type"])
            if isinstance(status.get("pen_state"), str):
                seen_pen_states.add(status["pen_state"])
            if runner_state == "FINISHED" and status.get("motion_active") is False:
                if finished_at is None:
                    finished_at = time.monotonic()
                elif time.monotonic() - finished_at >= 0.30:
                    return status, poses, seen_records, seen_types, seen_pen_states
            if runner_state in {"STOPPED", "ESTOPPED"}:
                raise RuntimeError(f"{description} interrupted: {diagnostics(status)}")
        time.sleep(0.05)
    raise RuntimeError(f"timeout waiting for {description}: {diagnostics(dict(link.last_status or {}))}")


def assert_endpoint(pose: Pose | None, x_mm: float, y_mm: float, label: str) -> None:
    if pose is None:
        raise RuntimeError(f"{label} has no final POSE")
    if abs(pose.x_mm - x_mm) > 25.0 or abs(pose.y_mm - y_mm) > 25.0:
        raise RuntimeError(f"{label} endpoint outside 25 mm tolerance: {pose}")
    if abs(pose.yaw_deg) > 6.0:
        raise RuntimeError(f"{label} yaw drift exceeds 6 deg: {pose}")


def assert_arc_samples(poses: list[Pose], label: str) -> None:
    # Isolate the 150 mm arc from the preceding pen-up travel and following
    # 150 mm line.  The circle occupies x=[200,350], y=[0,150].
    samples = [
        pose for pose in poses
        if 190.0 <= pose.x_mm <= 360.0 and -10.0 <= pose.y_mm <= 165.0
    ]
    if len(samples) < 3:
        raise RuntimeError(f"{label} produced too few POSE samples: {len(samples)}")
    errors = [abs(math.hypot(p.x_mm - 200.0, p.y_mm - 150.0) - 150.0) for p in samples]
    max_error = max(errors)
    rms_error = math.sqrt(sum(error * error for error in errors) / len(errors))
    if max_error > 30.0 or rms_error > 18.0:
        raise RuntimeError(f"{label} radial error too large: max={max_error:.1f} rms={rms_error:.1f}")
    print(f"PASS {label} radial=max {max_error:.1f} mm rms {rms_error:.1f} mm")


def wait_for_pen_interruption_window(link: SerialRobotLink, job_id: str) -> dict:
    # This hobby servo has no position feedback. Wait by the configured action
    # timing, then interrupt inside the fixture's 5 s PEN_DOWN hold window.
    # STATUS pen_state is diagnostic command state and is not an acceptance
    # prerequisite.
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        status = dict(link.last_status or {})
        if status.get("current_job") == job_id:
            runner = status.get("runner")
            if runner == "ERROR":
                raise RuntimeError(f"PEN hold failed before interruption: {diagnostics(status)}")
            if runner in {"FINISHED", "STOPPED", "ESTOPPED"}:
                raise RuntimeError(f"PEN hold ended before interruption: {diagnostics(status)}")
        time.sleep(0.05)

    status = dict(link.last_status or {})
    if status.get("current_job") == job_id and status.get("runner") in {
        "ERROR", "FINISHED", "STOPPED", "ESTOPPED"
    }:
        raise RuntimeError(f"PEN hold ended before timed interruption: {diagnostics(status)}")
    print("INFO using guarded 2.0 s actuator window; servo position has no feedback")
    return status


def assert_terminal_pen_up(link: SerialRobotLink, status: dict, label: str) -> None:
    if status.get("pen") != "UP" or status.get("pen_state") != "UP" or status.get("pen_busy"):
        raise RuntimeError(f"{label} did not finish with a settled UP pen: {diagnostics(status)}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--pen-only-traj", required=True, type=argparse.FileType("rb"))
    parser.add_argument("--line-traj", required=True, type=argparse.FileType("rb"))
    parser.add_argument("--mixed-traj", required=True, type=argparse.FileType("rb"))
    parser.add_argument("--hold-down-traj", required=True, type=argparse.FileType("rb"))
    parser.add_argument("--cubic-traj", required=True, type=argparse.FileType("rb"))
    parser.add_argument("--skip-interruptions", action="store_true")
    args = parser.parse_args()

    payloads = {name: handle.read() for name, handle in {
        "pen_only": args.pen_only_traj,
        "line": args.line_traj,
        "mixed": args.mixed_traj,
        "hold_down": args.hold_down_traj,
        "cubic": args.cubic_traj,
    }.items()}
    link = SerialRobotLink(port=args.port, baudrate=115200, ack_timeout_s=3.0, heartbeat_interval_s=1.0)
    try:
        link.connect()
        hello = link.wait_ready(timeout_s=5.0)
        expected = {
            "build_profile": "motion-circle-pen",
            "execution_mode": "HARDWARE_DRAW",
            "hardware_enabled": True,
            "motion_enabled": True,
            "circle_enabled": True,
            "rotate_enabled": False,
            "pen_enabled": True,
            "simulation_enabled": False,
        }
        for key, value in expected.items():
            if hello.get(key) != value:
                raise RuntimeError(f"unexpected motion-circle-pen HELLO: {key}={hello.get(key)!r}")
        print("PASS READY motion-circle-pen profile")

        status = wait_for_status(
            link,
            lambda item: item.get("motor_ready") is True
            and item.get("odom_ready") is True
            and item.get("motion_ready") is True
            and item.get("hardware_initializing") is False
            and item.get("estop") is False
            and item.get("pen_error") in {None, "NONE"},
            timeout_s=25.0,
            description="motor, odometry, and physical pen ready",
        )
        print(f"PASS MOTOR+ODOM+PEN READY status={diagnostics(status)}")

        receipt = upload_and_run(link, payloads["pen_only"], "motion-pen-only")
        status, _, seen_records, seen_types, seen_pen_states = observe_until_finished(
            link, receipt.job_id, timeout_s=20.0, description="PEN_ONLY")
        assert_terminal_pen_up(link, status, "PEN_ONLY")
        # A hobby servo has no position feedback.  STATUS is asynchronous, so
        # a short PEN_DOWN state can fall entirely between two STATUS frames.
        # Completion without PEN error plus a final, settled UP command is the
        # executable safety criterion; sampled intermediate states are only
        # diagnostic information.
        print(
            "PASS PEN_ONLY finished with final UP "
            f"sampled_pen_states={sorted(seen_pen_states)} "
            f"status={diagnostics(status)}")

        receipt = upload_and_run(link, payloads["line"], "motion-pen-line")
        status, _, seen_records, seen_types, _ = observe_until_finished(
            link, receipt.job_id, timeout_s=45.0, description="PEN LINE")
        assert_endpoint(link.last_pose, 200.0, 200.0, "PEN LINE")
        assert_terminal_pen_up(link, status, "PEN LINE")
        if status.get("record_count") != 5:
            raise RuntimeError(f"PEN LINE record progress is wrong: {diagnostics(status)} records={seen_records}")
        print(f"PASS PEN LINE FINISHED pose={link.last_pose} status={diagnostics(status)}")

        receipt = upload_and_run(link, payloads["mixed"], "motion-pen-mixed")
        status, poses, seen_records, seen_types, _ = observe_until_finished(
            link, receipt.job_id, timeout_s=65.0, description="PEN LINE+CIRCLE+LINE")
        assert_endpoint(link.last_pose, 350.0, 300.0, "PEN LINE+CIRCLE+LINE")
        assert_arc_samples(poses, "PEN CIRCLE")
        assert_terminal_pen_up(link, status, "PEN LINE+CIRCLE+LINE")
        if status.get("record_count") != 6:
            raise RuntimeError(f"mixed record progress is wrong: {diagnostics(status)} records={seen_records} types={seen_types}")
        print(f"PASS PEN LINE+CIRCLE+LINE FINISHED pose={link.last_pose} status={diagnostics(status)}")

        if not args.skip_interruptions:
            receipt = upload_and_run(link, payloads["hold_down"], "motion-pen-stop")
            wait_for_pen_interruption_window(link, receipt.job_id)
            link.stop()
            status = wait_for_status(
                link,
                lambda item: item.get("runner") in {"STOPPED", "IDLE"}
                and item.get("motion_active") is False
                and item.get("pen_state") == "UP"
                and item.get("pen_busy") is False,
                timeout_s=8.0,
                description="STOP while pen down",
            )
            print(f"PASS STOP raises pen -> {diagnostics(status)}")

            receipt = upload_and_run(link, payloads["hold_down"], "motion-pen-estop")
            wait_for_pen_interruption_window(link, receipt.job_id)
            link.estop()
            status = wait_for_status(
                link,
                lambda item: item.get("estop") is True
                and item.get("motion_active") is False
                and item.get("pen_state") == "UP"
                and item.get("pen_busy") is False,
                timeout_s=8.0,
                description="ESTOP while pen down",
            )
            expect_error(lambda: link.run(job_id=receipt.job_id, crc32=receipt.crc32), "ESTOP_ACTIVE")
            link.clear_estop()
            wait_for_status(link, lambda item: item.get("estop") is False, timeout_s=5.0, description="CLEAR_ESTOP")
            print(f"PASS ESTOP/CLEAR_ESTOP raises pen -> {diagnostics(status)}")

        receipt = link.upload(payloads["cubic"], job_id="motion-pen-cubic-reject")
        expect_error(lambda: link.run(job_id=receipt.job_id, crc32=receipt.crc32), "UNSUPPORTED_RECORD")
        print("PASS CUBIC_BEZIER rejected with UNSUPPORTED_RECORD")

        print("N3 MOTION-CIRCLE-PEN ACCEPTANCE PASSED")
        return 0
    finally:
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
