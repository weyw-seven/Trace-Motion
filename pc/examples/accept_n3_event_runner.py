"""Acceptance test for the safe firmware's TRJ2 event-only runner.

The script performs four checks without requiring motors, odometry, tracker,
or a physical pen:

1. PEN/WAIT event trajectory reaches FINISHED.
2. STOP interrupts a long WAIT and reaches STOPPED.
3. ESTOP interrupts a long WAIT and reaches ESTOPPED; CLEAR_ESTOP returns IDLE.
4. A motion trajectory is rejected synchronously with MOTION_DISABLED.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import time

from pc_trajectory.demo.robot_link import RobotLinkError, SerialRobotLink


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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="ESP32 COM port, for example COM9")
    parser.add_argument("--complete-traj", required=True, type=Path)
    parser.add_argument("--long-wait-traj", required=True, type=Path)
    parser.add_argument("--motion-traj", required=True, type=Path)
    args = parser.parse_args()

    for path in (args.complete_traj, args.long_wait_traj, args.motion_traj):
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
        expected = {
            "build_profile": "safe",
            "hardware_enabled": False,
            "motion_enabled": False,
            "pen_enabled": False,
        }
        for name, value in expected.items():
            if hello.get(name) != value:
                raise RuntimeError(f"not a safe firmware build: {name}={hello.get(name)!r}")
        print("PASS READY safe event-only profile")

        complete = link.upload(
            args.complete_traj.read_bytes(),
            job_id="event-complete",
        )
        link.run(job_id=complete.job_id, crc32=complete.crc32)
        status = wait_for_status(
            link,
            lambda item: item.get("runner") == "FINISHED",
            timeout_s=5.0,
            description="FINISHED",
        )
        if status.get("execution_mode") != "EVENT_ONLY":
            raise RuntimeError(f"unexpected execution mode: {status}")
        if status.get("motor_ready") or status.get("odom_ready"):
            raise RuntimeError(f"safe runner reported hardware ready: {status}")
        print(f"PASS EVENT FINISHED records={status.get('record_count')}")

        long_wait = link.upload(
            args.long_wait_traj.read_bytes(),
            job_id="event-stop",
        )
        link.run(job_id=long_wait.job_id, crc32=long_wait.crc32)
        wait_for_status(
            link,
            lambda item: item.get("runner") == "RUNNING",
            timeout_s=2.0,
            description="RUNNING before STOP",
        )
        link.stop()
        status = wait_for_status(
            link,
            lambda item: item.get("runner") == "STOPPED"
            and item.get("stop_requested") is False,
            timeout_s=2.0,
            description="STOPPED",
        )
        if status.get("pen") != "UP":
            raise RuntimeError(f"STOP did not leave pen logical state UP: {status}")
        print("PASS STOP -> STOPPED")

        long_wait = link.upload(
            args.long_wait_traj.read_bytes(),
            job_id="event-estop",
        )
        link.run(job_id=long_wait.job_id, crc32=long_wait.crc32)
        wait_for_status(
            link,
            lambda item: item.get("runner") == "RUNNING",
            timeout_s=2.0,
            description="RUNNING before ESTOP",
        )
        link.estop()
        status = wait_for_status(
            link,
            lambda item: item.get("runner") == "ESTOPPED"
            and item.get("estop") is True
            and item.get("stop_requested") is False,
            timeout_s=2.0,
            description="ESTOPPED",
        )
        print("PASS ESTOP -> ESTOPPED")
        link.clear_estop()
        wait_for_status(
            link,
            lambda item: item.get("runner") == "IDLE" and item.get("estop") is False,
            timeout_s=2.0,
            description="IDLE after CLEAR_ESTOP",
        )
        print("PASS CLEAR_ESTOP -> IDLE")

        motion = link.upload(
            args.motion_traj.read_bytes(),
            job_id="event-motion-rejected",
        )
        expect_error(
            lambda: link.run(job_id=motion.job_id, crc32=motion.crc32),
            "MOTION_DISABLED",
        )
        print("PASS motion trajectory rejected with MOTION_DISABLED")

        print("N3 EVENT RUNNER ACCEPTANCE PASSED")
        return 0
    finally:
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
