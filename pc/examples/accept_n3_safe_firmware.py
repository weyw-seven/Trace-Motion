r"""Acceptance test for the ESP32 N3 safe firmware.

This test verifies communication, upload, CRC/decoder acceptance, pose reset,
and safety commands.  It also verifies that the safe event-only runner accepts
an event trajectory while rejecting a motion trajectory with MOTION_DISABLED.
No motor, odometry, tracker, or physical pen driver is enabled by this build.

Example:
    .\.conda\python.exe -s examples/accept_n3_safe_firmware.py \\
        --port COM7 \\
        --traj D:\\esp-projects\\test-motor\\spiffs\\n3_event_done.traj \\
        --motion-traj D:\\esp-projects\\test-motor\\spiffs\\trj2_smooth_small.traj
"""

from __future__ import annotations

import argparse
from pathlib import Path
import time

from pc_trajectory.demo.robot_link import (
    RobotLinkError,
    SerialRobotLink,
    crc32_hex,
)
from pc_trajectory.demo.protocol import Pose


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="ESP32 COM port, for example COM7")
    parser.add_argument("--traj", required=True, type=Path, help="A valid event-only TRJ2 .traj file")
    parser.add_argument(
        "--motion-traj",
        required=True,
        type=Path,
        help="A valid TRJ2 file containing LINE/CIRCLE/CUBIC motion",
    )
    parser.add_argument("--job-id", default="", help="Upload job id; defaults to a unique timestamp id")
    args = parser.parse_args()

    payload = args.traj.read_bytes()
    if not payload:
        raise SystemExit("trajectory file is empty")

    job_id = args.job_id.strip() or f"safe-acceptance-{int(time.time() * 1000)}"

    link = SerialRobotLink(
        port=args.port,
        baudrate=115200,
        ack_timeout_s=3.0,
        heartbeat_interval_s=10.0,
    )
    try:
        link.connect()
        hello = link.wait_ready(timeout_s=5.0)
        features = set(hello.get("features", []))
        if "upload" not in features:
            raise RuntimeError(f"firmware does not advertise upload: {hello}")
        expected_safe = {
            "build_profile": "safe",
            "hardware_enabled": False,
            "motion_enabled": False,
            "pen_enabled": False,
        }
        for name, expected in expected_safe.items():
            if hello.get(name) != expected:
                raise RuntimeError(
                    f"firmware is not the safe build: {name}="
                    f"{hello.get(name)!r}, expected {expected!r}; hello={hello}"
                )
        print(f"PASS READY firmware={hello.get('firmware')} features={sorted(features)}")
        print("PASS SAFE PROFILE hardware=0 motion=0 pen=0")

        link.ping()
        print("PASS PING/PONG")

        receipt = link.upload(payload, job_id=job_id)
        if receipt.crc32 != crc32_hex(payload):
            raise RuntimeError("PC CRC calculation changed during upload")
        print(f"PASS UPLOAD size={receipt.size} crc32={receipt.crc32}")

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            status = link.last_status or {}
            if status.get("current_job") == job_id:
                break
            time.sleep(0.05)
        else:
            raise RuntimeError(f"STATUS did not confirm current_job={job_id!r}: {link.last_status}")
        print(f"PASS STATUS current_job={job_id}")

        link.run(job_id=receipt.job_id, crc32=receipt.crc32)
        print("PASS EVENT RUN accepted (event-only runner)")

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if (link.last_status or {}).get("runner") == "FINISHED":
                break
            time.sleep(0.05)
        else:
            raise RuntimeError(f"event runner did not finish: {link.last_status}")
        print("PASS EVENT RUN finished with no hardware drivers")

        motion_payload = args.motion_traj.read_bytes()
        if not motion_payload:
            raise SystemExit("motion trajectory file is empty")
        motion_job_id = f"{job_id}-motion"
        motion_receipt = link.upload(motion_payload, job_id=motion_job_id)
        print(f"PASS MOTION FIXTURE UPLOAD size={motion_receipt.size} crc32={motion_receipt.crc32}")
        try:
            link.run(job_id=motion_receipt.job_id, crc32=motion_receipt.crc32)
        except RobotLinkError as exc:
            if "MOTION_DISABLED" not in str(exc):
                raise
            print("PASS MOTION RUN blocked with MOTION_DISABLED (safe firmware)")
        else:
            raise RuntimeError("safe firmware unexpectedly accepted a motion trajectory")

        try:
            link.rotate_relative(angle_deg=10.0, speed_deg_s=45.0)
        except RobotLinkError as exc:
            if "MOTOR_NOT_READY" not in str(exc):
                raise
            print("PASS ROTATE_REL blocked with MOTOR_NOT_READY (safe firmware)")
        else:
            raise RuntimeError("safe firmware unexpectedly accepted ROTATE_REL")

        status = link.last_status or {}
        for name, expected in {
            "hardware_enabled": False,
            "motion_enabled": False,
            "pen_enabled": False,
        }.items():
            if status.get(name) != expected:
                raise RuntimeError(
                    f"STATUS lost safe profile field {name}: "
                    f"{status.get(name)!r}, expected {expected!r}; status={status}"
                )

        link.reset_pose(Pose(120.0, -40.0, 15.0))
        link.stop()
        link.estop()
        link.clear_estop()
        print("PASS RESET_POSE/STOP/ESTOP/CLEAR_ESTOP")
        print("N3 SAFE FIRMWARE ACCEPTANCE PASSED; no motion command was accepted")
        return 0
    finally:
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
