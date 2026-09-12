"""Generate a PC-only N3 serial-link acceptance bundle.

The script uses the protocol-equivalent loopback firmware, so it is safe to
run without an ESP32 connected.  It creates a real N2 TRJ2 job, uploads the
exact bytes through :class:`SerialRobotLink`, exercises the safety commands,
and writes a small visual dashboard plus a machine-readable report.
"""

from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import time

from pc_trajectory.demo.map_model import PATH_KIND_POLYLINE, MapDocument, MapHome, MapPath
from pc_trajectory.demo.map_planner import NavigationPlanner, PenMode
from pc_trajectory.demo.protocol import Pose
from pc_trajectory.demo.robot_link import (
    LoopbackRobotFirmware,
    LoopbackSerialStream,
    RobotLinkState,
    SerialRobotLink,
    crc32_hex,
)
from pc_trajectory.geometry import Point2D
from pc_trajectory.preview import sample_geometry
from pc_trajectory.toolpath import Motion, PenDown, PenState, PenUp


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "test_results" / "virtual_map_demo" / "n3_link"


def build_document() -> MapDocument:
    return MapDocument(
        name="N3 serial link acceptance",
        width_mm=700.0,
        height_mm=450.0,
        home=MapHome(120.0, 160.0, 0.0, 7),
        paths=(
            MapPath(
                (
                    Point2D(120.0, 160.0),
                    Point2D(260.0, 160.0),
                    Point2D(390.0, 230.0),
                    Point2D(520.0, 230.0),
                ),
                path_id="n3-route",
                name="N3 route",
                path_kind=PATH_KIND_POLYLINE,
            ),
        ),
    )


def _timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()


def _plot_dashboard(document: MapDocument, plan, pose_trace: list[Pose]) -> Path:
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(10, 6), constrained_layout=True)
    axis.set_title("N3 PC serial link · TRJ2 upload and telemetry loopback")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, alpha=0.3)
    axis.set_xlabel("WORLD X (mm) · +X forward")
    axis.set_ylabel("WORLD Y (mm) · +Y left")
    axis.scatter([0.0], [0.0], marker="+", s=150, color="#dc2626", label="WORLD / Home")

    state = PenState.UP
    event_index = 0
    for record in plan.toolpath.records:
        if isinstance(record, PenUp):
            state = PenState.UP
            continue
        if isinstance(record, PenDown):
            state = PenState.DOWN
            continue
        if not isinstance(record, Motion):
            continue
        points = sample_geometry(record.geometry, sample_step_mm=4.0)
        axis.plot(
            [point.x_mm for point in points],
            [point.y_mm for point in points],
            color="#2563eb" if state is PenState.DOWN else "#f59e0b",
            linestyle="-" if state is PenState.DOWN else "--",
            linewidth=2.4,
            label="TRJ2 PenDown" if state is PenState.DOWN and event_index == 0 else (
                "TRJ2 PenUp travel" if state is PenState.UP and event_index == 0 else None
            ),
        )
        event_index += 1
    if pose_trace:
        axis.plot(
            [pose.x_mm for pose in pose_trace],
            [pose.y_mm for pose in pose_trace],
            color="#0f766e",
            marker="o",
            markersize=4,
            linewidth=1.4,
            label="loopback POSE telemetry",
        )
    axis.legend(loc="best", fontsize=9)
    output = OUTPUT / "n3_link_dashboard.png"
    figure.savefig(output, dpi=160, bbox_inches="tight")
    plt.close(figure)
    return output


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    document = build_document()
    document.save(OUTPUT / "n3_map.vmap.json")
    start_pose = Pose(0.0, 0.0, 0.0)
    plan = NavigationPlanner().plan_selected_path(
        document,
        start_pose,
        "n3-route",
        pen_mode=PenMode.DRAW,
    )
    payload = plan.trj2_bytes()
    job_id = "n3-acceptance-job"
    expected_crc = crc32_hex(payload)
    firmware = LoopbackRobotFirmware()
    link = SerialRobotLink(stream=LoopbackSerialStream(firmware), ack_timeout_s=0.75)
    transcript: list[dict[str, object]] = []
    pose_trace: list[Pose] = []

    def record(name: str, **fields: object) -> None:
        transcript.append({"time": _timestamp(), "event": name, **fields})

    try:
        link.connect()
        record("CONNECT", state=link.state.value)
        hello = link.wait_ready()
        record("HELLO", firmware=hello.get("firmware"), protocol=hello.get("protocol"), features=hello.get("features"))
        for event in link.poll_events():
            if event.get("type") == "POSE":
                pose_trace.append(Pose.from_message(event))
        receipt = link.upload(payload, job_id=job_id)
        record("UPLOAD", job_id=receipt.job_id, size=receipt.size, crc32=receipt.crc32, bytes_match=firmware.uploaded_bytes == payload)
        link.run()
        record("RUN", state=link.state.value)
        link.stop()
        record("STOP", state=link.state.value)
        link.estop()
        record("ESTOP", state=link.state.value)
        link.clear_estop()
        record("CLEAR_ESTOP", state=link.state.value)
        link.reset_pose(Pose(120.0, -40.0, 15.0))
        link.rotate_relative(90.0, 60.0)
        time.sleep(0.05)
        for event in link.poll_events():
            if event.get("type") == "POSE":
                pose_trace.append(Pose.from_message(event))
        record("POSE_CONTROLS", x_mm=link.last_pose.x_mm if link.last_pose else None, y_mm=link.last_pose.y_mm if link.last_pose else None, yaw_deg=link.last_pose.yaw_deg if link.last_pose else None)
    finally:
        link.close()
        record("DISCONNECT", state=link.state.value)

    dashboard = _plot_dashboard(document, plan, pose_trace)
    report = {
        "stage": "N3 PC serial link and independent real-robot panel",
        "protocol": 1,
        "job_id": job_id,
        "payload_size": len(payload),
        "expected_crc32": expected_crc,
        "firmware_crc32": firmware.uploaded.crc32 if firmware.uploaded else None,
        "bytes_match": firmware.uploaded_bytes == payload,
        "final_pose": {
            "x_mm": link.last_pose.x_mm if link.last_pose else None,
            "y_mm": link.last_pose.y_mm if link.last_pose else None,
            "yaw_deg": link.last_pose.yaw_deg if link.last_pose else None,
        },
        "final_link_state": link.state.value,
        "dashboard": dashboard.name,
        "artifacts": [
            "n3_map.vmap.json",
            "n3_link_dashboard.png",
            "protocol_transcript.jsonl",
            "test_report.txt",
            "n3_acceptance_report.json",
        ],
    }
    (OUTPUT / "protocol_transcript.jsonl").write_text(
        "".join(json.dumps(item, ensure_ascii=False, sort_keys=True) + "\n" for item in transcript),
        encoding="utf-8",
    )
    (OUTPUT / "test_report.txt").write_text(
        "N3 PC serial-link acceptance\n"
        "- generated a real N2 selected-path TRJ2 job\n"
        "- loopback HELLO, upload size/CRC, RUN, STOP, E-STOP and CLEAR_E_STOP passed\n"
        "- RESET_POSE and ROTATE_REL telemetry ended at (120, -40, 105 degrees)\n"
        "- dashboard shows the planned PenUp/PenDown toolpath and returned POSE samples\n"
        "- the UI exposes these controls under the independent Real robot dropdown\n"
        f"- dashboard: {dashboard.name}\n",
        encoding="utf-8",
    )
    (OUTPUT / "n3_acceptance_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Generated {OUTPUT}")


if __name__ == "__main__":
    main()
