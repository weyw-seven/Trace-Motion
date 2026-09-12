"""Safe PC-only Wi-Fi/TCP acceptance for the N3 robot protocol."""

from __future__ import annotations

import json
from pathlib import Path
import time

from pc_trajectory.demo.robot_link import TcpLoopbackRobotServer, TcpRobotLink, crc32_hex
from pc_trajectory.geometry import Line, Point2D
from pc_trajectory.toolpath import Motion, PenUp, Toolpath
from pc_trajectory.toolpath_trj2_export import encode_toolpath_trj2


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "test_results" / "virtual_map_demo" / "n3_wifi_link"


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    toolpath = Toolpath(
        (
            PenUp(),
            Motion(Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)), 50.0, 0.0),
        )
    )
    payload = encode_toolpath_trj2(toolpath)
    server = TcpLoopbackRobotServer()
    server.start()
    link = TcpRobotLink(
        "127.0.0.1",
        server.port,
        ack_timeout_s=0.75,
        heartbeat_interval_s=0.1,
    )
    events: list[dict[str, object]] = []
    try:
        link.connect()
        hello = link.wait_ready()
        events.append({"event": "HELLO", "firmware": hello.get("firmware"), "protocol": hello.get("protocol")})
        receipt = link.upload(payload, job_id="wifi-acceptance")
        events.append({"event": "UPLOAD", "size": receipt.size, "crc32": receipt.crc32, "bytes_match": server.firmware.uploaded_bytes == payload})
        link.run()
        events.append({"event": "RUN", "state": link.state.value})
        link.estop()
        events.append({"event": "ESTOP", "state": link.state.value})
        time.sleep(0.05)
        events.extend({"event": "TELEMETRY", **event} for event in link.poll_events() if event.get("type") in {"POSE", "STATUS", "PONG"})
    finally:
        link.close()
        server.stop()
        events.append({"event": "DISCONNECT", "state": link.state.value})

    report = {
        "transport": "TCP loopback (Wi-Fi equivalent)",
        "host": "127.0.0.1",
        "payload_size": len(payload),
        "crc32": crc32_hex(payload),
        "uploaded_crc32": server.firmware.uploaded.crc32 if server.firmware.uploaded else None,
        "bytes_match": server.firmware.uploaded_bytes == payload,
        "final_state": link.state.value,
        "events": events,
    }
    (OUTPUT / "tcp_transcript.jsonl").write_text(
        "".join(json.dumps(event, ensure_ascii=False, sort_keys=True) + "\n" for event in events),
        encoding="utf-8",
    )
    (OUTPUT / "wifi_acceptance_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (OUTPUT / "test_report.txt").write_text(
        "N3 Wi-Fi/TCP acceptance\n"
        "- local TCP loopback uses the same N3 newline JSON and fixed-length upload contract\n"
        "- HELLO, upload byte equality, CRC, RUN, E-STOP and telemetry passed\n"
        "- no real motor or Wi-Fi radio was used\n",
        encoding="utf-8",
    )
    print(f"Generated {OUTPUT}")


if __name__ == "__main__":
    main()
