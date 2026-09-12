from __future__ import annotations

import time
import queue

import pytest

from pc_trajectory.demo.protocol import Pose
from pc_trajectory.demo.robot_link import (
    LoopbackRobotFirmware,
    LoopbackSerialStream,
    RobotLinkError,
    RobotLinkState,
    SerialRobotLink,
    TcpLoopbackRobotServer,
    TcpRobotLink,
    crc32_hex,
)


class _BootLogStream:
    """Serial-like stream that emits an ESP-IDF boot log but no N3 HELLO."""

    def __init__(self) -> None:
        self.is_open = False
        self._lines: queue.Queue[bytes] = queue.Queue()

    def open(self) -> None:
        self.is_open = True
        self._lines.put(b"I (123) motion_smooth_test: boot complete\\r\\n")

    def close(self) -> None:
        self.is_open = False

    def write(self, payload: bytes) -> int:
        return len(payload)

    def flush(self) -> None:
        return None

    def readline(self) -> bytes:
        if not self.is_open:
            return b""
        try:
            return self._lines.get(timeout=0.02)
        except queue.Empty:
            return b""


class _DelayedUploadReadyStream(LoopbackSerialStream):
    """Give the heartbeat a deterministic chance to contend with an upload."""

    def __init__(self) -> None:
        super().__init__()
        self._delayed = False

    def readline(self) -> bytes:
        line = super().readline()
        if not self._delayed and b"UPLOAD_READY" in line:
            self._delayed = True
            time.sleep(0.05)
        return line


def _wait_for(link: SerialRobotLink, predicate, timeout_s: float = 1.0) -> list[dict[str, object]]:
    deadline = time.monotonic() + timeout_s
    events: list[dict[str, object]] = []
    while time.monotonic() < deadline:
        events.extend(link.poll_events())
        if predicate(events):
            return events
        time.sleep(0.01)
    events.extend(link.poll_events())
    return events


def test_crc32_is_canonical_uppercase() -> None:
    assert crc32_hex(b"123456789") == "CBF43926"


def test_boot_logs_are_diagnostic_lines_until_n3_hello() -> None:
    link = SerialRobotLink(stream=_BootLogStream(), heartbeat_interval_s=0.02)
    try:
        link.connect()
        events = _wait_for(link, lambda values: any(value.get("type") == "LOG" for value in values), timeout_s=0.3)
        assert any(value.get("type") == "LOG" for value in events)
        assert not any(value.get("type") == "ERROR" for value in events)
        assert link.state is RobotLinkState.CONNECTING
    finally:
        link.close()


def test_loopback_upload_run_safety_and_pose_controls() -> None:
    firmware = LoopbackRobotFirmware()
    stream = LoopbackSerialStream(firmware)
    link = SerialRobotLink(stream=stream, ack_timeout_s=0.5)
    try:
        link.connect()
        hello = link.wait_ready()
        assert hello["firmware"] == "loopback-n3"
        assert link.state is RobotLinkState.READY

        payload = b"TRJ2\x00acceptance-payload"
        receipt = link.upload(payload, job_id="acceptance-1")
        assert receipt.size == len(payload)
        assert receipt.crc32 == crc32_hex(payload)
        assert firmware.uploaded_bytes == payload
        assert firmware.uploaded == receipt

        link.run()
        assert link.state is RobotLinkState.RUNNING
        link.estop()
        assert link.state is RobotLinkState.ESTOPPED
        with pytest.raises(RobotLinkError, match="ESTOP_ACTIVE"):
            link.run()
        link.clear_estop()
        assert link.state is RobotLinkState.READY

        link.reset_pose(Pose(120.0, -40.0, 15.0))
        link.rotate_relative(90.0, 60.0)
        events = _wait_for(link, lambda values: any(value.get("type") == "POSE" for value in values))
        assert link.last_pose is not None
        assert link.last_pose.x_mm == pytest.approx(120.0)
        assert link.last_pose.y_mm == pytest.approx(-40.0)
        assert link.last_pose.yaw_deg == pytest.approx(105.0)
        assert any(value.get("type") == "STATUS" for value in events)
    finally:
        link.close()
    assert link.state is RobotLinkState.DISCONNECTED


def test_loopback_heartbeat_emits_pong() -> None:
    link = SerialRobotLink(
        stream=LoopbackSerialStream(),
        ack_timeout_s=0.5,
        heartbeat_interval_s=0.02,
    )
    try:
        link.connect()
        link.wait_ready()
        events = _wait_for(link, lambda values: any(value.get("type") == "PONG" for value in values), timeout_s=0.5)
        assert any(value.get("type") == "PONG" for value in events)
    finally:
        link.close()


def test_heartbeat_cannot_interleave_with_binary_upload_transaction() -> None:
    stream = _DelayedUploadReadyStream()
    link = SerialRobotLink(
        stream=stream,
        ack_timeout_s=0.3,
        heartbeat_interval_s=0.005,
    )
    payload = b"TRJ2" + bytes(range(256)) * 2
    try:
        link.connect()
        link.wait_ready()
        receipt = link.upload(payload, job_id="atomic-upload")
        assert receipt.crc32 == crc32_hex(payload)
        assert stream.firmware.uploaded_bytes == payload
        assert link.state is RobotLinkState.READY
    finally:
        link.close()


def test_loopback_duplicate_command_id_is_idempotent() -> None:
    firmware = LoopbackRobotFirmware()
    link = SerialRobotLink(stream=LoopbackSerialStream(firmware), ack_timeout_s=0.5)
    command = {"type": "ROTATE_REL", "id": 9001, "angle_deg": 90.0, "speed_deg_s": 60.0}
    try:
        link.connect()
        link.wait_ready()
        first = link._request(command)
        second = link._request(command)
        assert first == second
        assert firmware.pose.yaw_deg == pytest.approx(90.0)
    finally:
        link.close()


def test_run_requires_an_uploaded_job() -> None:
    link = SerialRobotLink(stream=LoopbackSerialStream(), ack_timeout_s=0.5)
    try:
        link.connect()
        link.wait_ready()
        with pytest.raises(RobotLinkError, match="upload a trajectory"):
            link.run()
    finally:
        link.close()


def test_loopback_rejects_corrupt_upload_crc() -> None:
    stream = LoopbackSerialStream()
    link = SerialRobotLink(stream=stream, ack_timeout_s=0.5)
    try:
        link.connect()
        link.wait_ready()
        link._request({
            "type": "UPLOAD_BEGIN",
            "id": 9100,
            "job_id": "bad-crc",
            "size": 3,
            "crc32": crc32_hex(b"abc"),
        })
        stream.write(b"xyz")
        with pytest.raises(RobotLinkError, match="CRC_MISMATCH"):
            link._request({
                "type": "UPLOAD_END",
                "id": 9101,
                "job_id": "bad-crc",
                "size": 3,
                "crc32": crc32_hex(b"abc"),
            })
    finally:
        link.close()


def test_tcp_link_reuses_n3_protocol_and_uploads_exact_bytes() -> None:
    server = TcpLoopbackRobotServer()
    server.start()
    link = TcpRobotLink(
        "127.0.0.1",
        server.port,
        ack_timeout_s=0.5,
        heartbeat_interval_s=0.05,
    )
    try:
        link.connect()
        hello = link.wait_ready()
        assert hello["firmware"] == "loopback-n3"
        payload = b"TRJ2-over-tcp"
        receipt = link.upload(payload, job_id="tcp-1")
        assert receipt.crc32 == crc32_hex(payload)
        assert server.firmware.uploaded_bytes == payload
        link.run()
        assert link.state is RobotLinkState.RUNNING
        link.estop()
        assert link.state is RobotLinkState.ESTOPPED
    finally:
        link.close()
        server.stop()


def test_tcp_peer_disconnect_moves_link_to_disconnected() -> None:
    server = TcpLoopbackRobotServer()
    server.start()
    link = TcpRobotLink("127.0.0.1", server.port, ack_timeout_s=0.5, heartbeat_interval_s=0.05)
    try:
        link.connect()
        link.wait_ready()
        server.stop()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and link.state is not RobotLinkState.DISCONNECTED:
            time.sleep(0.01)
        assert link.state is RobotLinkState.DISCONNECTED
    finally:
        link.close()
