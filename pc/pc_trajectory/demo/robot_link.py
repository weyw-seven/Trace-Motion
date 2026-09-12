"""PC robot links for N3 serial communication and protocol-loopback tests.

The navigation planner and simulator do not know whether a task is going to a
real ESP32 or a deterministic in-memory firmware model.  Both links expose the
same small command surface; the serial implementation keeps all reads on a
background thread and reports validated JSON messages through ``poll_events``.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import queue
import socket
import threading
import time
from typing import Any, Mapping
import zlib

from .protocol import MessageType, Pose, ProtocolError, decode_message, encode_message


class RobotLinkError(RuntimeError):
    """Raised when a robot link cannot complete a command safely."""


class RobotLinkState(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    CONNECTING = "CONNECTING"
    READY = "READY"
    RUNNING = "RUNNING"
    ESTOPPED = "ESTOPPED"
    ERROR = "ERROR"


@dataclass(frozen=True)
class UploadReceipt:
    job_id: str
    size: int
    crc32: str


def crc32_hex(payload: bytes) -> str:
    """Return the protocol's canonical uppercase eight-digit CRC32."""

    return f"{zlib.crc32(payload) & 0xFFFFFFFF:08X}"


def list_serial_ports() -> tuple[str, ...]:
    """List available serial ports without making pyserial mandatory."""

    try:
        from serial.tools import list_ports  # type: ignore[import-not-found]
    except ImportError:
        return ()
    return tuple(sorted(port.device for port in list_ports.comports()))


class SerialRobotLink:
    """Threaded newline-JSON/length-delimited-binary serial link."""

    def __init__(
        self,
        port: str | None = None,
        *,
        baudrate: int = 115200,
        read_timeout_s: float = 0.1,
        ack_timeout_s: float = 1.5,
        heartbeat_interval_s: float = 2.0,
        stream: Any | None = None,
    ) -> None:
        if not isinstance(baudrate, int) or baudrate <= 0:
            raise ValueError("baudrate must be a positive integer")
        if read_timeout_s <= 0.0 or ack_timeout_s <= 0.0 or heartbeat_interval_s <= 0.0:
            raise ValueError("serial timeouts must be positive")
        self.port = port
        self.baudrate = baudrate
        self.read_timeout_s = float(read_timeout_s)
        self.ack_timeout_s = float(ack_timeout_s)
        self.heartbeat_interval_s = float(heartbeat_interval_s)
        self._stream = stream
        self._owns_stream = stream is None
        self._state = RobotLinkState.DISCONNECTED
        self._state_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._reader: threading.Thread | None = None
        self._heartbeat: threading.Thread | None = None
        self._events: queue.Queue[dict[str, Any]] = queue.Queue()
        self._pending: dict[int, queue.Queue[dict[str, Any]]] = {}
        self._pending_lock = threading.Lock()
        # N3 uploads temporarily switch the peer from newline-delimited JSON
        # to a fixed-length binary receive mode.  A re-entrant lock lets
        # ``upload`` hold the transport for that entire transaction while the
        # nested request/write helpers continue to use the same lock.
        self._write_lock = threading.RLock()
        self._id_lock = threading.Lock()
        self._next_id = 1
        self._last_upload: UploadReceipt | None = None
        self.hello: dict[str, Any] | None = None
        self.last_pose: Pose | None = None
        self.last_status: dict[str, Any] | None = None

    @property
    def state(self) -> RobotLinkState:
        with self._state_lock:
            return self._state

    @property
    def connected(self) -> bool:
        return self.state is not RobotLinkState.DISCONNECTED

    def _set_state(self, state: RobotLinkState) -> None:
        with self._state_lock:
            self._state = state

    def _allocate_id(self) -> int:
        with self._id_lock:
            value = self._next_id
            self._next_id += 1
            return value

    def connect(self) -> None:
        if self.connected:
            return
        if self._stream is None:
            if not self.port:
                raise RobotLinkError("select a serial port before connecting")
            try:
                import serial  # type: ignore[import-not-found]
            except ImportError as exc:
                raise RobotLinkError("pyserial is required for a real serial connection") from exc
            try:
                self._stream = serial.Serial(self.port, self.baudrate, timeout=self.read_timeout_s)
            except Exception as exc:  # pragma: no cover - depends on host hardware
                self._set_state(RobotLinkState.ERROR)
                raise RobotLinkError(f"cannot open serial port {self.port!r}: {exc}") from exc
        try:
            open_method = getattr(self._stream, "open", None)
            if callable(open_method) and not bool(getattr(self._stream, "is_open", True)):
                open_method()
        except Exception as exc:
            self._set_state(RobotLinkState.ERROR)
            raise RobotLinkError(f"cannot open robot stream: {exc}") from exc
        self._stop_event.clear()
        self._set_state(RobotLinkState.CONNECTING)
        self._reader = threading.Thread(target=self._read_loop, name="robot-link-reader", daemon=True)
        self._reader.start()
        self._heartbeat = threading.Thread(target=self._heartbeat_loop, name="robot-link-heartbeat", daemon=True)
        self._heartbeat.start()

    def close(self) -> None:
        self._stop_event.set()
        stream = self._stream
        if stream is not None:
            try:
                stream.close()
            except Exception:
                pass
        reader = self._reader
        if reader is not None and reader is not threading.current_thread():
            reader.join(timeout=max(0.2, self.read_timeout_s * 3.0))
        heartbeat = self._heartbeat
        if heartbeat is not None and heartbeat is not threading.current_thread():
            heartbeat.join(timeout=max(0.2, self.ack_timeout_s + 0.2))
        self._reader = None
        self._heartbeat = None
        self._stream = None if self._owns_stream else stream
        self._set_state(RobotLinkState.DISCONNECTED)
        with self._pending_lock:
            pending = tuple(self._pending.values())
            self._pending.clear()
        for waiter in pending:
            try:
                waiter.put_nowait({"type": MessageType.ERROR.value, "id": -1, "code": "DISCONNECTED", "message": "robot link closed"})
            except queue.Full:
                pass

    def _read_loop(self) -> None:
        assert self._stream is not None
        while not self._stop_event.is_set():
            try:
                raw = self._stream.readline()
            except Exception as exc:
                if not self._stop_event.is_set():
                    self._set_state(RobotLinkState.DISCONNECTED)
                    failure = {
                        "type": MessageType.ERROR.value,
                        "id": -1,
                        "code": "DISCONNECTED",
                        "message": str(exc),
                    }
                    self._events.put(failure)
                    with self._pending_lock:
                        pending = tuple(self._pending.values())
                    for waiter in pending:
                        waiter.put(failure)
                return
            if not raw:
                continue
            # ESP-IDF ROM/application logs can appear on the same USB/UART
            # stream during boot.  They are diagnostics, not N3 messages;
            # keep them visible to the UI without poisoning the protocol
            # state machine.  Once a line looks like JSON, malformed JSON is
            # still reported as a real protocol error.
            stripped = raw.strip()
            if not stripped:
                continue
            if not stripped.startswith(b"{"):
                self._events.put({
                    "type": "LOG",
                    "message": stripped.decode("utf-8", errors="replace"),
                })
                continue
            try:
                message = decode_message(raw)
            except ProtocolError as exc:
                self._events.put({"type": MessageType.ERROR.value, "id": -1, "code": "PROTOCOL", "message": str(exc)})
                continue
            message_type = message.get("type")
            if message_type == MessageType.HELLO.value:
                self.hello = message
                self._set_state(RobotLinkState.READY)
            elif message_type == MessageType.POSE.value:
                try:
                    self.last_pose = Pose.from_message(message)
                except ProtocolError:
                    pass
            elif message_type == MessageType.STATUS.value:
                self.last_status = message
                if message.get("estop"):
                    self._set_state(RobotLinkState.ESTOPPED)
                elif message.get("runner") == "RUNNING":
                    self._set_state(RobotLinkState.RUNNING)
                elif self.state is not RobotLinkState.ERROR:
                    self._set_state(RobotLinkState.READY)
            if message_type in {MessageType.ACK.value, MessageType.ERROR.value}:
                command_id = message.get("id")
                with self._pending_lock:
                    waiter = self._pending.get(command_id)
                if waiter is not None:
                    waiter.put(message)
                    continue
            self._events.put(message)

    def _heartbeat_loop(self) -> None:
        """Send periodic PINGs so cable loss is visible without a UI action."""

        while not self._stop_event.wait(self.heartbeat_interval_s):
            # Do not send commands before the peer has identified itself with
            # HELLO.  This keeps a legacy/test firmware quiet while the UI
            # reports the actual compatibility problem as "waiting for HELLO".
            if self.state not in {RobotLinkState.READY, RobotLinkState.RUNNING, RobotLinkState.ESTOPPED}:
                continue
            try:
                self.ping()
            except RobotLinkError as exc:
                if not self._stop_event.is_set():
                    self._set_state(RobotLinkState.ERROR)
                    self._events.put({
                        "type": MessageType.ERROR.value,
                        "id": -1,
                        "code": "HEARTBEAT",
                        "message": str(exc),
                    })

    def poll_events(self) -> list[dict[str, Any]]:
        events: list[dict[str, Any]] = []
        while True:
            try:
                events.append(self._events.get_nowait())
            except queue.Empty:
                return events

    def wait_ready(self, timeout_s: float = 2.0) -> dict[str, Any]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.state is RobotLinkState.READY and self.hello is not None:
                return dict(self.hello)
            time.sleep(0.01)
        raise RobotLinkError("timed out waiting for ESP32 HELLO")

    def _write(self, payload: bytes) -> None:
        if not self.connected or self._stream is None:
            raise RobotLinkError("robot link is disconnected")
        try:
            with self._write_lock:
                self._stream.write(payload)
                flush = getattr(self._stream, "flush", None)
                if callable(flush):
                    flush()
        except Exception as exc:
            self._set_state(RobotLinkState.ERROR)
            raise RobotLinkError(f"serial write failed: {exc}") from exc

    def _request(self, message: Mapping[str, Any], *, timeout_s: float | None = None) -> dict[str, Any]:
        command = dict(message)
        command_id = int(command["id"]) if "id" in command else self._allocate_id()
        command["id"] = command_id
        waiter: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=1)
        with self._pending_lock:
            self._pending[command_id] = waiter
        try:
            self._write(encode_message(command))
            response = waiter.get(timeout=self.ack_timeout_s if timeout_s is None else timeout_s)
        except queue.Empty as exc:
            raise RobotLinkError(f"timeout waiting for ACK id={command_id}") from exc
        finally:
            with self._pending_lock:
                self._pending.pop(command_id, None)
        if response.get("type") == MessageType.ERROR.value:
            raise RobotLinkError(f"ESP32 rejected id={command_id}: {response.get('code', 'ERROR')} {response.get('message', '')}")
        return response

    def ping(self) -> dict[str, Any]:
        return self._request({"type": MessageType.PING.value, "nonce": f"pc-{time.monotonic_ns()}"})

    def upload(self, payload: bytes, *, job_id: str = "") -> UploadReceipt:
        data = bytes(payload)
        if not data:
            raise RobotLinkError("cannot upload an empty trajectory")
        job = job_id.strip() or f"job-{int(time.time() * 1000)}"
        receipt = UploadReceipt(job, len(data), crc32_hex(data))
        # Keep heartbeat PINGs and UI commands out of the binary window.  Once
        # UPLOAD_BEGIN is accepted, every following byte is trajectory data
        # until the announced length has arrived; an interleaved JSON command
        # would otherwise be consumed as trajectory bytes.
        with self._write_lock:
            self._request({
                "type": MessageType.UPLOAD_BEGIN.value,
                "job_id": receipt.job_id,
                "size": receipt.size,
                "crc32": receipt.crc32,
            })
            self._write(data)
            self._request({
                "type": MessageType.UPLOAD_END.value,
                "job_id": receipt.job_id,
                "size": receipt.size,
                "crc32": receipt.crc32,
            })
        self._last_upload = receipt
        return receipt

    def run(self, *, job_id: str | None = None, crc32: str | None = None, reset_odometry: bool = False) -> dict[str, Any]:
        receipt = self._last_upload
        selected_job = job_id or (receipt.job_id if receipt else "")
        selected_crc = crc32 or (receipt.crc32 if receipt else "")
        if not selected_job or not selected_crc:
            raise RobotLinkError("upload a trajectory before RUN_TRAJECTORY")
        response = self._request({
            "type": MessageType.RUN_TRAJECTORY.value,
            "job_id": selected_job,
            "crc32": selected_crc,
            "reset_odometry": bool(reset_odometry),
        })
        self._set_state(RobotLinkState.RUNNING)
        return response

    def stop(self) -> dict[str, Any]:
        response = self._request({"type": MessageType.STOP.value})
        if self.state is not RobotLinkState.ESTOPPED:
            self._set_state(RobotLinkState.READY)
        return response

    def estop(self) -> dict[str, Any]:
        response = self._request({"type": MessageType.ESTOP.value})
        self._set_state(RobotLinkState.ESTOPPED)
        return response

    def clear_estop(self) -> dict[str, Any]:
        response = self._request({"type": MessageType.CLEAR_ESTOP.value})
        self._set_state(RobotLinkState.READY)
        return response

    def reset_pose(self, pose: Pose) -> dict[str, Any]:
        return self._request({
            "type": MessageType.RESET_POSE.value,
            "x_mm": pose.x_mm,
            "y_mm": pose.y_mm,
            "yaw_deg": pose.yaw_deg,
        })

    def rotate_relative(self, angle_deg: float, speed_deg_s: float = 45.0) -> dict[str, Any]:
        return self._request({
            "type": MessageType.ROTATE_REL.value,
            "angle_deg": angle_deg,
            "speed_deg_s": speed_deg_s,
        })


class TcpSocketStream:
    """Small serial-like adapter for the newline JSON/TCP transport.

    TCP is a byte stream, so ``readline`` keeps a receive buffer and returns
    exactly one newline-delimited protocol message at a time.  Raw trajectory
    bytes are written by the PC after ``UPLOAD_BEGIN``; the ESP32 side uses
    the announced byte count to switch its parser to binary mode.
    """

    def __init__(
        self,
        host: str,
        port: int,
        *,
        connect_timeout_s: float = 3.0,
        read_timeout_s: float = 0.1,
    ) -> None:
        host = str(host).strip()
        if not host:
            raise ValueError("TCP host must be non-empty")
        if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
            raise ValueError("TCP port must be an integer in [1, 65535]")
        if connect_timeout_s <= 0.0 or read_timeout_s <= 0.0:
            raise ValueError("TCP timeouts must be positive")
        self.host = host
        self.port = port
        self.connect_timeout_s = float(connect_timeout_s)
        self.read_timeout_s = float(read_timeout_s)
        self.socket: socket.socket | None = None
        self.is_open = False
        self._receive_buffer = bytearray()
        self._write_lock = threading.Lock()

    def open(self) -> None:
        if self.is_open:
            return
        try:
            sock = socket.create_connection((self.host, self.port), timeout=self.connect_timeout_s)
            sock.settimeout(self.read_timeout_s)
        except OSError:
            self.socket = None
            self.is_open = False
            raise
        self.socket = sock
        self.is_open = True
        self._receive_buffer.clear()

    def close(self) -> None:
        self.is_open = False
        sock = self.socket
        self.socket = None
        self._receive_buffer.clear()
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass

    def write(self, payload: bytes) -> int:
        sock = self.socket
        if not self.is_open or sock is None:
            raise OSError("TCP stream is closed")
        data = bytes(payload)
        with self._write_lock:
            sock.sendall(data)
        return len(data)

    def flush(self) -> None:
        return None

    def readline(self) -> bytes:
        sock = self.socket
        if not self.is_open or sock is None:
            return b""
        while True:
            newline = self._receive_buffer.find(b"\n")
            if newline >= 0:
                line = bytes(self._receive_buffer[:newline])
                del self._receive_buffer[: newline + 1]
                return line + b"\n"
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                return b""
            if not chunk:
                raise OSError("TCP peer closed the connection")
            self._receive_buffer.extend(chunk)


class TcpRobotLink(SerialRobotLink):
    """N3 robot link over Wi-Fi TCP using the same command contract as USB."""

    def __init__(
        self,
        host: str,
        port: int = 5000,
        *,
        connect_timeout_s: float = 3.0,
        read_timeout_s: float = 0.1,
        ack_timeout_s: float = 1.5,
        heartbeat_interval_s: float = 2.0,
    ) -> None:
        self.host = str(host).strip()
        self.tcp_port = int(port)
        stream = TcpSocketStream(
            self.host,
            self.tcp_port,
            connect_timeout_s=connect_timeout_s,
            read_timeout_s=read_timeout_s,
        )
        super().__init__(
            port=None,
            read_timeout_s=read_timeout_s,
            ack_timeout_s=ack_timeout_s,
            heartbeat_interval_s=heartbeat_interval_s,
            stream=stream,
        )


class _TcpLoopbackFirmwareStream:
    """Adapter used by ``TcpLoopbackRobotServer`` for acceptance tests."""

    def __init__(self, connection: socket.socket) -> None:
        self.connection = connection

    def emit(self, payload: bytes) -> None:
        self.connection.sendall(payload)


class TcpLoopbackRobotServer:
    """Local TCP server backed by the deterministic loopback firmware model."""

    def __init__(self, firmware: "LoopbackRobotFirmware | None" = None) -> None:
        self.firmware = firmware or LoopbackRobotFirmware()
        self.host = "127.0.0.1"
        self.port = 0
        self._listener: socket.socket | None = None
        self._connection: socket.socket | None = None
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self.host, 0))
        listener.listen(1)
        listener.settimeout(0.1)
        self._listener = listener
        self.port = int(listener.getsockname()[1])
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._serve, name="tcp-loopback-firmware", daemon=True)
        self._thread.start()

    def _serve(self) -> None:
        listener = self._listener
        if listener is None:
            return
        while not self._stop_event.is_set():
            try:
                connection, _address = listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            self._connection = connection
            connection.settimeout(0.1)
            stream = _TcpLoopbackFirmwareStream(connection)
            self.firmware.attach(stream)  # type: ignore[arg-type]
            try:
                while not self._stop_event.is_set():
                    try:
                        payload = connection.recv(4096)
                    except socket.timeout:
                        continue
                    if not payload:
                        break
                    self.firmware.receive(payload)
            except OSError:
                pass
            finally:
                try:
                    connection.close()
                except OSError:
                    pass
                self._connection = None
            return

    def stop(self) -> None:
        self._stop_event.set()
        connection = self._connection
        if connection is not None:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                connection.close()
            except OSError:
                pass
        listener = self._listener
        if listener is not None:
            try:
                listener.close()
            except OSError:
                pass
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._thread = None
        self._listener = None
        self._connection = None


class LoopbackRobotFirmware:
    """Tiny protocol-only firmware model for N3 acceptance tests."""

    def __init__(self) -> None:
        self.stream: LoopbackSerialStream | None = None
        self.pose = Pose(0.0, 0.0, 0.0)
        self.runner = "IDLE"
        self.pen = "UP"
        self.estop_active = False
        self.uploaded: UploadReceipt | None = None
        self.uploaded_bytes = b""
        self._upload_meta: UploadReceipt | None = None
        self._upload_buffer = bytearray()
        self._upload_remaining = 0
        self._line_buffer = bytearray()
        self._seq = 0
        self._command_responses: dict[int, dict[str, Any]] = {}

    def attach(self, stream: "LoopbackSerialStream") -> None:
        self.stream = stream
        self._emit({"type": MessageType.HELLO.value, "protocol": 1, "firmware": "loopback-n3", "features": ["upload", "pose", "status"]})
        self._emit_status()
        self._emit_pose()

    def receive(self, payload: bytes) -> None:
        offset = 0
        while offset < len(payload):
            if self._upload_remaining:
                count = min(self._upload_remaining, len(payload) - offset)
                self._upload_buffer.extend(payload[offset:offset + count])
                self._upload_remaining -= count
                offset += count
                continue
            newline = payload.find(b"\n", offset)
            if newline < 0:
                self._line_buffer.extend(payload[offset:])
                return
            self._line_buffer.extend(payload[offset:newline])
            offset = newline + 1
            try:
                message = decode_message(bytes(self._line_buffer))
            except ProtocolError as exc:
                self._emit({"type": MessageType.ERROR.value, "id": -1, "code": "PROTOCOL", "message": str(exc)})
                self._line_buffer.clear()
                continue
            self._line_buffer.clear()
            self._handle(message)

    def _emit(self, message: Mapping[str, Any]) -> None:
        if self.stream is not None:
            self.stream.emit(encode_message(message))

    def _emit_status(self) -> None:
        self._emit({
            "type": MessageType.STATUS.value,
            "runner": self.runner,
            "tracker": "IDLE",
            "pen": self.pen,
            "motor_ready": True,
            "odom_ready": True,
            "pose_valid": True,
            "estop": self.estop_active,
            "current_job": self.uploaded.job_id if self.uploaded else None,
            "error": None,
        })

    def _emit_pose(self) -> None:
        self._seq += 1
        self._emit(self.pose.to_message(self._seq))

    def _ack(self, message: Mapping[str, Any], ack_type: str) -> None:
        response = {"type": MessageType.ACK.value, "id": message.get("id", -1), "ack_type": ack_type}
        command_id = response["id"]
        if isinstance(command_id, int) and command_id >= 0:
            self._command_responses[command_id] = response
        self._emit(response)

    def _error(self, message: Mapping[str, Any], code: str, text: str) -> None:
        response = {"type": MessageType.ERROR.value, "id": message.get("id", -1), "code": code, "message": text}
        command_id = response["id"]
        if isinstance(command_id, int) and command_id >= 0:
            self._command_responses[command_id] = response
        self._emit(response)

    def _handle(self, message: Mapping[str, Any]) -> None:
        command_id = message.get("id")
        if isinstance(command_id, int) and command_id in self._command_responses:
            # Idempotency rule: a retransmitted command gets the original ACK
            # or ERROR and never executes the physical action twice.
            self._emit(self._command_responses[command_id])
            return
        kind = message.get("type")
        if kind == MessageType.PING.value:
            self._ack(message, MessageType.PING.value)
            self._emit({"type": MessageType.PONG.value, "id": message.get("id", -1), "nonce": message.get("nonce", "")})
        elif kind == MessageType.UPLOAD_BEGIN.value:
            self._upload_meta = UploadReceipt(message["job_id"], message["size"], str(message["crc32"]).upper())
            self._upload_buffer.clear()
            self._upload_remaining = self._upload_meta.size
            self._ack(message, MessageType.UPLOAD_READY.value)
        elif kind == MessageType.UPLOAD_END.value:
            if self._upload_meta is None or self._upload_remaining:
                self._error(message, "UPLOAD_INCOMPLETE", "trajectory payload is incomplete")
            elif len(self._upload_buffer) != message.get("size") or crc32_hex(bytes(self._upload_buffer)) != str(message.get("crc32")).upper():
                self._error(message, "CRC_MISMATCH", "trajectory CRC or size mismatch")
            else:
                self.uploaded_bytes = bytes(self._upload_buffer)
                self.uploaded = self._upload_meta
                self._upload_meta = None
                self._ack(message, MessageType.UPLOAD_END.value)
                self._emit_status()
        elif kind == MessageType.RUN_TRAJECTORY.value:
            if self.uploaded is None or message["job_id"] != self.uploaded.job_id or str(message["crc32"]).upper() != self.uploaded.crc32:
                self._error(message, "JOB_MISMATCH", "requested trajectory is not the uploaded job")
            elif self.estop_active:
                self._error(message, "ESTOP_ACTIVE", "clear emergency stop before running")
            else:
                self.runner = "RUNNING"
                self._ack(message, MessageType.RUN_TRAJECTORY.value)
                self._emit_status()
        elif kind == MessageType.STOP.value:
            self.runner = "IDLE"
            self.pen = "UP"
            self._ack(message, kind)
            self._emit_status()
        elif kind == MessageType.ESTOP.value:
            self.runner = "ESTOPPED"
            self.pen = "UP"
            self.estop_active = True
            self._ack(message, kind)
            self._emit_status()
        elif kind == MessageType.CLEAR_ESTOP.value:
            self.estop_active = False
            self.runner = "IDLE"
            self._ack(message, kind)
            self._emit_status()
        elif kind == MessageType.RESET_POSE.value:
            self.pose = Pose(message["x_mm"], message["y_mm"], message["yaw_deg"])
            self._ack(message, kind)
            self._emit_pose()
        elif kind == MessageType.ROTATE_REL.value:
            self.pose = Pose(self.pose.x_mm, self.pose.y_mm, self.pose.yaw_deg + message["angle_deg"])
            self._ack(message, kind)
            self._emit_pose()
        else:
            self._error(message, "UNKNOWN_COMMAND", str(kind))


class LoopbackSerialStream:
    """Serial-like duplex stream connected to :class:`LoopbackRobotFirmware`."""

    def __init__(self, firmware: LoopbackRobotFirmware | None = None) -> None:
        self.firmware = firmware or LoopbackRobotFirmware()
        self._outgoing: queue.Queue[bytes] = queue.Queue()
        self.is_open = False

    def open(self) -> None:
        self.is_open = True
        self.firmware.attach(self)

    def close(self) -> None:
        self.is_open = False
        self._outgoing.put(b"")

    def write(self, payload: bytes) -> int:
        if not self.is_open:
            raise OSError("loopback stream is closed")
        self.firmware.receive(bytes(payload))
        return len(payload)

    def flush(self) -> None:
        return None

    def emit(self, payload: bytes) -> None:
        self._outgoing.put(bytes(payload))

    def readline(self) -> bytes:
        if not self.is_open and self._outgoing.empty():
            return b""
        try:
            return self._outgoing.get(timeout=0.1)
        except queue.Empty:
            return b""


__all__ = [
    "LoopbackRobotFirmware",
    "LoopbackSerialStream",
    "RobotLinkError",
    "RobotLinkState",
    "SerialRobotLink",
    "TcpLoopbackRobotServer",
    "TcpRobotLink",
    "TcpSocketStream",
    "UploadReceipt",
    "crc32_hex",
    "list_serial_ports",
]
