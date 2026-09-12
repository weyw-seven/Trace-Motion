"""N0 PC/ESP32 message contract and coordinate helpers.

The N0 protocol is intentionally small and human-readable.  It is a line
delimited JSON protocol for commands, acknowledgements, errors and telemetry.
Trajectory bytes are uploaded by a later phase and are deliberately outside
this module.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
import math
from typing import Any, Mapping


PROTOCOL_VERSION = 1


class ProtocolError(ValueError):
    """Raised when a protocol message is malformed or unsafe to use."""


class MessageType(str, Enum):
    HELLO = "HELLO"
    PING = "PING"
    PONG = "PONG"
    ACK = "ACK"
    ERROR = "ERROR"
    UPLOAD_BEGIN = "UPLOAD_BEGIN"
    UPLOAD_READY = "UPLOAD_READY"
    UPLOAD_END = "UPLOAD_END"
    UPLOAD_ABORT = "UPLOAD_ABORT"
    RUN_TRAJECTORY = "RUN_TRAJECTORY"
    POSE = "POSE"
    STATUS = "STATUS"
    STOP = "STOP"
    ESTOP = "ESTOP"
    CLEAR_ESTOP = "CLEAR_ESTOP"
    RESET_POSE = "RESET_POSE"
    ROTATE_REL = "ROTATE_REL"


COMMAND_TYPES = frozenset(
    {
        MessageType.PING,
        MessageType.STOP,
        MessageType.ESTOP,
        MessageType.CLEAR_ESTOP,
        MessageType.RESET_POSE,
        MessageType.ROTATE_REL,
        MessageType.UPLOAD_BEGIN,
        MessageType.UPLOAD_END,
        MessageType.UPLOAD_ABORT,
        MessageType.RUN_TRAJECTORY,
    }
)
TELEMETRY_TYPES = frozenset({MessageType.HELLO, MessageType.PONG, MessageType.POSE, MessageType.STATUS})


def _finite(name: str, value: Any) -> float:
    if isinstance(value, bool):
        raise ProtocolError(f"{name} must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ProtocolError(f"{name} must be a finite number") from exc
    if not math.isfinite(number):
        raise ProtocolError(f"{name} must be a finite number")
    return number


def _integer(name: str, value: Any) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ProtocolError(f"{name} must be an integer")
    if value < 0:
        raise ProtocolError(f"{name} must be non-negative")
    return value


def wrap_to_180(angle_deg: float) -> float:
    """Return the shortest signed angle in [-180, 180)."""

    angle = _finite("angle_deg", angle_deg)
    return (angle + 180.0) % 360.0 - 180.0


def shortest_angle_deg(target_deg: float, current_deg: float) -> float:
    """Return the shortest relative rotation from current to target."""

    return wrap_to_180(_finite("target_deg", target_deg) - _finite("current_deg", current_deg))


def _require_fields(message: Mapping[str, Any], *names: str) -> None:
    missing = [name for name in names if name not in message]
    if missing:
        raise ProtocolError(f"missing field(s): {', '.join(missing)}")


def _validate_message(message: Mapping[str, Any]) -> dict[str, Any]:
    if not isinstance(message, Mapping):
        raise ProtocolError("message must be a JSON object")
    raw_type = message.get("type")
    try:
        message_type = MessageType(raw_type)
    except (TypeError, ValueError) as exc:
        raise ProtocolError(f"unknown message type: {raw_type!r}") from exc

    result = dict(message)
    result["type"] = message_type.value

    if message_type is MessageType.HELLO:
        _require_fields(result, "protocol")
        if result["protocol"] != PROTOCOL_VERSION:
            raise ProtocolError(f"unsupported protocol version: {result['protocol']!r}")
    elif message_type is MessageType.ACK:
        _require_fields(result, "id", "ack_type")
        _integer("id", result["id"])
        if not isinstance(result["ack_type"], str):
            raise ProtocolError("ack_type must be a string")
    elif message_type is MessageType.ERROR:
        _require_fields(result, "id", "code", "message")
        _integer("id", result["id"])
        if not isinstance(result["code"], str) or not isinstance(result["message"], str):
            raise ProtocolError("ERROR code and message must be strings")
    elif message_type is MessageType.POSE:
        _require_fields(
            result,
            "seq",
            "x_mm",
            "y_mm",
            "yaw_deg",
            "vx_world_mm_s",
            "vy_world_mm_s",
            "w_rad_s",
            "timestamp_ms",
        )
        _integer("seq", result["seq"])
        for name in (
            "x_mm",
            "y_mm",
            "yaw_deg",
            "vx_world_mm_s",
            "vy_world_mm_s",
            "w_rad_s",
            "timestamp_ms",
        ):
            _finite(name, result[name])
    elif message_type is MessageType.STATUS:
        _require_fields(result, "runner", "tracker", "pen", "motor_ready", "odom_ready", "estop")
        for name in ("runner", "tracker", "pen"):
            if not isinstance(result[name], str):
                raise ProtocolError(f"{name} must be a string")
        for name in ("motor_ready", "odom_ready", "estop"):
            if not isinstance(result[name], bool):
                raise ProtocolError(f"{name} must be a boolean")
        for name in ("pose_valid",):
            if name in result and not isinstance(result[name], bool):
                raise ProtocolError(f"{name} must be a boolean")
        for name in ("error", "current_job"):
            if name in result and result[name] is not None and not isinstance(result[name], str):
                raise ProtocolError(f"{name} must be a string or null")
    elif message_type in COMMAND_TYPES:
        _require_fields(result, "id")
        _integer("id", result["id"])
        if message_type is MessageType.PING:
            if "nonce" in result and not isinstance(result["nonce"], str):
                raise ProtocolError("nonce must be a string")
        elif message_type is MessageType.UPLOAD_BEGIN:
            _require_fields(result, "job_id", "size", "crc32")
            if not isinstance(result["job_id"], str) or not result["job_id"].strip():
                raise ProtocolError("job_id must be a non-empty string")
            _integer("size", result["size"])
            _validate_crc32(result["crc32"])
        elif message_type in {MessageType.UPLOAD_END, MessageType.UPLOAD_ABORT}:
            _require_fields(result, "job_id")
            if not isinstance(result["job_id"], str) or not result["job_id"].strip():
                raise ProtocolError("job_id must be a non-empty string")
            if "size" in result:
                _integer("size", result["size"])
            if "crc32" in result:
                _validate_crc32(result["crc32"])
        elif message_type is MessageType.RUN_TRAJECTORY:
            _require_fields(result, "job_id", "crc32")
            if not isinstance(result["job_id"], str) or not result["job_id"].strip():
                raise ProtocolError("job_id must be a non-empty string")
            _validate_crc32(result["crc32"])
            if "reset_odometry" in result and not isinstance(result["reset_odometry"], bool):
                raise ProtocolError("reset_odometry must be a boolean")
        elif message_type is MessageType.RESET_POSE:
            _require_fields(result, "x_mm", "y_mm", "yaw_deg")
            for name in ("x_mm", "y_mm", "yaw_deg"):
                _finite(name, result[name])
        elif message_type is MessageType.ROTATE_REL:
            _require_fields(result, "angle_deg", "speed_deg_s")
            _finite("angle_deg", result["angle_deg"])
            speed = _finite("speed_deg_s", result["speed_deg_s"])
            if speed <= 0.0:
                raise ProtocolError("speed_deg_s must be positive")

    return result


def _validate_crc32(value: Any) -> str:
    if isinstance(value, int) and not isinstance(value, bool):
        if 0 <= value <= 0xFFFFFFFF:
            return f"{value:08X}"
    if isinstance(value, str):
        normalized = value.strip().upper()
        if len(normalized) == 8:
            try:
                int(normalized, 16)
            except ValueError:
                pass
            else:
                return normalized
    raise ProtocolError("crc32 must be an 8-digit hexadecimal string or uint32")


def encode_message(message: Mapping[str, Any]) -> bytes:
    """Validate and encode one message as one UTF-8 JSON line."""

    validated = _validate_message(message)
    try:
        text = json.dumps(
            validated,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
            allow_nan=False,
        )
    except (TypeError, ValueError) as exc:
        raise ProtocolError(f"message is not JSON serializable: {exc}") from exc
    return (text + "\n").encode("utf-8")


def decode_message(line: bytes | str) -> dict[str, Any]:
    """Decode and validate one complete JSON line."""

    if isinstance(line, bytes):
        try:
            text = line.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ProtocolError("message is not valid UTF-8") from exc
    elif isinstance(line, str):
        text = line
    else:
        raise ProtocolError("message must be bytes or str")
    text = text.strip()
    if not text:
        raise ProtocolError("message is empty")
    try:
        value = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ProtocolError(f"invalid JSON: {exc.msg}") from exc
    return _validate_message(value)


def make_command(message_type: MessageType | str, command_id: int, **fields: Any) -> dict[str, Any]:
    """Build and validate a command object with a caller-owned ID."""

    try:
        kind = MessageType(message_type)
    except (TypeError, ValueError) as exc:
        raise ProtocolError(f"unknown command type: {message_type!r}") from exc
    if kind not in COMMAND_TYPES:
        raise ProtocolError(f"{kind.value} is not a command")
    message = {"type": kind.value, "id": command_id, **fields}
    return _validate_message(message)


@dataclass(frozen=True)
class Pose:
    """A WORLD-frame odometry pose and velocity snapshot."""

    x_mm: float
    y_mm: float
    yaw_deg: float
    vx_world_mm_s: float = 0.0
    vy_world_mm_s: float = 0.0
    w_rad_s: float = 0.0
    timestamp_ms: float = 0.0

    def __post_init__(self) -> None:
        for name in (
            "x_mm",
            "y_mm",
            "yaw_deg",
            "vx_world_mm_s",
            "vy_world_mm_s",
            "w_rad_s",
            "timestamp_ms",
        ):
            object.__setattr__(self, name, _finite(name, getattr(self, name)))

    def to_message(self, seq: int) -> dict[str, Any]:
        message = {
            "type": MessageType.POSE.value,
            "seq": seq,
            "x_mm": self.x_mm,
            "y_mm": self.y_mm,
            "yaw_deg": self.yaw_deg,
            "vx_world_mm_s": self.vx_world_mm_s,
            "vy_world_mm_s": self.vy_world_mm_s,
            "w_rad_s": self.w_rad_s,
            "timestamp_ms": self.timestamp_ms,
        }
        return _validate_message(message)

    @classmethod
    def from_message(cls, message: Mapping[str, Any]) -> "Pose":
        value = _validate_message(message)
        if value["type"] != MessageType.POSE.value:
            raise ProtocolError("message is not a POSE")
        return cls(
            x_mm=value["x_mm"],
            y_mm=value["y_mm"],
            yaw_deg=value["yaw_deg"],
            vx_world_mm_s=value["vx_world_mm_s"],
            vy_world_mm_s=value["vy_world_mm_s"],
            w_rad_s=value["w_rad_s"],
            timestamp_ms=value["timestamp_ms"],
        )


__all__ = [
    "COMMAND_TYPES",
    "MessageType",
    "Pose",
    "PROTOCOL_VERSION",
    "ProtocolError",
    "TELEMETRY_TYPES",
    "decode_message",
    "encode_message",
    "make_command",
    "shortest_angle_deg",
    "wrap_to_180",
]
