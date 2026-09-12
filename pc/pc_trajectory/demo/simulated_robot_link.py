"""Deterministic N0 robot link used before connecting real hardware."""

from __future__ import annotations

from collections import deque
import json
import math
from typing import Any, Mapping

from .protocol import (
    MessageType,
    Pose,
    ProtocolError,
    decode_message,
    make_command,
)


class SimulatedRobotLink:
    """A small synchronous robot model for protocol and coordinate tests.

    It never talks to hardware.  ``inject_body_motion`` uses the same BODY to
    WORLD rotation as the firmware odometry contract, so it is useful for
    catching mirrored axes and wrong yaw signs before N1.
    """

    def __init__(self, pose: Pose | None = None) -> None:
        self.pose = pose or Pose(0.0, 0.0, 0.0)
        self.connected = False
        self.estopped = False
        self.runner = "IDLE"
        self.tracker = "IDLE"
        self.pen = "UP"
        self.motor_ready = False
        self.odom_ready = False
        self._events: deque[dict[str, Any]] = deque()
        self.command_log: list[dict[str, Any]] = []
        self._seq = 0

    def connect(self) -> None:
        self.connected = True
        self.motor_ready = True
        self.odom_ready = True
        self._events.append({"type": MessageType.ACK.value, "id": 0, "ack_type": "HELLO"})
        self._emit_status()
        self._emit_pose()

    def disconnect(self) -> None:
        self.connected = False
        self.motor_ready = False
        self.odom_ready = False

    def _emit_pose(self) -> None:
        self._seq += 1
        self._events.append(self.pose.to_message(self._seq))

    def _emit_status(self) -> None:
        self._events.append(
            {
                "type": MessageType.STATUS.value,
                "runner": self.runner,
                "tracker": self.tracker,
                "pen": self.pen,
                "motor_ready": self.motor_ready,
                "odom_ready": self.odom_ready,
                "estop": self.estopped,
            }
        )

    def drain_events(self) -> list[dict[str, Any]]:
        events = list(self._events)
        self._events.clear()
        return events

    def send(self, message: Mapping[str, Any] | bytes | str) -> list[dict[str, Any]]:
        if not self.connected:
            raise ConnectionError("simulated robot is disconnected")
        decoded = decode_message(message) if not isinstance(message, Mapping) else decode_message(json.dumps(dict(message)))
        kind = MessageType(decoded["type"])
        if kind not in {
            MessageType.STOP,
            MessageType.ESTOP,
            MessageType.CLEAR_ESTOP,
            MessageType.RESET_POSE,
            MessageType.ROTATE_REL,
        }:
            raise ProtocolError(f"{kind.value} is not accepted by the simulated command link")
        self.command_log.append(decoded)
        command_id = decoded["id"]
        if kind is MessageType.ESTOP:
            self.estopped = True
            self.runner = "ESTOPPED"
            self.tracker = "IDLE"
            self.pose = Pose(self.pose.x_mm, self.pose.y_mm, self.pose.yaw_deg, timestamp_ms=self.pose.timestamp_ms)
        elif kind is MessageType.CLEAR_ESTOP:
            self.estopped = False
            self.runner = "IDLE"
        elif self.estopped:
            self._events.append(
                {
                    "type": MessageType.ERROR.value,
                    "id": command_id,
                    "code": "ESTOP_ACTIVE",
                    "message": "clear emergency stop before sending motion commands",
                }
            )
            self._emit_status()
            return self.drain_events()
        elif kind is MessageType.STOP:
            self.runner = "IDLE"
            self.tracker = "IDLE"
            self.pose = Pose(self.pose.x_mm, self.pose.y_mm, self.pose.yaw_deg, timestamp_ms=self.pose.timestamp_ms)
        elif kind is MessageType.RESET_POSE:
            self.pose = Pose(
                decoded["x_mm"],
                decoded["y_mm"],
                decoded["yaw_deg"],
                timestamp_ms=self.pose.timestamp_ms,
            )
        elif kind is MessageType.ROTATE_REL:
            self.pose = Pose(
                self.pose.x_mm,
                self.pose.y_mm,
                self.pose.yaw_deg + decoded["angle_deg"],
                timestamp_ms=self.pose.timestamp_ms,
            )
        self._events.append({"type": MessageType.ACK.value, "id": command_id, "ack_type": kind.value})
        self._emit_status()
        self._emit_pose()
        return self.drain_events()

    def inject_body_motion(
        self,
        vx_mm_s: float,
        vy_mm_s: float,
        dt_s: float,
        w_rad_s: float = 0.0,
    ) -> Pose:
        """Advance the model using BODY velocities and odometry conventions."""

        if not self.connected:
            raise ConnectionError("simulated robot is disconnected")
        if self.estopped:
            raise RuntimeError("cannot move while emergency stop is active")
        dt = float(dt_s)
        if not math.isfinite(dt) or dt < 0.0:
            raise ValueError("dt_s must be finite and non-negative")
        yaw_rad = math.radians(self.pose.yaw_deg)
        dx_world = (math.cos(yaw_rad) * vx_mm_s - math.sin(yaw_rad) * vy_mm_s) * dt
        dy_world = (math.sin(yaw_rad) * vx_mm_s + math.cos(yaw_rad) * vy_mm_s) * dt
        self.pose = Pose(
            self.pose.x_mm + dx_world,
            self.pose.y_mm + dy_world,
            self.pose.yaw_deg + math.degrees(w_rad_s * dt),
            vx_world_mm_s=dx_world / dt if dt else 0.0,
            vy_world_mm_s=dy_world / dt if dt else 0.0,
            w_rad_s=w_rad_s,
            timestamp_ms=self.pose.timestamp_ms + dt * 1000.0,
        )
        self._emit_pose()
        return self.pose


__all__ = ["SimulatedRobotLink"]
