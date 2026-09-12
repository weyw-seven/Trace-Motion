"""Deterministic N2 Toolpath simulator used by the desktop demo.

This is intentionally a kinematic review tool, not a replacement for the
ESP32 motor controller.  It executes the same Motion/Event stream that is
exported to TRJ2, advances by a caller supplied time step, and records the
pose trace used by the acceptance artifacts and UI.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from ..geometry import CubicBezier, Geometry, Line, Point2D
from ..toolpath import Motion, PenDown, PenState, PenUp, Toolpath, Wait
from .map_planner import NavigationPlan
from .protocol import Pose


class SimulationError(ValueError):
    """Raised for invalid simulation state or time steps."""


class SimulationState(str, Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    STOPPED = "STOPPED"
    ESTOPPED = "ESTOPPED"
    FINISHED = "FINISHED"


@dataclass(frozen=True)
class SimulationSnapshot:
    state: SimulationState
    pose: Pose
    pen_state: PenState
    elapsed_s: float
    active_record_index: int | None
    completed_records: int


@dataclass(frozen=True)
class _SampledMotion:
    record_index: int
    motion: Motion
    points: tuple[Point2D, ...]
    cumulative_mm: tuple[float, ...]
    length_mm: float

    def point_at_distance(self, distance_mm: float) -> Point2D:
        if distance_mm <= 0.0:
            return self.points[0]
        if distance_mm >= self.length_mm:
            return self.points[-1]
        for index in range(1, len(self.points)):
            upper = self.cumulative_mm[index]
            if distance_mm <= upper:
                lower = self.cumulative_mm[index - 1]
                span = upper - lower
                fraction = 0.0 if span <= 1.0e-12 else (distance_mm - lower) / span
                start = self.points[index - 1]
                end = self.points[index]
                return Point2D(
                    start.x_mm + (end.x_mm - start.x_mm) * fraction,
                    start.y_mm + (end.y_mm - start.y_mm) * fraction,
                )
        return self.points[-1]


def _sample_motion(motion: Motion, record_index: int, step_mm: float) -> _SampledMotion:
    geometry = motion.geometry
    length = geometry.length_mm()
    count = max(2, min(4096, int(math.ceil(length / step_mm)) + 1))
    points = tuple(geometry.point_at(i / (count - 1)) for i in range(count))
    cumulative = [0.0]
    for previous, current in zip(points, points[1:]):
        cumulative.append(cumulative[-1] + previous.distance_to(current))
    return _SampledMotion(record_index, motion, points, tuple(cumulative), cumulative[-1])


class SimulatedNavigationRunner:
    """Execute a :class:`NavigationPlan` with deterministic ``tick`` calls."""

    def __init__(
        self,
        pose: Pose | None = None,
        *,
        sample_step_mm: float = 4.0,
        pose_tolerance_mm: float = 1.0e-5,
    ) -> None:
        if not math.isfinite(sample_step_mm) or sample_step_mm <= 0.0:
            raise SimulationError("sample_step_mm must be > 0")
        if not math.isfinite(pose_tolerance_mm) or pose_tolerance_mm < 0.0:
            raise SimulationError("pose_tolerance_mm must be >= 0")
        self.pose = pose or Pose(0.0, 0.0, 0.0)
        self.sample_step_mm = float(sample_step_mm)
        self.pose_tolerance_mm = float(pose_tolerance_mm)
        self.state = SimulationState.IDLE
        self.pen_state = PenState.UP
        self.elapsed_s = 0.0
        self.active_record_index: int | None = None
        self._plan: NavigationPlan | None = None
        self._cursor = 0
        self._segment: _SampledMotion | None = None
        self._segment_distance_mm = 0.0
        self._wait_remaining_s = 0.0
        self._trace: list[Pose] = [self.pose]

    @property
    def plan(self) -> NavigationPlan | None:
        return self._plan

    @property
    def trace(self) -> tuple[Pose, ...]:
        return tuple(self._trace)

    @property
    def is_running(self) -> bool:
        return self.state is SimulationState.RUNNING

    def snapshot(self) -> SimulationSnapshot:
        return SimulationSnapshot(
            state=self.state,
            pose=self.pose,
            pen_state=self.pen_state,
            elapsed_s=self.elapsed_s,
            active_record_index=self.active_record_index,
            completed_records=self._cursor,
        )

    def start(self, plan: NavigationPlan) -> None:
        if not isinstance(plan, NavigationPlan):
            raise SimulationError("plan must be a NavigationPlan")
        if self.state is SimulationState.ESTOPPED:
            raise SimulationError("clear emergency stop before starting a plan")
        start = plan.start_point
        if self.pose_distance_to(start) > self.pose_tolerance_mm:
            raise SimulationError(
                "simulator pose does not match plan start: "
                f"error={self.pose_distance_to(start):.6f} mm"
            )
        self._plan = plan
        self._cursor = 0
        self._segment = None
        self._segment_distance_mm = 0.0
        self._wait_remaining_s = 0.0
        self.elapsed_s = 0.0
        self.active_record_index = None
        self.pen_state = PenState.UP
        self.state = SimulationState.RUNNING
        self._trace = [self.pose]

    def pose_distance_to(self, point: Point2D) -> float:
        return math.hypot(self.pose.x_mm - point.x_mm, self.pose.y_mm - point.y_mm)

    def _set_pose(self, point: Point2D, dt_s: float) -> None:
        if dt_s <= 0.0:
            vx = vy = 0.0
        else:
            vx = (point.x_mm - self.pose.x_mm) / dt_s
            vy = (point.y_mm - self.pose.y_mm) / dt_s
        self.pose = Pose(
            point.x_mm,
            point.y_mm,
            self.pose.yaw_deg,
            vx_world_mm_s=vx,
            vy_world_mm_s=vy,
            timestamp_ms=self.pose.timestamp_ms + dt_s * 1000.0,
        )
        if not self._trace or point.distance_to(Point2D(self._trace[-1].x_mm, self._trace[-1].y_mm)) > 1.0e-9:
            self._trace.append(self.pose)

    def _finish(self) -> None:
        self.state = SimulationState.FINISHED
        self.active_record_index = None
        self.pen_state = PenState.UP
        self.pose = Pose(
            self.pose.x_mm,
            self.pose.y_mm,
            self.pose.yaw_deg,
            timestamp_ms=self.pose.timestamp_ms,
        )

    def _advance_record(self) -> bool:
        """Prepare the next record; return False when the job has finished."""

        if self._plan is None:
            self._finish()
            return False
        records = self._plan.toolpath.records
        while self._cursor < len(records):
            record_index = self._cursor
            record = records[record_index]
            self._cursor += 1
            self.active_record_index = record_index
            if isinstance(record, PenUp):
                self.pen_state = PenState.UP
                continue
            if isinstance(record, PenDown):
                self.pen_state = PenState.DOWN
                continue
            if isinstance(record, Wait):
                self._wait_remaining_s = record.duration_s
                return True
            if isinstance(record, Motion):
                self._segment = _sample_motion(record, record_index, self.sample_step_mm)
                self._segment_distance_mm = 0.0
                return True
            raise SimulationError(f"unsupported Toolpath record: {type(record).__name__}")
        self._finish()
        return False

    def tick(self, dt_s: float) -> SimulationSnapshot:
        dt = float(dt_s)
        if not math.isfinite(dt) or dt < 0.0:
            raise SimulationError("dt_s must be finite and >= 0")
        if self.state is not SimulationState.RUNNING or dt == 0.0:
            return self.snapshot()

        remaining = dt
        while remaining > 1.0e-12 and self.state is SimulationState.RUNNING:
            if self._segment is None and self._wait_remaining_s <= 0.0:
                if not self._advance_record():
                    break

            if self._wait_remaining_s > 0.0:
                used = min(remaining, self._wait_remaining_s)
                self._wait_remaining_s -= used
                self.elapsed_s += used
                self.pose = Pose(
                    self.pose.x_mm,
                    self.pose.y_mm,
                    self.pose.yaw_deg,
                    timestamp_ms=self.pose.timestamp_ms + used * 1000.0,
                )
                remaining -= used
                continue

            if self._segment is None:
                continue
            segment = self._segment
            distance_left = segment.length_mm - self._segment_distance_mm
            travel = min(distance_left, segment.motion.speed_mm_s * remaining)
            used = travel / segment.motion.speed_mm_s if segment.motion.speed_mm_s > 0.0 else 0.0
            self._segment_distance_mm += travel
            self.elapsed_s += used
            self._set_pose(segment.point_at_distance(self._segment_distance_mm), used)
            remaining -= used
            if distance_left <= 1.0e-9 or self._segment_distance_mm >= segment.length_mm - 1.0e-9:
                self._set_pose(segment.points[-1], 0.0)
                self._segment = None
                self.active_record_index = None
                if used <= 1.0e-12 and remaining > 0.0:
                    continue

        return self.snapshot()

    def run_to_completion(self, *, dt_s: float = 0.02, max_steps: int = 1_000_000) -> SimulationSnapshot:
        if not math.isfinite(dt_s) or dt_s <= 0.0:
            raise SimulationError("dt_s must be > 0")
        for _ in range(max_steps):
            if self.state is not SimulationState.RUNNING:
                return self.snapshot()
            self.tick(dt_s)
        raise SimulationError("simulation exceeded max_steps")

    def stop(self) -> SimulationSnapshot:
        if self.state is SimulationState.RUNNING:
            self.state = SimulationState.STOPPED
            self.pen_state = PenState.UP
            self.active_record_index = None
        return self.snapshot()

    def emergency_stop(self) -> SimulationSnapshot:
        self.state = SimulationState.ESTOPPED
        self.pen_state = PenState.UP
        self.active_record_index = None
        self._segment = None
        return self.snapshot()

    def clear_emergency_stop(self) -> SimulationSnapshot:
        if self.state is SimulationState.ESTOPPED:
            self.state = SimulationState.IDLE
        return self.snapshot()

    def reset(self, pose: Pose | None = None) -> SimulationSnapshot:
        self.pose = pose or Pose(0.0, 0.0, 0.0)
        self.state = SimulationState.IDLE
        self.pen_state = PenState.UP
        self.elapsed_s = 0.0
        self.active_record_index = None
        self._plan = None
        self._cursor = 0
        self._segment = None
        self._segment_distance_mm = 0.0
        self._wait_remaining_s = 0.0
        self._trace = [self.pose]
        return self.snapshot()


__all__ = [
    "SimulationError",
    "SimulationSnapshot",
    "SimulationState",
    "SimulatedNavigationRunner",
]
