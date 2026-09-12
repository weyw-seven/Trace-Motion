"""Reviewable N2 navigation job wrapper.

The job object keeps the planning result, simulator lifecycle, and a compact
JSON-serialisable report together.  It is intentionally independent from the
Tk UI so the same flow can be exercised by tests and acceptance scripts.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

from .map_model import MapDocument
from .map_planner import NavigationPlan
from .navigation_simulator import SimulationState, SimulatedNavigationRunner


@dataclass
class NavigationJob:
    plan: NavigationPlan
    runner: SimulatedNavigationRunner

    @classmethod
    def create(cls, plan: NavigationPlan, *, runner: SimulatedNavigationRunner | None = None) -> "NavigationJob":
        simulator = runner or SimulatedNavigationRunner(plan.start_pose)
        return cls(plan=plan, runner=simulator)

    def validate_document(self, document: MapDocument) -> None:
        self.plan.ensure_current(document)

    def start(self) -> None:
        self.runner.start(self.plan)

    def tick(self, dt_s: float):
        return self.runner.tick(dt_s)

    def run_to_completion(self, *, dt_s: float = 0.02):
        return self.runner.run_to_completion(dt_s=dt_s)

    def stop(self):
        return self.runner.stop()

    def emergency_stop(self):
        return self.runner.emergency_stop()

    def clear_emergency_stop(self):
        return self.runner.clear_emergency_stop()

    def report(self) -> dict[str, Any]:
        snapshot = self.runner.snapshot()
        end = self.plan.end_point
        return {
            "mode": self.plan.mode.value,
            "pen_mode": self.plan.pen_mode.value,
            "path_id": self.plan.path_id,
            "path_direction": self.plan.path_direction.value if self.plan.path_direction else None,
            "path_ids": list(self.plan.path_ids),
            "path_directions": [item.value for item in self.plan.path_directions],
            "requested_target": {
                "x_mm": self.plan.requested_target_world.x_mm,
                "y_mm": self.plan.requested_target_world.y_mm,
            } if self.plan.requested_target_world is not None else None,
            "route_legs": [
                {
                    "path_id": leg.path_id,
                    "sequence_index": leg.sequence_index,
                    "direction": leg.direction.value,
                    "requested_start": {"x_mm": leg.requested_start.x_mm, "y_mm": leg.requested_start.y_mm},
                    "effective_start": {"x_mm": leg.effective_start.x_mm, "y_mm": leg.effective_start.y_mm},
                    "requested_end": {"x_mm": leg.requested_end.x_mm, "y_mm": leg.requested_end.y_mm},
                    "effective_end": {"x_mm": leg.effective_end.x_mm, "y_mm": leg.effective_end.y_mm},
                    "detour_count": leg.detour_count,
                }
                for leg in self.plan.route_legs
            ],
            "document_revision": self.plan.document_revision,
            "record_count": self.plan.toolpath.record_count,
            "motion_count": self.plan.toolpath.motion_count,
            "drawing_length_mm": self.plan.toolpath.drawing_length_mm(),
            "travel_length_mm": self.plan.toolpath.travel_length_mm(),
            "simulation": {
                "state": snapshot.state.value,
                "pen_state": snapshot.pen_state.value,
                "elapsed_s": snapshot.elapsed_s,
                "pose": {
                    "x_mm": snapshot.pose.x_mm,
                    "y_mm": snapshot.pose.y_mm,
                    "yaw_deg": snapshot.pose.yaw_deg,
                },
                "trace_points": len(self.runner.trace),
            },
            "plan_end": {
                "x_mm": end.x_mm,
                "y_mm": end.y_mm,
            } if end is not None else None,
        }

    def write_report(self, path: str | Path) -> Path:
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(self.report(), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return output

    def write_replay(self, path: str | Path) -> Path:
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        lines = []
        for pose in self.runner.trace:
            lines.append(json.dumps({
                "timestamp_ms": pose.timestamp_ms,
                "x_mm": pose.x_mm,
                "y_mm": pose.y_mm,
                "yaw_deg": pose.yaw_deg,
            }, ensure_ascii=False))
        output.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
        return output


__all__ = ["NavigationJob"]
