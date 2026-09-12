"""Review statistics for the exact TRJ2 bytes produced by a navigation plan."""

from __future__ import annotations

from dataclasses import dataclass
import math

from ..traj2_format import (
    Trj2CircleRecord,
    Trj2CubicBezierRecord,
    Trj2LineRecord,
    Trj2PenDownRecord,
    Trj2PenUpRecord,
    Trj2WaitRecord,
)
from ..traj2_reader import decode_trj2


@dataclass(frozen=True)
class ToolpathReview:
    """Counts and geometric checks for one canonical TRJ2 payload."""

    payload_size: int
    record_count: int
    line_count: int
    circle_count: int
    cubic_count: int
    pen_up_count: int
    pen_down_count: int
    wait_count: int
    circle_lengths_mm: tuple[float, ...]

    @property
    def min_circle_length_mm(self) -> float | None:
        return min(self.circle_lengths_mm) if self.circle_lengths_mm else None

    @property
    def max_circle_length_mm(self) -> float | None:
        return max(self.circle_lengths_mm) if self.circle_lengths_mm else None

    def warning_messages(
        self,
        *,
        max_circles: int = 200,
        min_circle_length_mm: float = 5.0,
    ) -> tuple[str, ...]:
        warnings: list[str] = []
        if max_circles > 0 and self.circle_count > max_circles:
            warnings.append(f"CIRCLE count {self.circle_count} exceeds soft warning {max_circles}")
        short_count = sum(length < min_circle_length_mm for length in self.circle_lengths_mm)
        if min_circle_length_mm > 0.0 and short_count:
            warnings.append(f"{short_count} CIRCLE records are shorter than {min_circle_length_mm:.1f} mm")
        return tuple(warnings)


def review_trj2(payload: bytes) -> ToolpathReview:
    """Decode and count the exact bytes that will be sent to the ESP32."""

    data = bytes(payload)
    file = decode_trj2(data)
    circles = tuple(record for record in file.records if isinstance(record, Trj2CircleRecord))
    circle_lengths = tuple(
        abs(math.radians(record.sweep_deg) * record.radius_mm)
        for record in circles
    )
    return ToolpathReview(
        payload_size=len(data),
        record_count=file.record_count,
        line_count=sum(isinstance(record, Trj2LineRecord) for record in file.records),
        circle_count=len(circles),
        cubic_count=sum(isinstance(record, Trj2CubicBezierRecord) for record in file.records),
        pen_up_count=sum(isinstance(record, Trj2PenUpRecord) for record in file.records),
        pen_down_count=sum(isinstance(record, Trj2PenDownRecord) for record in file.records),
        wait_count=sum(isinstance(record, Trj2WaitRecord) for record in file.records),
        circle_lengths_mm=circle_lengths,
    )


__all__ = ["ToolpathReview", "review_trj2"]
