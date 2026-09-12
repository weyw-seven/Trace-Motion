"""Generate deterministic TRJ2 fixtures for the safe event-only runner."""

from __future__ import annotations

import argparse
from pathlib import Path

from pc_trajectory.traj2_format import (
    Trj2File,
    Trj2Header,
    Trj2LineRecord,
    Trj2PenDownRecord,
    Trj2PenUpRecord,
    Trj2WaitRecord,
)
from pc_trajectory.traj2_writer import write_trj2


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    header = Trj2Header(0.0, 0.0, 0.0)
    complete = Trj2File(
        header,
        (
            Trj2PenUpRecord(),
            Trj2PenDownRecord(),
            Trj2WaitRecord(0.25),
            Trj2PenUpRecord(),
        ),
    )
    long_wait = Trj2File(
        header,
        (
            Trj2PenUpRecord(),
            Trj2WaitRecord(5.0),
            Trj2PenDownRecord(),
            Trj2WaitRecord(0.25),
            Trj2PenUpRecord(),
        ),
    )
    motion = Trj2File(
        header,
        (Trj2LineRecord(100.0, 0.0, 20.0, 50.0),),
    )

    outputs = {
        "n3_event_done.traj": complete,
        "n3_event_wait.traj": long_wait,
        "n3_event_motion.traj": motion,
    }
    for name, trajectory in outputs.items():
        path = write_trj2(trajectory, args.output_dir / name)
        print(f"wrote {path} ({path.stat().st_size} bytes, {trajectory.record_count} records)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
