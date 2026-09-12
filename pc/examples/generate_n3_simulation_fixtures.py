"""Generate deterministic TRJ2 fixtures for the hardware-free N3 simulator."""

from __future__ import annotations

import argparse
from pathlib import Path

from pc_trajectory.traj2_format import (
    Trj2CircleRecord,
    Trj2CubicBezierRecord,
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
    mixed = Trj2File(
        header,
        (
            Trj2PenUpRecord(),
            Trj2LineRecord(100.0, 0.0, 50.0, 100.0),
            # The circle starts at (100, 0) and ends at (100, 100).
            Trj2CircleRecord(100.0, 50.0, 50.0, -90.0, 180.0, 40.0, 100.0),
            Trj2PenDownRecord(),
            Trj2WaitRecord(0.25),
            Trj2LineRecord(0.0, 100.0, 50.0, 100.0),
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
    long_line = Trj2File(
        header,
        (Trj2LineRecord(300.0, 0.0, 20.0, 40.0),),
    )
    cubic = Trj2File(
        header,
        (
            Trj2CubicBezierRecord(
                25.0,
                0.0,
                75.0,
                100.0,
                100.0,
                100.0,
                30.0,
                60.0,
            ),
        ),
    )

    outputs = {
        "n3_sim_mixed.traj": mixed,
        "n3_sim_wait.traj": long_wait,
        "n3_sim_line.traj": long_line,
        "n3_sim_cubic.traj": cubic,
    }
    for name, trajectory in outputs.items():
        path = write_trj2(trajectory, args.output_dir / name)
        print(f"wrote {path} ({path.stat().st_size} bytes, {trajectory.record_count} records)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
