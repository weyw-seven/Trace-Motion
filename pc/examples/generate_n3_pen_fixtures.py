"""Generate deterministic TRJ2 fixtures for the guarded physical-pen profile."""

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
    fixtures = {
        "n3_pen_only.traj": Trj2File(
            header,
            (
                Trj2PenUpRecord(),
                Trj2WaitRecord(0.5),
                Trj2PenDownRecord(),
                Trj2WaitRecord(0.8),
                Trj2PenUpRecord(),
                Trj2WaitRecord(0.5),
            ),
        ),
        # The first 200 mm is a pen-up travel move.  The second 200 mm is the
        # physical drawing move and remains above the project's speed floor.
        "n3_pen_line.traj": Trj2File(
            header,
            (
                Trj2PenUpRecord(),
                Trj2LineRecord(200.0, 0.0, 80.0, 150.0),
                Trj2PenDownRecord(),
                Trj2LineRecord(200.0, 200.0, 80.0, 150.0),
                Trj2PenUpRecord(),
            ),
        ),
        # Pen-down CIRCLE with radius 150 mm, followed by a 150 mm line.
        "n3_pen_mixed.traj": Trj2File(
            header,
            (
                Trj2PenUpRecord(),
                Trj2LineRecord(200.0, 0.0, 80.0, 150.0),
                Trj2PenDownRecord(),
                Trj2CircleRecord(200.0, 150.0, 150.0, -90.0, 90.0, 80.0, 150.0),
                Trj2LineRecord(350.0, 300.0, 80.0, 150.0),
                Trj2PenUpRecord(),
            ),
        ),
        # A long WAIT makes it possible to issue STOP/ESTOP while the pen is
        # physically DOWN and verify that the terminal path raises it.
        "n3_pen_hold_down.traj": Trj2File(
            header,
            (
                Trj2PenUpRecord(),
                Trj2PenDownRecord(),
                Trj2WaitRecord(5.0),
                Trj2PenUpRecord(),
            ),
        ),
        "n3_pen_cubic_reject.traj": Trj2File(
            header,
            (Trj2CubicBezierRecord(50.0, 0.0, 100.0, 100.0, 150.0, 0.0, 80.0, 150.0),),
        ),
    }

    for name, trajectory in fixtures.items():
        path = write_trj2(trajectory, args.output_dir / name)
        print(f"wrote {path} ({path.stat().st_size} bytes, {trajectory.record_count} records)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
