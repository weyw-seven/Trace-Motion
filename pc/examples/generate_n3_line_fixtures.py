"""Generate deterministic TRJ2 fixtures for the guarded motion-line profile."""

from __future__ import annotations

import argparse
from pathlib import Path

from pc_trajectory.traj2_format import (
    Trj2CircleRecord,
    Trj2File,
    Trj2Header,
    Trj2LineRecord,
)
from pc_trajectory.traj2_writer import write_trj2


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    header = Trj2Header(0.0, 0.0, 0.0)
    fixtures = {
        "n3_motion_line.traj": Trj2File(
            header,
            (Trj2LineRecord(200.0, 0.0, 60.0, 150.0),),
        ),
        "n3_motion_long.traj": Trj2File(
            header,
            (Trj2LineRecord(600.0, 0.0, 60.0, 150.0),),
        ),
        "n3_motion_circle.traj": Trj2File(
            header,
            (Trj2CircleRecord(120.0, 0.0, 120.0, 180.0, 90.0, 60.0, 150.0),),
        ),
        "n3_motion_multi.traj": Trj2File(
            header,
            (
                Trj2LineRecord(160.0, 0.0, 60.0, 150.0),
                Trj2LineRecord(320.0, 0.0, 60.0, 150.0),
            ),
        ),
        "n3_motion_low_speed.traj": Trj2File(
            header,
            (Trj2LineRecord(200.0, 0.0, 25.0, 150.0),),
        ),
    }

    for name, trajectory in fixtures.items():
        path = write_trj2(trajectory, args.output_dir / name)
        print(f"wrote {path} ({path.stat().st_size} bytes, {trajectory.record_count} records)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
