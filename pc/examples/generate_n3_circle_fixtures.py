"""Generate deterministic TRJ2 fixtures for the guarded motion-circle profile."""

from __future__ import annotations

import argparse
from pathlib import Path

from pc_trajectory.traj2_format import (
    Trj2CircleRecord,
    Trj2CubicBezierRecord,
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
        "n3_c_line.traj": Trj2File(
            header,
            (Trj2LineRecord(200.0, 0.0, 80.0, 150.0),),
        ),
        # (0, 0) -> (150, 150), positive sweep: vehicle travels left/CCW.
        "n3_c_ccw.traj": Trj2File(
            header,
            (Trj2CircleRecord(0.0, 150.0, 150.0, -90.0, 90.0, 80.0, 150.0),),
        ),
        # (0, 0) -> (150, -150), negative sweep: vehicle travels right/CW.
        "n3_c_cw.traj": Trj2File(
            header,
            (Trj2CircleRecord(0.0, -150.0, 150.0, 90.0, -90.0, 80.0, 150.0),),
        ),
        # Tangent-continuous LINE -> CIRCLE -> LINE, ending at (350, 350).
        "n3_c_mixed.traj": Trj2File(
            header,
            (
                Trj2LineRecord(200.0, 0.0, 80.0, 150.0),
                Trj2CircleRecord(200.0, 150.0, 150.0, -90.0, 90.0, 80.0, 150.0),
                Trj2LineRecord(350.0, 350.0, 80.0, 150.0),
            ),
        ),
        # Geometrically continuous but non-tangent junction, for future barrier tuning.
        "n3_c_sharp.traj": Trj2File(
            header,
            (
                Trj2LineRecord(200.0, 0.0, 80.0, 150.0),
                Trj2CircleRecord(350.0, 0.0, 150.0, 180.0, 90.0, 80.0, 150.0),
            ),
        ),
        "n3_c_small.traj": Trj2File(
            header,
            (Trj2CircleRecord(0.0, 80.0, 80.0, -90.0, 90.0, 80.0, 150.0),),
        ),
        "n3_c_low.traj": Trj2File(
            header,
            (Trj2CircleRecord(0.0, 150.0, 150.0, -90.0, 90.0, 25.0, 150.0),),
        ),
        "n3_c_cubic.traj": Trj2File(
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
