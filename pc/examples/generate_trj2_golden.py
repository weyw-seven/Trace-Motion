"""Generate and verify the frozen 340-byte TRJ2 Golden file."""

from pathlib import Path

from pc_trajectory.traj2_format import (
    Trj2File,
    Trj2Header,
    Trj2LineRecord,
    Trj2PenDownRecord,
    Trj2PenUpRecord,
)
from pc_trajectory.traj2_reader import (
    format_trj2,
    read_trj2,
)
from pc_trajectory.traj2_writer import write_trj2


OUTPUT = Path(__file__).resolve().parent / "trj2_golden.traj"


def build_golden() -> Trj2File:
    return Trj2File(
        header=Trj2Header(
            start_x_mm=0.0,
            start_y_mm=0.0,
            start_yaw_deg=0.0,
        ),
        records=(
            Trj2PenDownRecord(),
            Trj2LineRecord(100.0, 0.0, 300.0, 0.0),
            Trj2PenUpRecord(),
            Trj2LineRecord(200.0, 0.0, 500.0, 0.0),
            Trj2PenDownRecord(),
            Trj2LineRecord(300.0, 0.0, 300.0, 0.0),
            Trj2PenUpRecord(),
        ),
    )


def main() -> None:
    file = build_golden()
    write_trj2(file, OUTPUT)

    decoded = read_trj2(OUTPUT)

    assert decoded == file
    assert OUTPUT.stat().st_size == 340

    print(f"Generated : {OUTPUT}")
    print("Records   : 7")
    print("Size      : 340 bytes")
    print("Validation: PASS")
    print()
    print(format_trj2(decoded))


if __name__ == "__main__":
    main()
