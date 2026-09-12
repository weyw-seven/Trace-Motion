from pathlib import Path

from pc_trajectory import (
    CircleSegment,
    LineSegment,
    Trajectory,
    TrajectoryHeader,
    decode_traj,
    encode_traj,
    read_traj,
    write_traj,
)


# Independently frozen fixture: exact expected 164-byte wire image.
# This is deliberately NOT assembled with HEADER_FMT / RECORD_FMT.
GOLDEN_BYTES = bytes.fromhex(
    "54524a3101002000030000002c000000"
    "00000000000000000000000000000000"
    "0100000000009643000000000000fa43"
    "00000000000000000000000000000000"
    "000000000000000000000000"
    "0200000000005c43000000000000fa43"
    "00007a4300007a430000b4c200003443"
    "000000000000000000000000"
    "01000000000096430000000000000000"
    "0000fa43000000000000000000000000"
    "000000000000000000000000"
)


def golden_trajectory():
    return Trajectory(
        header=TrajectoryHeader(0.0, 0.0, 0.0),
        segments=[
            LineSegment(500.0, 0.0, 300.0, 0.0),
            CircleSegment(
                500.0,
                250.0,
                250.0,
                -90.0,
                180.0,
                220.0,
                0.0,
            ),
            LineSegment(0.0, 500.0, 300.0, 0.0),
        ],
    )


def test_golden_fixture_size():
    assert len(GOLDEN_BYTES) == 164


def test_writer_matches_exact_golden_bytes():
    actual = encode_traj(golden_trajectory())
    assert len(actual) == 164
    assert actual == GOLDEN_BYTES


def test_reader_decodes_independent_golden_bytes():
    trajectory = decode_traj(GOLDEN_BYTES)

    assert trajectory.header == TrajectoryHeader(0.0, 0.0, 0.0)
    assert len(trajectory.segments) == 3

    assert trajectory.segments[0] == LineSegment(500.0, 0.0, 300.0, 0.0)
    assert trajectory.segments[1] == CircleSegment(
        500.0, 250.0, 250.0, -90.0, 180.0, 220.0, 0.0
    )
    assert trajectory.segments[2] == LineSegment(0.0, 500.0, 300.0, 0.0)


def test_file_round_trip(tmp_path: Path):
    path = tmp_path / "test.traj"
    original = golden_trajectory()

    write_traj(original, path)

    assert path.stat().st_size == 164
    assert path.read_bytes() == GOLDEN_BYTES
    assert read_traj(path) == original


def test_float32_round_trip_uses_float32_semantics():
    trajectory = Trajectory(
        TrajectoryHeader(0.0, 0.0, 0.0),
        [LineSegment(123.123456789, 5.25, 10.125, 0.0)],
    )
    decoded = decode_traj(encode_traj(trajectory))

    # The encoded coordinate is binary32, not Python's binary64 input value.
    assert decoded.segments[0].end_x_mm != 123.123456789
    assert abs(decoded.segments[0].end_x_mm - 123.123456789) < 1e-4
