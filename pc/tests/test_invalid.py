import math
import struct

import pytest

from pc_trajectory import (
    CircleSegment,
    LineSegment,
    Trajectory,
    TrajectoryFormatError,
    TrajectoryHeader,
    ValidationError,
    decode_traj,
    encode_traj,
)
from pc_trajectory.traj_format import (
    HEADER_FMT,
    RECORD_FMT,
    HEADER_SIZE,
    RECORD_SIZE,
)


def traj_with(segment, *, header=TrajectoryHeader(0.0, 0.0, 0.0)):
    return Trajectory(header, [segment])


@pytest.mark.parametrize(
    "header",
    [
        TrajectoryHeader(math.nan, 0.0, 0.0),
        TrajectoryHeader(0.0, math.inf, 0.0),
        TrajectoryHeader(0.0, 0.0, -math.inf),
    ],
)
def test_reject_nonfinite_header(header):
    with pytest.raises(ValidationError):
        encode_traj(Trajectory(header, []))


@pytest.mark.parametrize("speed", [0.0, -1.0, math.nan, math.inf, -math.inf])
def test_reject_invalid_speed(speed):
    with pytest.raises(ValidationError):
        encode_traj(traj_with(LineSegment(1.0, 0.0, speed, 0.0)))


@pytest.mark.parametrize("acceleration", [-1.0, math.nan, math.inf, -math.inf])
def test_reject_invalid_acceleration(acceleration):
    with pytest.raises(ValidationError):
        encode_traj(
            traj_with(LineSegment(1.0, 0.0, 1.0, acceleration))
        )


@pytest.mark.parametrize(
    "x,y",
    [
        (math.nan, 0.0),
        (0.0, math.inf),
        (-math.inf, 0.0),
    ],
)
def test_reject_nonfinite_line_endpoint(x, y):
    with pytest.raises(ValidationError):
        encode_traj(traj_with(LineSegment(x, y, 100.0)))


def test_reject_zero_length_line():
    with pytest.raises(ValidationError, match="LINE length"):
        encode_traj(traj_with(LineSegment(0.0, 0.0, 100.0)))


@pytest.mark.parametrize("radius", [0.0, -1.0, math.nan, math.inf])
def test_reject_invalid_circle_radius(radius):
    with pytest.raises(ValidationError):
        encode_traj(
            traj_with(
                CircleSegment(
                    0.0, 1.0, radius, -90.0, 90.0, 100.0
                )
            )
        )


@pytest.mark.parametrize("sweep", [0.0, 0.5e-6, -0.5e-6, math.nan, math.inf])
def test_reject_invalid_circle_sweep(sweep):
    with pytest.raises(ValidationError):
        encode_traj(
            traj_with(
                CircleSegment(
                    0.0, 1.0, 1.0, -90.0, sweep, 100.0
                )
            )
        )


def test_reject_circle_arc_too_short_for_executor():
    # Decoder sweep threshold is met, but physical arc length is <= 1e-3 mm.
    with pytest.raises(ValidationError, match="arc length"):
        encode_traj(
            traj_with(
                CircleSegment(
                    0.0,
                    1.0e-3,
                    1.0e-3,
                    -90.0,
                    1.0,
                    100.0,
                )
            )
        )


def test_reject_circle_start_discontinuity():
    trajectory = Trajectory(
        TrajectoryHeader(0.0, 0.0, 0.0),
        [
            CircleSegment(
                10.0,
                10.0,
                1.0,
                0.0,
                90.0,
                100.0,
            )
        ],
    )
    with pytest.raises(ValidationError, match="discontinuous"):
        encode_traj(trajectory)


def make_header(
    *,
    magic=b"TRJ1",
    version=1,
    header_size=32,
    count=0,
    record_size=44,
    flags=0,
    reserved=0,
):
    return struct.pack(
        HEADER_FMT,
        magic,
        version,
        header_size,
        count,
        record_size,
        flags,
        0.0,
        0.0,
        0.0,
        reserved,
    )


@pytest.mark.parametrize(
    "data",
    [
        b"",
        b"TRJ1",
        b"\x00" * 31,
    ],
)
def test_reader_rejects_truncated_header(data):
    with pytest.raises(TrajectoryFormatError):
        decode_traj(data)


def test_reader_rejects_bad_magic():
    with pytest.raises(TrajectoryFormatError, match="bad magic"):
        decode_traj(make_header(magic=b"BAD!"))


def test_reader_rejects_bad_version():
    with pytest.raises(TrajectoryFormatError, match="version"):
        decode_traj(make_header(version=2))


def test_reader_rejects_noncanonical_header_size():
    with pytest.raises(TrajectoryFormatError, match="header_size"):
        decode_traj(make_header(header_size=33))


def test_reader_rejects_noncanonical_record_size():
    with pytest.raises(TrajectoryFormatError, match="record_size"):
        decode_traj(make_header(record_size=45))


def test_reader_rejects_header_flags():
    with pytest.raises(TrajectoryFormatError, match="header flags"):
        decode_traj(make_header(flags=1))


def test_reader_rejects_header_reserved():
    with pytest.raises(TrajectoryFormatError, match="header reserved"):
        decode_traj(make_header(reserved=1))


def test_reader_rejects_truncated_record():
    data = make_header(count=1) + b"\x00" * (RECORD_SIZE - 1)
    with pytest.raises(TrajectoryFormatError, match="truncated"):
        decode_traj(data)


def test_reader_rejects_trailing_bytes():
    data = make_header() + b"\x00"
    with pytest.raises(TrajectoryFormatError, match="trailing"):
        decode_traj(data)


def make_line_record(
    *,
    type_value=1,
    flags=0,
    reserved=0,
    speed=100.0,
    acceleration=0.0,
    end_x=1.0,
    end_y=0.0,
    extra_data=None,
):
    data = [end_x, end_y, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    if extra_data is not None:
        index, value = extra_data
        data[index] = value
    return struct.pack(
        RECORD_FMT,
        type_value,
        flags,
        reserved,
        speed,
        acceleration,
        *data,
    )


def test_reader_rejects_unknown_segment_type():
    data = make_header(count=1) + make_line_record(type_value=99)
    with pytest.raises(TrajectoryFormatError, match="unsupported segment"):
        decode_traj(data)


def test_reader_rejects_none_segment_type():
    data = make_header(count=1) + make_line_record(type_value=0)
    with pytest.raises(TrajectoryFormatError, match="unsupported segment"):
        decode_traj(data)


def test_reader_rejects_record_flags():
    data = make_header(count=1) + make_line_record(flags=1)
    with pytest.raises(TrajectoryFormatError, match="record flags"):
        decode_traj(data)


def test_reader_rejects_record_reserved():
    data = make_header(count=1) + make_line_record(reserved=1)
    with pytest.raises(TrajectoryFormatError, match="record reserved"):
        decode_traj(data)


def test_reader_rejects_nonzero_line_unused_data():
    data = make_header(count=1) + make_line_record(extra_data=(2, 123.0))
    with pytest.raises(TrajectoryFormatError, match="unused"):
        decode_traj(data)


def test_reader_rejects_semantically_invalid_record():
    data = make_header(count=1) + make_line_record(speed=0.0)
    with pytest.raises(TrajectoryFormatError, match="semantics"):
        decode_traj(data)
