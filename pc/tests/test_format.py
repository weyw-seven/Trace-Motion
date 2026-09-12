import struct

from pc_trajectory.traj_format import (
    HEADER_FMT,
    HEADER_SIZE,
    RECORD_FMT,
    RECORD_SIZE,
    SegmentType,
    CircleSegment,
    LineSegment,
    TrajectoryHeader,
    pack_header,
    pack_segment,
)


def test_struct_sizes_are_frozen():
    assert struct.calcsize(HEADER_FMT) == 32 == HEADER_SIZE
    assert struct.calcsize(RECORD_FMT) == 44 == RECORD_SIZE


def test_segment_type_values_are_frozen():
    assert int(SegmentType.NONE) == 0
    assert int(SegmentType.LINE) == 1
    assert int(SegmentType.CIRCLE) == 2


def test_header_is_little_endian_and_canonical():
    raw = pack_header(
        TrajectoryHeader(1.0, 2.0, 3.0),
        segment_count=0x01020304,
    )
    assert len(raw) == 32
    assert raw[0:4] == b"TRJ1"
    assert raw[4:6] == b"\x01\x00"
    assert raw[6:8] == b"\x20\x00"
    assert raw[8:12] == b"\x04\x03\x02\x01"
    assert raw[12:14] == b"\x2c\x00"
    assert raw[14:16] == b"\x00\x00"
    assert raw[28:32] == b"\x00\x00\x00\x00"


def test_line_record_is_44_bytes_and_unused_slots_are_zero():
    raw = pack_segment(LineSegment(500.0, 0.0, 300.0, 0.0))
    assert len(raw) == 44
    assert raw[0] == 1
    assert raw[1:4] == b"\x00\x00\x00"

    values = struct.unpack(RECORD_FMT, raw)
    data = values[5:]
    assert data[0] == 500.0
    assert data[1] == 0.0
    assert data[2:] == (0.0,) * 6


def test_circle_record_is_44_bytes_and_unused_slots_are_zero():
    raw = pack_segment(
        CircleSegment(500.0, 250.0, 250.0, -90.0, 180.0, 220.0, 0.0)
    )
    assert len(raw) == 44
    assert raw[0] == 2
    assert raw[1:4] == b"\x00\x00\x00"

    values = struct.unpack(RECORD_FMT, raw)
    data = values[5:]
    assert data[:5] == (500.0, 250.0, 250.0, -90.0, 180.0)
    assert data[5:] == (0.0,) * 3
