import math
import struct

import pytest

from pc_trajectory.traj2_format import (
    TRJ2_HEADER_FMT,
    TRJ2_HEADER_SIZE,
    TRJ2_MAGIC,
    TRJ2_RECORD_FMT,
    TRJ2_RECORD_SIZE,
    TRJ2_VERSION,
    Trj2CircleRecord,
    Trj2CubicBezierRecord,
    Trj2Header,
    Trj2LineRecord,
    Trj2PenDownRecord,
    Trj2PenUpRecord,
    Trj2RecordType,
    Trj2WaitRecord,
    header_from_raw,
    pack_header,
    pack_record,
    record_from_raw,
    unpack_header_raw,
    unpack_record_raw,
)


def assert_close(actual, expected, tol=1e-7):
    assert math.isclose(actual, expected, rel_tol=0.0, abs_tol=tol)


def test_struct_sizes_are_frozen():
    assert struct.calcsize(TRJ2_HEADER_FMT) == 32
    assert struct.calcsize(TRJ2_RECORD_FMT) == 44
    assert TRJ2_HEADER_SIZE == 32
    assert TRJ2_RECORD_SIZE == 44


def test_magic_and_version_are_frozen():
    assert TRJ2_MAGIC == b"TRJ2"
    assert TRJ2_VERSION == 2


def test_record_type_values_are_frozen():
    assert int(Trj2RecordType.NONE) == 0x00
    assert int(Trj2RecordType.LINE) == 0x01
    assert int(Trj2RecordType.CIRCLE) == 0x02
    assert int(Trj2RecordType.CUBIC_BEZIER) == 0x03
    assert int(Trj2RecordType.PEN_UP) == 0x20
    assert int(Trj2RecordType.PEN_DOWN) == 0x21
    assert int(Trj2RecordType.WAIT) == 0x22


def test_header_pack_raw_mapping():
    header = Trj2Header(
        start_x_mm=1.25,
        start_y_mm=-2.5,
        start_yaw_deg=30.0,
    )
    data = pack_header(header, record_count=7)

    assert len(data) == 32

    raw = unpack_header_raw(data)

    assert raw.magic == b"TRJ2"
    assert raw.version == 2
    assert raw.header_size == 32
    assert raw.record_count == 7
    assert raw.record_size == 44
    assert raw.flags == 0
    assert raw.reserved == 0
    assert_close(raw.start_x_mm, 1.25)
    assert_close(raw.start_y_mm, -2.5)
    assert_close(raw.start_yaw_deg, 30.0)


def test_header_from_raw_keeps_only_semantic_pose():
    header = Trj2Header(1.0, 2.0, 3.0)
    raw = unpack_header_raw(pack_header(header, record_count=99))

    semantic = header_from_raw(raw)

    assert semantic == header
    assert not hasattr(semantic, "record_count")
    assert not hasattr(semantic, "magic")


def test_line_raw_mapping_and_unused_zero():
    record = Trj2LineRecord(
        end_x_mm=100.0,
        end_y_mm=-25.0,
        speed_mm_s=300.0,
        acceleration_mm_s2=1200.0,
    )

    raw = unpack_record_raw(pack_record(record))

    assert raw.type_value == 0x01
    assert raw.flags == 0
    assert raw.reserved == 0
    assert_close(raw.speed_mm_s, 300.0)
    assert_close(raw.acceleration_mm_s2, 1200.0)
    assert_close(raw.data[0], 100.0)
    assert_close(raw.data[1], -25.0)
    assert raw.data[2:] == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def test_circle_raw_mapping_and_signed_sweep():
    record = Trj2CircleRecord(
        center_x_mm=10.0,
        center_y_mm=20.0,
        radius_mm=30.0,
        start_angle_deg=45.0,
        sweep_deg=-270.0,
        speed_mm_s=80.0,
        acceleration_mm_s2=5.0,
    )

    raw = unpack_record_raw(pack_record(record))

    assert raw.type_value == 0x02
    assert_close(raw.data[0], 10.0)
    assert_close(raw.data[1], 20.0)
    assert_close(raw.data[2], 30.0)
    assert_close(raw.data[3], 45.0)
    assert_close(raw.data[4], -270.0)
    assert raw.data[5:] == (0.0, 0.0, 0.0)


def test_bezier_raw_mapping_is_p1_p2_p3_with_implicit_p0():
    record = Trj2CubicBezierRecord(
        control1_x_mm=10.0,
        control1_y_mm=11.0,
        control2_x_mm=20.0,
        control2_y_mm=21.0,
        end_x_mm=30.0,
        end_y_mm=31.0,
        speed_mm_s=100.0,
        acceleration_mm_s2=7.0,
    )

    raw = unpack_record_raw(pack_record(record))

    assert raw.type_value == 0x03
    assert raw.data == (
        10.0,
        11.0,
        20.0,
        21.0,
        30.0,
        31.0,
        0.0,
        0.0,
    )


@pytest.mark.parametrize(
    "record,type_value",
    [
        (Trj2PenUpRecord(), 0x20),
        (Trj2PenDownRecord(), 0x21),
    ],
)
def test_pen_event_records_have_zero_motion_fields_and_payload(
    record,
    type_value,
):
    raw = unpack_record_raw(pack_record(record))

    assert raw.type_value == type_value
    assert raw.speed_mm_s == 0.0
    assert raw.acceleration_mm_s2 == 0.0
    assert raw.data == (0.0,) * 8


def test_wait_mapping():
    raw = unpack_record_raw(
        pack_record(Trj2WaitRecord(duration_s=0.25))
    )

    assert raw.type_value == 0x22
    assert raw.speed_mm_s == 0.0
    assert raw.acceleration_mm_s2 == 0.0
    assert_close(raw.data[0], 0.25)
    assert raw.data[1:] == (0.0,) * 7


@pytest.mark.parametrize(
    "record",
    [
        Trj2LineRecord(1.0, 2.0, 3.0),
        Trj2CircleRecord(0.0, 0.0, 1.0, 0.0, 90.0, 3.0),
        Trj2CubicBezierRecord(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0
        ),
        Trj2PenUpRecord(),
        Trj2PenDownRecord(),
        Trj2WaitRecord(1.0),
    ],
)
def test_every_record_packs_to_exactly_44_bytes(record):
    assert len(pack_record(record)) == 44


@pytest.mark.parametrize(
    "record",
    [
        Trj2LineRecord(1.0, 2.0, 3.0, 4.0),
        Trj2CircleRecord(0.0, 0.0, 2.0, 0.0, 90.0, 3.0, 4.0),
        Trj2CubicBezierRecord(
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0
        ),
        Trj2PenUpRecord(),
        Trj2PenDownRecord(),
        Trj2WaitRecord(1.5),
    ],
)
def test_pack_raw_record_from_raw_round_trip(record):
    raw = unpack_record_raw(pack_record(record))
    decoded = record_from_raw(raw)

    if isinstance(record, Trj2WaitRecord):
        assert_close(decoded.duration_s, record.duration_s)
    else:
        assert decoded == record


def test_unpack_header_rejects_wrong_buffer_size():
    with pytest.raises(ValueError):
        unpack_header_raw(b"\x00" * 31)


def test_unpack_record_rejects_wrong_buffer_size():
    with pytest.raises(ValueError):
        unpack_record_raw(b"\x00" * 43)


def test_record_from_raw_rejects_none_type():
    raw = unpack_record_raw(
        bytes.fromhex(
            "00000000"
            "00000000"
            "00000000"
            + "00000000" * 8
        )
    )
    with pytest.raises(ValueError):
        record_from_raw(raw)
