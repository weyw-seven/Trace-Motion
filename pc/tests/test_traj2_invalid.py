import math
import struct

import pytest

from pc_trajectory.traj2_format import (
    TRJ2_HEADER_FMT,
    TRJ2_HEADER_SIZE,
    TRJ2_RECORD_FMT,
    TRJ2_RECORD_SIZE,
    Trj2CircleRecord,
    Trj2CubicBezierRecord,
    Trj2File,
    Trj2Header,
    Trj2LineRecord,
    Trj2PenDownRecord,
    Trj2PenUpRecord,
    Trj2WaitRecord,
)
from pc_trajectory.traj2_reader import (
    Trj2DecodeError,
    decode_trj2,
)
from pc_trajectory.traj2_validate import (
    Trj2ContinuityError,
    Trj2ValidationError,
    validate_record,
    validate_trj2_file,
)
from pc_trajectory.traj2_writer import encode_trj2


GOLDEN = bytes.fromhex(
    "54524a3202002000070000002c0000000000000000000000000000000000000021000000000000000000000000000000000000000000000000000000000000000000000000000000000000000100000000009643000000000000c842000000000000000000000000000000000000000000000000000000002000000000000000000000000000000000000000000000000000000000000000000000000000000000000000010000000000fa43000000000000484300000000000000000000000000000000000000000000000000000000210000000000000000000000000000000000000000000000000000000000000000000000000000000000000001000000000096430000000000009643000000000000000000000000000000000000000000000000000000002000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"
)


def valid_header():
    return Trj2Header(0.0, 0.0, 0.0)


@pytest.mark.parametrize(
    "field",
    ["start_x_mm", "start_y_mm", "start_yaw_deg"],
)
@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_nonfinite_header_pose_is_rejected(field, value):
    kwargs = dict(
        start_x_mm=0.0,
        start_y_mm=0.0,
        start_yaw_deg=0.0,
    )
    kwargs[field] = value

    with pytest.raises(Trj2ValidationError):
        encode_trj2(
            Trj2File(
                header=Trj2Header(**kwargs),
                records=(),
            )
        )


@pytest.mark.parametrize(
    "speed",
    [0.0, -1.0, math.nan, math.inf, -math.inf],
)
def test_line_invalid_speed_rejected(speed):
    record = Trj2LineRecord(
        100.0,
        0.0,
        speed_mm_s=speed,
    )
    with pytest.raises(Trj2ValidationError):
        validate_record(record)


@pytest.mark.parametrize(
    "acceleration",
    [-1.0, math.nan, math.inf, -math.inf],
)
def test_line_invalid_acceleration_rejected(acceleration):
    record = Trj2LineRecord(
        100.0,
        0.0,
        speed_mm_s=100.0,
        acceleration_mm_s2=acceleration,
    )
    with pytest.raises(Trj2ValidationError):
        validate_record(record)


@pytest.mark.parametrize("radius", [0.0, -1.0, math.nan, math.inf])
def test_circle_invalid_radius_rejected(radius):
    record = Trj2CircleRecord(
        0.0,
        0.0,
        radius,
        0.0,
        90.0,
        100.0,
    )
    with pytest.raises(Trj2ValidationError):
        validate_record(record)


@pytest.mark.parametrize(
    "sweep",
    [0.0, 1e-7, -1e-7, math.nan, math.inf],
)
def test_circle_invalid_sweep_rejected(sweep):
    record = Trj2CircleRecord(
        0.0,
        0.0,
        1.0,
        0.0,
        sweep,
        100.0,
    )
    with pytest.raises(Trj2ValidationError):
        validate_record(record)


@pytest.mark.parametrize(
    "duration",
    [0.0, -1.0, math.nan, math.inf, -math.inf],
)
def test_wait_invalid_duration_rejected(duration):
    with pytest.raises(Trj2ValidationError):
        validate_record(Trj2WaitRecord(duration))


def test_zero_length_line_rejected_in_file_context():
    file = Trj2File(
        valid_header(),
        (
            Trj2LineRecord(
                0.0,
                0.0,
                speed_mm_s=100.0,
            ),
        ),
    )

    with pytest.raises(Trj2ValidationError):
        validate_trj2_file(file)


def test_circle_discontinuity_rejected():
    file = Trj2File(
        valid_header(),
        (
            Trj2CircleRecord(
                center_x_mm=100.0,
                center_y_mm=100.0,
                radius_mm=10.0,
                start_angle_deg=0.0,
                sweep_deg=90.0,
                speed_mm_s=100.0,
            ),
        ),
    )

    with pytest.raises(Trj2ContinuityError):
        validate_trj2_file(file)


def test_event_does_not_change_logical_xy_before_circle():
    file = Trj2File(
        valid_header(),
        (
            Trj2PenDownRecord(),
            Trj2WaitRecord(0.1),
            Trj2PenUpRecord(),
            Trj2CircleRecord(
                center_x_mm=0.0,
                center_y_mm=10.0,
                radius_mm=10.0,
                start_angle_deg=-90.0,
                sweep_deg=90.0,
                speed_mm_s=100.0,
            ),
        ),
    )

    validate_trj2_file(file)


def test_fully_collapsed_bezier_rejected():
    file = Trj2File(
        valid_header(),
        (
            Trj2CubicBezierRecord(
                control1_x_mm=0.0,
                control1_y_mm=0.0,
                control2_x_mm=0.0,
                control2_y_mm=0.0,
                end_x_mm=0.0,
                end_y_mm=0.0,
                speed_mm_s=100.0,
            ),
        ),
    )

    with pytest.raises(Trj2ValidationError):
        validate_trj2_file(file)


def test_closed_noncollapsed_bezier_is_valid():
    file = Trj2File(
        valid_header(),
        (
            Trj2CubicBezierRecord(
                control1_x_mm=100.0,
                control1_y_mm=0.0,
                control2_x_mm=100.0,
                control2_y_mm=100.0,
                end_x_mm=0.0,
                end_y_mm=0.0,
                speed_mm_s=100.0,
            ),
        ),
    )

    validate_trj2_file(file)


def mutate_header(blob: bytes, field_index: int, value) -> bytes:
    fields = list(struct.unpack(
        TRJ2_HEADER_FMT,
        blob[:TRJ2_HEADER_SIZE],
    ))
    fields[field_index] = value
    return (
        struct.pack(TRJ2_HEADER_FMT, *fields)
        + blob[TRJ2_HEADER_SIZE:]
    )


def mutate_record(
    blob: bytes,
    record_index: int,
    field_index: int,
    value,
) -> bytes:
    offset = TRJ2_HEADER_SIZE + record_index * TRJ2_RECORD_SIZE
    fields = list(struct.unpack(
        TRJ2_RECORD_FMT,
        blob[offset:offset + TRJ2_RECORD_SIZE],
    ))
    fields[field_index] = value
    record = struct.pack(TRJ2_RECORD_FMT, *fields)
    return blob[:offset] + record + blob[offset + TRJ2_RECORD_SIZE:]


def test_truncated_header_rejected():
    with pytest.raises(Trj2DecodeError):
        decode_trj2(GOLDEN[:31])


def test_truncated_record_rejected():
    with pytest.raises(Trj2DecodeError):
        decode_trj2(GOLDEN[:-1])


def test_extra_trailing_byte_rejected():
    with pytest.raises(Trj2DecodeError):
        decode_trj2(GOLDEN + b"\x00")


def test_bad_magic_rejected():
    broken = mutate_header(GOLDEN, 0, b"BAD!")
    with pytest.raises(Trj2DecodeError):
        decode_trj2(broken)


def test_bad_version_rejected():
    broken = mutate_header(GOLDEN, 1, 3)
    with pytest.raises(Trj2DecodeError):
        decode_trj2(broken)


def test_noncanonical_header_size_rejected():
    broken = mutate_header(GOLDEN, 2, 33)
    with pytest.raises(Trj2DecodeError):
        decode_trj2(broken)


def test_record_count_mismatch_rejected():
    broken = mutate_header(GOLDEN, 3, 6)
    with pytest.raises(Trj2DecodeError):
        decode_trj2(broken)


def test_noncanonical_record_size_rejected():
    broken = mutate_header(GOLDEN, 4, 45)
    with pytest.raises(Trj2DecodeError):
        decode_trj2(broken)


def test_nonzero_header_flags_rejected():
    broken = mutate_header(GOLDEN, 5, 1)
    with pytest.raises(Trj2DecodeError):
        decode_trj2(broken)


def test_nonzero_header_reserved_rejected():
    broken = mutate_header(GOLDEN, 9, 1)
    with pytest.raises(Trj2DecodeError):
        decode_trj2(broken)


def test_unknown_record_type_rejected():
    broken = mutate_record(GOLDEN, 0, 0, 0x7F)
    with pytest.raises(Trj2DecodeError):
        decode_trj2(broken)


def test_none_record_type_rejected():
    broken = mutate_record(GOLDEN, 0, 0, 0x00)
    with pytest.raises(Trj2DecodeError):
        decode_trj2(broken)


def test_nonzero_record_flags_rejected():
    broken = mutate_record(GOLDEN, 1, 1, 1)
    with pytest.raises(Trj2DecodeError):
        decode_trj2(broken)


def test_nonzero_record_reserved_rejected():
    broken = mutate_record(GOLDEN, 1, 2, 1)
    with pytest.raises(Trj2DecodeError):
        decode_trj2(broken)


def test_event_nonzero_speed_rejected():
    broken = mutate_record(GOLDEN, 0, 3, 1.0)
    with pytest.raises(Trj2DecodeError):
        decode_trj2(broken)


def test_event_nonzero_acceleration_rejected():
    broken = mutate_record(GOLDEN, 0, 4, 1.0)
    with pytest.raises(Trj2DecodeError):
        decode_trj2(broken)


def test_pen_event_nonzero_payload_rejected():
    # field index 5 is data[0]
    broken = mutate_record(GOLDEN, 0, 5, 1.0)
    with pytest.raises(Trj2DecodeError):
        decode_trj2(broken)


def test_line_nonzero_unused_payload_rejected():
    # LINE data[2] => struct field index 7
    broken = mutate_record(GOLDEN, 1, 7, 123.0)
    with pytest.raises(Trj2DecodeError):
        decode_trj2(broken)


def test_raw_nan_speed_rejected():
    broken = mutate_record(GOLDEN, 1, 3, math.nan)
    with pytest.raises(Trj2DecodeError):
        decode_trj2(broken)


def test_raw_inf_payload_rejected():
    broken = mutate_record(GOLDEN, 1, 5, math.inf)
    with pytest.raises(Trj2DecodeError):
        decode_trj2(broken)


def test_wait_unused_payload_rejected():
    # Replace record 0 with WAIT and deliberately dirty data[1].
    offset = TRJ2_HEADER_SIZE
    wait = struct.pack(
        TRJ2_RECORD_FMT,
        0x22,
        0,
        0,
        0.0,
        0.0,
        0.5,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    broken = GOLDEN[:offset] + wait + GOLDEN[offset + 44:]

    with pytest.raises(Trj2DecodeError):
        decode_trj2(broken)


def test_circle_unused_payload_rejected():
    file = Trj2File(
        valid_header(),
        (
            Trj2CircleRecord(
                0.0, 10.0, 10.0, -90.0, 90.0, 100.0
            ),
        ),
    )
    blob = bytearray(encode_trj2(file))

    # record 0 data[5] field index = 10
    broken = mutate_record(bytes(blob), 0, 10, 1.0)

    with pytest.raises(Trj2DecodeError):
        decode_trj2(broken)


def test_bezier_unused_payload_rejected():
    file = Trj2File(
        valid_header(),
        (
            Trj2CubicBezierRecord(
                10.0, 0.0,
                20.0, 10.0,
                30.0, 10.0,
                100.0,
            ),
        ),
    )
    blob = encode_trj2(file)

    # record 0 data[6] field index = 11
    broken = mutate_record(blob, 0, 11, 1.0)

    with pytest.raises(Trj2DecodeError):
        decode_trj2(broken)


def test_empty_trj2_file_is_canonically_valid():
    file = Trj2File(valid_header(), ())

    blob = encode_trj2(file)

    assert len(blob) == 32
    assert decode_trj2(blob) == file
