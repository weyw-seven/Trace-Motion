import math

import pytest

import pc_trajectory.toolpath_trj2_export as bridge
from pc_trajectory.drawing import Drawing, Stroke
from pc_trajectory.geometry import (
    Arc,
    CubicBezier,
    Line,
    Point2D,
)
from pc_trajectory.toolpath import (
    Motion,
    PenDown,
    PenUp,
    Toolpath,
    Wait,
)
from pc_trajectory.toolpath_compiler import (
    ToolpathCompilerConfig,
    compile_drawing,
)
from pc_trajectory.toolpath_trj2_export import (
    ToolpathTrj2ExportError,
    Trj2ExportContinuityError,
    Trj2ExportStartPoseError,
    encode_toolpath_trj2,
    motion_to_trj2_record,
    toolpath_record_to_trj2_record,
    toolpath_to_trj2_file,
    write_toolpath_trj2,
)
from pc_trajectory.traj2_format import (
    Trj2CircleRecord,
    Trj2CubicBezierRecord,
    Trj2File,
    Trj2LineRecord,
    Trj2PenDownRecord,
    Trj2PenUpRecord,
    Trj2WaitRecord,
)
from pc_trajectory.traj2_reader import decode_trj2
from pc_trajectory.traj2_writer import encode_trj2


GOLDEN_TRJ2_BYTES = bytes.fromhex(
    "54524a3202002000070000002c0000000000000000000000000000000000000021000000000000000000000000000000000000000000000000000000000000000000000000000000000000000100000000009643000000000000c842000000000000000000000000000000000000000000000000000000002000000000000000000000000000000000000000000000000000000000000000000000000000000000000000010000000000fa43000000000000484300000000000000000000000000000000000000000000000000000000210000000000000000000000000000000000000000000000000000000000000000000000000000000000000001000000000096430000000000009643000000000000000000000000000000000000000000000000000000002000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"
)


ABS = 1.0e-9


def assert_close(actual, expected, tol=ABS):
    assert math.isclose(actual, expected, rel_tol=0.0, abs_tol=tol)


def line_motion(
    start,
    end,
    *,
    speed=300.0,
    acceleration=0.0,
):
    return Motion(
        Line(start, end),
        speed_mm_s=speed,
        acceleration_mm_s2=acceleration,
    )


def two_stroke_drawing():
    stroke0 = Stroke(
        (
            Line(
                Point2D(0.0, 0.0),
                Point2D(100.0, 0.0),
            ),
        )
    )
    stroke1 = Stroke(
        (
            Line(
                Point2D(200.0, 0.0),
                Point2D(300.0, 0.0),
            ),
        )
    )
    return Drawing((stroke0, stroke1))


def test_frozen_golden_fixture_is_340_bytes():
    assert len(GOLDEN_TRJ2_BYTES) == 340
    assert len(GOLDEN_TRJ2_BYTES) == 32 + 7 * 44


def test_line_motion_maps_to_trj2_line_record():
    motion = line_motion(
        Point2D(10.0, 20.0),
        Point2D(30.0, 40.0),
        speed=123.0,
        acceleration=456.0,
    )

    record = motion_to_trj2_record(motion)

    assert isinstance(record, Trj2LineRecord)
    assert_close(record.end_x_mm, 30.0)
    assert_close(record.end_y_mm, 40.0)
    assert_close(record.speed_mm_s, 123.0)
    assert_close(record.acceleration_mm_s2, 456.0)


def test_line_record_does_not_store_explicit_phase_b_start():
    motion = line_motion(
        Point2D(10.0, 20.0),
        Point2D(30.0, 40.0),
    )

    record = motion_to_trj2_record(motion)

    assert not hasattr(record, "start_x_mm")
    assert not hasattr(record, "start_y_mm")


def test_arc_motion_maps_without_reparameterization():
    motion = Motion(
        Arc(
            center=Point2D(500.0, 250.0),
            radius_mm=250.0,
            start_angle_deg=-90.0,
            sweep_deg=180.0,
        ),
        speed_mm_s=220.0,
        acceleration_mm_s2=12.0,
    )

    record = motion_to_trj2_record(motion)

    assert isinstance(record, Trj2CircleRecord)
    assert_close(record.center_x_mm, 500.0)
    assert_close(record.center_y_mm, 250.0)
    assert_close(record.radius_mm, 250.0)
    assert_close(record.start_angle_deg, -90.0)
    assert_close(record.sweep_deg, 180.0)
    assert_close(record.speed_mm_s, 220.0)
    assert_close(record.acceleration_mm_s2, 12.0)


def test_negative_arc_sweep_is_preserved():
    motion = Motion(
        Arc(
            center=Point2D(0.0, 0.0),
            radius_mm=10.0,
            start_angle_deg=90.0,
            sweep_deg=-135.0,
        ),
        speed_mm_s=50.0,
    )

    record = motion_to_trj2_record(motion)

    assert isinstance(record, Trj2CircleRecord)
    assert_close(record.sweep_deg, -135.0)


def test_bezier_motion_maps_p1_p2_p3_and_omits_p0():
    curve = CubicBezier(
        start=Point2D(1.0, 2.0),
        control1=Point2D(10.0, 11.0),
        control2=Point2D(20.0, 21.0),
        end=Point2D(30.0, 31.0),
    )
    motion = Motion(
        curve,
        speed_mm_s=100.0,
        acceleration_mm_s2=7.0,
    )

    record = motion_to_trj2_record(motion)

    assert isinstance(record, Trj2CubicBezierRecord)
    assert_close(record.control1_x_mm, 10.0)
    assert_close(record.control1_y_mm, 11.0)
    assert_close(record.control2_x_mm, 20.0)
    assert_close(record.control2_y_mm, 21.0)
    assert_close(record.end_x_mm, 30.0)
    assert_close(record.end_y_mm, 31.0)
    assert_close(record.speed_mm_s, 100.0)
    assert_close(record.acceleration_mm_s2, 7.0)

    assert not hasattr(record, "start_x_mm")
    assert not hasattr(record, "start_y_mm")


def test_motion_to_record_rejects_non_motion():
    with pytest.raises(ToolpathTrj2ExportError):
        motion_to_trj2_record("not motion")  # type: ignore[arg-type]


@pytest.mark.parametrize(
    "tool_record,trj2_type",
    [
        (PenUp(), Trj2PenUpRecord),
        (PenDown(), Trj2PenDownRecord),
        (Wait(0.25), Trj2WaitRecord),
    ],
)
def test_event_mapping(tool_record, trj2_type):
    record = toolpath_record_to_trj2_record(tool_record)

    assert isinstance(record, trj2_type)

    if isinstance(record, Trj2WaitRecord):
        assert_close(record.duration_s, 0.25)


def test_record_order_is_preserved_exactly():
    motion = line_motion(
        Point2D(0.0, 0.0),
        Point2D(100.0, 0.0),
    )
    toolpath = Toolpath(
        (
            PenDown(),
            motion,
            Wait(0.1),
            PenUp(),
        )
    )

    file = toolpath_to_trj2_file(toolpath)

    assert [
        type(record)
        for record in file.records
    ] == [
        Trj2PenDownRecord,
        Trj2LineRecord,
        Trj2WaitRecord,
        Trj2PenUpRecord,
    ]


def test_header_xy_is_derived_from_first_motion_even_with_leading_events():
    motion = line_motion(
        Point2D(123.5, -44.25),
        Point2D(200.0, 0.0),
    )
    toolpath = Toolpath(
        (
            PenDown(),
            Wait(0.1),
            motion,
            PenUp(),
        )
    )

    file = toolpath_to_trj2_file(
        toolpath,
        start_yaw_deg=17.0,
    )

    assert_close(file.header.start_x_mm, 123.5)
    assert_close(file.header.start_y_mm, -44.25)
    assert_close(file.header.start_yaw_deg, 17.0)


def test_start_yaw_is_not_inferred_from_first_motion_tangent():
    motion = line_motion(
        Point2D(0.0, 0.0),
        Point2D(0.0, 100.0),
    )

    file = toolpath_to_trj2_file(
        Toolpath((motion,)),
        start_yaw_deg=0.0,
    )

    assert_close(file.header.start_yaw_deg, 0.0)


@pytest.mark.parametrize("yaw", [math.nan, math.inf, -math.inf])
def test_nonfinite_start_yaw_is_rejected(yaw):
    motion = line_motion(
        Point2D(0.0, 0.0),
        Point2D(100.0, 0.0),
    )

    with pytest.raises(ToolpathTrj2ExportError):
        toolpath_to_trj2_file(
            Toolpath((motion,)),
            start_yaw_deg=yaw,
        )


def test_explicit_start_point_matching_first_motion_is_accepted():
    motion = line_motion(
        Point2D(10.0, 20.0),
        Point2D(100.0, 20.0),
    )

    file = toolpath_to_trj2_file(
        Toolpath((motion,)),
        start_point=Point2D(10.0, 20.0),
    )

    assert_close(file.header.start_x_mm, 10.0)
    assert_close(file.header.start_y_mm, 20.0)


def test_explicit_start_point_is_only_assertion_not_geometry_override():
    motion = line_motion(
        Point2D(10.0, 20.0),
        Point2D(100.0, 20.0),
    )
    toolpath = Toolpath((motion,))

    # 0.005 mm assertion mismatch is within the default 0.01 mm tolerance.
    file = toolpath_to_trj2_file(
        toolpath,
        start_point=Point2D(10.005, 20.0),
    )

    # Header still uses the actual Phase-B Motion start, not the asserted point.
    assert_close(file.header.start_x_mm, 10.0)
    assert_close(file.header.start_y_mm, 20.0)


def test_explicit_start_point_mismatch_above_tolerance_is_rejected():
    motion = line_motion(
        Point2D(10.0, 20.0),
        Point2D(100.0, 20.0),
    )

    with pytest.raises(Trj2ExportStartPoseError):
        toolpath_to_trj2_file(
            Toolpath((motion,)),
            start_point=Point2D(10.02, 20.0),
        )


def test_invalid_explicit_start_point_type_is_rejected():
    motion = line_motion(
        Point2D(0.0, 0.0),
        Point2D(100.0, 0.0),
    )

    with pytest.raises(Trj2ExportStartPoseError):
        toolpath_to_trj2_file(
            Toolpath((motion,)),
            start_point=(0.0, 0.0),  # type: ignore[arg-type]
        )


def test_empty_toolpath_requires_explicit_start_point():
    with pytest.raises(Trj2ExportStartPoseError):
        toolpath_to_trj2_file(Toolpath())


def test_empty_toolpath_with_start_point_exports_32_byte_trj2():
    toolpath = Toolpath()

    file = toolpath_to_trj2_file(
        toolpath,
        start_point=Point2D(12.0, 34.0),
        start_yaw_deg=56.0,
    )
    data = encode_toolpath_trj2(
        toolpath,
        start_point=Point2D(12.0, 34.0),
        start_yaw_deg=56.0,
    )

    assert file.record_count == 0
    assert_close(file.header.start_x_mm, 12.0)
    assert_close(file.header.start_y_mm, 34.0)
    assert_close(file.header.start_yaw_deg, 56.0)
    assert len(data) == 32


def test_event_only_toolpath_requires_start_point():
    toolpath = Toolpath(
        (
            PenDown(),
            Wait(0.25),
            PenUp(),
        )
    )

    with pytest.raises(Trj2ExportStartPoseError):
        toolpath_to_trj2_file(toolpath)


def test_event_only_toolpath_with_start_point_preserves_events():
    toolpath = Toolpath(
        (
            PenDown(),
            Wait(0.25),
            PenUp(),
        )
    )

    file = toolpath_to_trj2_file(
        toolpath,
        start_point=Point2D(50.0, 60.0),
    )

    assert file.record_count == 3
    assert isinstance(file.records[0], Trj2PenDownRecord)
    assert isinstance(file.records[1], Trj2WaitRecord)
    assert isinstance(file.records[2], Trj2PenUpRecord)
    assert_close(file.header.start_x_mm, 50.0)
    assert_close(file.header.start_y_mm, 60.0)


def test_events_do_not_change_logical_xy_during_export():
    motion0 = line_motion(
        Point2D(0.0, 0.0),
        Point2D(100.0, 0.0),
    )
    motion1 = Motion(
        Arc(
            center=Point2D(100.0, 10.0),
            radius_mm=10.0,
            start_angle_deg=-90.0,
            sweep_deg=90.0,
        ),
        speed_mm_s=100.0,
    )

    toolpath = Toolpath(
        (
            PenDown(),
            motion0,
            PenUp(),
            Wait(0.1),
            PenDown(),
            motion1,
            PenUp(),
        )
    )

    file = toolpath_to_trj2_file(toolpath)

    assert file.record_count == 7
    assert isinstance(file.records[5], Trj2CircleRecord)


def test_export_rechecks_explicit_motion_continuity_with_stricter_override():
    motion0 = line_motion(
        Point2D(0.0, 0.0),
        Point2D(100.0, 0.0),
    )
    motion1 = line_motion(
        Point2D(100.5, 0.0),
        Point2D(200.0, 0.0),
    )

    # Toolpath itself accepts because its configured tolerance is loose.
    toolpath = Toolpath(
        (motion0, motion1),
        continuity_tolerance_mm=1.0,
    )

    with pytest.raises(Trj2ExportContinuityError) as exc_info:
        toolpath_to_trj2_file(
            toolpath,
            continuity_tolerance_mm=0.01,
        )

    error = exc_info.value
    assert error.record_index == 1
    assert error.motion_index == 1
    assert_close(error.position_error_mm, 0.5)
    assert_close(error.tolerance_mm, 0.01)


def test_default_export_uses_toolpath_continuity_tolerance():
    motion0 = line_motion(
        Point2D(0.0, 0.0),
        Point2D(100.0, 0.0),
    )
    motion1 = line_motion(
        Point2D(100.5, 0.0),
        Point2D(200.0, 0.0),
    )
    toolpath = Toolpath(
        (motion0, motion1),
        continuity_tolerance_mm=1.0,
    )

    file = toolpath_to_trj2_file(toolpath)

    assert file.record_count == 2


@pytest.mark.parametrize("tol", [-0.1, math.nan, math.inf, -math.inf])
def test_invalid_export_continuity_tolerance_rejected(tol):
    motion = line_motion(
        Point2D(0.0, 0.0),
        Point2D(100.0, 0.0),
    )

    with pytest.raises(ToolpathTrj2ExportError):
        toolpath_to_trj2_file(
            Toolpath((motion,)),
            continuity_tolerance_mm=tol,
        )


def test_toolpath_to_file_rejects_non_toolpath():
    with pytest.raises(ToolpathTrj2ExportError):
        toolpath_to_trj2_file("not toolpath")  # type: ignore[arg-type]


def test_b4_two_stroke_compiler_maps_to_exact_expected_record_types():
    toolpath = compile_drawing(two_stroke_drawing())

    file = toolpath_to_trj2_file(toolpath)

    assert [
        type(record)
        for record in file.records
    ] == [
        Trj2PenDownRecord,
        Trj2LineRecord,
        Trj2PenUpRecord,
        Trj2LineRecord,
        Trj2PenDownRecord,
        Trj2LineRecord,
        Trj2PenUpRecord,
    ]


def test_b4_two_stroke_draw_and_travel_speeds_survive_lowering():
    toolpath = compile_drawing(two_stroke_drawing())

    file = toolpath_to_trj2_file(toolpath)

    assert_close(file.records[1].speed_mm_s, 300.0)
    assert_close(file.records[3].speed_mm_s, 500.0)
    assert_close(file.records[5].speed_mm_s, 300.0)


def test_custom_compiler_accelerations_survive_lowering():
    config = ToolpathCompilerConfig(
        drawing_speed_mm_s=250.0,
        drawing_acceleration_mm_s2=111.0,
        travel_speed_mm_s=700.0,
        travel_acceleration_mm_s2=222.0,
    )
    toolpath = compile_drawing(
        two_stroke_drawing(),
        config,
    )

    file = toolpath_to_trj2_file(toolpath)

    assert_close(file.records[1].speed_mm_s, 250.0)
    assert_close(file.records[1].acceleration_mm_s2, 111.0)

    assert_close(file.records[3].speed_mm_s, 700.0)
    assert_close(file.records[3].acceleration_mm_s2, 222.0)

    assert_close(file.records[5].speed_mm_s, 250.0)
    assert_close(file.records[5].acceleration_mm_s2, 111.0)


def test_full_drawing_to_toolpath_to_trj2_matches_b6_340_byte_golden():
    toolpath = compile_drawing(two_stroke_drawing())

    actual = encode_toolpath_trj2(
        toolpath,
        start_yaw_deg=0.0,
    )

    assert len(actual) == 340
    assert actual == GOLDEN_TRJ2_BYTES


def test_b7_bytes_decode_with_b6_reader_to_same_semantics():
    toolpath = compile_drawing(two_stroke_drawing())

    decoded = decode_trj2(
        encode_toolpath_trj2(toolpath)
    )

    assert decoded.record_count == 7
    assert isinstance(decoded.records[0], Trj2PenDownRecord)
    assert isinstance(decoded.records[1], Trj2LineRecord)
    assert isinstance(decoded.records[2], Trj2PenUpRecord)
    assert isinstance(decoded.records[3], Trj2LineRecord)
    assert isinstance(decoded.records[4], Trj2PenDownRecord)
    assert isinstance(decoded.records[5], Trj2LineRecord)
    assert isinstance(decoded.records[6], Trj2PenUpRecord)


def test_encode_bridge_delegates_binary_serialization_to_b6(monkeypatch):
    sentinel = b"B6-writer-called"
    captured = {}

    def fake_encode(file):
        captured["file"] = file
        return sentinel

    monkeypatch.setattr(bridge, "encode_trj2", fake_encode)

    toolpath = compile_drawing(two_stroke_drawing())
    actual = bridge.encode_toolpath_trj2(toolpath)

    assert actual == sentinel
    assert isinstance(captured["file"], Trj2File)


def test_write_bridge_delegates_file_output_to_b6(monkeypatch, tmp_path):
    output = tmp_path / "out.traj"
    captured = {}

    def fake_write(file, path):
        captured["file"] = file
        captured["path"] = path
        return path

    monkeypatch.setattr(bridge, "write_trj2", fake_write)

    toolpath = compile_drawing(two_stroke_drawing())
    returned = bridge.write_toolpath_trj2(toolpath, output)

    assert returned == output
    assert isinstance(captured["file"], Trj2File)
    assert captured["path"] == output


def test_write_toolpath_trj2_writes_exact_340_byte_golden(tmp_path):
    output = tmp_path / "two_stroke.traj"
    toolpath = compile_drawing(two_stroke_drawing())

    returned = write_toolpath_trj2(
        toolpath,
        output,
        start_yaw_deg=0.0,
    )

    assert returned == output
    assert output.stat().st_size == 340
    assert output.read_bytes() == GOLDEN_TRJ2_BYTES


def test_bezier_toolpath_can_be_encoded_and_read_by_b6():
    curve = CubicBezier(
        start=Point2D(0.0, 0.0),
        control1=Point2D(50.0, 0.0),
        control2=Point2D(50.0, 100.0),
        end=Point2D(100.0, 100.0),
    )
    toolpath = Toolpath(
        (
            Motion(
                curve,
                speed_mm_s=150.0,
                acceleration_mm_s2=10.0,
            ),
        )
    )

    decoded = decode_trj2(
        encode_toolpath_trj2(toolpath)
    )

    assert decoded.record_count == 1
    record = decoded.records[0]
    assert isinstance(record, Trj2CubicBezierRecord)
    assert_close(record.control1_x_mm, 50.0)
    assert_close(record.control2_y_mm, 100.0)
    assert_close(record.end_x_mm, 100.0)
    assert_close(record.end_y_mm, 100.0)


def test_wait_duration_survives_binary_round_trip():
    motion = line_motion(
        Point2D(0.0, 0.0),
        Point2D(100.0, 0.0),
    )
    toolpath = Toolpath(
        (
            PenDown(),
            motion,
            Wait(0.375),
            PenUp(),
        )
    )

    decoded = decode_trj2(
        encode_toolpath_trj2(toolpath)
    )

    wait = decoded.records[2]
    assert isinstance(wait, Trj2WaitRecord)
    assert_close(wait.duration_s, 0.375, tol=1e-7)


def test_same_motion_type_used_for_draw_and_travel():
    toolpath = compile_drawing(two_stroke_drawing())
    file = toolpath_to_trj2_file(toolpath)

    # Draw lines and the pen-up travel are all the same TRJ2 LINE record type.
    assert type(file.records[1]) is Trj2LineRecord
    assert type(file.records[3]) is Trj2LineRecord
    assert type(file.records[5]) is Trj2LineRecord


def test_bridge_module_does_not_duplicate_binary_layout_constants():
    forbidden_names = {
        "HEADER_FMT",
        "RECORD_FMT",
        "HEADER_SIZE",
        "RECORD_SIZE",
        "TRJ2_HEADER_FMT",
        "TRJ2_RECORD_FMT",
        "TRJ2_HEADER_SIZE",
        "TRJ2_RECORD_SIZE",
    }

    assert forbidden_names.isdisjoint(set(vars(bridge)))


def test_bridge_module_does_not_expose_record_type_numbers():
    assert "Trj2RecordType" not in vars(bridge)


def test_direct_b6_encoding_of_lowered_file_equals_bridge_encoding():
    toolpath = compile_drawing(two_stroke_drawing())
    file = toolpath_to_trj2_file(toolpath)

    assert encode_toolpath_trj2(toolpath) == encode_trj2(file)
