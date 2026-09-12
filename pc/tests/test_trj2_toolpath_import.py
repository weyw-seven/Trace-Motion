import math
from pathlib import Path

import pytest

import pc_trajectory.trj2_toolpath_import as importer
from pc_trajectory.drawing import Drawing, Stroke
from pc_trajectory.geometry import Arc, CubicBezier, Line, Point2D
from pc_trajectory.toolpath import Motion, PenDown, PenState, PenUp, Toolpath, Wait
from pc_trajectory.toolpath_compiler import compile_drawing
from pc_trajectory.toolpath_trj2_export import (
    encode_toolpath_trj2,
    toolpath_to_trj2_file,
)
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
from pc_trajectory.traj2_reader import decode_trj2
ImportedTrj2Toolpath = importer.ImportedTrj2Toolpath
Trj2ImportContinuityError = importer.Trj2ImportContinuityError
Trj2ToolpathImportError = importer.Trj2ToolpathImportError
decode_toolpath_trj2 = importer.decode_toolpath_trj2
read_toolpath_trj2 = importer.read_toolpath_trj2
trj2_file_to_toolpath = importer.trj2_file_to_toolpath
trj2_record_to_toolpath_record = importer.trj2_record_to_toolpath_record
from pc_trajectory.traj2_validate import TRJ2_CONTINUITY_TOLERANCE_MM
from pc_trajectory.traj2_writer import encode_trj2


GOLDEN = bytes.fromhex("54524a3202002000070000002c0000000000000000000000000000000000000021000000000000000000000000000000000000000000000000000000000000000000000000000000000000000100000000009643000000000000c842000000000000000000000000000000000000000000000000000000002000000000000000000000000000000000000000000000000000000000000000000000000000000000000000010000000000fa43000000000000484300000000000000000000000000000000000000000000000000000000210000000000000000000000000000000000000000000000000000000000000000000000000000000000000001000000000096430000000000009643000000000000000000000000000000000000000000000000000000002000000000000000000000000000000000000000000000000000000000000000000000000000000000000000")


def close(a, b, tol=1e-7):
    assert math.isclose(a, b, rel_tol=0.0, abs_tol=tol)


def two_strokes():
    return Drawing((
        Stroke((Line(Point2D(0, 0), Point2D(100, 0)),)),
        Stroke((Line(Point2D(200, 0), Point2D(300, 0)),)),
    ))


def mixed_toolpath():
    line = Motion(
        Line(Point2D(0, 0), Point2D(100, 0)),
        123.456,
        12.25,
    )
    arc = Motion(
        Arc(Point2D(100, 50), 50, -90, 90),
        87.5,
        3.25,
    )
    bezier = Motion(
        CubicBezier(
            arc.end_point(),
            Point2D(160, 50),
            Point2D(180, 100),
            Point2D(200, 100),
        ),
        66.75,
        2.5,
    )
    return Toolpath((
        PenDown(),
        line,
        arc,
        Wait(0.375),
        bezier,
        PenUp(),
    ))


def test_empty_file_preserves_header_pose():
    imported = trj2_file_to_toolpath(
        Trj2File(Trj2Header(12, 34, 56), ())
    )
    assert isinstance(imported, ImportedTrj2Toolpath)
    assert imported.toolpath == Toolpath()
    assert imported.start_point == Point2D(12, 34)
    close(imported.start_yaw_deg, 56)


def test_line_uses_current_as_explicit_start():
    record, next_point = trj2_record_to_toolpath_record(
        Trj2LineRecord(30, 40, 123, 4),
        current_point=Point2D(10, 20),
    )
    assert isinstance(record, Motion)
    assert isinstance(record.geometry, Line)
    assert record.start_point() == Point2D(10, 20)
    assert record.end_point() == Point2D(30, 40)
    assert next_point == Point2D(30, 40)
    close(record.speed_mm_s, 123)
    close(record.acceleration_mm_s2, 4)


def test_circle_preserves_parameters():
    record, next_point = trj2_record_to_toolpath_record(
        Trj2CircleRecord(500, 250, 250, -90, 180, 220, 12),
        current_point=Point2D(500, 0),
    )
    assert isinstance(record, Motion)
    assert isinstance(record.geometry, Arc)
    arc = record.geometry
    assert arc.center == Point2D(500, 250)
    close(arc.radius_mm, 250)
    close(arc.start_angle_deg, -90)
    close(arc.sweep_deg, 180)
    assert next_point.distance_to(Point2D(500, 500)) < 1e-6


def test_negative_circle_sweep_preserved():
    record, next_point = trj2_record_to_toolpath_record(
        Trj2CircleRecord(0, 0, 10, 90, -90, 50),
        current_point=Point2D(0, 10),
    )
    assert isinstance(record, Motion)
    close(record.geometry.sweep_deg, -90)
    assert next_point.distance_to(Point2D(10, 0)) < 1e-6


def test_circle_bad_current_rejected():
    with pytest.raises(Trj2ImportContinuityError):
        trj2_record_to_toolpath_record(
            Trj2CircleRecord(0, 10, 10, -90, 90, 100),
            current_point=Point2D(1, 0),
        )


def test_bezier_reconstructs_implicit_p0():
    record, next_point = trj2_record_to_toolpath_record(
        Trj2CubicBezierRecord(
            10, 11, 20, 21, 30, 31, 100, 7
        ),
        current_point=Point2D(1, 2),
    )
    assert isinstance(record, Motion)
    assert isinstance(record.geometry, CubicBezier)
    curve = record.geometry
    assert curve.start == Point2D(1, 2)
    assert curve.control1 == Point2D(10, 11)
    assert curve.control2 == Point2D(20, 21)
    assert curve.end == Point2D(30, 31)
    assert next_point == Point2D(30, 31)


@pytest.mark.parametrize(
    "source,expected",
    [
        (Trj2PenDownRecord(), PenDown),
        (Trj2PenUpRecord(), PenUp),
        (Trj2WaitRecord(0.25), Wait),
    ],
)
def test_events_preserve_type_and_xy(source, expected):
    current = Point2D(12, 34)
    record, next_point = trj2_record_to_toolpath_record(
        source,
        current_point=current,
    )
    assert isinstance(record, expected)
    assert next_point is current
    if isinstance(record, Wait):
        close(record.duration_s, 0.25)


def test_bad_current_point_type_rejected():
    with pytest.raises(Trj2ToolpathImportError):
        trj2_record_to_toolpath_record(
            Trj2PenUpRecord(),
            current_point=(0, 0),  # type: ignore[arg-type]
        )


def test_event_only_preserves_header_and_order():
    imported = trj2_file_to_toolpath(
        Trj2File(
            Trj2Header(50, 60, 30),
            (
                Trj2PenDownRecord(),
                Trj2WaitRecord(1),
                Trj2PenUpRecord(),
            ),
        )
    )
    assert imported.start_point == Point2D(50, 60)
    close(imported.start_yaw_deg, 30)
    assert [type(r) for r in imported.toolpath.records] == [
        PenDown, Wait, PenUp
    ]


def test_events_do_not_move_xy():
    imported = trj2_file_to_toolpath(
        Trj2File(
            Trj2Header(0, 0, 0),
            (
                Trj2LineRecord(100, 0, 100),
                Trj2PenUpRecord(),
                Trj2WaitRecord(0.1),
                Trj2PenDownRecord(),
                Trj2LineRecord(200, 0, 100),
            ),
        )
    )
    second = imported.toolpath.records[4]
    assert isinstance(second, Motion)
    assert second.start_point() == Point2D(100, 0)


def test_circle_end_becomes_next_line_start():
    imported = trj2_file_to_toolpath(
        Trj2File(
            Trj2Header(0, 0, 0),
            (
                Trj2CircleRecord(0, 10, 10, -90, 90, 100),
                Trj2LineRecord(50, 10, 100),
            ),
        )
    )
    a, b = imported.toolpath.records
    assert isinstance(a, Motion)
    assert isinstance(b, Motion)
    assert a.end_point().distance_to(b.start_point()) < 1e-9


def test_first_circle_wrapper_keeps_exact_header_pose():
    imported = trj2_file_to_toolpath(
        Trj2File(
            Trj2Header(0, 0, 17),
            (Trj2CircleRecord(0, 10, 10, -90, 90, 100),),
        )
    )
    assert imported.start_point == Point2D(0, 0)
    close(imported.start_yaw_deg, 17)
    first = imported.toolpath.records[0]
    assert isinstance(first, Motion)
    assert (
        first.start_point().distance_to(imported.start_point)
        <= TRJ2_CONTINUITY_TOLERANCE_MM
    )


def test_golden_sequence():
    imported = decode_toolpath_trj2(GOLDEN)
    assert [type(r) for r in imported.toolpath.records] == [
        PenDown, Motion, PenUp, Motion, PenDown, Motion, PenUp
    ]


def test_golden_header():
    imported = decode_toolpath_trj2(GOLDEN)
    assert imported.start_point == Point2D(0, 0)
    close(imported.start_yaw_deg, 0)


def test_golden_line_starts():
    imported = decode_toolpath_trj2(GOLDEN).toolpath
    m0, m1, m2 = (
        imported.records[1],
        imported.records[3],
        imported.records[5],
    )
    assert isinstance(m0, Motion)
    assert isinstance(m1, Motion)
    assert isinstance(m2, Motion)
    assert m0.start_point() == Point2D(0, 0)
    assert m0.end_point() == Point2D(100, 0)
    assert m1.start_point() == Point2D(100, 0)
    assert m1.end_point() == Point2D(200, 0)
    assert m2.start_point() == Point2D(200, 0)
    assert m2.end_point() == Point2D(300, 0)


def test_golden_speeds():
    toolpath = decode_toolpath_trj2(GOLDEN).toolpath
    speeds = [
        r.speed_mm_s for r in toolpath.records
        if isinstance(r, Motion)
    ]
    assert speeds == [300, 500, 300]


def test_golden_lengths_and_pen_state():
    toolpath = decode_toolpath_trj2(GOLDEN).toolpath
    close(toolpath.drawing_length_mm(), 200)
    close(toolpath.travel_length_mm(), 100)
    close(toolpath.total_motion_length_mm(), 300)
    assert toolpath.final_pen_state() is PenState.UP


def test_b4_export_import_record_kinds():
    original = compile_drawing(two_strokes())
    imported = decode_toolpath_trj2(
        encode_toolpath_trj2(original)
    ).toolpath
    assert [type(r) for r in imported.records] == [
        type(r) for r in original.records
    ]


def test_b4_golden_export_import_export_exact():
    original = compile_drawing(two_strokes())
    a = encode_toolpath_trj2(original, start_yaw_deg=0)
    imported = decode_toolpath_trj2(a)
    b = encode_toolpath_trj2(
        imported.toolpath,
        start_point=imported.start_point,
        start_yaw_deg=imported.start_yaw_deg,
    )
    assert a == GOLDEN
    assert b == a


def test_independent_golden_reexport_exact():
    imported = decode_toolpath_trj2(GOLDEN)
    assert encode_toolpath_trj2(
        imported.toolpath,
        start_point=imported.start_point,
        start_yaw_deg=imported.start_yaw_deg,
    ) == GOLDEN


def test_mixed_line_arc_bezier_wait_binary_stable():
    a = encode_toolpath_trj2(mixed_toolpath(), start_yaw_deg=23.5)
    imported = decode_toolpath_trj2(a)
    b = encode_toolpath_trj2(
        imported.toolpath,
        start_point=imported.start_point,
        start_yaw_deg=imported.start_yaw_deg,
    )
    assert b == a


def test_mixed_types_and_order_preserved():
    imported = decode_toolpath_trj2(
        encode_toolpath_trj2(mixed_toolpath())
    ).toolpath
    assert [type(r) for r in imported.records] == [
        PenDown, Motion, Motion, Wait, Motion, PenUp
    ]
    motions = [r for r in imported.records if isinstance(r, Motion)]
    assert isinstance(motions[0].geometry, Line)
    assert isinstance(motions[1].geometry, Arc)
    assert isinstance(motions[2].geometry, CubicBezier)


def test_mixed_constraints_and_wait_preserved():
    imported = decode_toolpath_trj2(
        encode_toolpath_trj2(mixed_toolpath())
    ).toolpath
    line = imported.records[1]
    arc = imported.records[2]
    wait = imported.records[3]
    bezier = imported.records[4]
    assert isinstance(line, Motion)
    assert isinstance(arc, Motion)
    assert isinstance(wait, Wait)
    assert isinstance(bezier, Motion)
    close(line.speed_mm_s, 123.456, 1e-4)
    close(line.acceleration_mm_s2, 12.25)
    close(arc.speed_mm_s, 87.5)
    close(wait.duration_s, 0.375)
    close(bezier.speed_mm_s, 66.75)


def test_float32_values_stabilize_after_one_round_trip():
    original = Toolpath((
        Motion(
            Line(
                Point2D(0.1, -0.2),
                Point2D(123.456789, 98.7654321),
            ),
            234.567891,
            12.345678,
        ),
    ))
    a = encode_toolpath_trj2(original, start_yaw_deg=17.123456)
    imported = decode_toolpath_trj2(a)
    b = encode_toolpath_trj2(
        imported.toolpath,
        start_point=imported.start_point,
        start_yaw_deg=imported.start_yaw_deg,
    )
    assert b == a


def test_empty_file_round_trip_exact():
    source = Trj2File(Trj2Header(12.25, -34.5, 67.75), ())
    a = encode_trj2(source)
    imported = decode_toolpath_trj2(a)
    b = encode_toolpath_trj2(
        imported.toolpath,
        start_point=imported.start_point,
        start_yaw_deg=imported.start_yaw_deg,
    )
    assert b == a
    assert len(b) == 32


def test_event_only_round_trip_exact():
    source = Trj2File(
        Trj2Header(50, 60, 30),
        (
            Trj2PenDownRecord(),
            Trj2WaitRecord(0.25),
            Trj2PenUpRecord(),
        ),
    )
    a = encode_trj2(source)
    imported = decode_toolpath_trj2(a)
    b = encode_toolpath_trj2(
        imported.toolpath,
        start_point=imported.start_point,
        start_yaw_deg=imported.start_yaw_deg,
    )
    assert b == a


def test_first_circle_generated_by_b7_round_trip_exact():
    original = Toolpath((
        Motion(
            Arc(Point2D(0, 10), 10, -90, 180),
            speed_mm_s=100,
        ),
    ))
    a = encode_toolpath_trj2(original, start_yaw_deg=5)
    imported = decode_toolpath_trj2(a)
    b = encode_toolpath_trj2(
        imported.toolpath,
        start_point=imported.start_point,
        start_yaw_deg=imported.start_yaw_deg,
    )
    assert b == a


def test_imported_toolpath_uses_b6_tolerance():
    imported = decode_toolpath_trj2(GOLDEN)
    close(
        imported.toolpath.continuity_tolerance_mm,
        TRJ2_CONTINUITY_TOLERANCE_MM,
    )


def test_decode_accepts_bytearray_and_memoryview():
    assert decode_toolpath_trj2(bytearray(GOLDEN)) == (
        decode_toolpath_trj2(memoryview(GOLDEN))
    )


def test_read_convenience(tmp_path):
    path = tmp_path / "golden.traj"
    path.write_bytes(GOLDEN)
    imported = read_toolpath_trj2(path)
    assert imported.start_point == Point2D(0, 0)
    assert imported.toolpath.record_count == 7


def test_decode_delegates_to_b6(monkeypatch):
    sentinel = Trj2File(Trj2Header(1, 2, 3), ())
    seen = {}

    def fake(data):
        seen["data"] = data
        return sentinel

    monkeypatch.setattr(importer, "decode_trj2", fake)
    result = importer.decode_toolpath_trj2(b"abc")
    assert seen["data"] == b"abc"
    assert result.start_point == Point2D(1, 2)


def test_read_delegates_to_b6(monkeypatch):
    sentinel = Trj2File(Trj2Header(4, 5, 6), ())
    seen = {}

    def fake(path):
        seen["path"] = path
        return sentinel

    monkeypatch.setattr(importer, "read_trj2", fake)
    result = importer.read_toolpath_trj2("x.traj")
    assert seen["path"] == "x.traj"
    assert result.start_point == Point2D(4, 5)


def test_non_file_rejected():
    with pytest.raises(Trj2ToolpathImportError):
        trj2_file_to_toolpath("bad")  # type: ignore[arg-type]


def test_invalid_semantic_file_rejected():
    invalid = Trj2File(
        Trj2Header(0, 0, 0),
        (Trj2LineRecord(0, 0, 100),),
    )
    with pytest.raises(Trj2ToolpathImportError):
        trj2_file_to_toolpath(invalid)


def test_import_does_not_mutate_source():
    source = decode_trj2(GOLDEN)
    before = source
    imported = trj2_file_to_toolpath(source)
    assert source == before
    assert imported.toolpath.record_count == source.record_count


def test_draw_travel_recovered_from_pen_state():
    imported = decode_toolpath_trj2(GOLDEN).toolpath
    assert all(
        isinstance(r.geometry, Line)
        for r in imported.records
        if isinstance(r, Motion)
    )
    close(imported.drawing_length_mm(), 200)
    close(imported.travel_length_mm(), 100)


def test_repeated_pen_events_preserved():
    imported = trj2_file_to_toolpath(
        Trj2File(
            Trj2Header(0, 0, 0),
            (
                Trj2PenUpRecord(),
                Trj2PenUpRecord(),
                Trj2PenDownRecord(),
                Trj2PenDownRecord(),
                Trj2PenUpRecord(),
            ),
        )
    ).toolpath
    assert [type(r) for r in imported.records] == [
        PenUp, PenUp, PenDown, PenDown, PenUp
    ]
    assert imported.final_pen_state() is PenState.UP


def test_golden_can_lower_to_same_b6_semantic_file():
    source = decode_trj2(GOLDEN)
    imported = trj2_file_to_toolpath(source)
    lowered = toolpath_to_trj2_file(
        imported.toolpath,
        start_point=imported.start_point,
        start_yaw_deg=imported.start_yaw_deg,
    )
    assert lowered == source


def test_b8_has_no_binary_layout_constants():
    forbidden = {
        "HEADER_FMT", "RECORD_FMT", "HEADER_SIZE", "RECORD_SIZE",
        "TRJ2_HEADER_FMT", "TRJ2_RECORD_FMT",
        "TRJ2_HEADER_SIZE", "TRJ2_RECORD_SIZE",
    }
    assert forbidden.isdisjoint(set(vars(importer)))


def test_b8_has_no_type_number_enum():
    assert "Trj2RecordType" not in vars(importer)


def test_b8_source_has_no_struct_unpack():
    source = Path(importer.__file__).read_text(encoding="utf-8")
    assert "import struct" not in source
    assert "struct.unpack" not in source
