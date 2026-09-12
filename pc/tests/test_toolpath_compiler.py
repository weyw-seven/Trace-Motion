import math

import pytest

from pc_trajectory.drawing import Drawing, Stroke, StrokeBuilder
from pc_trajectory.geometry import Arc, CubicBezier, Line, Point2D
from pc_trajectory.toolpath import (
    Motion,
    PenDown,
    PenState,
    PenUp,
    Toolpath,
    Wait,
)
from pc_trajectory.toolpath_compiler import (
    ToolpathCompiler,
    ToolpathCompilerConfig,
    ToolpathCompilerError,
    compile_drawing,
)


ABS = 1.0e-9


def assert_close(actual, expected, tol=ABS):
    assert math.isclose(actual, expected, rel_tol=0.0, abs_tol=tol)


def assert_point_close(actual: Point2D, expected: Point2D, tol=ABS):
    assert_close(actual.x_mm, expected.x_mm, tol)
    assert_close(actual.y_mm, expected.y_mm, tol)


def one_line_stroke(
    start_x: float,
    start_y: float,
    end_x: float,
    end_y: float,
) -> Stroke:
    return Stroke(
        (
            Line(
                Point2D(start_x, start_y),
                Point2D(end_x, end_y),
            ),
        )
    )


def golden_stroke() -> Stroke:
    builder = StrokeBuilder(Point2D(0.0, 0.0))
    builder.line_to(500.0, 0.0)
    builder.arc(
        center=Point2D(500.0, 250.0),
        sweep_deg=180.0,
    )
    builder.line_to(0.0, 500.0)
    return builder.build()


def test_default_config_is_locked():
    config = ToolpathCompilerConfig()

    assert_close(config.drawing_speed_mm_s, 300.0)
    assert_close(config.drawing_acceleration_mm_s2, 0.0)
    assert_close(config.travel_speed_mm_s, 500.0)
    assert_close(config.travel_acceleration_mm_s2, 0.0)
    assert_close(config.position_tolerance_mm, 0.01)


@pytest.mark.parametrize(
    "field,value",
    [
        ("drawing_speed_mm_s", 0.0),
        ("drawing_speed_mm_s", -1.0),
        ("drawing_speed_mm_s", math.nan),
        ("drawing_speed_mm_s", math.inf),
        ("travel_speed_mm_s", 0.0),
        ("travel_speed_mm_s", -1.0),
        ("travel_speed_mm_s", math.nan),
        ("travel_speed_mm_s", math.inf),
        ("drawing_acceleration_mm_s2", -0.1),
        ("drawing_acceleration_mm_s2", math.nan),
        ("drawing_acceleration_mm_s2", math.inf),
        ("travel_acceleration_mm_s2", -0.1),
        ("travel_acceleration_mm_s2", math.nan),
        ("travel_acceleration_mm_s2", math.inf),
        ("position_tolerance_mm", -0.1),
        ("position_tolerance_mm", math.nan),
        ("position_tolerance_mm", math.inf),
    ],
)
def test_config_rejects_invalid_values(field, value):
    kwargs = {field: value}
    with pytest.raises(ToolpathCompilerError):
        ToolpathCompilerConfig(**kwargs)


def test_compiler_rejects_invalid_config_object():
    with pytest.raises(ToolpathCompilerError):
        ToolpathCompiler(config="not config")  # type: ignore[arg-type]


def test_compile_rejects_non_drawing_input():
    compiler = ToolpathCompiler()

    with pytest.raises(ToolpathCompilerError):
        compiler.compile("not drawing")  # type: ignore[arg-type]


def test_empty_drawing_compiles_to_empty_toolpath():
    toolpath = compile_drawing(Drawing())

    assert isinstance(toolpath, Toolpath)
    assert toolpath.is_empty()
    assert toolpath.record_count == 0
    assert toolpath.final_pen_state() is PenState.UP


def test_functional_and_class_api_are_equivalent():
    drawing = Drawing(
        (
            one_line_stroke(0.0, 0.0, 100.0, 0.0),
        )
    )
    config = ToolpathCompilerConfig(
        drawing_speed_mm_s=222.0,
        drawing_acceleration_mm_s2=333.0,
    )

    by_function = compile_drawing(drawing, config)
    by_class = ToolpathCompiler(config).compile(drawing)

    assert by_function == by_class


def test_single_line_stroke_compiles_to_pen_down_motion_pen_up():
    stroke = one_line_stroke(0.0, 0.0, 100.0, 0.0)
    toolpath = compile_drawing(Drawing((stroke,)))

    assert toolpath.record_count == 3
    assert isinstance(toolpath.records[0], PenDown)
    assert isinstance(toolpath.records[1], Motion)
    assert isinstance(toolpath.records[2], PenUp)

    motion = toolpath.records[1]
    assert motion.geometry is stroke.geometries[0]
    assert_close(motion.speed_mm_s, 300.0)
    assert_close(motion.acceleration_mm_s2, 0.0)

    assert_close(toolpath.drawing_length_mm(), 100.0)
    assert_close(toolpath.travel_length_mm(), 0.0)
    assert toolpath.final_pen_state() is PenState.UP


def test_compiler_emits_no_leading_pen_up_event():
    toolpath = compile_drawing(
        Drawing((one_line_stroke(0.0, 0.0, 10.0, 0.0),))
    )

    assert isinstance(toolpath.records[0], PenDown)
    assert not isinstance(toolpath.records[0], PenUp)


def test_golden_line_arc_line_preserves_geometry_order_and_identity():
    stroke = golden_stroke()
    toolpath = compile_drawing(Drawing((stroke,)))

    assert toolpath.record_count == 5
    assert isinstance(toolpath.records[0], PenDown)
    assert isinstance(toolpath.records[4], PenUp)

    motions = [
        record
        for record in toolpath.records
        if isinstance(record, Motion)
    ]
    assert len(motions) == 3

    assert motions[0].geometry is stroke.geometries[0]
    assert motions[1].geometry is stroke.geometries[1]
    assert motions[2].geometry is stroke.geometries[2]

    assert isinstance(motions[0].geometry, Line)
    assert isinstance(motions[1].geometry, Arc)
    assert isinstance(motions[2].geometry, Line)

    assert_close(
        toolpath.drawing_length_mm(),
        1000.0 + 250.0 * math.pi,
    )
    assert_close(toolpath.travel_length_mm(), 0.0)


def test_uniform_drawing_speed_applies_to_all_geometry_in_first_baseline_compiler():
    stroke = golden_stroke()
    config = ToolpathCompilerConfig(
        drawing_speed_mm_s=275.0,
        drawing_acceleration_mm_s2=444.0,
    )

    toolpath = compile_drawing(Drawing((stroke,)), config)

    motions = [
        record
        for record in toolpath.records
        if isinstance(record, Motion)
    ]

    assert len(motions) == 3
    for motion in motions:
        assert_close(motion.speed_mm_s, 275.0)
        assert_close(motion.acceleration_mm_s2, 444.0)


def test_two_stroke_golden_inserts_one_pen_up_travel_motion():
    stroke0 = one_line_stroke(0.0, 0.0, 100.0, 0.0)
    stroke1 = one_line_stroke(200.0, 0.0, 300.0, 0.0)

    toolpath = compile_drawing(Drawing((stroke0, stroke1)))

    assert toolpath.record_count == 7

    assert isinstance(toolpath.records[0], PenDown)
    assert isinstance(toolpath.records[1], Motion)
    assert isinstance(toolpath.records[2], PenUp)
    assert isinstance(toolpath.records[3], Motion)
    assert isinstance(toolpath.records[4], PenDown)
    assert isinstance(toolpath.records[5], Motion)
    assert isinstance(toolpath.records[6], PenUp)

    travel = toolpath.records[3]
    assert isinstance(travel.geometry, Line)
    assert_point_close(travel.start_point(), Point2D(100.0, 0.0))
    assert_point_close(travel.end_point(), Point2D(200.0, 0.0))

    assert_close(travel.speed_mm_s, 500.0)
    assert_close(travel.acceleration_mm_s2, 0.0)

    assert_close(toolpath.drawing_length_mm(), 200.0)
    assert_close(toolpath.travel_length_mm(), 100.0)
    assert_close(toolpath.total_motion_length_mm(), 300.0)
    assert toolpath.final_pen_state() is PenState.UP


def test_travel_uses_custom_speed_and_acceleration():
    drawing = Drawing(
        (
            one_line_stroke(0.0, 0.0, 100.0, 0.0),
            one_line_stroke(200.0, 0.0, 300.0, 0.0),
        )
    )
    config = ToolpathCompilerConfig(
        travel_speed_mm_s=777.0,
        travel_acceleration_mm_s2=888.0,
    )

    toolpath = compile_drawing(drawing, config)
    travel = toolpath.records[3]

    assert isinstance(travel, Motion)
    assert_close(travel.speed_mm_s, 777.0)
    assert_close(travel.acceleration_mm_s2, 888.0)


def test_same_endpoint_start_strokes_keep_pen_cycle_but_emit_no_travel():
    stroke0 = one_line_stroke(0.0, 0.0, 100.0, 0.0)
    stroke1 = one_line_stroke(100.0, 0.0, 200.0, 0.0)

    toolpath = compile_drawing(Drawing((stroke0, stroke1)))

    assert toolpath.record_count == 6
    assert isinstance(toolpath.records[0], PenDown)
    assert isinstance(toolpath.records[1], Motion)
    assert isinstance(toolpath.records[2], PenUp)
    assert isinstance(toolpath.records[3], PenDown)
    assert isinstance(toolpath.records[4], Motion)
    assert isinstance(toolpath.records[5], PenUp)

    assert toolpath.motion_count == 2
    assert_close(toolpath.travel_length_mm(), 0.0)
    assert_close(toolpath.drawing_length_mm(), 200.0)


def test_tiny_interstroke_gap_within_tolerance_emits_no_travel():
    stroke0 = one_line_stroke(0.0, 0.0, 100.0, 0.0)
    stroke1 = one_line_stroke(100.005, 0.0, 200.0, 0.0)

    toolpath = compile_drawing(Drawing((stroke0, stroke1)))

    assert toolpath.motion_count == 2
    assert_close(toolpath.travel_length_mm(), 0.0)
    toolpath.validate_position_continuity()


def test_interstroke_gap_above_tolerance_emits_travel():
    stroke0 = one_line_stroke(0.0, 0.0, 100.0, 0.0)
    stroke1 = one_line_stroke(100.02, 0.0, 200.0, 0.0)

    toolpath = compile_drawing(Drawing((stroke0, stroke1)))

    assert toolpath.motion_count == 3
    travel = toolpath.records[3]
    assert isinstance(travel, Motion)
    assert_close(travel.length_mm(), 0.02)


def test_custom_position_tolerance_controls_travel_insertion():
    drawing = Drawing(
        (
            one_line_stroke(0.0, 0.0, 100.0, 0.0),
            one_line_stroke(100.5, 0.0, 200.0, 0.0),
        )
    )

    default_toolpath = compile_drawing(drawing)
    loose_toolpath = compile_drawing(
        drawing,
        ToolpathCompilerConfig(position_tolerance_mm=1.0),
    )

    assert default_toolpath.motion_count == 3
    assert loose_toolpath.motion_count == 2
    assert_close(loose_toolpath.continuity_tolerance_mm, 1.0)


def test_three_strokes_preserve_original_order():
    stroke0 = one_line_stroke(0.0, 0.0, 10.0, 0.0)
    stroke1 = one_line_stroke(100.0, 0.0, 110.0, 0.0)
    stroke2 = one_line_stroke(50.0, 0.0, 60.0, 0.0)

    drawing = Drawing((stroke0, stroke1, stroke2))
    toolpath = compile_drawing(drawing)

    draw_motions = []
    pen_down = False
    for record in toolpath.records:
        if isinstance(record, PenDown):
            pen_down = True
        elif isinstance(record, PenUp):
            pen_down = False
        elif isinstance(record, Motion) and pen_down:
            draw_motions.append(record)

    assert [m.geometry for m in draw_motions] == [
        stroke0.geometries[0],
        stroke1.geometries[0],
        stroke2.geometries[0],
    ]


def test_compiler_does_not_choose_shorter_stroke_order():
    # Stroke2 is spatially closer to Stroke0 than Stroke1, but deterministic
    # baseline compiler must preserve Drawing order.
    stroke0 = one_line_stroke(0.0, 0.0, 10.0, 0.0)
    stroke1 = one_line_stroke(1000.0, 0.0, 1010.0, 0.0)
    stroke2 = one_line_stroke(20.0, 0.0, 30.0, 0.0)

    toolpath = compile_drawing(Drawing((stroke0, stroke1, stroke2)))

    draw_geometries = []
    state_down = False
    for record in toolpath.records:
        if isinstance(record, PenDown):
            state_down = True
        elif isinstance(record, PenUp):
            state_down = False
        elif isinstance(record, Motion) and state_down:
            draw_geometries.append(record.geometry)

    assert draw_geometries == [
        stroke0.geometries[0],
        stroke1.geometries[0],
        stroke2.geometries[0],
    ]


def test_compiler_does_not_reverse_stroke_direction():
    stroke = one_line_stroke(100.0, 0.0, 0.0, 0.0)

    toolpath = compile_drawing(Drawing((stroke,)))
    motion = toolpath.records[1]

    assert isinstance(motion, Motion)
    assert_point_close(motion.start_point(), Point2D(100.0, 0.0))
    assert_point_close(motion.end_point(), Point2D(0.0, 0.0))


def test_cubic_bezier_is_preserved_as_motion_not_lowered():
    curve = CubicBezier(
        start=Point2D(0.0, 0.0),
        control1=Point2D(0.0, 50.0),
        control2=Point2D(100.0, 50.0),
        end=Point2D(100.0, 0.0),
    )
    stroke = Stroke((curve,))

    toolpath = compile_drawing(Drawing((stroke,)))
    motion = toolpath.records[1]

    assert isinstance(motion, Motion)
    assert motion.geometry is curve
    assert isinstance(motion.geometry, CubicBezier)


def test_compiler_does_not_insert_wait_events():
    drawing = Drawing(
        (
            one_line_stroke(0.0, 0.0, 100.0, 0.0),
            one_line_stroke(200.0, 0.0, 300.0, 0.0),
        )
    )
    toolpath = compile_drawing(drawing)

    assert not any(isinstance(record, Wait) for record in toolpath.records)


def test_compiled_toolpath_automatically_passes_toolpath_position_validation():
    drawing = Drawing(
        (
            golden_stroke(),
            one_line_stroke(1000.0, 1000.0, 1100.0, 1000.0),
        )
    )

    toolpath = compile_drawing(drawing)

    toolpath.validate_position_continuity()


def test_compiled_drawing_length_matches_source_drawing_length():
    stroke0 = golden_stroke()
    stroke1 = one_line_stroke(1000.0, 0.0, 1123.0, 0.0)
    drawing = Drawing((stroke0, stroke1))

    toolpath = compile_drawing(drawing)

    assert_close(
        toolpath.drawing_length_mm(),
        drawing.total_drawing_length_mm(),
        tol=1.0e-8,
    )


def test_total_motion_is_drawing_plus_travel():
    drawing = Drawing(
        (
            one_line_stroke(0.0, 0.0, 100.0, 0.0),
            one_line_stroke(200.0, 0.0, 250.0, 0.0),
            one_line_stroke(400.0, 0.0, 425.0, 0.0),
        )
    )

    toolpath = compile_drawing(drawing)

    assert_close(
        toolpath.total_motion_length_mm(),
        toolpath.drawing_length_mm() + toolpath.travel_length_mm(),
    )


def test_every_stroke_emits_its_own_pen_down_and_pen_up():
    drawing = Drawing(
        (
            one_line_stroke(0.0, 0.0, 10.0, 0.0),
            one_line_stroke(10.0, 0.0, 20.0, 0.0),
            one_line_stroke(20.0, 0.0, 30.0, 0.0),
        )
    )

    toolpath = compile_drawing(drawing)

    assert sum(isinstance(r, PenDown) for r in toolpath.records) == 3
    assert sum(isinstance(r, PenUp) for r in toolpath.records) == 3
    assert toolpath.final_pen_state() is PenState.UP


def test_compiler_is_stateless_across_multiple_compile_calls():
    compiler = ToolpathCompiler()

    drawing0 = Drawing((one_line_stroke(0.0, 0.0, 10.0, 0.0),))
    drawing1 = Drawing((one_line_stroke(100.0, 0.0, 120.0, 0.0),))

    toolpath0 = compiler.compile(drawing0)
    toolpath1 = compiler.compile(drawing1)

    assert_point_close(toolpath0.start_point(), Point2D(0.0, 0.0))
    assert_point_close(toolpath1.start_point(), Point2D(100.0, 0.0))
    assert toolpath0.record_count == 3
    assert toolpath1.record_count == 3


def test_compiler_config_is_exposed_unchanged():
    config = ToolpathCompilerConfig(
        drawing_speed_mm_s=123.0,
        travel_speed_mm_s=456.0,
    )
    compiler = ToolpathCompiler(config)

    assert compiler.config is config


def test_compiler_does_not_add_binary_fields_or_trj_records():
    toolpath = compile_drawing(
        Drawing((one_line_stroke(0.0, 0.0, 10.0, 0.0),))
    )

    # Architectural regression: B4.2 outputs Toolpath IR, not packed TRJ records.
    motion = toolpath.records[1]
    assert isinstance(motion, Motion)
    assert not hasattr(motion, "type")
    assert not hasattr(motion, "flags")
    assert not hasattr(motion, "data")
    assert not hasattr(toolpath, "header")
    assert not hasattr(toolpath, "start_yaw_deg")
