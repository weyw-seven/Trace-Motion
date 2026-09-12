import math

import matplotlib

matplotlib.use("Agg", force=True)

import matplotlib.pyplot as plt
import pytest

from pc_trajectory.drawing import Drawing, Stroke, StrokeBuilder
from pc_trajectory.geometry import Arc, CubicBezier, Line, Point2D
from pc_trajectory.preview import (
    PreviewError,
    plot_drawing,
    plot_toolpath,
    sample_geometry,
    save_drawing_preview,
    save_toolpath_preview,
)
from pc_trajectory.toolpath import Motion, PenDown, PenUp, Toolpath, Wait
from pc_trajectory.toolpath_compiler import compile_drawing


ABS = 1.0e-8


def assert_close(actual, expected, tol=ABS):
    assert math.isclose(actual, expected, rel_tol=0.0, abs_tol=tol)


def text_strings(ax):
    return [artist.get_text() for artist in ax.texts]


def golden_stroke() -> Stroke:
    builder = StrokeBuilder(Point2D(0.0, 0.0))
    builder.line_to(500.0, 0.0)
    builder.arc(
        center=Point2D(500.0, 250.0),
        sweep_deg=180.0,
    )
    builder.line_to(0.0, 500.0)
    return builder.build()


def test_sample_line_returns_exactly_two_endpoints():
    line = Line(Point2D(1.0, 2.0), Point2D(11.0, 2.0))

    points = sample_geometry(line, sample_step_mm=0.1)

    assert len(points) == 2
    assert points[0] == line.start_point()
    assert points[-1] == line.end_point()


def test_sample_arc_uses_multiple_render_points():
    arc = Arc(
        center=Point2D(0.0, 0.0),
        radius_mm=10.0,
        start_angle_deg=0.0,
        sweep_deg=90.0,
    )

    points = sample_geometry(arc, sample_step_mm=2.0)

    assert len(points) > 2
    assert points[0] == arc.start_point()
    assert points[-1].distance_to(arc.end_point()) < 1.0e-9


def test_sample_full_circle_does_not_collapse_to_one_point():
    arc = Arc(
        center=Point2D(10.0, 20.0),
        radius_mm=5.0,
        start_angle_deg=17.0,
        sweep_deg=360.0,
    )

    points = sample_geometry(arc, sample_step_mm=1.0)

    assert len(points) > 10
    assert points[0].distance_to(points[-1]) < 1.0e-9
    assert max(point.x_mm for point in points) > 14.9
    assert min(point.x_mm for point in points) < 5.1


def test_sample_negative_sweep_preserves_cw_direction():
    arc = Arc(
        center=Point2D(0.0, 0.0),
        radius_mm=10.0,
        start_angle_deg=90.0,
        sweep_deg=-90.0,
    )

    points = sample_geometry(arc, sample_step_mm=2.0)

    assert_close(points[0].x_mm, 0.0)
    assert_close(points[0].y_mm, 10.0)
    assert_close(points[-1].x_mm, 10.0)
    assert_close(points[-1].y_mm, 0.0)

    midpoint = points[len(points) // 2]
    assert midpoint.x_mm > 0.0
    assert midpoint.y_mm > 0.0


def test_sample_bezier_uses_multiple_points():
    curve = CubicBezier(
        start=Point2D(0.0, 0.0),
        control1=Point2D(0.0, 20.0),
        control2=Point2D(20.0, 20.0),
        end=Point2D(20.0, 0.0),
    )

    points = sample_geometry(curve, sample_step_mm=2.0)

    assert len(points) > 2
    assert points[0] == curve.start_point()
    assert points[-1] == curve.end_point()


@pytest.mark.parametrize("step", [0.0, -1.0, math.nan, math.inf])
def test_invalid_sample_step_is_rejected(step):
    line = Line(Point2D(0.0, 0.0), Point2D(1.0, 0.0))
    with pytest.raises(PreviewError):
        sample_geometry(line, sample_step_mm=step)


@pytest.mark.parametrize("max_samples", [0, 1, -1, 2.5, True])
def test_invalid_max_samples_is_rejected(max_samples):
    arc = Arc(
        center=Point2D(0.0, 0.0),
        radius_mm=1.0,
        start_angle_deg=0.0,
        sweep_deg=90.0,
    )
    with pytest.raises(PreviewError):
        sample_geometry(
            arc,
            max_samples_per_geometry=max_samples,
        )


def test_sampling_respects_max_sample_cap():
    arc = Arc(
        center=Point2D(0.0, 0.0),
        radius_mm=1000.0,
        start_angle_deg=0.0,
        sweep_deg=360.0,
    )

    points = sample_geometry(
        arc,
        sample_step_mm=0.01,
        max_samples_per_geometry=50,
    )

    assert len(points) == 50


def test_empty_drawing_preview_does_not_crash_and_uses_world_axes():
    fig, ax = plot_drawing(Drawing())

    try:
        assert ax.get_xlabel() == "WORLD X (mm)"
        assert ax.get_ylabel() == "WORLD Y (mm)"
        assert ax.get_title() == "Drawing Preview"
        assert_close(float(ax.get_aspect()), 1.0)
        assert len(ax.lines) == 0

        # WORLD +Y must point upward. There is no inverted image Y axis.
        y0, y1 = ax.get_ylim()
        assert y1 > y0
    finally:
        plt.close(fig)


def test_single_line_drawing_preview_has_endpoint_data():
    drawing = Drawing(
        (
            Stroke(
                (
                    Line(
                        Point2D(0.0, 0.0),
                        Point2D(100.0, 0.0),
                    ),
                )
            ),
        )
    )

    fig, ax = plot_drawing(
        drawing,
        show_direction=False,
        show_stroke_labels=False,
        show_start_end=False,
        show_warnings=False,
    )

    try:
        assert len(ax.lines) == 1
        xdata = list(ax.lines[0].get_xdata())
        ydata = list(ax.lines[0].get_ydata())
        assert xdata == [0.0, 100.0]
        assert ydata == [0.0, 0.0]
    finally:
        plt.close(fig)


def test_arc_preview_is_sampled_curve_not_endpoint_chord():
    drawing = Drawing(
        (
            Stroke(
                (
                    Arc(
                        center=Point2D(0.0, 0.0),
                        radius_mm=10.0,
                        start_angle_deg=0.0,
                        sweep_deg=90.0,
                    ),
                )
            ),
        )
    )

    fig, ax = plot_drawing(
        drawing,
        sample_step_mm=1.0,
        show_direction=False,
        show_stroke_labels=False,
        show_start_end=False,
        show_warnings=False,
    )

    try:
        assert len(ax.lines) == 1
        xdata = list(ax.lines[0].get_xdata())
        ydata = list(ax.lines[0].get_ydata())
        assert len(xdata) > 2
        assert max(xdata) >= 10.0 - 1.0e-9
        assert max(ydata) >= 10.0 - 1.0e-9
    finally:
        plt.close(fig)


def test_bezier_drawing_preview_renders():
    curve = CubicBezier(
        start=Point2D(0.0, 0.0),
        control1=Point2D(0.0, 20.0),
        control2=Point2D(20.0, 20.0),
        end=Point2D(20.0, 0.0),
    )
    drawing = Drawing((Stroke((curve,)),))

    fig, ax = plot_drawing(
        drawing,
        show_direction=False,
        show_stroke_labels=False,
        show_start_end=False,
        show_warnings=False,
    )

    try:
        assert len(ax.lines) == 1
        assert len(ax.lines[0].get_xdata()) > 2
    finally:
        plt.close(fig)


def test_golden_drawing_preview_is_right_half_circle():
    drawing = Drawing((golden_stroke(),))

    fig, ax = plot_drawing(
        drawing,
        sample_step_mm=5.0,
        show_direction=False,
        show_stroke_labels=False,
        show_start_end=False,
        show_warnings=False,
    )

    try:
        assert len(ax.lines) == 3

        # The middle artist is the +180 deg CCW arc.
        arc_x = list(ax.lines[1].get_xdata())
        arc_y = list(ax.lines[1].get_ydata())

        assert min(arc_x) >= 500.0 - 1.0e-8
        assert max(arc_x) > 749.0
        assert min(arc_y) >= -1.0e-8
        assert max(arc_y) <= 500.0 + 1.0e-8

        # Explicit regression against an accidental left half-circle.
        assert min(arc_x) > 499.0
        assert_close(float(ax.get_aspect()), 1.0)
        y0, y1 = ax.get_ylim()
        assert y1 > y0
    finally:
        plt.close(fig)


def test_sharp_corner_drawing_marks_stop_required_from_b3_analysis():
    stroke = Stroke(
        (
            Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),
            Line(Point2D(100.0, 0.0), Point2D(100.0, 100.0)),
        )
    )

    fig, ax = plot_drawing(
        Drawing((stroke,)),
        show_direction=False,
        show_stroke_labels=False,
        show_start_end=False,
        show_warnings=True,
    )

    try:
        texts = text_strings(ax)
        assert any("STOP" in text and "90.0" in text for text in texts)
    finally:
        plt.close(fig)


def test_smooth_golden_drawing_has_no_stop_warning():
    fig, ax = plot_drawing(
        Drawing((golden_stroke(),)),
        show_direction=False,
        show_stroke_labels=False,
        show_start_end=False,
        show_warnings=True,
    )

    try:
        assert not any("STOP" in text for text in text_strings(ax))
    finally:
        plt.close(fig)


def test_drawing_preview_can_show_geometry_labels():
    drawing = Drawing((golden_stroke(),))

    fig, ax = plot_drawing(
        drawing,
        show_direction=False,
        show_stroke_labels=False,
        show_geometry_labels=True,
        show_start_end=False,
        show_warnings=False,
    )

    try:
        texts = text_strings(ax)
        assert "G0" in texts
        assert "G1" in texts
        assert "G2" in texts
    finally:
        plt.close(fig)


def test_toolpath_two_strokes_uses_solid_draw_dashed_travel_solid_draw():
    stroke0 = Stroke(
        (Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),)
    )
    stroke1 = Stroke(
        (Line(Point2D(200.0, 100.0), Point2D(300.0, 100.0)),)
    )
    toolpath = compile_drawing(Drawing((stroke0, stroke1)))

    fig, ax = plot_toolpath(
        toolpath,
        show_direction=False,
        show_events=False,
        show_start_end=False,
        show_warnings=False,
        show_legend=False,
    )

    try:
        assert len(ax.lines) == 3
        assert [line.get_linestyle() for line in ax.lines] == ["-", "--", "-"]

        travel_x = list(ax.lines[1].get_xdata())
        travel_y = list(ax.lines[1].get_ydata())
        assert travel_x == [100.0, 200.0]
        assert travel_y == [0.0, 100.0]
    finally:
        plt.close(fig)


def test_toolpath_event_labels_are_placed_at_logical_xy():
    stroke = Stroke(
        (Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),)
    )
    toolpath = compile_drawing(Drawing((stroke,)))

    fig, ax = plot_toolpath(
        toolpath,
        show_direction=False,
        show_events=True,
        show_start_end=False,
        show_warnings=False,
        show_legend=False,
    )

    try:
        texts = text_strings(ax)
        assert "PEN DOWN" in texts
        assert "PEN UP" in texts
    finally:
        plt.close(fig)


def test_toolpath_pen_state_review_has_event_markers_and_legend():
    stroke0 = Stroke(
        (Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),)
    )
    stroke1 = Stroke(
        (Line(Point2D(200.0, 100.0), Point2D(300.0, 100.0)),)
    )
    toolpath = compile_drawing(Drawing((stroke0, stroke1)))

    fig, ax = plot_toolpath(
        toolpath,
        show_direction=False,
        show_warnings=False,
        show_start_end=False,
    )

    try:
        assert ax.get_title() == "Toolpath Preview (PEN UP / PEN DOWN)"
        texts = text_strings(ax)
        assert texts.count("PEN DOWN") == 2
        assert texts.count("PEN UP") == 2
        legend = ax.get_legend()
        assert legend is not None
        labels = [text.get_text() for text in legend.get_texts()]
        assert "PEN DOWN event" in labels
        assert "PEN UP event" in labels
    finally:
        plt.close(fig)


def test_wait_label_renders_without_changing_xy():
    motion0 = Motion(
        Line(Point2D(0.0, 0.0), Point2D(50.0, 0.0)),
        speed_mm_s=100.0,
    )
    motion1 = Motion(
        Line(Point2D(50.0, 0.0), Point2D(100.0, 0.0)),
        speed_mm_s=100.0,
    )
    toolpath = Toolpath(
        (
            PenDown(),
            motion0,
            Wait(0.25),
            motion1,
            PenUp(),
        )
    )

    fig, ax = plot_toolpath(
        toolpath,
        show_direction=False,
        show_events=True,
        show_start_end=False,
        show_warnings=False,
        show_legend=False,
    )

    try:
        assert any("WAIT 0.250 s" == text for text in text_strings(ax))
    finally:
        plt.close(fig)


def test_toolpath_sharp_corner_marks_stop_only_within_unbroken_draw_motion_run():
    motion0 = Motion(
        Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),
        speed_mm_s=100.0,
    )
    motion1 = Motion(
        Line(Point2D(100.0, 0.0), Point2D(100.0, 100.0)),
        speed_mm_s=100.0,
    )
    toolpath = Toolpath((PenDown(), motion0, motion1, PenUp()))

    fig, ax = plot_toolpath(
        toolpath,
        show_direction=False,
        show_events=False,
        show_start_end=False,
        show_warnings=True,
        show_legend=False,
    )

    try:
        assert any(
            "STOP" in text and "90.0" in text
            for text in text_strings(ax)
        )
    finally:
        plt.close(fig)


def test_wait_barrier_prevents_false_junction_warning_across_event():
    motion0 = Motion(
        Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),
        speed_mm_s=100.0,
    )
    motion1 = Motion(
        Line(Point2D(100.0, 0.0), Point2D(100.0, 100.0)),
        speed_mm_s=100.0,
    )
    toolpath = Toolpath(
        (
            PenDown(),
            motion0,
            Wait(0.1),
            motion1,
            PenUp(),
        )
    )

    fig, ax = plot_toolpath(
        toolpath,
        show_direction=False,
        show_events=False,
        show_start_end=False,
        show_warnings=True,
        show_legend=False,
    )

    try:
        # WAIT is already a zero-speed barrier, so B5 must not pretend this is
        # one uninterrupted drawing junction requiring a separate B3 warning.
        assert not any("STOP" in text for text in text_strings(ax))
    finally:
        plt.close(fig)


def test_toolpath_start_end_and_motion_labels_render():
    toolpath = compile_drawing(
        Drawing(
            (
                Stroke(
                    (
                        Line(
                            Point2D(0.0, 0.0),
                            Point2D(100.0, 0.0),
                        ),
                    )
                ),
            )
        )
    )

    fig, ax = plot_toolpath(
        toolpath,
        show_direction=False,
        show_events=False,
        show_motion_labels=True,
        show_start_end=True,
        show_warnings=False,
        show_legend=False,
    )

    try:
        texts = text_strings(ax)
        assert "M0" in texts
        assert "START" in texts
        assert "END" in texts
    finally:
        plt.close(fig)


def test_event_only_toolpath_preview_does_not_crash():
    toolpath = Toolpath((PenDown(), Wait(0.1), PenUp()))

    fig, ax = plot_toolpath(toolpath)

    try:
        assert len(ax.lines) == 0
        assert ax.get_xlabel() == "WORLD X (mm)"
        assert ax.get_ylabel() == "WORLD Y (mm)"
    finally:
        plt.close(fig)


def test_custom_axes_are_reused():
    fig, ax = plt.subplots()
    drawing = Drawing(
        (
            Stroke(
                (
                    Line(
                        Point2D(0.0, 0.0),
                        Point2D(10.0, 0.0),
                    ),
                )
            ),
        )
    )

    try:
        returned_fig, returned_ax = plot_drawing(
            drawing,
            ax=ax,
            show_direction=False,
        )
        assert returned_fig is fig
        assert returned_ax is ax
    finally:
        plt.close(fig)


def test_invalid_axes_object_is_rejected():
    with pytest.raises(PreviewError):
        plot_drawing(Drawing(), ax="not axes")


def test_save_drawing_preview_creates_nonempty_png(tmp_path):
    output = tmp_path / "drawing.png"

    returned = save_drawing_preview(
        Drawing((golden_stroke(),)),
        output,
        show_direction=False,
    )

    assert returned == output
    assert output.exists()
    assert output.stat().st_size > 1000


def test_save_toolpath_preview_creates_nonempty_png(tmp_path):
    drawing = Drawing(
        (
            Stroke(
                (Line(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),)
            ),
            Stroke(
                (Line(Point2D(200.0, 100.0), Point2D(300.0, 100.0)),)
            ),
        )
    )
    output = tmp_path / "toolpath.png"

    returned = save_toolpath_preview(
        compile_drawing(drawing),
        output,
        show_direction=False,
    )

    assert returned == output
    assert output.exists()
    assert output.stat().st_size > 1000


@pytest.mark.parametrize("dpi", [0, -1, 1.5, True])
def test_save_preview_rejects_invalid_dpi(tmp_path, dpi):
    drawing = Drawing((golden_stroke(),))
    with pytest.raises(PreviewError):
        save_drawing_preview(drawing, tmp_path / "x.png", dpi=dpi)


def test_preview_does_not_mutate_drawing_or_toolpath():
    drawing = Drawing((golden_stroke(),))
    toolpath = compile_drawing(drawing)

    original_drawing = drawing
    original_toolpath = toolpath
    original_records = toolpath.records
    original_geometry = tuple(
        geometry
        for stroke in drawing.strokes
        for geometry in stroke.geometries
    )

    fig0, _ = plot_drawing(drawing)
    fig1, _ = plot_toolpath(toolpath)

    try:
        assert drawing == original_drawing
        assert toolpath == original_toolpath
        assert toolpath.records == original_records
        assert tuple(
            geometry
            for stroke in drawing.strokes
            for geometry in stroke.geometries
        ) == original_geometry
    finally:
        plt.close(fig0)
        plt.close(fig1)


def test_preview_objects_have_no_trj_binary_side_effects():
    drawing = Drawing((golden_stroke(),))
    toolpath = compile_drawing(drawing)

    fig, ax = plot_toolpath(toolpath)
    try:
        assert not hasattr(toolpath, "header")
        assert not hasattr(toolpath, "binary")
        assert not hasattr(ax, "traj_record_size")
    finally:
        plt.close(fig)
