"""
Phase B / B5 engineering preview and visualization.

The preview layer visualizes already-created Drawing / Toolpath objects.
It does not change geometry, toolpath records, or binary representation.

Frozen coordinate convention:
    TRAJECTORY WORLD frame

    +X = WORLD +X
         robot forward direction when chassis yaw = 0 deg

    +Y = WORLD +Y
         robot left direction when chassis yaw = 0 deg

    positive angle / positive arc sweep = CCW

Display convention:
    screen right = WORLD +X
    screen up    = WORLD +Y

The preview MUST NOT invert the Y axis.

Layer responsibilities:
    Geometry      -> mathematical curves
    Drawing       -> continuous drawn Strokes
    Path Analysis -> junction quality / STOP_REQUIRED
    Toolpath      -> Motion + Event execution IR
    Preview       -> visualization only

Preview sampling is rendering-only. Sampling points never become trajectory
records and never mutate Geometry/Toolpath.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Iterable

from .drawing import Drawing
from .geometry import Arc, CubicBezier, Geometry, Line, Point2D
from .path_analysis import analyze_junction, analyze_stroke
from .toolpath import (
    INITIAL_PEN_STATE,
    Motion,
    PenDown,
    PenState,
    PenUp,
    Toolpath,
    Wait,
)


DEFAULT_SAMPLE_STEP_MM = 5.0
MAX_RENDER_SAMPLES_PER_GEOMETRY = 4096

# Stable preview palette for execution state. The same colors are used by the
# CLI artifact and the desktop UI acceptance tab.
DRAWING_MOTION_COLOR = "#1f77b4"
TRAVEL_MOTION_COLOR = "#d95f02"
PEN_DOWN_EVENT_COLOR = "#2ca02c"
PEN_UP_EVENT_COLOR = "#d62728"
WAIT_EVENT_COLOR = "#9467bd"


class PreviewError(ValueError):
    """Raised for invalid preview options or unsupported preview input."""


def _finite_positive(name: str, value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise PreviewError(f"{name} must be finite")
    if value <= 0.0:
        raise PreviewError(f"{name} must be > 0")
    return value


def _validate_max_samples(value: int) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise PreviewError("max_samples_per_geometry must be an integer")
    if value < 2:
        raise PreviewError("max_samples_per_geometry must be >= 2")
    return value


def _new_or_existing_axes(ax: Any | None):
    if ax is not None:
        if not hasattr(ax, "plot") or not hasattr(ax, "figure"):
            raise PreviewError("ax must be a matplotlib Axes")
        return ax.figure, ax

    import matplotlib.pyplot as plt

    fig, created_ax = plt.subplots()
    return fig, created_ax


def _configure_world_axes(ax: Any, *, title: str) -> None:
    """
    Configure standard Trajectory WORLD display.

    X grows to the right.
    Y grows upward.
    """
    ax.set_xlabel("WORLD X (mm)")
    ax.set_ylabel("WORLD Y (mm)")
    ax.set_title(title)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True)


def _parameter_values(count: int) -> tuple[float, ...]:
    if count < 2:
        raise PreviewError("render sample count must be >= 2")
    denominator = count - 1
    return tuple(index / denominator for index in range(count))


def sample_geometry(
    geometry: Geometry,
    *,
    sample_step_mm: float = DEFAULT_SAMPLE_STEP_MM,
    max_samples_per_geometry: int = MAX_RENDER_SAMPLES_PER_GEOMETRY,
) -> tuple[Point2D, ...]:
    """
    Return render-only samples for one geometry primitive.

    LINE always returns exactly its two endpoints.
    ARC / CUBIC_BEZIER use length-based parameter sampling.

    The resulting parameter samples are NOT time samples and NOT controller
    reference points.
    """
    step = _finite_positive("sample_step_mm", sample_step_mm)
    max_samples = _validate_max_samples(max_samples_per_geometry)

    if isinstance(geometry, Line):
        return (geometry.start_point(), geometry.end_point())

    if not isinstance(geometry, (Arc, CubicBezier)):
        raise PreviewError(
            f"Unsupported geometry type {type(geometry).__name__}"
        )

    length_mm = geometry.length_mm()
    requested_count = max(2, int(math.ceil(length_mm / step)) + 1)
    sample_count = min(requested_count, max_samples)

    return tuple(
        geometry.point_at(parameter)
        for parameter in _parameter_values(sample_count)
    )


def _plot_geometry(
    ax: Any,
    geometry: Geometry,
    *,
    linestyle: str,
    sample_step_mm: float,
    max_samples_per_geometry: int,
    label: str | None = None,
    color: str | None = None,
):
    points = sample_geometry(
        geometry,
        sample_step_mm=sample_step_mm,
        max_samples_per_geometry=max_samples_per_geometry,
    )
    (line_artist,) = ax.plot(
        [point.x_mm for point in points],
        [point.y_mm for point in points],
        linestyle=linestyle,
        label=label,
        color=color,
    )
    return line_artist


def _add_direction_arrow(
    ax: Any,
    geometry: Geometry,
) -> None:
    # Use the geometry parameter only for display direction.
    p0 = geometry.point_at(0.45)
    p1 = geometry.point_at(0.55)

    # Avoid a meaningless arrow when a pathological render interval collapses.
    if p0.distance_to(p1) <= 1.0e-12:
        return

    ax.annotate(
        "",
        xy=(p1.x_mm, p1.y_mm),
        xytext=(p0.x_mm, p0.y_mm),
        arrowprops={"arrowstyle": "->"},
    )


def _mark_point(
    ax: Any,
    point: Point2D,
    *,
    marker: str,
    label_text: str,
    label_offset_points: tuple[float, float] = (6.0, 7.0),
    color: str | None = None,
) -> None:
    scatter_options = {"marker": marker, "zorder": 4}
    if color is not None:
        scatter_options["color"] = color
    ax.scatter([point.x_mm], [point.y_mm], **scatter_options)
    ax.annotate(
        label_text,
        xy=(point.x_mm, point.y_mm),
        xytext=label_offset_points,
        textcoords="offset points",
    )


def _mark_stop_required(
    ax: Any,
    point: Point2D,
    *,
    angle_deg: float,
) -> None:
    ax.scatter([point.x_mm], [point.y_mm], marker="X", zorder=5)
    ax.annotate(
        f"STOP {angle_deg:.1f} deg",
        xy=(point.x_mm, point.y_mm),
        xytext=(8, 8),
        textcoords="offset points",
    )


def _finish_axes(ax: Any) -> None:
    # relim/autoscale after all line artists have been created.
    ax.relim()
    ax.autoscale_view()
    ax.margins(x=0.08, y=0.08)


def plot_drawing(
    drawing: Drawing,
    *,
    ax: Any | None = None,
    show_stroke_labels: bool = True,
    show_geometry_labels: bool = False,
    show_direction: bool = True,
    show_start_end: bool = True,
    show_warnings: bool = True,
    sample_step_mm: float = DEFAULT_SAMPLE_STEP_MM,
    max_samples_per_geometry: int = MAX_RENDER_SAMPLES_PER_GEOMETRY,
):
    """
    Plot the requested Drawing geometry only.

    No inter-Stroke travel is shown because Drawing describes what should be
    drawn, not how the robot travels between Strokes.

    Returns:
        (figure, axes)
    """
    if not isinstance(drawing, Drawing):
        raise PreviewError("drawing must be a Drawing")

    _finite_positive("sample_step_mm", sample_step_mm)
    _validate_max_samples(max_samples_per_geometry)

    fig, axes = _new_or_existing_axes(ax)
    _configure_world_axes(axes, title="Drawing Preview")

    geometry_global_index = 0

    for stroke_index, stroke in enumerate(drawing.strokes):
        if show_stroke_labels:
            start = stroke.start_point()
            axes.annotate(
                f"Stroke {stroke_index}",
                xy=(start.x_mm, start.y_mm),
                xytext=(6, -20),
                textcoords="offset points",
            )

        for geometry in stroke.geometries:
            _plot_geometry(
                axes,
                geometry,
                linestyle="-",
                sample_step_mm=sample_step_mm,
                max_samples_per_geometry=max_samples_per_geometry,
            )

            if show_direction:
                _add_direction_arrow(axes, geometry)

            if show_geometry_labels:
                midpoint = geometry.point_at(0.5)
                axes.annotate(
                    f"G{geometry_global_index}",
                    xy=(midpoint.x_mm, midpoint.y_mm),
                    xytext=(5, 5),
                    textcoords="offset points",
                )

            geometry_global_index += 1

        if show_warnings and stroke.geometry_count > 1:
            analysis = analyze_stroke(stroke)
            for junction in analysis.junctions:
                if junction.stop_required:
                    junction_point = stroke.geometries[
                        junction.previous_index
                    ].end_point()
                    _mark_stop_required(
                        axes,
                        junction_point,
                        angle_deg=junction.tangent_angle_deg,
                    )

    if show_start_end and not drawing.is_empty():
        _mark_point(
            axes,
            drawing.start_point(),
            marker="o",
            label_text="START",
            label_offset_points=(8, 8),
        )
        _mark_point(
            axes,
            drawing.end_point(),
            marker="s",
            label_text="END",
            label_offset_points=(8, 8),
        )

    _finish_axes(axes)
    return fig, axes


def _toolpath_event_positions(
    toolpath: Toolpath,
) -> dict[int, Point2D]:
    """
    Resolve event record positions from Toolpath logical XY.

    Events before the first Motion occur at first_motion.start.
    Events after a Motion occur at the most recent Motion endpoint.

    Event-only Toolpaths have no defined XY, so their events are omitted.
    """
    if not toolpath.has_motion():
        return {}

    current = toolpath.start_point()
    positions: dict[int, Point2D] = {}

    for record_index, record in enumerate(toolpath.records):
        if isinstance(record, Motion):
            current = record.end_point()
        elif isinstance(record, (PenUp, PenDown, Wait)):
            positions[record_index] = current

    return positions


def plot_toolpath(
    toolpath: Toolpath,
    *,
    ax: Any | None = None,
    show_events: bool = True,
    show_pen_state: bool = True,
    show_direction: bool = True,
    show_motion_labels: bool = False,
    show_start_end: bool = True,
    show_warnings: bool = True,
    show_legend: bool = True,
    sample_step_mm: float = DEFAULT_SAMPLE_STEP_MM,
    max_samples_per_geometry: int = MAX_RENDER_SAMPLES_PER_GEOMETRY,
):
    """
    Plot the robot execution Toolpath.

    Visual semantics:
        solid Motion  -> current pen state DOWN -> drawing
        dashed Motion -> current pen state UP   -> travel
        green down-triangle / red up-triangle -> explicit pen-state events

    Event records do not change XY.
    """
    if not isinstance(toolpath, Toolpath):
        raise PreviewError("toolpath must be a Toolpath")

    _finite_positive("sample_step_mm", sample_step_mm)
    _validate_max_samples(max_samples_per_geometry)

    fig, axes = _new_or_existing_axes(ax)
    _configure_world_axes(axes, title="Toolpath Preview (PEN UP / PEN DOWN)")

    event_positions = _toolpath_event_positions(toolpath)

    state = INITIAL_PEN_STATE
    motion_index = 0

    first_drawing_label_used = False
    first_travel_label_used = False
    pen_down_event_seen = False
    pen_up_event_seen = False

    previous_drawing_motion: Motion | None = None

    for record_index, record in enumerate(toolpath.records):
        if isinstance(record, PenUp):
            if show_events and show_pen_state and record_index in event_positions:
                _mark_point(
                    axes,
                    event_positions[record_index],
                    marker="^",
                    label_text="PEN UP",
                    label_offset_points=(6, -18),
                    color=PEN_UP_EVENT_COLOR,
                )
                pen_up_event_seen = True
            state = PenState.UP
            # Any Event is a blocking execution barrier.
            previous_drawing_motion = None
            continue

        if isinstance(record, PenDown):
            if show_events and show_pen_state and record_index in event_positions:
                _mark_point(
                    axes,
                    event_positions[record_index],
                    marker="v",
                    label_text="PEN DOWN",
                    label_offset_points=(6, -18),
                    color=PEN_DOWN_EVENT_COLOR,
                )
                pen_down_event_seen = True
            state = PenState.DOWN
            previous_drawing_motion = None
            continue

        if isinstance(record, Wait):
            if show_events and record_index in event_positions:
                point = event_positions[record_index]
                axes.scatter(
                    [point.x_mm],
                    [point.y_mm],
                    marker="P",
                    color=WAIT_EVENT_COLOR,
                    zorder=4,
                )
                axes.annotate(
                    f"WAIT {record.duration_s:.3f} s",
                    xy=(point.x_mm, point.y_mm),
                    xytext=(6, -18),
                    textcoords="offset points",
                )
            previous_drawing_motion = None
            continue

        if not isinstance(record, Motion):
            raise PreviewError(
                f"Unsupported Toolpath record type "
                f"{type(record).__name__}"
            )

        drawing_motion = state is PenState.DOWN
        linestyle = "-" if drawing_motion else "--"

        legend_label = None
        if drawing_motion and not first_drawing_label_used:
            legend_label = "Drawing motion"
            first_drawing_label_used = True
        elif not drawing_motion and not first_travel_label_used:
            legend_label = "Travel motion"
            first_travel_label_used = True

        _plot_geometry(
            axes,
            record.geometry,
            linestyle=linestyle,
            sample_step_mm=sample_step_mm,
            max_samples_per_geometry=max_samples_per_geometry,
            label=legend_label,
            color=DRAWING_MOTION_COLOR if drawing_motion else TRAVEL_MOTION_COLOR,
        )

        if show_direction:
            _add_direction_arrow(axes, record.geometry)

        if show_motion_labels:
            midpoint = record.geometry.point_at(0.5)
            axes.annotate(
                f"M{motion_index}",
                xy=(midpoint.x_mm, midpoint.y_mm),
                xytext=(5, 5),
                textcoords="offset points",
            )

        if (
            show_warnings
            and drawing_motion
            and previous_drawing_motion is not None
        ):
            junction = analyze_junction(
                previous_drawing_motion.geometry,
                record.geometry,
            )
            if junction.stop_required:
                _mark_stop_required(
                    axes,
                    previous_drawing_motion.end_point(),
                    angle_deg=junction.tangent_angle_deg,
                )

        previous_drawing_motion = record if drawing_motion else None
        motion_index += 1

    if show_start_end and toolpath.has_motion():
        _mark_point(
            axes,
            toolpath.start_point(),
            marker="o",
            label_text="START",
            label_offset_points=(8, 8),
        )
        _mark_point(
            axes,
            toolpath.end_point(),
            marker="s",
            label_text="END",
            label_offset_points=(8, 8),
        )

    if show_legend and (
        first_drawing_label_used
        or first_travel_label_used
        or pen_down_event_seen
        or pen_up_event_seen
    ):
        # Motion labels are attached to plotted lines. Event labels use proxy
        # handles so the pen-state layer is visible without adding artificial
        # data lines to the axes.
        handles, labels = axes.get_legend_handles_labels()
        from matplotlib.lines import Line2D

        if pen_down_event_seen:
            handles.append(
                Line2D(
                    [],
                    [],
                    marker="v",
                    linestyle="None",
                    color=PEN_DOWN_EVENT_COLOR,
                    label="PEN DOWN event",
                )
            )
            labels.append("PEN DOWN event")
        if pen_up_event_seen:
            handles.append(
                Line2D(
                    [],
                    [],
                    marker="^",
                    linestyle="None",
                    color=PEN_UP_EVENT_COLOR,
                    label="PEN UP event",
                )
            )
            labels.append("PEN UP event")
        axes.legend(handles, labels)

    _finish_axes(axes)
    return fig, axes


def save_drawing_preview(
    drawing: Drawing,
    path: str | Path,
    *,
    dpi: int = 150,
    **plot_options,
) -> Path:
    """Render and save a Drawing preview without calling plt.show()."""
    import matplotlib.pyplot as plt

    if not isinstance(dpi, int) or isinstance(dpi, bool) or dpi <= 0:
        raise PreviewError("dpi must be a positive integer")

    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)

    fig, _ = plot_drawing(drawing, **plot_options)
    try:
        fig.savefig(output, dpi=dpi, bbox_inches="tight")
    finally:
        plt.close(fig)

    return output


def save_toolpath_preview(
    toolpath: Toolpath,
    path: str | Path,
    *,
    dpi: int = 150,
    **plot_options,
) -> Path:
    """Render and save a Toolpath preview without calling plt.show()."""
    import matplotlib.pyplot as plt

    if not isinstance(dpi, int) or isinstance(dpi, bool) or dpi <= 0:
        raise PreviewError("dpi must be a positive integer")

    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)

    fig, _ = plot_toolpath(toolpath, **plot_options)
    try:
        fig.savefig(output, dpi=dpi, bbox_inches="tight")
    finally:
        plt.close(fig)

    return output


__all__ = [
    "DEFAULT_SAMPLE_STEP_MM",
    "MAX_RENDER_SAMPLES_PER_GEOMETRY",
    "PreviewError",
    "plot_drawing",
    "plot_toolpath",
    "sample_geometry",
    "save_drawing_preview",
    "save_toolpath_preview",
]
