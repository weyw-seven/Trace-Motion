"""Shared artifact writer used by the CLI and the local desktop UI."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import shutil

from .preview import save_toolpath_preview
from .raster.trace_preview import save_trace_preview
from .toolpath import Motion, PenDown, PenState, PenUp


@dataclass(frozen=True)
class ArtifactPaths:
    output_dir: Path
    trajectory: Path
    trace_preview: Path
    toolpath_preview: Path | None
    report: Path
    stages: dict[str, Path]
    input_copy: Path | None = None


def _report_for(result, config, paths: ArtifactPaths, source: Path | None) -> dict:
    bounds = result.world_bounds
    state = PenState.UP
    pen_up_events = 0
    pen_down_events = 0
    drawing_motions = 0
    travel_motions = 0
    drawing_length_mm = 0.0
    travel_length_mm = 0.0
    for record in result.toolpath.records:
        if isinstance(record, PenUp):
            state = PenState.UP
            pen_up_events += 1
        elif isinstance(record, PenDown):
            state = PenState.DOWN
            pen_down_events += 1
        elif isinstance(record, Motion):
            if state is PenState.DOWN:
                drawing_motions += 1
                drawing_length_mm += record.length_mm()
            else:
                travel_motions += 1
                travel_length_mm += record.length_mm()
    return {
        "source": str(source.resolve()) if source is not None else None,
        "trajectory": str(paths.trajectory.resolve()),
        "trace_preview": str(paths.trace_preview.resolve()),
        "toolpath_preview": (
            str(paths.toolpath_preview.resolve())
            if paths.toolpath_preview is not None else None
        ),
        "stages": {name: str(path.resolve()) for name, path in paths.stages.items()},
        "input_copy": str(paths.input_copy.resolve()) if paths.input_copy else None,
        "pixel_paths": result.trace.path_count,
        "planned_strokes": result.stroke_plan.stroke_count,
        "drawing_strokes": result.drawing.stroke_count,
        "line_records": result.line_count,
        "circle_records": result.circle_count,
        "motion_records": result.motion_count,
        "record_count": result.traj2_file.record_count,
        "pen_state": {
            "initial": PenState.UP.value,
            "final": result.toolpath.final_pen_state().value,
            "pen_up_events": pen_up_events,
            "pen_down_events": pen_down_events,
            "drawing_motion_records": drawing_motions,
            "travel_motion_records": travel_motions,
            "drawing_length_mm": drawing_length_mm,
            "travel_length_mm": travel_length_mm,
        },
        "mm_per_pixel": result.transform.mm_per_pixel,
        "smoothing": {
            "iterations": config.smoothing_iterations,
            "strength": config.smoothing_strength,
            "simplify_tolerance_mm": config.simplify_tolerance_mm,
            "arc_fitting_enabled": config.enable_arc_fitting,
            "min_arc_points": config.min_arc_points,
            "min_arc_sweep_deg": config.min_arc_sweep_deg,
            "min_arc_radius_mm": config.min_arc_radius_mm,
        },
        "world_bounds_mm": {
            "min_x": bounds.min_x_mm,
            "min_y": bounds.min_y_mm,
            "max_x": bounds.max_x_mm,
            "max_y": bounds.max_y_mm,
        },
        "diagnostics": [
            {
                "code": diagnostic.code,
                "severity": diagnostic.severity,
                "message": diagnostic.message,
            }
            for diagnostic in result.diagnostics
        ],
    }


def write_lineart_artifacts(
    result,
    config,
    output_dir: str | Path,
    *,
    source: str | Path | None = None,
    basename: str | None = None,
    include_preview: bool = True,
    copy_input: bool = True,
) -> ArtifactPaths:
    """Write every user-facing result from one successful pipeline run."""
    from .raster.lineart import LineartConfig, LineartResult

    if not isinstance(result, LineartResult):
        raise TypeError("result must be LineartResult")
    if not isinstance(config, LineartConfig):
        raise TypeError("config must be LineartConfig")
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    source_path = Path(source) if source is not None else None
    stem = basename or (source_path.stem if source_path is not None else "lineart")
    trajectory = result.write_traj(output / f"{stem}.traj")
    stages = result.preprocess.save_previews(output / "stages")
    trace_preview = save_trace_preview(result.trace, output / "trace.png")
    toolpath_preview = None
    if include_preview:
        toolpath_preview = save_toolpath_preview(
            result.toolpath,
            output / "toolpath.png",
            # This image is the visual acceptance view for the executable
            # pen-state layer, so keep the explicit events visible.
            show_events=True,
            show_pen_state=True,
            show_direction=False,
            show_warnings=False,
            show_motion_labels=False,
        )
    input_copy = None
    if copy_input and source_path is not None and source_path.is_file():
        input_copy = output / f"input{source_path.suffix.lower()}"
        shutil.copy2(source_path, input_copy)
    paths = ArtifactPaths(
        output_dir=output,
        trajectory=trajectory,
        trace_preview=trace_preview,
        toolpath_preview=toolpath_preview,
        report=output / "report.json",
        stages=stages,
        input_copy=input_copy,
    )
    paths.report.write_text(
        json.dumps(_report_for(result, config, paths, source_path), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return paths


__all__ = ["ArtifactPaths", "write_lineart_artifacts"]
