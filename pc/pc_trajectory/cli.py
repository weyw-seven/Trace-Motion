"""Command-line entry points for the PC line-art compiler."""

from __future__ import annotations

import argparse
from pathlib import Path

from .artifacts import write_lineart_artifacts
from .raster import (
    LineartConfig,
    PreprocessConfig,
    SizeConfig,
    lineart_to_trj2,
)
from .toolpath_compiler import ToolpathCompilerConfig


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="pc-trajectory")
    subparsers = parser.add_subparsers(dest="command", required=True)
    lineart = subparsers.add_parser("lineart", help="compile PNG/JPEG line art to TRJ2")
    lineart.add_argument("source", type=Path)
    lineart.add_argument("--output", type=Path, required=True, help="output directory")
    lineart.add_argument("--threshold", type=int, default=127)
    lineart.add_argument("--invert", action="store_true")
    lineart.add_argument("--min-component-pixels", type=int, default=1)
    lineart.add_argument("--max-extent-mm", type=float, default=100.0)
    lineart.add_argument("--simplify-mm", type=float, default=0.35)
    lineart.add_argument("--smooth-iterations", type=int, default=3)
    lineart.add_argument("--smooth-strength", type=float, default=0.5)
    lineart.add_argument("--no-arc-fit", action="store_true")
    lineart.add_argument("--min-arc-points", type=int, default=6)
    lineart.add_argument("--min-arc-sweep-deg", type=float, default=12.0)
    lineart.add_argument("--min-arc-radius-mm", type=float, default=1.0)
    lineart.add_argument("--drawing-speed", type=float, default=300.0)
    lineart.add_argument("--travel-speed", type=float, default=500.0)
    lineart.add_argument("--drawing-acceleration", type=float, default=0.0)
    lineart.add_argument("--travel-acceleration", type=float, default=0.0)
    lineart.add_argument("--start-yaw-deg", type=float, default=0.0)
    lineart.add_argument("--no-preview", action="store_true")
    return parser


def _run_lineart(args: argparse.Namespace) -> int:
    config = LineartConfig(
        preprocess=PreprocessConfig(
            threshold=args.threshold,
            invert=args.invert,
            min_component_pixels=args.min_component_pixels,
        ),
        size=SizeConfig(max_extent_mm=args.max_extent_mm),
        simplify_tolerance_mm=args.simplify_mm,
        smoothing_iterations=args.smooth_iterations,
        smoothing_strength=args.smooth_strength,
        enable_arc_fitting=not args.no_arc_fit,
        min_arc_points=args.min_arc_points,
        min_arc_sweep_deg=args.min_arc_sweep_deg,
        min_arc_radius_mm=args.min_arc_radius_mm,
        compiler=ToolpathCompilerConfig(
            drawing_speed_mm_s=args.drawing_speed,
            travel_speed_mm_s=args.travel_speed,
            drawing_acceleration_mm_s2=args.drawing_acceleration,
            travel_acceleration_mm_s2=args.travel_acceleration,
        ),
        start_yaw_deg=args.start_yaw_deg,
    )
    result = lineart_to_trj2(args.source, config)
    paths = write_lineart_artifacts(
        result,
        config,
        args.output,
        source=args.source,
        include_preview=not args.no_preview,
    )
    print(f"TRJ2: {paths.trajectory}")
    print(
        f"strokes={result.drawing.stroke_count}, lines={result.line_count}, "
        f"circles={result.circle_count}, records={result.traj2_file.record_count}"
    )
    print(f"report: {paths.report}")
    return 0


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.command == "lineart":
        return _run_lineart(args)
    raise AssertionError(f"unhandled command {args.command!r}")


if __name__ == "__main__":
    raise SystemExit(main())
