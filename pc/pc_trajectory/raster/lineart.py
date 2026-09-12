"""R3 line-art to canonical TRJ2 pipeline.

The pipeline is intentionally small and inspectable:

    PNG/JPEG -> preprocess -> skeleton/trace -> stroke plan
              -> WORLD polylines -> Drawing -> Toolpath -> TRJ2

Only executable LINE, CIRCLE, PEN_UP and PEN_DOWN records are emitted.  The
returned intermediate objects make every stage available to a UI or an
acceptance report without re-running the image processing.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path

from PIL import Image

from ..drawing import Drawing, Stroke
from ..geometry import Arc, Geometry, Line, Point2D
from ..toolpath import Motion, Toolpath
from ..toolpath_compiler import ToolpathCompilerConfig, compile_drawing
from ..toolpath_trj2_export import encode_toolpath_trj2
from ..traj2_format import Trj2File
from ..traj2_reader import decode_trj2
from ..trj2_toolpath_import import decode_toolpath_trj2
from .config import PreprocessConfig, RasterError, SizeConfig
from .coordinates import PixelWorldTransform, fit_pixel_transform_points
from .curve_fit import fit_polyline_geometry
from .preprocess import PreprocessResult, RasterDiagnostic, preprocess_image, read_lineart
from .simplify import simplify_polyline, smooth_polyline
from .skeleton import SkeletonConfig, SkeletonResult
from .stroke_planner import PlannedStroke, StrokePlan, plan_strokes
from .trace import TraceConfig, TraceResult, extract_pixel_paths


@dataclass(frozen=True)
class LineartConfig:
    preprocess: PreprocessConfig = field(default_factory=PreprocessConfig)
    skeleton: SkeletonConfig = field(default_factory=SkeletonConfig)
    trace: TraceConfig = field(default_factory=TraceConfig)
    size: SizeConfig = field(default_factory=SizeConfig)
    simplify_tolerance_mm: float = 0.35
    smoothing_iterations: int = 3
    smoothing_strength: float = 0.5
    enable_arc_fitting: bool = True
    min_arc_points: int = 6
    min_arc_sweep_deg: float = 12.0
    min_arc_radius_mm: float = 1.0
    compiler: ToolpathCompilerConfig = field(default_factory=ToolpathCompilerConfig)
    start_yaw_deg: float = 0.0
    anchor_world: Point2D = Point2D(0.0, 0.0)

    def __post_init__(self) -> None:
        for name, expected in (
            ("preprocess", PreprocessConfig),
            ("skeleton", SkeletonConfig),
            ("trace", TraceConfig),
            ("size", SizeConfig),
            ("compiler", ToolpathCompilerConfig),
        ):
            if not isinstance(getattr(self, name), expected):
                raise RasterError(f"{name} has an invalid configuration type")
        if isinstance(self.simplify_tolerance_mm, bool):
            raise RasterError("simplify_tolerance_mm must be non-negative and finite")
        if type(self.smoothing_iterations) is not int or self.smoothing_iterations < 0:
            raise RasterError("smoothing_iterations must be a non-negative integer")
        if type(self.enable_arc_fitting) is not bool:
            raise RasterError("enable_arc_fitting must be bool")
        if type(self.min_arc_points) is not int or self.min_arc_points < 3:
            raise RasterError("min_arc_points must be an integer >= 3")
        if isinstance(self.smoothing_strength, bool):
            raise RasterError("smoothing_strength must be in [0, 1]")
        try:
            tolerance = float(self.simplify_tolerance_mm)
            strength = float(self.smoothing_strength)
            min_sweep = float(self.min_arc_sweep_deg)
            min_radius = float(self.min_arc_radius_mm)
            yaw = float(self.start_yaw_deg)
        except (TypeError, ValueError, OverflowError) as exc:
            raise RasterError("simplify_tolerance_mm/start_yaw_deg must be finite numbers") from exc
        if not math.isfinite(tolerance) or tolerance < 0.0:
            raise RasterError("simplify_tolerance_mm must be non-negative and finite")
        if not math.isfinite(strength) or not 0.0 <= strength <= 1.0:
            raise RasterError("smoothing_strength must be in [0, 1]")
        if not math.isfinite(min_sweep) or not 0.0 < min_sweep <= 180.0:
            raise RasterError("min_arc_sweep_deg must be in (0, 180]")
        if not math.isfinite(min_radius) or min_radius <= 0.0:
            raise RasterError("min_arc_radius_mm must be positive and finite")
        if not math.isfinite(yaw):
            raise RasterError("start_yaw_deg must be finite")
        if not isinstance(self.anchor_world, Point2D):
            raise RasterError("anchor_world must be Point2D")
        object.__setattr__(self, "simplify_tolerance_mm", tolerance)
        object.__setattr__(self, "smoothing_strength", strength)
        object.__setattr__(self, "min_arc_sweep_deg", min_sweep)
        object.__setattr__(self, "min_arc_radius_mm", min_radius)
        object.__setattr__(self, "start_yaw_deg", yaw)


@dataclass(frozen=True)
class LineartResult:
    preprocess: PreprocessResult
    skeleton: SkeletonResult
    trace: TraceResult
    stroke_plan: StrokePlan
    transform: PixelWorldTransform
    drawing: Drawing
    toolpath: Toolpath
    traj2_file: Trj2File
    traj2_bytes: bytes
    roundtrip_toolpath: Toolpath
    diagnostics: tuple[RasterDiagnostic, ...]

    @property
    def world_bounds(self):
        return self.drawing.bounding_box()

    @property
    def stroke_count(self) -> int:
        return self.drawing.stroke_count

    @property
    def line_count(self) -> int:
        return sum(
            isinstance(record, Motion) and isinstance(record.geometry, Line)
            for record in self.toolpath.records
        )

    @property
    def circle_count(self) -> int:
        return sum(
            isinstance(record, Motion) and isinstance(record.geometry, Arc)
            for record in self.toolpath.records
        )

    @property
    def motion_count(self) -> int:
        return self.toolpath.motion_count

    def write_traj(self, path: str | Path) -> Path:
        output = Path(path)
        output.write_bytes(self.traj2_bytes)
        return output


def _raise_diagnostics(diagnostics: tuple[RasterDiagnostic, ...]) -> None:
    errors = [diagnostic for diagnostic in diagnostics if diagnostic.severity == "error"]
    if errors:
        raise RasterError("; ".join(diagnostic.message for diagnostic in errors))


def _world_points(
    path,
    transform: PixelWorldTransform,
    tolerance_mm: float,
    smoothing_iterations: int,
    smoothing_strength: float,
) -> tuple[Point2D, ...]:
    points = tuple(transform.to_world(point) for point in path.points)
    smoothed = smooth_polyline(
        points,
        iterations=smoothing_iterations,
        strength=smoothing_strength,
        closed=path.closed,
    )
    return smoothed


def _planned_stroke_to_stroke(
    planned: PlannedStroke,
    transform: PixelWorldTransform,
    tolerance_mm: float,
    smoothing_iterations: int,
    smoothing_strength: float,
    enable_arc_fitting: bool,
    min_arc_points: int,
    min_arc_sweep_deg: float,
    min_arc_radius_mm: float,
) -> Stroke:
    geometries: list[Geometry] = []
    for path in planned.paths:
        points = _world_points(
            path,
            transform,
            tolerance_mm,
            smoothing_iterations,
            smoothing_strength,
        )
        if enable_arc_fitting:
            geometries.extend(fit_polyline_geometry(
                points,
                tolerance_mm=tolerance_mm,
                closed=path.closed,
                min_arc_points=min_arc_points,
                min_arc_sweep_deg=min_arc_sweep_deg,
                min_arc_radius_mm=min_arc_radius_mm,
            ))
        else:
            simplified = simplify_polyline(points, tolerance_mm=tolerance_mm, closed=path.closed)
            for start, end in zip(simplified, simplified[1:]):
                if start.distance_to(end) > 1.0e-12:
                    geometries.append(Line(start, end))
    if not geometries:
        raise RasterError("stroke simplification removed every segment")
    return Stroke(tuple(geometries))


def _make_drawing(
    plan: StrokePlan,
    transform: PixelWorldTransform,
    tolerance_mm: float,
    smoothing_iterations: int,
    smoothing_strength: float,
    enable_arc_fitting: bool,
    min_arc_points: int,
    min_arc_sweep_deg: float,
    min_arc_radius_mm: float,
) -> Drawing:
    strokes = tuple(
        _planned_stroke_to_stroke(
            stroke,
            transform,
            tolerance_mm,
            smoothing_iterations,
            smoothing_strength,
            enable_arc_fitting,
            min_arc_points,
            min_arc_sweep_deg,
            min_arc_radius_mm,
        )
        for stroke in plan.strokes
    )
    if not strokes:
        raise RasterError("no drawable strokes remain after tracing")
    return Drawing(strokes)


def lineart_to_trj2(
    source: Image.Image | str | Path,
    config: LineartConfig | None = None,
) -> LineartResult:
    """Generate a canonical TRJ2 file and all inspectable intermediate stages."""
    config = config if config is not None else LineartConfig()
    if not isinstance(config, LineartConfig):
        raise RasterError("config must be LineartConfig")
    if isinstance(source, Image.Image):
        preprocess = preprocess_image(source, config.preprocess)
    elif isinstance(source, (str, Path)):
        preprocess = read_lineart(source, config.preprocess)
    else:
        raise RasterError("source must be a Pillow Image or PNG/JPEG path")
    _raise_diagnostics(preprocess.diagnostics)

    skeleton, trace = extract_pixel_paths(
        preprocess.cleaned_mask,
        skeleton_config=config.skeleton,
        trace_config=config.trace,
    )
    # ``extract_pixel_paths`` carries skeleton diagnostics into the returned
    # trace, so append that aggregate once instead of duplicating warnings.
    diagnostics = tuple(preprocess.diagnostics) + tuple(trace.diagnostics)
    _raise_diagnostics(diagnostics)
    plan = plan_strokes(trace)
    if not plan.strokes:
        raise RasterError("no drawable strokes were found")
    all_points = tuple(point for stroke in plan.strokes for point in stroke.points)
    anchor = plan.strokes[0].points[0]
    transform = fit_pixel_transform_points(
        all_points,
        anchor_pixel=anchor,
        anchor_world=config.anchor_world,
        size=config.size,
    )
    drawing = _make_drawing(
        plan,
        transform,
        config.simplify_tolerance_mm,
        config.smoothing_iterations,
        config.smoothing_strength,
        config.enable_arc_fitting,
        config.min_arc_points,
        config.min_arc_sweep_deg,
        config.min_arc_radius_mm,
    )
    toolpath = compile_drawing(drawing, config.compiler)
    encoded = encode_toolpath_trj2(
        toolpath,
        start_yaw_deg=config.start_yaw_deg,
        start_point=drawing.start_point(),
    )
    roundtrip_file = decode_trj2(encoded)
    roundtrip = decode_toolpath_trj2(encoded)
    return LineartResult(
        preprocess=preprocess,
        skeleton=skeleton,
        trace=trace,
        stroke_plan=plan,
        transform=transform,
        drawing=drawing,
        toolpath=toolpath,
        traj2_file=roundtrip_file,
        traj2_bytes=encoded,
        roundtrip_toolpath=roundtrip.toolpath,
        diagnostics=diagnostics,
    )


__all__ = ["LineartConfig", "LineartResult", "lineart_to_trj2"]
