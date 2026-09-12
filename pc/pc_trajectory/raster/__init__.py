"""Raster preprocessing, skeleton extraction and pixel-path tracing."""

from .config import PreprocessConfig, RasterError, SizeConfig
from .coordinates import PixelPoint, PixelWorldTransform, fit_pixel_transform, fit_pixel_transform_points
from .preprocess import PreprocessResult, RasterDiagnostic, preprocess_image, read_lineart
from .skeleton import SkeletonConfig, SkeletonResult, skeletonize_mask
from .trace import (
    NodeKind,
    PixelPath,
    TraceConfig,
    TraceNode,
    TraceResult,
    extract_pixel_paths,
    suppress_small_parallel_loops,
    trace_skeleton,
)
from .trace_preview import save_trace_preview
from .simplify import simplify_polyline, smooth_polyline
from .curve_fit import fit_polyline_geometry
from .stroke_planner import PlannedStroke, StrokePlan, plan_strokes
from .lineart import LineartConfig, LineartResult, lineart_to_trj2

__all__ = [
    "PreprocessConfig", "RasterError", "SizeConfig", "PixelPoint",
    "PixelWorldTransform", "fit_pixel_transform", "fit_pixel_transform_points", "PreprocessResult",
    "RasterDiagnostic", "preprocess_image", "read_lineart",
    "SkeletonConfig", "SkeletonResult", "skeletonize_mask", "NodeKind",
    "PixelPath", "TraceConfig", "TraceNode", "TraceResult",
    "extract_pixel_paths", "trace_skeleton",
    "suppress_small_parallel_loops",
    "save_trace_preview",
    "simplify_polyline", "smooth_polyline", "PlannedStroke", "StrokePlan", "plan_strokes",
    "fit_polyline_geometry",
    "LineartConfig", "LineartResult", "lineart_to_trj2",
]
