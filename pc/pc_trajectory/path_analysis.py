"""
Phase B / B3 path analysis and trajectory-quality diagnostics.

This module sits above:
    geometry.py
    drawing.py

and below:
    toolpath.py / toolpath_compiler.py
    preview.py
    curve fitting / optimization

It answers questions such as:
    - What is the position error at a geometry junction?
    - Are the outgoing/incoming tangents continuous by current firmware policy?
    - Does the current ESP32 Executor require a zero-speed junction here?
    - Are there suspiciously short geometry primitives?
    - What are the aggregate statistics for a Stroke / Drawing?

It intentionally does NOT:
    - generate motion profiles
    - predict exact junction speed
    - simulate acceleration/braking in time
    - generate 10 ms control points
    - plan PEN_UP / PEN_DOWN / travel
    - serialize TRJ1/TRJ2
    - perform preview rendering

Firmware-aligned policies currently represented here:
    - position continuity acceptance: 2 mm
    - tangent continuity: dot >= cos(5 deg)
    - executor geometry length epsilon: 1e-3 mm

PC generation quality remains stricter:
    - position continuity target: 0.01 mm

Important:
    Tangent angle is reported for diagnostics, but tangent-continuous
    classification mirrors the firmware's dot-product test directly.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
import math

from .drawing import (
    Drawing,
    PC_CONTINUITY_TOLERANCE_MM,
    Stroke,
)
from .geometry import BoundingBox, Geometry, Vec2


FIRMWARE_CONTINUITY_TOLERANCE_MM = 2.0
FIRMWARE_TANGENT_CONTINUITY_DEG = 5.0
FIRMWARE_LENGTH_EPSILON_MM = 1.0e-3

# Quality warning only. This is NOT a protocol rejection threshold.
SHORT_GEOMETRY_WARNING_MM = 2.0


class AnalysisError(ValueError):
    """Raised for invalid analysis configuration or unsupported input."""


class DiagnosticSeverity(str, Enum):
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"


class DiagnosticCode(str, Enum):
    PC_POSITION_TOLERANCE_EXCEEDED = "PC_POSITION_TOLERANCE_EXCEEDED"
    FIRMWARE_POSITION_DISCONTINUITY = "FIRMWARE_POSITION_DISCONTINUITY"
    STOP_REQUIRED = "STOP_REQUIRED"
    SHORT_GEOMETRY = "SHORT_GEOMETRY"
    FIRMWARE_GEOMETRY_TOO_SHORT = "FIRMWARE_GEOMETRY_TOO_SHORT"


@dataclass(frozen=True)
class Diagnostic:
    severity: DiagnosticSeverity
    code: DiagnosticCode
    message: str

    stroke_index: int | None = None
    geometry_index: int | None = None
    junction_index: int | None = None


@dataclass(frozen=True)
class JunctionAnalysis:
    previous_index: int
    next_index: int

    position_error_mm: float
    pc_position_continuous: bool
    firmware_position_continuous: bool

    outgoing_tangent: Vec2
    incoming_tangent: Vec2
    tangent_dot: float
    tangent_angle_deg: float

    tangent_continuous: bool
    stop_required: bool


@dataclass(frozen=True)
class GeometryAnalysis:
    geometry_index: int
    length_mm: float
    bounding_box: BoundingBox

    firmware_length_valid: bool
    short_geometry: bool


@dataclass(frozen=True)
class StrokeAnalysis:
    geometry: tuple[GeometryAnalysis, ...]
    junctions: tuple[JunctionAnalysis, ...]
    diagnostics: tuple[Diagnostic, ...]

    geometry_count: int
    junction_count: int

    total_length_mm: float
    bounding_box: BoundingBox

    tangent_continuous_count: int
    stop_required_count: int
    short_geometry_count: int
    firmware_too_short_count: int

    worst_junction_angle_deg: float | None

    @property
    def error_count(self) -> int:
        return sum(
            diagnostic.severity is DiagnosticSeverity.ERROR
            for diagnostic in self.diagnostics
        )

    @property
    def warning_count(self) -> int:
        return sum(
            diagnostic.severity is DiagnosticSeverity.WARNING
            for diagnostic in self.diagnostics
        )

    @property
    def passes_execution_policy(self) -> bool:
        """
        True when B3 found no execution-policy ERROR.

        This is intentionally NOT named "firmware_compatible": B3 does not
        decide whether a primitive has a native TRJ/ESP32 representation
        (for example CubicBezier support is an exporter/firmware capability).
        """
        return self.error_count == 0


@dataclass(frozen=True)
class DrawingAnalysis:
    strokes: tuple[StrokeAnalysis, ...]
    diagnostics: tuple[Diagnostic, ...]

    stroke_count: int
    geometry_count: int
    total_junction_count: int

    total_drawing_length_mm: float
    bounding_box: BoundingBox | None

    total_tangent_continuous_count: int
    total_stop_required_count: int
    total_short_geometry_count: int
    total_firmware_too_short_count: int

    worst_junction_angle_deg: float | None

    @property
    def error_count(self) -> int:
        return sum(
            diagnostic.severity is DiagnosticSeverity.ERROR
            for diagnostic in self.diagnostics
        )

    @property
    def warning_count(self) -> int:
        return sum(
            diagnostic.severity is DiagnosticSeverity.WARNING
            for diagnostic in self.diagnostics
        )

    @property
    def passes_execution_policy(self) -> bool:
        """
        True when B3 found no execution-policy ERROR.

        This is intentionally NOT named "firmware_compatible": B3 does not
        decide whether a primitive has a native TRJ/ESP32 representation
        (for example CubicBezier support is an exporter/firmware capability).
        """
        return self.error_count == 0


def _finite_nonnegative(name: str, value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise AnalysisError(f"{name} must be finite")
    if value < 0.0:
        raise AnalysisError(f"{name} must be >= 0")
    return value


def _finite_positive(name: str, value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise AnalysisError(f"{name} must be finite")
    if value <= 0.0:
        raise AnalysisError(f"{name} must be > 0")
    return value


def _clamp_unit(value: float) -> float:
    return max(-1.0, min(1.0, value))


def analyze_junction(
    previous: Geometry,
    next_geometry: Geometry,
    *,
    previous_index: int = 0,
    next_index: int = 1,
    pc_continuity_tolerance_mm: float = PC_CONTINUITY_TOLERANCE_MM,
    firmware_continuity_tolerance_mm: float = FIRMWARE_CONTINUITY_TOLERANCE_MM,
    tangent_tolerance_deg: float = FIRMWARE_TANGENT_CONTINUITY_DEG,
) -> JunctionAnalysis:
    """
    Analyze one geometry junction.

    stop_required has current firmware execution meaning:
        True only when position continuity is firmware-acceptable but the
        tangent dot-product fails the firmware tangent-continuity threshold.

    If position continuity itself is not firmware-acceptable, the junction
    is an execution ERROR rather than a meaningful "stop then continue" case.
    """
    pc_tol = _finite_nonnegative(
        "pc_continuity_tolerance_mm",
        pc_continuity_tolerance_mm,
    )
    firmware_tol = _finite_nonnegative(
        "firmware_continuity_tolerance_mm",
        firmware_continuity_tolerance_mm,
    )
    tangent_tol = _finite_nonnegative(
        "tangent_tolerance_deg",
        tangent_tolerance_deg,
    )
    if tangent_tol > 180.0:
        raise AnalysisError("tangent_tolerance_deg must be <= 180")

    position_error = previous.end_point().distance_to(
        next_geometry.start_point()
    )

    outgoing = previous.end_tangent().normalized()
    incoming = next_geometry.start_tangent().normalized()

    tangent_dot = _clamp_unit(outgoing.dot(incoming))
    tangent_angle_deg = math.degrees(math.acos(tangent_dot))

    # Mirrors trajectory_executor_tangents_continuous():
    #     dot >= cos(tolerance)
    tangent_threshold = math.cos(math.radians(tangent_tol))
    tangent_continuous = tangent_dot >= tangent_threshold

    firmware_position_continuous = position_error <= firmware_tol
    pc_position_continuous = position_error <= pc_tol

    stop_required = (
        firmware_position_continuous
        and not tangent_continuous
    )

    return JunctionAnalysis(
        previous_index=previous_index,
        next_index=next_index,
        position_error_mm=position_error,
        pc_position_continuous=pc_position_continuous,
        firmware_position_continuous=firmware_position_continuous,
        outgoing_tangent=outgoing,
        incoming_tangent=incoming,
        tangent_dot=tangent_dot,
        tangent_angle_deg=tangent_angle_deg,
        tangent_continuous=tangent_continuous,
        stop_required=stop_required,
    )


def analyze_geometry(
    geometry: Geometry,
    *,
    geometry_index: int = 0,
    firmware_length_epsilon_mm: float = FIRMWARE_LENGTH_EPSILON_MM,
    short_geometry_warning_mm: float = SHORT_GEOMETRY_WARNING_MM,
) -> GeometryAnalysis:
    firmware_epsilon = _finite_positive(
        "firmware_length_epsilon_mm",
        firmware_length_epsilon_mm,
    )
    short_threshold = _finite_positive(
        "short_geometry_warning_mm",
        short_geometry_warning_mm,
    )
    if short_threshold < firmware_epsilon:
        raise AnalysisError(
            "short_geometry_warning_mm must be >= firmware_length_epsilon_mm"
        )

    length_mm = geometry.length_mm()

    # Current Executor accepts only length > epsilon.
    firmware_length_valid = length_mm > firmware_epsilon

    # A geometry rejected by firmware is classified as ERROR rather than
    # redundantly receiving the softer SHORT_GEOMETRY warning.
    short_geometry = (
        firmware_length_valid
        and length_mm < short_threshold
    )

    return GeometryAnalysis(
        geometry_index=geometry_index,
        length_mm=length_mm,
        bounding_box=geometry.bounding_box(),
        firmware_length_valid=firmware_length_valid,
        short_geometry=short_geometry,
    )


def _diagnostics_for_geometry(
    analysis: GeometryAnalysis,
    *,
    firmware_length_epsilon_mm: float,
    short_geometry_warning_mm: float,
) -> list[Diagnostic]:
    diagnostics: list[Diagnostic] = []

    if not analysis.firmware_length_valid:
        diagnostics.append(
            Diagnostic(
                severity=DiagnosticSeverity.ERROR,
                code=DiagnosticCode.FIRMWARE_GEOMETRY_TOO_SHORT,
                message=(
                    f"Geometry {analysis.geometry_index} length "
                    f"{analysis.length_mm:.9f} mm does not exceed the "
                    f"current executor length epsilon "
                    f"{firmware_length_epsilon_mm:.9f} mm."
                ),
                geometry_index=analysis.geometry_index,
            )
        )
    elif analysis.short_geometry:
        diagnostics.append(
            Diagnostic(
                severity=DiagnosticSeverity.WARNING,
                code=DiagnosticCode.SHORT_GEOMETRY,
                message=(
                    f"Geometry {analysis.geometry_index} is short "
                    f"({analysis.length_mm:.6f} mm < "
                    f"{short_geometry_warning_mm:.6f} mm quality threshold)."
                ),
                geometry_index=analysis.geometry_index,
            )
        )

    return diagnostics


def _diagnostics_for_junction(
    analysis: JunctionAnalysis,
    *,
    junction_index: int,
    pc_continuity_tolerance_mm: float,
    firmware_continuity_tolerance_mm: float,
    tangent_tolerance_deg: float,
) -> list[Diagnostic]:
    diagnostics: list[Diagnostic] = []

    if not analysis.firmware_position_continuous:
        diagnostics.append(
            Diagnostic(
                severity=DiagnosticSeverity.ERROR,
                code=DiagnosticCode.FIRMWARE_POSITION_DISCONTINUITY,
                message=(
                    f"Junction {junction_index} position error "
                    f"{analysis.position_error_mm:.6f} mm exceeds the current "
                    f"executor continuity-tolerance reference "
                    f"{firmware_continuity_tolerance_mm:.6f} mm."
                ),
                junction_index=junction_index,
            )
        )
    elif not analysis.pc_position_continuous:
        diagnostics.append(
            Diagnostic(
                severity=DiagnosticSeverity.WARNING,
                code=DiagnosticCode.PC_POSITION_TOLERANCE_EXCEEDED,
                message=(
                    f"Junction {junction_index} position error "
                    f"{analysis.position_error_mm:.6f} mm exceeds PC quality "
                    f"tolerance {pc_continuity_tolerance_mm:.6f} mm, "
                    "while remaining within the current executor "
                    "continuity-tolerance reference."
                ),
                junction_index=junction_index,
            )
        )

    if analysis.stop_required:
        diagnostics.append(
            Diagnostic(
                severity=DiagnosticSeverity.WARNING,
                code=DiagnosticCode.STOP_REQUIRED,
                message=(
                    f"Junction {junction_index} tangent angle "
                    f"{analysis.tangent_angle_deg:.6f} deg fails the current "
                    f"executor tangent-continuity policy "
                    f"({tangent_tolerance_deg:.6f} deg); "
                    "zero-speed junction required."
                ),
                junction_index=junction_index,
            )
        )

    return diagnostics


def analyze_stroke(
    stroke: Stroke,
    *,
    pc_continuity_tolerance_mm: float = PC_CONTINUITY_TOLERANCE_MM,
    firmware_continuity_tolerance_mm: float = FIRMWARE_CONTINUITY_TOLERANCE_MM,
    tangent_tolerance_deg: float = FIRMWARE_TANGENT_CONTINUITY_DEG,
    firmware_length_epsilon_mm: float = FIRMWARE_LENGTH_EPSILON_MM,
    short_geometry_warning_mm: float = SHORT_GEOMETRY_WARNING_MM,
) -> StrokeAnalysis:
    geometry_analyses = tuple(
        analyze_geometry(
            geometry,
            geometry_index=index,
            firmware_length_epsilon_mm=firmware_length_epsilon_mm,
            short_geometry_warning_mm=short_geometry_warning_mm,
        )
        for index, geometry in enumerate(stroke.geometries)
    )

    junctions = tuple(
        analyze_junction(
            stroke.geometries[index],
            stroke.geometries[index + 1],
            previous_index=index,
            next_index=index + 1,
            pc_continuity_tolerance_mm=pc_continuity_tolerance_mm,
            firmware_continuity_tolerance_mm=firmware_continuity_tolerance_mm,
            tangent_tolerance_deg=tangent_tolerance_deg,
        )
        for index in range(stroke.geometry_count - 1)
    )

    diagnostics: list[Diagnostic] = []

    for geometry_analysis in geometry_analyses:
        diagnostics.extend(
            _diagnostics_for_geometry(
                geometry_analysis,
                firmware_length_epsilon_mm=firmware_length_epsilon_mm,
                short_geometry_warning_mm=short_geometry_warning_mm,
            )
        )

    for junction_index, junction in enumerate(junctions):
        diagnostics.extend(
            _diagnostics_for_junction(
                junction,
                junction_index=junction_index,
                pc_continuity_tolerance_mm=pc_continuity_tolerance_mm,
                firmware_continuity_tolerance_mm=firmware_continuity_tolerance_mm,
                tangent_tolerance_deg=tangent_tolerance_deg,
            )
        )

    tangent_continuous_count = sum(
        junction.tangent_continuous
        for junction in junctions
    )
    stop_required_count = sum(
        junction.stop_required
        for junction in junctions
    )
    short_geometry_count = sum(
        geometry.short_geometry
        for geometry in geometry_analyses
    )
    firmware_too_short_count = sum(
        not geometry.firmware_length_valid
        for geometry in geometry_analyses
    )

    worst_junction_angle_deg = (
        max(junction.tangent_angle_deg for junction in junctions)
        if junctions
        else None
    )

    return StrokeAnalysis(
        geometry=geometry_analyses,
        junctions=junctions,
        diagnostics=tuple(diagnostics),
        geometry_count=stroke.geometry_count,
        junction_count=len(junctions),
        total_length_mm=stroke.length_mm(),
        bounding_box=stroke.bounding_box(),
        tangent_continuous_count=tangent_continuous_count,
        stop_required_count=stop_required_count,
        short_geometry_count=short_geometry_count,
        firmware_too_short_count=firmware_too_short_count,
        worst_junction_angle_deg=worst_junction_angle_deg,
    )


def analyze_drawing(
    drawing: Drawing,
    **stroke_analysis_options,
) -> DrawingAnalysis:
    stroke_analyses = tuple(
        analyze_stroke(
            stroke,
            **stroke_analysis_options,
        )
        for stroke in drawing.strokes
    )

    diagnostics: list[Diagnostic] = []
    for stroke_index, stroke_analysis in enumerate(stroke_analyses):
        diagnostics.extend(
            replace(diagnostic, stroke_index=stroke_index)
            for diagnostic in stroke_analysis.diagnostics
        )

    worst_angles = [
        analysis.worst_junction_angle_deg
        for analysis in stroke_analyses
        if analysis.worst_junction_angle_deg is not None
    ]

    return DrawingAnalysis(
        strokes=stroke_analyses,
        diagnostics=tuple(diagnostics),
        stroke_count=drawing.stroke_count,
        geometry_count=drawing.geometry_count,
        total_junction_count=sum(
            analysis.junction_count
            for analysis in stroke_analyses
        ),
        total_drawing_length_mm=drawing.total_drawing_length_mm(),
        bounding_box=None if drawing.is_empty() else drawing.bounding_box(),
        total_tangent_continuous_count=sum(
            analysis.tangent_continuous_count
            for analysis in stroke_analyses
        ),
        total_stop_required_count=sum(
            analysis.stop_required_count
            for analysis in stroke_analyses
        ),
        total_short_geometry_count=sum(
            analysis.short_geometry_count
            for analysis in stroke_analyses
        ),
        total_firmware_too_short_count=sum(
            analysis.firmware_too_short_count
            for analysis in stroke_analyses
        ),
        worst_junction_angle_deg=max(worst_angles) if worst_angles else None,
    )


__all__ = [
    "AnalysisError",
    "Diagnostic",
    "DiagnosticCode",
    "DiagnosticSeverity",
    "DrawingAnalysis",
    "FIRMWARE_CONTINUITY_TOLERANCE_MM",
    "FIRMWARE_LENGTH_EPSILON_MM",
    "FIRMWARE_TANGENT_CONTINUITY_DEG",
    "GeometryAnalysis",
    "JunctionAnalysis",
    "SHORT_GEOMETRY_WARNING_MM",
    "StrokeAnalysis",
    "analyze_drawing",
    "analyze_geometry",
    "analyze_junction",
    "analyze_stroke",
]
