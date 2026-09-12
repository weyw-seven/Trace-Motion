"""Convert a one-pixel skeleton into deterministic, edge-complete paths.

The public result remains in image pixel coordinates. R3 owns conversion to
WORLD millimetres and construction of Geometry/Stroke objects.
"""

from dataclasses import dataclass, replace
from enum import Enum
import math

import numpy as np

from .config import RasterError
from .coordinates import PixelPoint
from .preprocess import RasterDiagnostic
from .skeleton import SkeletonConfig, SkeletonResult, skeletonize_mask


_NEIGHBOR_OFFSETS = (
    (-1, 0), (-1, 1), (0, 1), (1, 1),
    (1, 0), (1, -1), (0, -1), (-1, -1),
)


class NodeKind(str, Enum):
    ENDPOINT = "ENDPOINT"
    JUNCTION = "JUNCTION"


@dataclass(frozen=True)
class TraceConfig:
    """Graph policies. No spur pruning or gap bridging is done by default.

    The small-loop filter only applies when ``extract_pixel_paths`` also has
    the pre-skeleton source mask. It targets two parallel routes between the
    same pair of degree-3 junctions whose combined perimeter is comparable to
    the estimated ink width. Standalone closed loops are never affected.
    """

    allow_diagonal: bool = True
    suppress_small_parallel_loops: bool = True
    artifact_loop_perimeter_width_factor: float = 5.0

    def __post_init__(self) -> None:
        if type(self.allow_diagonal) is not bool:
            raise RasterError("allow_diagonal must be bool")
        if type(self.suppress_small_parallel_loops) is not bool:
            raise RasterError("suppress_small_parallel_loops must be bool")
        value = self.artifact_loop_perimeter_width_factor
        if isinstance(value, bool):
            raise RasterError("artifact_loop_perimeter_width_factor must be positive and finite")
        try:
            value = float(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise RasterError("artifact_loop_perimeter_width_factor must be positive and finite") from exc
        if not math.isfinite(value) or value <= 0.0:
            raise RasterError("artifact_loop_perimeter_width_factor must be positive and finite")
        object.__setattr__(self, "artifact_loop_perimeter_width_factor", value)


@dataclass(frozen=True)
class TraceNode:
    node_id: int
    kind: NodeKind
    representative: PixelPoint
    members: tuple[PixelPoint, ...]
    degree: int
    component_index: int


@dataclass(frozen=True)
class PixelPath:
    points: tuple[PixelPoint, ...]
    closed: bool
    component_index: int
    start_node_id: int | None = None
    end_node_id: int | None = None

    @property
    def point_count(self) -> int:
        return len(self.points)

    @property
    def length_px(self) -> float:
        return sum(
            math.hypot(b.x - a.x, b.y - a.y)
            for a, b in zip(self.points, self.points[1:])
        )


@dataclass(frozen=True)
class TraceResult:
    skeleton_mask: np.ndarray
    paths: tuple[PixelPath, ...]
    nodes: tuple[TraceNode, ...]
    component_count: int
    raw_edge_count: int
    contracted_edge_count: int
    external_edge_count: int
    traced_edge_count: int
    diagnostics: tuple[RasterDiagnostic, ...]

    @property
    def untraced_edge_count(self) -> int:
        return self.external_edge_count - self.traced_edge_count

    @property
    def path_count(self) -> int:
        return len(self.paths)


Pixel = tuple[int, int]  # (y, x)
Edge = tuple[Pixel, Pixel]


def _pixel_key(pixel: Pixel) -> tuple[int, int]:
    return pixel[0], pixel[1]


def _edge(a: Pixel, b: Pixel) -> Edge:
    return (a, b) if _pixel_key(a) <= _pixel_key(b) else (b, a)


def _pixel_point(pixel: Pixel) -> PixelPoint:
    return PixelPoint(pixel[1], pixel[0])


def _valid_pixel(mask: np.ndarray, y: int, x: int) -> bool:
    return 0 <= y < mask.shape[0] and 0 <= x < mask.shape[1] and bool(mask[y, x])


def _neighbors_for(mask: np.ndarray, pixel: Pixel, *, diagonal: bool) -> tuple[Pixel, ...]:
    y, x = pixel
    result: list[Pixel] = []
    for dy, dx in _NEIGHBOR_OFFSETS:
        if not diagonal and dy != 0 and dx != 0:
            continue
        ny, nx = y + dy, x + dx
        if not _valid_pixel(mask, ny, nx):
            continue
        if dy != 0 and dx != 0:
            # A diagonal link is redundant when either orthogonal bridge exists.
            if _valid_pixel(mask, y, x + dx) or _valid_pixel(mask, y + dy, x):
                continue
        result.append((ny, nx))
    return tuple(result)


def _build_adjacency(mask: np.ndarray, config: TraceConfig) -> dict[Pixel, tuple[Pixel, ...]]:
    pixels = [(y, x) for y, x in zip(*np.nonzero(mask))]
    return {
        pixel: _neighbors_for(mask, pixel, diagonal=config.allow_diagonal)
        for pixel in pixels
    }


def _component_labels(adjacency: dict[Pixel, tuple[Pixel, ...]]) -> tuple[dict[Pixel, int], int]:
    labels: dict[Pixel, int] = {}
    component = 0
    for start in sorted(adjacency):
        if start in labels:
            continue
        stack = [start]
        labels[start] = component
        while stack:
            current = stack.pop()
            for neighbor in adjacency[current]:
                if neighbor not in labels:
                    labels[neighbor] = component
                    stack.append(neighbor)
        component += 1
    return labels, component


def _group_junction_pixels(
    adjacency: dict[Pixel, tuple[Pixel, ...]],
    degrees: dict[Pixel, int],
    labels: dict[Pixel, int],
) -> list[tuple[Pixel, ...]]:
    junctions = {pixel for pixel, degree in degrees.items() if degree >= 3}
    groups: list[tuple[Pixel, ...]] = []
    unseen = set(junctions)
    while unseen:
        start = min(unseen)
        unseen.remove(start)
        stack = [start]
        group = [start]
        while stack:
            current = stack.pop()
            for neighbor in adjacency[current]:
                if neighbor in unseen and degrees[neighbor] >= 3:
                    unseen.remove(neighbor)
                    stack.append(neighbor)
                    group.append(neighbor)
        groups.append(tuple(sorted(group)))
    groups.sort(key=lambda group: min(group))
    return groups


def _representative(members: tuple[Pixel, ...]) -> Pixel:
    center_y = sum(pixel[0] for pixel in members) / len(members)
    center_x = sum(pixel[1] for pixel in members) / len(members)
    return min(members, key=lambda pixel: ((pixel[0] - center_y) ** 2 + (pixel[1] - center_x) ** 2, pixel))


def _canonicalize(
    points: list[PixelPoint],
    *,
    closed: bool,
) -> tuple[PixelPoint, ...]:
    compact: list[PixelPoint] = []
    for point in points:
        if not compact or point != compact[-1]:
            compact.append(point)
    if closed:
        if compact and compact[0] == compact[-1]:
            compact.pop()
        if len(compact) < 2:
            return tuple(compact + compact[:1])
        candidates: list[tuple[PixelPoint, ...]] = []
        for sequence in (compact, list(reversed(compact))):
            index = min(range(len(sequence)), key=lambda i: (sequence[i].y, sequence[i].x, i))
            rotated = sequence[index:] + sequence[:index]
            candidates.append(tuple(rotated + rotated[:1]))
        return min(candidates, key=lambda sequence: tuple((p.y, p.x) for p in sequence))
    if len(compact) >= 2 and (compact[-1].y, compact[-1].x) < (compact[0].y, compact[0].x):
        compact.reverse()
    return tuple(compact)


def trace_skeleton(
    skeleton_mask: np.ndarray,
    config: TraceConfig | None = None,
) -> TraceResult:
    """Trace every external skeleton edge once into open paths or closed loops."""
    config = config if config is not None else TraceConfig()
    if not isinstance(config, TraceConfig):
        raise RasterError("config must be TraceConfig")
    array = np.asarray(skeleton_mask)
    if array.ndim != 2 or array.size == 0:
        raise RasterError("skeleton_mask must be a nonempty two-dimensional array")
    if array.dtype != np.bool_:
        if not np.issubdtype(array.dtype, np.number):
            raise RasterError("skeleton_mask must contain boolean or numeric values")
        array = array != 0
    array = np.array(array, dtype=bool, copy=True)
    array.setflags(write=False)
    if not array.any():
        return TraceResult(
            skeleton_mask=array,
            paths=(),
            nodes=(),
            component_count=0,
            raw_edge_count=0,
            contracted_edge_count=0,
            external_edge_count=0,
            traced_edge_count=0,
            diagnostics=(RasterDiagnostic(
                "EMPTY_SKELETON", "Skeleton has no foreground pixels.", "error"),),
        )
    adjacency = _build_adjacency(array, config)
    labels, component_count = _component_labels(adjacency)
    degrees = {pixel: len(neighbors) for pixel, neighbors in adjacency.items()}
    raw_edges = {
        _edge(pixel, neighbor)
        for pixel, neighbors in adjacency.items()
        for neighbor in neighbors
    }
    junction_groups = _group_junction_pixels(adjacency, degrees, labels)
    node_members: list[tuple[Pixel, ...]] = []
    node_kinds: list[NodeKind] = []
    for group in junction_groups:
        node_members.append(group)
        node_kinds.append(NodeKind.JUNCTION)
    for pixel in sorted(pixel for pixel, degree in degrees.items() if degree == 1):
        node_members.append((pixel,))
        node_kinds.append(NodeKind.ENDPOINT)
    order = sorted(range(len(node_members)), key=lambda index: (_representative(node_members[index]), node_kinds[index].value))
    node_members = [node_members[index] for index in order]
    node_kinds = [node_kinds[index] for index in order]
    pixel_to_node = {
        pixel: node_id
        for node_id, members in enumerate(node_members)
        for pixel in members
    }
    nodes: list[TraceNode] = []
    for node_id, (kind, members) in enumerate(zip(node_kinds, node_members)):
        representative = _representative(members)
        boundary_edges = {
            _edge(pixel, neighbor)
            for pixel in members
            for neighbor in adjacency[pixel]
            if neighbor not in members
        }
        nodes.append(TraceNode(
            node_id=node_id,
            kind=kind,
            representative=_pixel_point(representative),
            members=tuple(_pixel_point(pixel) for pixel in members),
            degree=len(boundary_edges),
            component_index=labels[representative],
        ))
    internal_edges = {
        _edge(pixel, neighbor)
        for node_id, members in enumerate(node_members)
        for pixel in members
        for neighbor in adjacency[pixel]
        if neighbor in members and pixel_to_node.get(neighbor) == node_id
    }
    internal_edges = {_edge(a, b) for a, b in internal_edges}
    external_edges = raw_edges - internal_edges
    visited = set(internal_edges)
    diagnostics: list[RasterDiagnostic] = []
    for pixel, degree in sorted(degrees.items()):
        if degree == 0:
            diagnostics.append(RasterDiagnostic(
                "ISOLATED_SKELETON_PIXEL",
                f"Pixel ({pixel[1]}, {pixel[0]}) is isolated and cannot form a path.",
                "error",
            ))
    paths: list[PixelPath] = []

    def follow(start_pixel: Pixel, next_pixel: Pixel, start_node: int) -> PixelPath | None:
        edge = _edge(start_pixel, next_pixel)
        if edge in visited:
            return None
        visited.add(edge)
        points = [_pixel_point(_representative(node_members[start_node])), _pixel_point(next_pixel)]
        previous, current = start_pixel, next_pixel
        end_node: int | None = None
        while current not in pixel_to_node:
            candidates = [neighbor for neighbor in adjacency[current] if neighbor != previous and _edge(current, neighbor) not in visited]
            if len(candidates) != 1:
                diagnostics.append(RasterDiagnostic(
                    "AMBIGUOUS_SKELETON_TRACE",
                    f"Trace at ({current[1]}, {current[0]}) has {len(candidates)} unvisited continuations.",
                    "error",
                ))
                return None
            following = candidates[0]
            visited.add(_edge(current, following))
            points.append(_pixel_point(following))
            previous, current = current, following
        end_node = pixel_to_node[current]
        points[-1] = _pixel_point(_representative(node_members[end_node]))
        closed = end_node == start_node
        canonical = _canonicalize(points, closed=closed)
        if len(canonical) < (3 if closed else 2):
            diagnostics.append(RasterDiagnostic(
                "DEGENERATE_PIXEL_PATH",
                "A traced path has too few distinct points and was omitted.",
                "error",
            ))
            return None
        return PixelPath(tuple(canonical), closed, labels[start_pixel], start_node, end_node)

    ports = []
    for node_id, members in enumerate(node_members):
        for pixel in members:
            for neighbor in adjacency[pixel]:
                if _edge(pixel, neighbor) in external_edges:
                    ports.append((node_id, pixel, neighbor))
    for node_id, pixel, neighbor in sorted(ports, key=lambda item: (item[0], item[1], item[2])):
        path = follow(pixel, neighbor, node_id)
        if path is not None:
            paths.append(path)

    # Components with no endpoint/junction are pure cycles. Trace each leftover
    # cycle from its lexicographically smallest edge and choose a canonical side.
    for first, second in sorted(external_edges - visited):
        if _edge(first, second) in visited:
            continue
        start, current = first, second
        visited.add(_edge(start, current))
        points = [_pixel_point(start), _pixel_point(current)]
        previous = start
        while current != start:
            candidates = [neighbor for neighbor in adjacency[current] if neighbor != previous]
            if start in candidates:
                visited.add(_edge(current, start))
                points.append(_pixel_point(start))
                current = start
                break
            candidates = [neighbor for neighbor in candidates if _edge(current, neighbor) not in visited]
            if not candidates:
                diagnostics.append(RasterDiagnostic(
                    "OPEN_CYCLE_TRACE",
                    f"Cycle trace stopped at ({current[1]}, {current[0]}).",
                    "error",
                ))
                break
            following = min(candidates)
            visited.add(_edge(current, following))
            points.append(_pixel_point(following))
            previous, current = current, following
        if current == start:
            canonical = _canonicalize(points, closed=True)
            if len(canonical) >= 3:
                paths.append(PixelPath(tuple(canonical), True, labels[start]))
            else:
                diagnostics.append(RasterDiagnostic(
                    "DEGENERATE_PIXEL_PATH", "A closed component has too few points.", "error"))

    traced = len(visited - internal_edges)
    if traced != len(external_edges):
        diagnostics.append(RasterDiagnostic(
            "UNTRACED_SKELETON_EDGE",
            f"{len(external_edges) - traced} external skeleton edges were not traced.",
            "error",
        ))
    paths.sort(key=lambda path: (
        path.component_index,
        path.points[0].y if path.points else math.inf,
        path.points[0].x if path.points else math.inf,
        path.points[-1].y if path.points else math.inf,
        path.points[-1].x if path.points else math.inf,
    ))
    return TraceResult(
        skeleton_mask=array,
        paths=tuple(paths),
        nodes=tuple(nodes),
        component_count=component_count,
        raw_edge_count=len(raw_edges),
        contracted_edge_count=len(internal_edges),
        external_edge_count=len(external_edges),
        traced_edge_count=traced,
        diagnostics=tuple(diagnostics),
    )


def suppress_small_parallel_loops(
    source_mask: np.ndarray,
    raw_trace: TraceResult,
    config: TraceConfig | None = None,
) -> TraceResult:
    """Remove cap-like parallel loops and retrace the cleaned skeleton.

    A candidate must consist of exactly two paths joining the same pair of
    degree-3 junctions. Only the longer route is removed, and only when the
    pair's perimeter is at most ``estimated_ink_width * factor``. A fresh trace
    after removal collapses the two artificial junctions into the surrounding
    continuous path. The decision is reported explicitly.
    """
    config = config if config is not None else TraceConfig()
    if not isinstance(config, TraceConfig):
        raise RasterError("config must be TraceConfig")
    if not isinstance(raw_trace, TraceResult):
        raise RasterError("raw_trace must be TraceResult")
    array = np.asarray(source_mask)
    if array.ndim != 2 or array.shape != raw_trace.skeleton_mask.shape:
        raise RasterError("source_mask must match the traced skeleton shape")
    if array.dtype != np.bool_:
        if not np.issubdtype(array.dtype, np.number):
            raise RasterError("source_mask must contain boolean or numeric values")
        array = array != 0
    if not config.suppress_small_parallel_loops or not array.any() or not raw_trace.skeleton_mask.any():
        return raw_trace

    estimated_width = float(array.sum()) / float(raw_trace.skeleton_mask.sum())
    perimeter_limit = estimated_width * config.artifact_loop_perimeter_width_factor
    node_by_id = {node.node_id: node for node in raw_trace.nodes}
    parallel: dict[tuple[int, int], list[PixelPath]] = {}
    for path in raw_trace.paths:
        if path.closed or path.start_node_id is None or path.end_node_id is None:
            continue
        if path.start_node_id == path.end_node_id:
            continue
        start = node_by_id.get(path.start_node_id)
        end = node_by_id.get(path.end_node_id)
        if (
            start is None or end is None
            or start.kind is not NodeKind.JUNCTION
            or end.kind is not NodeKind.JUNCTION
            or start.degree != 3 or end.degree != 3
        ):
            continue
        key = tuple(sorted((path.start_node_id, path.end_node_id)))
        parallel.setdefault(key, []).append(path)

    skeleton = np.array(raw_trace.skeleton_mask, copy=True)
    suppressed: list[tuple[PixelPath, float]] = []
    for paths in parallel.values():
        if len(paths) != 2:
            continue
        perimeter = paths[0].length_px + paths[1].length_px
        if perimeter > perimeter_limit:
            continue
        longer = max(
            paths,
            key=lambda path: (
                path.length_px,
                tuple((point.y, point.x) for point in path.points),
            ),
        )
        # Endpoints belong to the junctions and must remain. Interior pixels
        # are exclusive to this graph edge because the path has no critical
        # point between its endpoints.
        for point in longer.points[1:-1]:
            skeleton[int(round(point.y)), int(round(point.x))] = False
        suppressed.append((longer, perimeter))

    if not suppressed:
        return raw_trace

    cleaned = trace_skeleton(skeleton, replace(config, suppress_small_parallel_loops=False))
    diagnostics = list(raw_trace.diagnostics)
    diagnostics.extend(cleaned.diagnostics)
    for path, perimeter in suppressed:
        diagnostics.append(RasterDiagnostic(
            "SMALL_PARALLEL_LOOP_SUPPRESSED",
            "Suppressed a longer parallel route between junctions "
            f"{path.start_node_id} and {path.end_node_id}: perimeter "
            f"{perimeter:.3f}px <= {perimeter_limit:.3f}px "
            f"(estimated ink width {estimated_width:.3f}px).",
            "warning",
        ))
    return replace(cleaned, diagnostics=tuple(diagnostics))


def extract_pixel_paths(
    mask: np.ndarray,
    *,
    skeleton_config: SkeletonConfig | None = None,
    trace_config: TraceConfig | None = None,
) -> tuple[SkeletonResult, TraceResult]:
    """Convenience R2 entry point: mask -> skeleton -> deterministic paths."""
    skeleton = skeletonize_mask(mask, skeleton_config)
    trace_config = trace_config if trace_config is not None else TraceConfig()
    trace = trace_skeleton(skeleton.skeleton_mask, trace_config)
    trace = suppress_small_parallel_loops(mask, trace, trace_config)
    diagnostics = tuple(skeleton.diagnostics) + tuple(trace.diagnostics)
    if diagnostics != trace.diagnostics:
        trace = TraceResult(
            trace.skeleton_mask, trace.paths, trace.nodes, trace.component_count,
            trace.raw_edge_count, trace.contracted_edge_count, trace.external_edge_count,
            trace.traced_edge_count, diagnostics)
    return skeleton, trace


__all__ = [
    "NodeKind", "PixelPath", "TraceConfig", "TraceNode", "TraceResult",
    "extract_pixel_paths", "suppress_small_parallel_loops", "trace_skeleton",
]
