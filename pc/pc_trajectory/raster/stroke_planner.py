"""Turn traced pixel graph edges into continuous, deterministic strokes.

``trace_skeleton`` deliberately returns one :class:`PixelPath` per graph
edge.  A drawing executor, however, should keep the pen down while it can
walk through adjacent edges.  This module decomposes each connected graph
into the minimum number of open trails (Euler trail decomposition), preserving
every traced edge exactly once.  Closed pixel loops remain individual strokes.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from .config import RasterError
from .coordinates import PixelPoint
from .trace import PixelPath, TraceResult


@dataclass(frozen=True)
class PlannedStroke:
    """An ordered, oriented sequence of traced paths."""

    paths: tuple[PixelPath, ...]
    points: tuple[PixelPoint, ...]
    component_index: int
    closed: bool = False

    def __post_init__(self) -> None:
        paths = tuple(self.paths)
        points = tuple(self.points)
        if not paths:
            raise RasterError("PlannedStroke must contain at least one path")
        if len(points) < 2:
            raise RasterError("PlannedStroke must contain at least two points")
        object.__setattr__(self, "paths", paths)
        object.__setattr__(self, "points", points)

    @property
    def length_px(self) -> float:
        return sum(
            math.hypot(b.x - a.x, b.y - a.y)
            for a, b in zip(self.points, self.points[1:])
        )

    @property
    def path_count(self) -> int:
        return len(self.paths)


@dataclass(frozen=True)
class StrokePlan:
    """The complete deterministic stroke ordering for one trace result."""

    strokes: tuple[PlannedStroke, ...]
    traced_path_count: int

    def __post_init__(self) -> None:
        strokes = tuple(self.strokes)
        if any(not isinstance(stroke, PlannedStroke) for stroke in strokes):
            raise RasterError("strokes must contain PlannedStroke values")
        if type(self.traced_path_count) is not int or self.traced_path_count < 0:
            raise RasterError("traced_path_count must be a non-negative integer")
        if sum(stroke.path_count for stroke in strokes) != self.traced_path_count:
            raise RasterError("stroke plan does not account for every traced path")
        object.__setattr__(self, "strokes", strokes)

    @property
    def stroke_count(self) -> int:
        return len(self.strokes)


@dataclass(frozen=True)
class _GraphEdge:
    edge_id: int
    path_index: int | None
    start: int
    end: int

    @property
    def is_virtual(self) -> bool:
        return self.path_index is None


def _node_key(node_id: int, trace: TraceResult) -> tuple[float, float, int]:
    node = next(node for node in trace.nodes if node.node_id == node_id)
    return node.representative.y, node.representative.x, node_id


def _oriented_path(path: PixelPath, start_node: int, end_node: int) -> tuple[PixelPoint, ...]:
    if path.start_node_id == start_node and path.end_node_id == end_node:
        return path.points
    if path.start_node_id == end_node and path.end_node_id == start_node:
        return tuple(reversed(path.points))
    raise RasterError("stroke planner found a path/node incidence mismatch")


def _join_paths(
    edge_sequence: list[tuple[_GraphEdge, int, int]],
    paths: tuple[PixelPath, ...],
    component_index: int,
) -> PlannedStroke | None:
    real = [item for item in edge_sequence if not item[0].is_virtual]
    if not real:
        return None
    oriented_paths: list[PixelPath] = []
    points: list[PixelPoint] = []
    for edge, start, end in real:
        path = paths[edge.path_index]  # type: ignore[index]
        oriented_points = _oriented_path(path, start, end)
        if not points:
            points.extend(oriented_points)
        else:
            if points[-1] != oriented_points[0]:
                raise RasterError("Euler trail contains discontinuous path endpoints")
            points.extend(oriented_points[1:])
        # Preserve orientation without mutating the immutable source path.
        oriented_paths.append(
            path if oriented_points == path.points else PixelPath(
                points=oriented_points,
                closed=path.closed,
                component_index=path.component_index,
                start_node_id=start,
                end_node_id=end,
            )
        )
    return PlannedStroke(tuple(oriented_paths), tuple(points), component_index)


def _euler_circuit(
    start: int,
    adjacency: dict[int, list[int]],
    edges: dict[int, _GraphEdge],
) -> list[tuple[_GraphEdge, int, int]]:
    """Return an ordered circuit using Hierholzer's algorithm."""
    used: set[int] = set()
    stack_nodes = [start]
    stack_incoming: list[tuple[_GraphEdge, int, int]] = []
    circuit: list[tuple[_GraphEdge, int, int]] = []
    while stack_nodes:
        current = stack_nodes[-1]
        candidate = next(
            (edge_id for edge_id in adjacency[current] if edge_id not in used),
            None,
        )
        if candidate is not None:
            used.add(candidate)
            edge = edges[candidate]
            other = edge.end if edge.start == current else edge.start
            stack_nodes.append(other)
            stack_incoming.append((edge, current, other))
            continue
        stack_nodes.pop()
        if stack_incoming:
            circuit.append(stack_incoming.pop())
    circuit.reverse()
    if len(circuit) != len(edges):
        raise RasterError("Euler circuit did not consume every graph edge")
    return circuit


def _component_nodes(edges: tuple[_GraphEdge, ...]) -> tuple[int, ...]:
    result = set()
    for edge in edges:
        result.add(edge.start)
        result.add(edge.end)
    return tuple(sorted(result))


def plan_strokes(trace: TraceResult) -> StrokePlan:
    """Create the minimum deterministic trail decomposition per component.

    For a connected graph with ``2k`` odd vertices, the decomposition contains
    ``max(1, k)`` trails.  Virtual pairing edges are used only internally and
    are removed before the public plan is returned.
    """
    if not isinstance(trace, TraceResult):
        raise RasterError("trace must be TraceResult")
    paths = trace.paths
    edge_paths: dict[int, PixelPath] = {}
    grouped: dict[int, list[int]] = {}
    closed_paths: list[PixelPath] = []
    for index, path in enumerate(paths):
        if path.closed or path.start_node_id is None or path.end_node_id is None:
            closed_paths.append(path)
            continue
        edge_paths[index] = path
        grouped.setdefault(path.component_index, []).append(index)

    planned: list[PlannedStroke] = []
    for path in sorted(
        closed_paths,
        key=lambda p: (
            p.component_index,
            tuple((point.y, point.x) for point in p.points),
        ),
    ):
        if len(path.points) < 2:
            continue
        planned.append(PlannedStroke((path,), path.points, path.component_index, closed=True))

    for component_index in sorted(grouped):
        path_indices = tuple(sorted(grouped[component_index]))
        incident: dict[int, list[int]] = {}
        real_edges: dict[int, _GraphEdge] = {}
        for local_id, path_index in enumerate(path_indices):
            path = paths[path_index]
            edge = _GraphEdge(local_id, path_index, path.start_node_id, path.end_node_id)  # type: ignore[arg-type]
            real_edges[local_id] = edge
            incident.setdefault(edge.start, []).append(local_id)
            incident.setdefault(edge.end, []).append(local_id)
        nodes = _component_nodes(tuple(real_edges.values()))
        odd = [node for node in nodes if len(incident[node]) % 2 == 1]
        # Pairing in representative order makes the result stable and avoids
        # introducing a geometric optimizer into this stage.
        odd.sort(key=lambda node: _node_key(node, trace))
        all_edges = dict(real_edges)
        next_id = len(all_edges)
        for first, second in zip(odd[::2], odd[1::2]):
            all_edges[next_id] = _GraphEdge(next_id, None, first, second)
            incident.setdefault(first, []).append(next_id)
            incident.setdefault(second, []).append(next_id)
            next_id += 1
        adjacency = {
            node: sorted(edge_ids, key=lambda edge_id: (
                1 if all_edges[edge_id].is_virtual else 0,
                min(all_edges[edge_id].start, all_edges[edge_id].end),
                max(all_edges[edge_id].start, all_edges[edge_id].end),
                edge_id,
            ))
            for node, edge_ids in incident.items()
        }
        start = min(nodes, key=lambda node: _node_key(node, trace))
        circuit = _euler_circuit(start, adjacency, all_edges)
        virtual_positions = [index for index, (edge, _, _) in enumerate(circuit) if edge.is_virtual]
        segments: list[list[tuple[_GraphEdge, int, int]]] = []
        if not virtual_positions:
            segments = [circuit]
        else:
            # The circuit is cyclic.  Rotate just after the first virtual edge
            # so every returned segment begins at a real edge.
            pivot = (virtual_positions[0] + 1) % len(circuit)
            rotated = circuit[pivot:] + circuit[:pivot]
            current: list[tuple[_GraphEdge, int, int]] = []
            for item in rotated:
                if item[0].is_virtual:
                    if current:
                        segments.append(current)
                        current = []
                else:
                    current.append(item)
            if current:
                segments.append(current)
        component_strokes = [
            _join_paths(segment, paths, component_index)
            for segment in segments
        ]
        planned.extend(stroke for stroke in component_strokes if stroke is not None)

    planned.sort(key=lambda stroke: (
        stroke.component_index,
        stroke.points[0].y,
        stroke.points[0].x,
        stroke.points[-1].y,
        stroke.points[-1].x,
        tuple((point.y, point.x) for point in stroke.points),
    ))
    return StrokePlan(tuple(planned), len(paths))


__all__ = ["PlannedStroke", "StrokePlan", "plan_strokes"]
