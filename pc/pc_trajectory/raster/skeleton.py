"""Topology-preserving 2-D skeletonization for line-art masks.

The implementation is the Zhang-Suen thinning algorithm expressed with NumPy
array operations. Keeping it here avoids a mandatory OpenCV/scikit-image
dependency while leaving the public result independent of the thinning backend.
Input and output use image coordinates: array[y, x] is a foreground pixel.
"""

from dataclasses import dataclass
import math

import numpy as np

from .config import RasterError
from .preprocess import RasterDiagnostic, _filter_components


@dataclass(frozen=True)
class SkeletonConfig:
    """Controls the bounded thinning loop."""

    max_iterations: int = 1000

    def __post_init__(self) -> None:
        if type(self.max_iterations) is not int or self.max_iterations < 1:
            raise RasterError("max_iterations must be a positive integer")


@dataclass(frozen=True)
class SkeletonResult:
    input_mask: np.ndarray
    skeleton_mask: np.ndarray
    iterations: int
    converged: bool
    diagnostics: tuple[RasterDiagnostic, ...]

    @property
    def input_pixel_count(self) -> int:
        return int(self.input_mask.sum())

    @property
    def skeleton_pixel_count(self) -> int:
        return int(self.skeleton_mask.sum())


def _neighbors(mask: np.ndarray) -> tuple[np.ndarray, ...]:
    padded = np.pad(mask, 1, mode="constant", constant_values=False)
    # Clockwise order around the current pixel: N, NE, E, SE, S, SW, W, NW.
    return (
        padded[:-2, 1:-1],
        padded[:-2, 2:],
        padded[1:-1, 2:],
        padded[2:, 2:],
        padded[2:, 1:-1],
        padded[2:, :-2],
        padded[1:-1, :-2],
        padded[:-2, :-2],
    )


def _deletion_mask(mask: np.ndarray, phase: int) -> np.ndarray:
    neighbors = _neighbors(mask)
    transitions = sum(
        (~neighbors[index] & neighbors[(index + 1) % 8])
        for index in range(8)
    )
    count = sum(neighbors)
    north, northeast, east, southeast, south, southwest, west, northwest = neighbors
    if phase == 1:
        # Zhang-Suen sub-iteration 1:
        # P2*P4*P6 == 0 and P4*P6*P8 == 0.
        connectivity = (~north | ~east | ~south) & (~east | ~south | ~west)
    else:
        # Zhang-Suen sub-iteration 2:
        # P2*P4*P8 == 0 and P2*P6*P8 == 0.
        connectivity = (~north | ~east | ~west) & (~north | ~south | ~west)
    return mask & (count >= 2) & (count <= 6) & (transitions == 1) & connectivity


def skeletonize_mask(
    mask: np.ndarray,
    config: SkeletonConfig | None = None,
) -> SkeletonResult:
    """Return a read-only one-pixel skeleton without modifying ``mask``.

    The input is interpreted as a boolean foreground mask. A non-converged
    result is returned with an ERROR diagnostic so callers can decide whether
    to stop or display the intermediate image.
    """
    config = config if config is not None else SkeletonConfig()
    if not isinstance(config, SkeletonConfig):
        raise RasterError("config must be SkeletonConfig")
    array = np.asarray(mask)
    if array.ndim != 2:
        raise RasterError("mask must be a two-dimensional array")
    if array.size == 0:
        raise RasterError("mask must have nonzero dimensions")
    if array.dtype != np.bool_:
        if not np.issubdtype(array.dtype, np.number):
            raise RasterError("mask must contain boolean or numeric values")
        array = array != 0
    original = np.array(array, dtype=bool, copy=True)
    working = original.copy()
    diagnostics: list[RasterDiagnostic] = []
    if not working.any():
        diagnostics.append(RasterDiagnostic(
            "EMPTY_SKELETON", "Input mask has no foreground pixels.", "error"))

    iterations = 0
    converged = False
    for iterations in range(1, config.max_iterations + 1):
        changed = False
        for phase in (1, 2):
            remove = _deletion_mask(working, phase)
            if remove.any():
                working[remove] = False
                changed = True
        if not changed:
            converged = True
            break
    if not converged and working.any():
        diagnostics.append(RasterDiagnostic(
            "SKELETON_MAX_ITERATIONS",
            f"Skeletonization did not converge within {config.max_iterations} iterations.",
            "error",
        ))
    _, input_components, _ = _filter_components(original, 1)
    _, skeleton_components, _ = _filter_components(working, 1)
    if input_components != skeleton_components:
        diagnostics.append(RasterDiagnostic(
            "SKELETON_COMPONENT_COUNT_CHANGED",
            "Skeletonization changed the 8-connected component count from "
            f"{input_components} to {skeleton_components}.",
            "error",
        ))
    if working.any() and int(working.sum()) == 1:
        diagnostics.append(RasterDiagnostic(
            "SINGLE_PIXEL_SKELETON",
            "Skeleton contains one foreground pixel and cannot form a path.",
            "error",
        ))
    for result in (original, working):
        result.setflags(write=False)
    return SkeletonResult(original, working, iterations, converged, tuple(diagnostics))


__all__ = ["SkeletonConfig", "SkeletonResult", "skeletonize_mask"]
