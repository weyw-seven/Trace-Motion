"""Planar camera-to-paper calibration without an OpenCV dependency.

The user clicks four paper corners in image order::

    top-left, top-right, bottom-right, bottom-left

The paper coordinate frame has its origin at the bottom-left corner, +X to
the right and +Y upward.  A normalized DLT homography maps camera pixels to
that WORLD-like paper frame in millimetres.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np


class CalibrationError(ValueError):
    """Raised when paper calibration input is invalid or degenerate."""


def _points_array(points: Iterable[Iterable[float]], *, name: str) -> np.ndarray:
    try:
        array = np.asarray(tuple(tuple(point) for point in points), dtype=float)
    except (TypeError, ValueError) as exc:
        raise CalibrationError(f"{name} must contain numeric 2-D points") from exc
    if array.shape != (4, 2):
        raise CalibrationError(f"{name} must contain exactly four (x, y) points")
    if not np.isfinite(array).all():
        raise CalibrationError(f"{name} must contain only finite points")
    return array


def _validate_quad(points: np.ndarray, *, name: str) -> None:
    # Consecutive clicked corners must not collapse. A zero polygon area also
    # catches collinear or repeated points before SVD produces a bad matrix.
    edges = np.roll(points, -1, axis=0) - points
    if np.any(np.linalg.norm(edges, axis=1) <= 1.0e-9):
        raise CalibrationError(f"{name} contains repeated or too-close corners")
    area = 0.5 * float(
        np.sum(points[:, 0] * np.roll(points[:, 1], -1))
        - np.sum(points[:, 1] * np.roll(points[:, 0], -1))
    )
    if abs(area) <= 1.0e-9:
        raise CalibrationError(f"{name} corners are collinear")


def _normalize_points(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    centroid = np.mean(points, axis=0)
    shifted = points - centroid
    mean_distance = float(np.mean(np.linalg.norm(shifted, axis=1)))
    if mean_distance <= 1.0e-12:
        raise CalibrationError("calibration points have zero spread")
    scale = math.sqrt(2.0) / mean_distance
    transform = np.array(
        [
            [scale, 0.0, -scale * centroid[0]],
            [0.0, scale, -scale * centroid[1]],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    homogeneous = np.column_stack((points, np.ones(len(points))))
    normalized = (transform @ homogeneous.T).T
    return normalized[:, :2], transform


def compute_homography(
    source_points: Iterable[Iterable[float]],
    destination_points: Iterable[Iterable[float]],
) -> np.ndarray:
    """Return a 3x3 projective transform mapping source to destination."""
    source = _points_array(source_points, name="source_points")
    destination = _points_array(destination_points, name="destination_points")
    _validate_quad(source, name="source_points")
    _validate_quad(destination, name="destination_points")

    source_normalized, source_transform = _normalize_points(source)
    destination_normalized, destination_transform = _normalize_points(destination)

    matrix: list[list[float]] = []
    for (x, y), (u, v) in zip(source_normalized, destination_normalized):
        matrix.append([-x, -y, -1.0, 0.0, 0.0, 0.0, u * x, u * y, u])
        matrix.append([0.0, 0.0, 0.0, -x, -y, -1.0, v * x, v * y, v])
    _, singular_values, vh = np.linalg.svd(np.asarray(matrix, dtype=float))
    if singular_values[-1] <= 1.0e-14 and singular_values[-2] <= 1.0e-14:
        raise CalibrationError("calibration homography is underdetermined")
    normalized_h = vh[-1].reshape(3, 3)
    try:
        homography = np.linalg.inv(destination_transform) @ normalized_h @ source_transform
    except np.linalg.LinAlgError as exc:
        raise CalibrationError("calibration homography is not invertible") from exc
    if abs(homography[2, 2]) <= 1.0e-14:
        raise CalibrationError("calibration homography has an invalid scale")
    homography /= homography[2, 2]
    return homography


def _apply_homography(homography: np.ndarray, point: tuple[float, float]) -> tuple[float, float]:
    vector = homography @ np.array([float(point[0]), float(point[1]), 1.0], dtype=float)
    if abs(vector[2]) <= 1.0e-12:
        raise CalibrationError("point maps to an invalid projective location")
    return float(vector[0] / vector[2]), float(vector[1] / vector[2])


@dataclass(frozen=True)
class PaperCalibration:
    """Saved camera pixel -> paper millimetre calibration."""

    image_points: tuple[tuple[float, float], ...]
    paper_width_mm: float
    paper_height_mm: float
    homography: tuple[float, ...]
    max_reprojection_error_mm: float
    version: int = 1

    @classmethod
    def from_points(
        cls,
        image_points: Iterable[Iterable[float]],
        paper_width_mm: float,
        paper_height_mm: float,
    ) -> "PaperCalibration":
        image = _points_array(image_points, name="image_points")
        _validate_quad(image, name="image_points")
        try:
            width = float(paper_width_mm)
            height = float(paper_height_mm)
        except (TypeError, ValueError, OverflowError) as exc:
            raise CalibrationError("paper dimensions must be finite positive numbers") from exc
        if not math.isfinite(width) or width <= 0.0 or not math.isfinite(height) or height <= 0.0:
            raise CalibrationError("paper dimensions must be finite positive numbers")

        # Destination order is bottom-left, but image click order is top-left.
        paper = np.asarray(
            [
                (0.0, height),
                (width, height),
                (width, 0.0),
                (0.0, 0.0),
            ],
            dtype=float,
        )
        homography = compute_homography(image, paper)
        mapped = np.asarray([_apply_homography(homography, tuple(point)) for point in image])
        reprojection_error = float(np.max(np.linalg.norm(mapped - paper, axis=1)))
        return cls(
            image_points=tuple((float(x), float(y)) for x, y in image),
            paper_width_mm=width,
            paper_height_mm=height,
            homography=tuple(float(value) for value in homography.reshape(-1)),
            max_reprojection_error_mm=reprojection_error,
        )

    @property
    def matrix(self) -> np.ndarray:
        return np.asarray(self.homography, dtype=float).reshape(3, 3)

    def image_to_world(self, point: tuple[float, float]) -> tuple[float, float]:
        return _apply_homography(self.matrix, point)

    def world_to_image(self, point: tuple[float, float]) -> tuple[float, float]:
        try:
            inverse = np.linalg.inv(self.matrix)
        except np.linalg.LinAlgError as exc:
            raise CalibrationError("calibration homography cannot be inverted") from exc
        return _apply_homography(inverse, point)

    def image_points_to_world(self, points: Iterable[Iterable[float]]) -> tuple[tuple[float, float], ...]:
        return tuple(self.image_to_world((float(x), float(y))) for x, y in points)

    def as_dict(self) -> dict:
        return {
            "version": self.version,
            "corner_order": ["top_left", "top_right", "bottom_right", "bottom_left"],
            "image_points": [list(point) for point in self.image_points],
            "paper_width_mm": self.paper_width_mm,
            "paper_height_mm": self.paper_height_mm,
            "homography": list(self.homography),
            "max_reprojection_error_mm": self.max_reprojection_error_mm,
            "world_frame": "paper_bottom_left_x_right_y_up_mm",
        }

    def save(self, path: str | Path) -> Path:
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(self.as_dict(), ensure_ascii=False, indent=2), encoding="utf-8")
        return output

    @classmethod
    def load(cls, path: str | Path) -> "PaperCalibration":
        input_path = Path(path)
        try:
            data = json.loads(input_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise CalibrationError(f"cannot read calibration file: {input_path}") from exc
        try:
            calibration = cls.from_points(
                data["image_points"],
                data["paper_width_mm"],
                data["paper_height_mm"],
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise CalibrationError(f"invalid calibration file: {input_path}") from exc
        return calibration


__all__ = ["CalibrationError", "PaperCalibration", "compute_homography"]
