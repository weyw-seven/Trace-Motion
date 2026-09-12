"""PNG/JPEG -> oriented RGB, normalized grayscale and cleaned foreground mask.

Coordinates remain in the EXIF-oriented full image. Foreground bounds are
half-open (left, top, right, bottom), like Pillow crop boxes. True means ink.
Filtering never bridges gaps; removed component counts are exposed explicitly.
"""

from dataclasses import dataclass
from pathlib import Path
import warnings

import numpy as np
from PIL import Image, ImageOps, UnidentifiedImageError

from .config import PreprocessConfig, RasterError


@dataclass(frozen=True)
class RasterDiagnostic:
    code: str
    message: str
    severity: str = "warning"


@dataclass(frozen=True)
class PreprocessResult:
    rgb: np.ndarray
    grayscale: np.ndarray
    binary_mask: np.ndarray
    cleaned_mask: np.ndarray
    content_bounds: tuple[int, int, int, int] | None
    component_count_before: int
    component_count_after: int
    removed_pixels: int
    diagnostics: tuple[RasterDiagnostic, ...]
    config: PreprocessConfig
    source: str | None = None

    def save_previews(self, directory: str | Path) -> dict[str, Path]:
        """Save lossless stage images; foreground renders black on white."""
        folder = Path(directory)
        folder.mkdir(parents=True, exist_ok=True)
        stages = {
            "source": self.rgb,
            "grayscale": self.grayscale,
            "binary": np.where(self.binary_mask, 0, 255).astype(np.uint8),
            "cleaned": np.where(self.cleaned_mask, 0, 255).astype(np.uint8),
        }
        paths = {}
        for name, array in stages.items():
            path = folder / f"{name}.png"
            Image.fromarray(array).save(path)
            paths[name] = path
        return paths


def _filter_components(mask: np.ndarray, minimum: int) -> tuple[np.ndarray, int, int]:
    """Row runs + union/find: 8-connected components without per-pixel objects."""
    parents: list[int] = []
    sizes: list[int] = []
    runs: list[tuple[int, int, int, int]] = []
    previous: list[tuple[int, int, int]] = []

    def root(label: int) -> int:
        while parents[label] != label:
            parents[label] = parents[parents[label]]
            label = parents[label]
        return label

    for y, row in enumerate(mask):
        transitions = np.flatnonzero(np.diff(np.pad(row.astype(np.int8), (1, 1))))
        current = []
        first_possible = 0
        for left, right in zip(transitions[::2], transitions[1::2]):
            left, right = int(left), int(right)  # right is exclusive
            label = len(parents)
            parents.append(label)
            sizes.append(right - left)
            while first_possible < len(previous) and previous[first_possible][1] < left:
                first_possible += 1
            index = first_possible
            while index < len(previous) and previous[index][0] <= right:
                other = root(previous[index][2])
                own = root(label)
                if own != other:
                    if sizes[own] < sizes[other]:
                        own, other = other, own
                    parents[other] = own
                    sizes[own] += sizes[other]
                index += 1
            current.append((left, right, label))
            runs.append((y, left, right, label))
        previous = current

    roots = {root(i) for i in range(len(parents))}
    kept = {i for i in roots if sizes[i] >= minimum}
    cleaned = np.zeros_like(mask)
    for y, left, right, label in runs:
        if root(label) in kept:
            cleaned[y, left:right] = True
    return cleaned, len(roots), len(kept)


def preprocess_image(image: Image.Image, config: PreprocessConfig | None = None) -> PreprocessResult:
    """Process a Pillow image without mutating it. In-memory images aid testing/UI."""
    config = config if config is not None else PreprocessConfig()
    if not isinstance(config, PreprocessConfig):
        raise RasterError("config must be PreprocessConfig")
    if not isinstance(image, Image.Image):
        raise RasterError("image must be a Pillow Image")
    width, height = image.size
    if width < 1 or height < 1:
        raise RasterError("image must have nonzero dimensions")
    if width * height > config.max_image_pixels:
        raise RasterError(f"image exceeds max_image_pixels={config.max_image_pixels}; resize explicitly")
    if getattr(image, "n_frames", 1) != 1:
        raise RasterError("multi-frame images are not supported")

    oriented = ImageOps.exif_transpose(image)
    rgba = oriented.convert("RGBA")
    # Light-ink mode composites on black before inversion, so transparent pixels
    # remain background instead of turning into false foreground.
    background = (0, 0, 0, 255) if config.invert else (255, 255, 255, 255)
    composited = Image.alpha_composite(Image.new("RGBA", rgba.size, background), rgba).convert("RGB")
    rgb = np.array(composited, dtype=np.uint8)
    gray = np.array(composited.convert("L"), dtype=np.uint8)
    if config.invert:
        gray = 255 - gray
    binary = gray <= config.threshold
    cleaned, before, after = _filter_components(binary, config.min_component_pixels)
    removed = int(binary.sum() - cleaned.sum())
    diagnostics = []
    if before != after:
        diagnostics.append(RasterDiagnostic(
            "COMPONENTS_REMOVED", f"Removed {before - after} components ({removed} pixels)."))
    if after == 0:
        bounds = None
        diagnostics.append(RasterDiagnostic("EMPTY_FOREGROUND", "No foreground remains; no drawing can be generated.", "error"))
    else:
        ys = np.flatnonzero(cleaned.any(axis=1))
        xs = np.flatnonzero(cleaned.any(axis=0))
        bounds = (int(xs[0]), int(ys[0]), int(xs[-1]) + 1, int(ys[-1]) + 1)
        if bounds[2] - bounds[0] == 1 and bounds[3] - bounds[1] == 1:
            diagnostics.append(RasterDiagnostic("SINGLE_PIXEL", "A single pixel has no path extent.", "error"))
    for array in (rgb, gray, binary, cleaned):
        array.setflags(write=False)
    return PreprocessResult(rgb, gray, binary, cleaned, bounds, before, after, removed,
                            tuple(diagnostics), config)


def read_lineart(path: str | Path, config: PreprocessConfig | None = None) -> PreprocessResult:
    """Decode by file contents; allow PNG/JPEG only and report input errors."""
    from dataclasses import replace

    config = config if config is not None else PreprocessConfig()
    if not isinstance(config, PreprocessConfig):
        raise RasterError("config must be PreprocessConfig")
    source = Path(path)
    try:
        with warnings.catch_warnings():
            warnings.simplefilter("error", Image.DecompressionBombWarning)
            with Image.open(source) as image:
                if image.format not in ("PNG", "JPEG"):
                    raise RasterError("only PNG and JPEG inputs are supported")
                result = preprocess_image(image, config)
    except (OSError, UnidentifiedImageError, Image.DecompressionBombError,
            Image.DecompressionBombWarning, SyntaxError) as exc:
        raise RasterError(f"Cannot read lineart {source}: {exc}") from exc
    return replace(result, source=str(source.resolve()))
