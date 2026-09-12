"""Small dependency-free diagnostic renderer for R2 pixel paths."""

from pathlib import Path

from PIL import Image, ImageDraw

from .config import RasterError
from .trace import NodeKind, TraceResult


_PATH_COLORS = (
    (220, 40, 40), (40, 110, 220), (30, 155, 80),
    (190, 90, 190), (220, 135, 30), (30, 160, 170),
)


def save_trace_preview(
    result: TraceResult,
    path: str | Path,
    *,
    scale: int | None = None,
    margin: int = 12,
) -> Path:
    """Render skeleton, colored paths, endpoints and junctions to a PNG.

    ``scale=None`` chooses 1..8 automatically and keeps the longest output
    dimension near 1600 pixels. An explicit scale is still available for
    pixel-level debugging.
    """
    if not isinstance(result, TraceResult):
        raise RasterError("result must be TraceResult")
    if type(margin) is not int or margin < 0:
        raise RasterError("margin must be a nonnegative integer")
    height, width = result.skeleton_mask.shape
    if scale is None:
        scale = max(1, min(8, 1600 // max(width + margin * 2, height + margin * 2)))
    elif type(scale) is not int or scale < 1:
        raise RasterError("scale must be a positive integer or None")
    image = Image.new("RGB", ((width + margin * 2) * scale, (height + margin * 2) * scale), "white")
    draw = ImageDraw.Draw(image)

    def xy(point):
        return ((point.x + margin) * scale, (point.y + margin) * scale)

    for y, x in zip(*result.skeleton_mask.nonzero()):
        left = (x + margin) * scale
        top = (y + margin) * scale
        draw.rectangle((left, top, left + scale - 1, top + scale - 1), fill=(55, 55, 55))
    for index, pixel_path in enumerate(result.paths):
        if len(pixel_path.points) >= 2:
            draw.line([xy(point) for point in pixel_path.points], fill=_PATH_COLORS[index % len(_PATH_COLORS)], width=max(1, scale // 2))
    for node in result.nodes:
        point = xy(node.representative)
        radius = max(2, scale // 2)
        color = (20, 150, 50) if node.kind is NodeKind.ENDPOINT else (210, 30, 30)
        draw.ellipse((point[0] - radius, point[1] - radius, point[0] + radius, point[1] + radius), fill=color)
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output)
    return output


__all__ = ["save_trace_preview"]
