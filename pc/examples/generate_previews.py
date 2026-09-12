"""Generate the three fixed B5 engineering preview fixtures."""

from pathlib import Path

from pc_trajectory.drawing import Drawing, Stroke, StrokeBuilder
from pc_trajectory.geometry import Line, Point2D
from pc_trajectory.preview import (
    save_drawing_preview,
    save_toolpath_preview,
)
from pc_trajectory.toolpath_compiler import compile_drawing


OUTPUT_DIR = Path(__file__).resolve().parent


def build_golden_drawing() -> Drawing:
    builder = StrokeBuilder(Point2D(0.0, 0.0))
    builder.line_to(500.0, 0.0)
    builder.arc(
        center=Point2D(500.0, 250.0),
        sweep_deg=180.0,
    )
    builder.line_to(0.0, 500.0)
    return Drawing((builder.build(),))


def build_two_stroke_drawing() -> Drawing:
    stroke0 = Stroke(
        (
            Line(
                Point2D(0.0, 0.0),
                Point2D(100.0, 0.0),
            ),
        )
    )
    stroke1 = Stroke(
        (
            Line(
                Point2D(200.0, 100.0),
                Point2D(300.0, 100.0),
            ),
        )
    )
    return Drawing((stroke0, stroke1))


def build_sharp_corner_drawing() -> Drawing:
    stroke = Stroke(
        (
            Line(
                Point2D(0.0, 0.0),
                Point2D(100.0, 0.0),
            ),
            Line(
                Point2D(100.0, 0.0),
                Point2D(100.0, 100.0),
            ),
        )
    )
    return Drawing((stroke,))


def main() -> None:
    golden = build_golden_drawing()
    save_drawing_preview(
        golden,
        OUTPUT_DIR / "golden_preview.png",
        show_geometry_labels=True,
    )

    two_strokes = build_two_stroke_drawing()
    save_toolpath_preview(
        compile_drawing(two_strokes),
        OUTPUT_DIR / "two_strokes_preview.png",
        show_motion_labels=True,
    )

    sharp_corner = build_sharp_corner_drawing()
    save_drawing_preview(
        sharp_corner,
        OUTPUT_DIR / "sharp_corner_preview.png",
        show_geometry_labels=True,
    )

    print("Generated:")
    print("  golden_preview.png")
    print("  two_strokes_preview.png")
    print("  sharp_corner_preview.png")


if __name__ == "__main__":
    main()
