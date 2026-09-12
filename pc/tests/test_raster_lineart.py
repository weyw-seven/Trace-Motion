from pathlib import Path

from PIL import Image, ImageDraw

from pc_trajectory.raster import (
    LineartConfig,
    PreprocessConfig,
    SizeConfig,
    lineart_to_trj2,
)
from pc_trajectory.traj2_reader import decode_trj2
from pc_trajectory.traj2_validate import validate_trj2_file


def _line_image() -> Image.Image:
    image = Image.new("L", (32, 24), 255)
    draw = ImageDraw.Draw(image)
    draw.line((4, 4, 27, 4), fill=0, width=1)
    draw.line((15, 4, 15, 19), fill=0, width=1)
    return image


def test_lineart_pipeline_emits_only_firmware_executable_motion_and_pen_records():
    result = lineart_to_trj2(
        _line_image(),
        LineartConfig(size=SizeConfig(max_extent_mm=100), simplify_tolerance_mm=0.01),
    )
    validate_trj2_file(result.traj2_file)
    decoded = decode_trj2(result.traj2_bytes)

    assert result.traj2_file.record_count == decoded.record_count
    assert result.transform.to_world(result.stroke_plan.strokes[0].points[0]).x_mm == 0.0
    assert result.transform.to_world(result.stroke_plan.strokes[0].points[0]).y_mm == 0.0
    assert all(type(record).__name__ in {
        "Trj2LineRecord", "Trj2CircleRecord", "Trj2PenUpRecord", "Trj2PenDownRecord"
    }
               for record in result.traj2_file.records)
    assert result.drawing.bounding_box().width_mm <= 100.0 + 1.0e-9
    assert result.drawing.bounding_box().height_mm <= 100.0 + 1.0e-9


def test_lineart_result_can_write_bytes(tmp_path: Path):
    result = lineart_to_trj2(_line_image())
    output = result.write_traj(tmp_path / "drawing.traj")
    assert output.read_bytes() == result.traj2_bytes


def test_lineart_output_is_deterministic():
    first = lineart_to_trj2(_line_image())
    second = lineart_to_trj2(_line_image())
    assert first.traj2_bytes == second.traj2_bytes
