import json
from pathlib import Path

from PIL import Image, ImageDraw

from pc_trajectory.artifacts import write_lineart_artifacts
from pc_trajectory.raster import LineartConfig, PreprocessConfig, lineart_to_trj2


def test_shared_artifact_writer_creates_complete_bundle(tmp_path: Path):
    image_path = tmp_path / "input.png"
    image = Image.new("L", (24, 24), 255)
    ImageDraw.Draw(image).line((3, 3, 20, 20), fill=0, width=2)
    image.save(image_path)

    config = LineartConfig(preprocess=PreprocessConfig(min_component_pixels=1))
    result = lineart_to_trj2(image_path, config)
    paths = write_lineart_artifacts(result, config, tmp_path / "artifacts", source=image_path)

    assert paths.trajectory.is_file()
    assert paths.trace_preview.is_file()
    assert paths.toolpath_preview is not None and paths.toolpath_preview.is_file()
    assert paths.input_copy is not None and paths.input_copy.is_file()
    assert set(paths.stages) == {"source", "grayscale", "binary", "cleaned"}
    report = json.loads(paths.report.read_text(encoding="utf-8"))
    assert report["source"] == str(image_path.resolve())
    assert report["record_count"] == result.traj2_file.record_count
    assert report["pen_state"]["pen_down_events"] == result.drawing.stroke_count
    assert report["pen_state"]["pen_up_events"] == result.drawing.stroke_count
    assert report["pen_state"]["final"] == "UP"
