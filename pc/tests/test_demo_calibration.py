import io
import json
import math
from pathlib import Path
from urllib.request import Request, urlopen

import pytest
from PIL import Image

from pc_trajectory.demo.browser_camera import BrowserCameraServer
from pc_trajectory.demo.calibration import CalibrationError, PaperCalibration, compute_homography
from pc_trajectory.demo.camera import CameraError, parse_camera_source


def close_pair(actual, expected, tolerance=1.0e-6):
    assert math.isclose(actual[0], expected[0], abs_tol=tolerance, rel_tol=0.0)
    assert math.isclose(actual[1], expected[1], abs_tol=tolerance, rel_tol=0.0)


def test_paper_calibration_maps_perspective_corners_to_bottom_left_world():
    calibration = PaperCalibration.from_points(
        ((100.0, 80.0), (900.0, 60.0), (940.0, 700.0), (80.0, 740.0)),
        210.0,
        297.0,
    )

    close_pair(calibration.image_to_world((100.0, 80.0)), (0.0, 297.0), 1.0e-4)
    close_pair(calibration.image_to_world((900.0, 60.0)), (210.0, 297.0), 1.0e-4)
    close_pair(calibration.image_to_world((940.0, 700.0)), (210.0, 0.0), 1.0e-4)
    close_pair(calibration.image_to_world((80.0, 740.0)), (0.0, 0.0), 1.0e-4)
    assert calibration.max_reprojection_error_mm < 1.0e-4


def test_paper_calibration_round_trip_and_json(tmp_path: Path):
    calibration = PaperCalibration.from_points(
        ((0.0, 0.0), (1000.0, 0.0), (1000.0, 500.0), (0.0, 500.0)),
        200.0,
        100.0,
    )
    path = calibration.save(tmp_path / "paper.json")
    loaded = PaperCalibration.load(path)

    point = (345.0, 123.0)
    world = loaded.image_to_world(point)
    close_pair(loaded.world_to_image(world), point, 1.0e-6)
    data = json.loads(path.read_text(encoding="utf-8"))
    assert data["corner_order"] == ["top_left", "top_right", "bottom_right", "bottom_left"]
    assert loaded.paper_width_mm == 200.0


@pytest.mark.parametrize(
    "source, expected",
    [("0", 0), ("2", 2), ("http://192.168.1.5:8080/video", "http://192.168.1.5:8080/video")],
)
def test_parse_camera_source(source, expected):
    assert parse_camera_source(source) == expected


def test_invalid_camera_source_is_rejected():
    with pytest.raises(CameraError):
        parse_camera_source("")
    with pytest.raises(CameraError):
        parse_camera_source(-1)


def test_degenerate_calibration_is_rejected():
    with pytest.raises(CalibrationError):
        compute_homography(
            ((0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)),
            ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)),
        )


def test_phone_browser_camera_server_receives_jpeg_frame():
    server = BrowserCameraServer(port=0, advertised_host="127.0.0.1", secure=False)
    url = server.start()
    try:
        page = urlopen(url, timeout=2).read()
        assert b"Enable camera" in page
        image = Image.new("RGB", (32, 24), (12, 34, 56))
        payload = io.BytesIO()
        image.save(payload, format="JPEG")
        request = Request(
            url + "frame",
            data=payload.getvalue(),
            method="POST",
            headers={"Content-Type": "image/jpeg"},
        )
        assert urlopen(request, timeout=2).status == 204
        frame = server.latest()
        assert frame is not None
        assert frame.image.size == (32, 24)
        assert frame.image.getpixel((0, 0))[0] > 0
        assert server.latest() is None
    finally:
        server.stop()


def test_phone_browser_page_draws_video_and_exposes_camera_controls():
    server = BrowserCameraServer(port=0, advertised_host="127.0.0.1", secure=False)
    url = server.start()
    try:
        page = urlopen(url, timeout=2).read().decode("utf-8")
        assert "context.drawImage(video" in page
        assert "getCapabilities" in page
        assert "zoom: 1" in page
        assert "height: {ideal: 960}" in page
        assert 'id="camera"' in page
    finally:
        server.stop()
