import json
import math
from pathlib import Path

import pytest

from pc_trajectory.demo.map_model import (
    PATH_KIND_POLYLINE,
    MapBackground,
    MapDocument,
    MapHome,
    MapLandmark,
    MapModelError,
    MapObstacle,
    MapPath,
)
from pc_trajectory.demo.map_view import MapViewTransform
from pc_trajectory.geometry import Arc, Line, Point2D


def demo_document() -> MapDocument:
    return MapDocument(
        name="demo",
        width_mm=1200.0,
        height_mm=800.0,
        home=MapHome(200.0, 100.0, 0.0, 7),
        paths=(
            MapPath(
                (Point2D(200.0, 100.0), Point2D(500.0, 100.0), Point2D(700.0, 300.0)),
                path_id="road-1",
                name="main road",
            ),
        ),
        landmarks=(MapLandmark("station-a", "A", 700.0, 300.0),),
        background=MapBackground(
            source_type="phone_photo",
            relative_path="demo.assets/background.jpg",
            map_bounds_mm=(0.0, 0.0, 1200.0, 800.0),
            calibration_path="demo.assets/calibration.json",
        ),
    )


def test_home_offset_converts_map_paths_to_world_without_moving_map_points():
    document = demo_document()
    path = document.paths[0]
    assert path.points_map_mm[0] == Point2D(200.0, 100.0)
    assert document.path_world_points("road-1") == (
        Point2D(0.0, 0.0),
        Point2D(300.0, 0.0),
        Point2D(500.0, 200.0),
    )
    moved = document.with_home(MapHome(300.0, 200.0))
    assert moved.paths[0].points_map_mm == path.points_map_mm
    assert moved.path_world_points("road-1")[0] == Point2D(-100.0, -100.0)


def test_map_view_transform_round_trips_and_keeps_world_y_up():
    transform = MapViewTransform.fit_workspace(1200.0, 800.0, 1000.0, 700.0)
    point = Point2D(300.0, 200.0)
    pixel = transform.map_to_pixel(point)
    assert pixel[0] > transform.origin_u_px
    assert pixel[1] < transform.origin_v_px
    restored = transform.pixel_to_map(*pixel)
    assert math.isclose(restored.x_mm, point.x_mm, abs_tol=1.0e-9)
    assert math.isclose(restored.y_mm, point.y_mm, abs_tol=1.0e-9)


def test_map_json_round_trip_preserves_background_and_semantics(tmp_path: Path):
    original = demo_document()
    path = original.save(tmp_path / "demo.vmap.json")
    loaded = MapDocument.load(path)
    assert loaded.to_dict() == original.to_dict()
    assert loaded.background is not None
    assert loaded.background.relative_path == "demo.assets/background.jpg"
    assert loaded.path_world_points("road-1") == original.path_world_points("road-1")


def test_map_json_round_trip_preserves_ui_settings(tmp_path: Path):
    settings = {
        "schema": 1,
        "planner": {"smoothing_iterations": 4, "simplify_tolerance_mm": 0.25},
        "vehicle": {"enabled": True, "front_mm": 80.0},
        "navigation": {
            "mode": "path",
            "selected_path_ids": ["road-1"],
            "target_map": [640.0, 320.0],
        },
        "transport": {"type": "wifi", "host": "192.168.4.1", "tcp_port": "5000"},
    }
    original = demo_document()._replace(settings=settings)
    path = original.save(tmp_path / "settings.vmap.json")
    loaded = MapDocument.load(path)
    assert loaded.settings == settings
    raw = json.loads(path.read_text(encoding="utf-8"))
    assert raw["settings"]["navigation"]["selected_path_ids"] == ["road-1"]


def test_map_json_is_human_readable_and_has_stable_version(tmp_path: Path):
    path = demo_document().save(tmp_path / "demo.vmap.json")
    raw = json.loads(path.read_text(encoding="utf-8"))
    assert raw["version"] == 1
    assert raw["paths"][0]["points_map_mm"][0] == [200.0, 100.0]
    assert "background" in raw


def test_invalid_map_data_is_rejected():
    with pytest.raises(MapModelError):
        MapDocument(name="bad", width_mm=0.0, height_mm=100.0)
    with pytest.raises(MapModelError):
        MapPath((Point2D(0.0, 0.0),), path_id="p")
    with pytest.raises(MapModelError):
        MapDocument.from_dict(
            {
                "version": 1,
                "name": "bad",
                "workspace": {"width_mm": 100, "height_mm": 100},
                "paths": [
                    {"id": "same", "name": "a", "points_map_mm": [[0, 0], [1, 0]]},
                    {"id": "same", "name": "b", "points_map_mm": [[0, 0], [0, 1]]},
                ],
            }
        )
    with pytest.raises(MapModelError):
        MapDocument.from_dict(
            {
                "version": 1,
                "name": "bad-settings",
                "workspace": {"width_mm": 100, "height_mm": 100},
                "settings": [],
            }
        )
    with pytest.raises(MapModelError):
        MapDocument(name="bad-settings", width_mm=100.0, height_mm=100.0, settings={"x": math.nan})


def test_fitting_preserves_order_and_produces_line_or_arc():
    path = MapPath(
        tuple(Point2D(float(x), 0.0) for x in range(0, 101, 10)),
        path_id="line",
    )
    fitted = path.fitted_geometry(tolerance_mm=0.5)
    assert fitted
    assert all(isinstance(item, (Line, Arc)) for item in fitted)
    assert fitted[0].start == Point2D(0.0, 0.0)
    assert fitted[-1].end == Point2D(100.0, 0.0)


def test_polyline_path_preserves_each_vertex_without_smoothing():
    path = MapPath(
        (Point2D(0.0, 0.0), Point2D(100.0, 100.0), Point2D(200.0, 0.0)),
        path_id="exact-polyline",
        path_kind=PATH_KIND_POLYLINE,
    )
    fitted = path.fitted_geometry()
    assert len(fitted) == 2
    assert [geometry.start_point() for geometry in fitted] == [
        Point2D(0.0, 0.0),
        Point2D(100.0, 100.0),
    ]
    assert [geometry.end_point() for geometry in fitted] == [
        Point2D(100.0, 100.0),
        Point2D(200.0, 0.0),
    ]


def test_processing_is_in_map_mm_and_keeps_open_endpoints_exact():
    path = MapPath(
        (
            Point2D(0.0, 0.0),
            Point2D(10.0, 4.0),
            Point2D(20.0, -3.0),
            Point2D(30.0, 0.0),
        ),
        path_id="jitter",
    )
    processed = path.processed_points(smoothing_iterations=3, smoothing_strength=0.5, simplify_tolerance_mm=0.0)
    assert processed[0] == path.points_map_mm[0]
    assert processed[-1] == path.points_map_mm[-1]
    assert len(processed) == len(path.points_map_mm)


def test_landmark_can_be_added_replaced_and_removed():
    document = demo_document()
    extra = MapLandmark("station-b", "B", 900.0, 500.0)
    added = document.with_landmark(extra)
    assert [item.landmark_id for item in added.landmarks] == ["station-a", "station-b"]
    removed = added.without_landmark("station-b")
    assert [item.landmark_id for item in removed.landmarks] == ["station-a"]


def test_path_can_be_removed_without_affecting_landmarks():
    document = demo_document().without_path("road-1")
    assert document.paths == ()
    assert len(document.landmarks) == 1


def test_obstacle_round_trip_and_rejects_self_intersection(tmp_path: Path):
    obstacle = MapObstacle(
        (Point2D(10.0, 10.0), Point2D(60.0, 10.0), Point2D(60.0, 50.0), Point2D(10.0, 50.0)),
        obstacle_id="wall",
        name="Wall",
    )
    document = MapDocument.new().with_obstacle(obstacle)
    saved = document.save(tmp_path / "obstacles.vmap.json")
    loaded = MapDocument.load(saved)
    assert loaded.obstacles[0].to_dict() == obstacle.to_dict()
    with pytest.raises(MapModelError, match="self-intersect"):
        MapObstacle(
            (Point2D(0.0, 0.0), Point2D(50.0, 50.0), Point2D(0.0, 50.0), Point2D(50.0, 0.0)),
        )
