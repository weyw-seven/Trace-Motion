"""Generate deterministic N1 map-model and fitted-path acceptance artifacts."""

from __future__ import annotations

import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import matplotlib.pyplot as plt

from pc_trajectory.demo.map_model import MapDocument, MapHome, MapLandmark, MapPath
from pc_trajectory.geometry import Arc, CubicBezier, Line, Point2D


def _sample_geometry(geometry, count: int = 32) -> list[Point2D]:
    if isinstance(geometry, Line):
        return [geometry.start_point(), geometry.end_point()]
    if isinstance(geometry, Arc):
        return [geometry.point_at(index / count) for index in range(count + 1)]
    if isinstance(geometry, CubicBezier):
        return [geometry.point_at(index / count) for index in range(count + 1)]
    raise TypeError(f"unsupported geometry: {type(geometry)!r}")


def main() -> None:
    output = ROOT / "test_results" / "virtual_map_demo" / "n1_editor"
    output.mkdir(parents=True, exist_ok=True)
    path = MapPath(
        (
            Point2D(180.0, 120.0),
            Point2D(320.0, 120.0),
            Point2D(460.0, 130.0),
            Point2D(590.0, 230.0),
            Point2D(700.0, 360.0),
            Point2D(840.0, 420.0),
        ),
        path_id="road-1",
        name="主路线",
    )
    document = MapDocument(
        name="n1_acceptance_map",
        width_mm=1200.0,
        height_mm=800.0,
        home=MapHome(180.0, 120.0, 0.0, 7),
        paths=(path,),
        landmarks=(MapLandmark("station-a", "Station A", 840.0, 420.0),),
    )
    document.save(output / "demo_map.vmap.json")
    fitted = path.fitted_geometry(tolerance_mm=3.0)
    (output / "map_report.json").write_text(
        json.dumps(
            {
                "map_name": document.name,
                "workspace_mm": [document.width_mm, document.height_mm],
                "home_map_mm": [document.home.map_x_mm, document.home.map_y_mm],
                "home_world_mm": [0.0, 0.0],
                "path_count": len(document.paths),
                "raw_point_count": len(path.points_map_mm),
                "fitted_primitive_count": len(fitted),
                "fitted_primitive_types": [type(item).__name__ for item in fitted],
                "path_world_start_mm": [document.path_world_points(path.path_id)[0].x_mm, document.path_world_points(path.path_id)[0].y_mm],
                "path_world_end_mm": [document.path_world_points(path.path_id)[-1].x_mm, document.path_world_points(path.path_id)[-1].y_mm],
                "background_reserved": document.background is None,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    figure, axis = plt.subplots(figsize=(10, 6), constrained_layout=True)
    axis.add_patch(plt.Rectangle((0.0, 0.0), document.width_mm, document.height_mm, fill=False, edgecolor="#64748b", linewidth=2))
    for value in range(0, 1201, 100):
        axis.axvline(value, color="#e2e8f0", linewidth=0.6)
    for value in range(0, 801, 100):
        axis.axhline(value, color="#e2e8f0", linewidth=0.6)

    raw_x = [point.x_mm for point in path.points_map_mm]
    raw_y = [point.y_mm for point in path.points_map_mm]
    axis.plot(raw_x, raw_y, "o--", color="#94a3b8", label="raw hand-drawn points")
    for index, geometry in enumerate(fitted):
        points = _sample_geometry(geometry)
        axis.plot(
            [point.x_mm for point in points],
            [point.y_mm for point in points],
            color="#2563eb",
            linewidth=3,
            label="fitted LINE/CIRCLE" if index == 0 else None,
        )

    axis.scatter([document.home.map_x_mm], [document.home.map_y_mm], color="#dc2626", marker="x", s=100, label="Home (map frame)")
    axis.annotate("Home → WORLD (0,0)", (document.home.map_x_mm, document.home.map_y_mm), xytext=(document.home.map_x_mm + 40, document.home.map_y_mm - 45), arrowprops={"arrowstyle": "->", "color": "#dc2626"}, color="#991b1b")
    for landmark in document.landmarks:
        axis.scatter([landmark.map_x_mm], [landmark.map_y_mm], facecolors="none", edgecolors="#7c3aed", s=100)
        axis.text(landmark.map_x_mm + 12, landmark.map_y_mm, landmark.name, color="#6b21a8", va="center")
    axis.set_title("N1 virtual map acceptance · Map mm / Home offset / fitted path")
    axis.set_xlabel("Map X (mm)")
    axis.set_ylabel("Map Y (mm, upward)")
    axis.set_xlim(-30, document.width_mm + 30)
    axis.set_ylim(-30, document.height_mm + 30)
    axis.set_aspect("equal", adjustable="box")
    axis.legend(loc="upper left")
    axis.grid(False)
    figure.savefig(output / "editor_acceptance.png", dpi=150)
    plt.close(figure)
    (output / "test_report.txt").write_text(
        "N1 map model acceptance generated.\n"
        "Map-local path points remain fixed when Home is changed.\n"
        "Home converts the path into WORLD coordinates at execution time.\n"
        "Raw hand-drawn points and fitted LINE/CIRCLE geometry are shown.\n"
        "Photo background remains a reserved N1B layer.\n",
        encoding="utf-8",
    )
    print(output / "editor_acceptance.png")


if __name__ == "__main__":
    main()
