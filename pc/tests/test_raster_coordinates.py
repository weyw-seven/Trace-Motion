import math

import pytest

from pc_trajectory.geometry import Point2D
from pc_trajectory.raster import PixelPoint, PixelWorldTransform, RasterError, SizeConfig, fit_pixel_transform


def test_asymmetric_bounds_scale_y_flip_and_nonzero_crop_offset():
    # 11 x 6 pixel centers span 10 x 5 units, NOT 11 x 6.
    transform = fit_pixel_transform((7, 13, 18, 19), anchor_pixel=PixelPoint(7, 18))
    assert transform.mm_per_pixel == 10
    assert transform.to_world(PixelPoint(7, 18)) == Point2D(0, 0)
    assert transform.to_world(PixelPoint(17, 13)) == Point2D(100, 50)
    assert transform.to_world(PixelPoint(8, 17)) == Point2D(10, 10)
    bounds = transform.bounds_in_world((7, 13, 18, 19))
    assert (bounds.min_x_mm, bounds.min_y_mm, bounds.max_x_mm, bounds.max_y_mm) == (0, 0, 100, 50)


def test_arbitrary_first_stroke_anchor_and_inverse():
    transform = fit_pixel_transform((5, 8, 16, 29), anchor_pixel=PixelPoint(10, 18),
                                    anchor_world=Point2D(30, -5), size=SizeConfig(width_mm=75))
    assert transform.to_world(PixelPoint(10, 18)) == Point2D(30, -5)
    for point in [PixelPoint(5, 8), PixelPoint(15, 28), PixelPoint(7.25, 19.3)]:
        restored = transform.to_pixel(transform.to_world(point))
        assert restored.x == pytest.approx(point.x)
        assert restored.y == pytest.approx(point.y)
    assert transform.as_dict()["anchor_world_mm"] == {"x": 30, "y": -5}


@pytest.mark.parametrize("size,expected", [
    (SizeConfig(width_mm=80), 8),
    (SizeConfig(height_mm=80), 16),
    (SizeConfig(width_mm=80, height_mm=30), 6),
    (SizeConfig(max_extent_mm=25), 2.5),
])
def test_fit_dimensions_preserves_aspect(size, expected):
    transform = fit_pixel_transform((0, 0, 11, 6), anchor_pixel=PixelPoint(0, 0), size=size)
    assert transform.mm_per_pixel == expected
    box = transform.bounds_in_world((0, 0, 11, 6))
    assert (box.max_x_mm - box.min_x_mm) / (box.max_y_mm - box.min_y_mm) == 2


@pytest.mark.parametrize("bounds", [(3, 4, 4, 15), (3, 4, 14, 5)])
def test_single_axis_paths_use_nonzero_span(bounds):
    transform = fit_pixel_transform(bounds, anchor_pixel=PixelPoint(3, 4))
    assert transform.mm_per_pixel == 10


def test_degenerate_axis_constraint_is_explicit():
    with pytest.raises(RasterError, match="zero extent"):
        fit_pixel_transform((0, 0, 1, 10), anchor_pixel=PixelPoint(0, 0), size=SizeConfig(width_mm=100))
    transform = fit_pixel_transform((0, 0, 1, 11), anchor_pixel=PixelPoint(0, 0),
                                    size=SizeConfig(width_mm=100, height_mm=50))
    assert transform.mm_per_pixel == 5


@pytest.mark.parametrize("bounds", [None, (0, 0, 1, 1), (1, 1, 0, 0), (-1, 0, 4, 4),
                                        (0, 0, 0, 3), (0.0, 0, 3, 3), (0, 0, 3)])
def test_no_scale_for_invalid_or_empty_extent(bounds):
    with pytest.raises(RasterError):
        fit_pixel_transform(bounds, anchor_pixel=PixelPoint(0, 0))


@pytest.mark.parametrize("kwargs", [
    {"max_extent_mm": 0}, {"width_mm": -1}, {"height_mm": math.inf},
    {"width_mm": math.nan}, {"width_mm": True}, {"height_mm": "bad"},
    {"width_mm": 100, "max_extent_mm": 100},
])
def test_invalid_size(kwargs):
    with pytest.raises(RasterError):
        SizeConfig(**kwargs)


@pytest.mark.parametrize("value", [math.inf, math.nan, True, "bad"])
def test_invalid_pixel_point(value):
    with pytest.raises(RasterError):
        PixelPoint(value, 0)


def test_invalid_transform_scale():
    with pytest.raises(RasterError):
        PixelWorldTransform(0, PixelPoint(0, 0))
