from collections import deque

import numpy as np
from PIL import Image
import pytest

from pc_trajectory.raster import PreprocessConfig, RasterError, preprocess_image, read_lineart


def ink_image(mask):
    return Image.fromarray(np.where(mask, 0, 255).astype(np.uint8))


def test_asymmetric_foreground_bounds_and_input_preserved():
    mask = np.zeros((8, 12), dtype=bool)
    mask[2:7, 3] = True
    mask[6, 3:10] = True
    image = ink_image(mask)
    original = image.tobytes()
    result = preprocess_image(image)
    assert result.content_bounds == (3, 2, 10, 7)
    np.testing.assert_array_equal(result.cleaned_mask, mask)
    assert result.component_count_before == result.component_count_after == 1
    assert result.removed_pixels == 0
    assert image.tobytes() == original
    assert not result.cleaned_mask.flags.writeable


def test_threshold_is_inclusive_and_white_stays_background():
    image = Image.fromarray(np.array([[0, 126, 127, 128, 254, 255]], dtype=np.uint8))
    result = preprocess_image(image)
    assert result.binary_mask.tolist() == [[True, True, True, False, False, False]]
    assert preprocess_image(image, PreprocessConfig(threshold=254)).binary_mask[0, -1] == False


@pytest.mark.parametrize("invert", [False, True])
def test_transparency_always_background_including_light_ink(invert):
    rgba = np.zeros((3, 5, 4), dtype=np.uint8)
    ink = 255 if invert else 0
    rgba[1, 1:4] = (ink, ink, ink, 255)
    rgba[0, 0] = (ink, ink, ink, 0)  # invisible ink
    rgba[2, 0] = (ink, ink, ink, 64)  # mostly transparent ink
    result = preprocess_image(Image.fromarray(rgba), PreprocessConfig(invert=invert))
    assert result.cleaned_mask.sum() == 3
    assert result.content_bounds == (1, 1, 4, 2)
    assert result.grayscale[0, 0] == 255


def test_light_on_dark_equals_dark_on_light():
    mask = np.eye(7, dtype=bool)
    dark = preprocess_image(ink_image(mask))
    light = preprocess_image(Image.fromarray(np.where(mask, 255, 0).astype(np.uint8)), PreprocessConfig(invert=True))
    np.testing.assert_array_equal(dark.cleaned_mask, light.cleaned_mask)
    assert dark.component_count_after == 1  # diagonal is connected


def test_noise_filter_keeps_exact_threshold_and_does_not_bridge_gap():
    mask = np.zeros((9, 12), dtype=bool)
    mask[1, 1] = True
    mask[4, 1:4] = True
    mask[4, 5:8] = True
    result = preprocess_image(ink_image(mask), PreprocessConfig(min_component_pixels=3))
    assert (result.component_count_before, result.component_count_after) == (3, 2)
    assert result.removed_pixels == 1
    assert not result.cleaned_mask[4, 4]
    assert result.cleaned_mask.sum() == 6
    assert result.diagnostics[0].code == "COMPONENTS_REMOVED"
    assert preprocess_image(ink_image(mask)).cleaned_mask.sum() == 7


def flood_reference(mask, minimum):
    """Independent small-image flood-fill oracle for row-run connectivity."""
    unseen = set(zip(*np.nonzero(mask)))
    cleaned = np.zeros_like(mask)
    before = after = 0
    while unseen:
        seed = unseen.pop()
        queue, component = deque([seed]), {seed}
        while queue:
            y, x = queue.popleft()
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    neighbor = (y + dy, x + dx)
                    if neighbor in unseen:
                        unseen.remove(neighbor)
                        queue.append(neighbor)
                        component.add(neighbor)
        before += 1
        if len(component) >= minimum:
            after += 1
            for y, x in component:
                cleaned[y, x] = True
    return cleaned, before, after


def test_run_components_match_independent_flood_fill():
    rng = np.random.default_rng(4701)
    for probability in (0.1, 0.3, 0.6, 1.0):
        for minimum in (1, 2, 7, 30):
            for _ in range(5):
                mask = rng.random((13, 17)) < probability
                expected, before, after = flood_reference(mask, minimum)
                actual = preprocess_image(ink_image(mask), PreprocessConfig(min_component_pixels=minimum))
                np.testing.assert_array_equal(actual.cleaned_mask, expected)
                assert (actual.component_count_before, actual.component_count_after) == (before, after)


@pytest.mark.parametrize("filtered", [False, True])
def test_empty_result_has_diagnostic_and_no_bounds(filtered):
    mask = np.zeros((4, 4), dtype=bool)
    if filtered:
        mask[1, 1] = True
    result = preprocess_image(ink_image(mask), PreprocessConfig(min_component_pixels=2))
    assert result.content_bounds is None
    assert result.component_count_after == 0
    assert "EMPTY_FOREGROUND" in {d.code for d in result.diagnostics}


def test_single_pixel_preserved_but_diagnosed():
    result = preprocess_image(Image.new("L", (1, 1), 0))
    assert result.cleaned_mask.sum() == 1
    assert result.diagnostics[0].code == "SINGLE_PIXEL"


@pytest.mark.parametrize("extension", ["png", "jpg"])
def test_read_file_and_save_stage_previews(tmp_path, extension):
    image = Image.new("RGB", (20, 10), "white")
    for x in range(4, 16):
        image.putpixel((x, 5), (0, 0, 0))
    path = tmp_path / f"input.{extension}"
    image.save(path)
    result = read_lineart(path)
    assert result.source == str(path.resolve())
    assert result.content_bounds == (4, 5, 16, 6)
    paths = result.save_previews(tmp_path / "stages")
    assert set(paths) == {"source", "grayscale", "binary", "cleaned"}
    with Image.open(paths["cleaned"]) as saved:
        np.testing.assert_array_equal(np.asarray(saved) == 0, result.cleaned_mask)


def test_palette_png_transparency(tmp_path):
    image = Image.new("P", (4, 3), 0)
    image.putpalette([0, 0, 0, 0, 0, 0] + [0] * 762)
    image.putpixel((2, 1), 1)
    path = tmp_path / "palette.png"
    image.save(path, transparency=0)
    result = read_lineart(path)
    assert result.cleaned_mask.sum() == 1
    assert result.cleaned_mask[1, 2]


def test_exif_orientation_applied_before_bounds(tmp_path):
    image = Image.new("RGB", (9, 5), "white")
    for x in range(1, 5):
        image.putpixel((x, 1), (0, 0, 0))
    exif = Image.Exif()
    exif[274] = 6  # rotate 90 degrees clockwise for display
    path = tmp_path / "oriented.jpg"
    image.save(path, exif=exif, quality=100, subsampling=0)
    result = read_lineart(path)
    assert result.rgb.shape == (9, 5, 3)
    assert result.content_bounds == (3, 1, 4, 5)


@pytest.mark.parametrize("kind", ["missing", "corrupt", "gif", "animated"])
def test_bad_file_inputs_are_reported(tmp_path, kind):
    path = tmp_path / "input.png"
    if kind == "corrupt":
        path.write_bytes(b"not an image")
    elif kind == "gif":
        Image.new("RGB", (3, 3)).save(path, format="GIF")
    elif kind == "animated":
        Image.new("RGB", (3, 3), "white").save(
            path, save_all=True, append_images=[Image.new("RGB", (3, 3), "black")])
    with pytest.raises(RasterError):
        read_lineart(path)


def test_image_limit_rejects_before_conversion():
    with pytest.raises(RasterError, match="max_image_pixels"):
        preprocess_image(Image.new("L", (11, 10)), PreprocessConfig(max_image_pixels=100))


@pytest.mark.parametrize("kwargs", [
    {"threshold": -1}, {"threshold": 255}, {"threshold": True}, {"threshold": 1.5},
    {"invert": 1}, {"min_component_pixels": 0}, {"min_component_pixels": 1.5},
    {"max_image_pixels": 0}, {"max_image_pixels": True},
])
def test_invalid_configuration(kwargs):
    with pytest.raises(RasterError):
        PreprocessConfig(**kwargs)
