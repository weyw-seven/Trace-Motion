from pathlib import Path

from pc_trajectory.raster import (
    PreprocessConfig,
    extract_pixel_paths,
    read_lineart,
    save_trace_preview,
)

input_path = Path(r"D:\esp-projects\pc_trajectory\graph\image.png")
output_dir = Path(r"D:\esp-projects\pc_trajectory\r2_check")

lineart = read_lineart(
    input_path,
    PreprocessConfig(
        threshold=127,
        invert=False,
        min_component_pixels=80,
    ),
)

lineart.save_previews(output_dir / "r1")

skeleton, trace = extract_pixel_paths(lineart.cleaned_mask)
save_trace_preview(trace, output_dir / "r2_trace.png")

print("foreground pixels:", skeleton.input_pixel_count)
print("skeleton pixels:", skeleton.skeleton_pixel_count)
print("components:", trace.component_count)
print("paths:", trace.path_count)
print("external edges:", trace.external_edge_count)
print("traced edges:", trace.traced_edge_count)
print("untraced edges:", trace.untraced_edge_count)

for diagnostic in trace.diagnostics:
    print(diagnostic.severity, diagnostic.code, diagnostic.message)