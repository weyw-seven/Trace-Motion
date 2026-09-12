"""Small Windows desktop UI for the line-art to TRJ2 compiler.

The UI intentionally calls the same ``lineart_to_trj2`` service as the CLI.
It keeps the latest successful result and marks the export state invalid as
soon as the source or any parameter changes.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from PIL import Image, ImageTk

from .artifacts import ArtifactPaths, write_lineart_artifacts
from .raster import LineartConfig, PreprocessConfig, SizeConfig, lineart_to_trj2
from .toolpath import PenDown, PenUp
from .toolpath_compiler import ToolpathCompilerConfig


class LineartApp:
    def __init__(self, root: tk.Tk, *, source: str | Path | None = None, output: str | Path | None = None):
        self.root = root
        self.root.title("Lineart → TRJ2")
        self.root.geometry("1180x760")
        self.root.minsize(900, 620)
        self.result = None
        self.config = None
        self.artifacts: ArtifactPaths | None = None
        self._photo_images: dict[str, ImageTk.PhotoImage] = {}

        self.source_var = tk.StringVar(value=str(source) if source else "")
        self.output_var = tk.StringVar(value=str(output) if output else "test_results/ui_output")
        self.threshold_var = tk.IntVar(value=127)
        self.min_component_var = tk.IntVar(value=80)
        self.max_extent_var = tk.DoubleVar(value=100.0)
        self.simplify_var = tk.DoubleVar(value=0.35)
        self.smooth_iterations_var = tk.IntVar(value=3)
        self.smooth_strength_var = tk.DoubleVar(value=0.5)
        self.arc_fit_var = tk.BooleanVar(value=True)
        self.drawing_speed_var = tk.DoubleVar(value=300.0)
        self.travel_speed_var = tk.DoubleVar(value=500.0)
        self.status_var = tk.StringVar(value="Ready. Choose an image and generate a trajectory.")
        self.stats_var = tk.StringVar(value="No generated result")

        self._build_layout()
        self._bind_stale_state()
        if source:
            self.root.after(150, self.generate)

    def _build_layout(self) -> None:
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)

        controls = ttk.Frame(self.root, padding=12)
        controls.grid(row=0, column=0, sticky="ns")
        preview = ttk.Frame(self.root, padding=(0, 12, 12, 12))
        preview.grid(row=0, column=1, sticky="nsew")
        preview.columnconfigure(0, weight=1)
        preview.rowconfigure(1, weight=1)

        ttk.Label(controls, text="Lineart → TRJ2", font=("Segoe UI", 16, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 14)
        )
        row = self._path_row(controls, 1, "Source image", self.source_var, self._choose_source)
        row = self._path_row(controls, row, "Output folder", self.output_var, self._choose_output)

        ttk.Separator(controls).grid(row=row, column=0, columnspan=2, sticky="ew", pady=12)
        row += 1
        ttk.Label(controls, text="Image and geometry", font=("Segoe UI", 10, "bold")).grid(
            row=row, column=0, columnspan=2, sticky="w", pady=(0, 6)
        )
        row += 1
        row = self._spin_row(controls, row, "Threshold", self.threshold_var, 0, 254)
        row = self._spin_row(controls, row, "Min component px", self.min_component_var, 1, 1000000)
        row = self._spin_row(controls, row, "Max extent mm", self.max_extent_var, 0.1, 10000, increment=1.0)
        row = self._spin_row(controls, row, "Simplify mm", self.simplify_var, 0.01, 100, increment=0.05)
        row = self._spin_row(controls, row, "Smooth iterations", self.smooth_iterations_var, 0, 20)
        row = self._spin_row(controls, row, "Smooth strength", self.smooth_strength_var, 0, 1, increment=0.05)
        ttk.Checkbutton(controls, text="Enable LINE + CIRCLE fitting", variable=self.arc_fit_var).grid(
            row=row, column=0, columnspan=2, sticky="w", pady=5
        )
        row += 1
        ttk.Label(controls, text="Motion", font=("Segoe UI", 10, "bold")).grid(
            row=row, column=0, columnspan=2, sticky="w", pady=(10, 6)
        )
        row += 1
        row = self._spin_row(controls, row, "Drawing speed", self.drawing_speed_var, 1, 5000, increment=10.0)
        row = self._spin_row(controls, row, "Travel speed", self.travel_speed_var, 1, 5000, increment=10.0)

        ttk.Separator(controls).grid(row=row, column=0, columnspan=2, sticky="ew", pady=12)
        row += 1
        self.generate_button = ttk.Button(controls, text="Generate trajectory", command=self.generate)
        self.generate_button.grid(row=row, column=0, columnspan=2, sticky="ew", pady=(0, 6))
        row += 1
        self.export_button = ttk.Button(controls, text="Export current result", command=self.export, state="disabled")
        self.export_button.grid(row=row, column=0, columnspan=2, sticky="ew")
        row += 1
        ttk.Label(controls, textvariable=self.status_var, wraplength=280, foreground="#555").grid(
            row=row, column=0, columnspan=2, sticky="w", pady=(14, 0)
        )

        ttk.Label(preview, textvariable=self.stats_var, anchor="w").grid(row=0, column=0, sticky="ew", pady=(0, 8))
        self.notebook = ttk.Notebook(preview)
        self.notebook.grid(row=1, column=0, sticky="nsew")
        self.preview_labels: dict[str, ttk.Label] = {}
        for key, title in (
            ("source", "Source"),
            ("grayscale", "Grayscale"),
            ("cleaned", "Cleaned"),
            ("trace", "R2 trace"),
            ("toolpath", "WORLD toolpath + pen"),
        ):
            tab = ttk.Frame(self.notebook, padding=8)
            tab.rowconfigure(0, weight=1)
            tab.columnconfigure(0, weight=1)
            label = ttk.Label(tab, text="Generate a result to preview this stage", anchor="center")
            label.grid(row=0, column=0, sticky="nsew")
            self.notebook.add(tab, text=title)
            self.preview_labels[key] = label

    @staticmethod
    def _path_row(parent, row, label, variable, browse_command):
        ttk.Label(parent, text=label).grid(row=row, column=0, columnspan=2, sticky="w")
        entry = ttk.Entry(parent, textvariable=variable, width=34)
        entry.grid(row=row + 1, column=0, sticky="ew", pady=(2, 8))
        ttk.Button(parent, text="…", width=3, command=browse_command).grid(row=row + 1, column=1, padx=(5, 0), pady=(2, 8))
        return row + 2

    @staticmethod
    def _spin_row(parent, row, label, variable, minimum, maximum, *, increment=1.0):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=2)
        ttk.Spinbox(parent, textvariable=variable, from_=minimum, to=maximum, increment=increment, width=12).grid(
            row=row, column=1, sticky="e", pady=2
        )
        return row + 1

    def _bind_stale_state(self) -> None:
        watched = (
            self.source_var, self.output_var, self.threshold_var, self.min_component_var,
            self.max_extent_var, self.simplify_var, self.smooth_iterations_var,
            self.smooth_strength_var, self.arc_fit_var, self.drawing_speed_var,
            self.travel_speed_var,
        )
        for variable in watched:
            variable.trace_add("write", self._mark_stale)

    def _mark_stale(self, *_args) -> None:
        if self.result is not None:
            self.status_var.set("Parameters changed. Generate again before exporting.")
            self.export_button.configure(state="disabled")

    def _choose_source(self) -> None:
        selected = filedialog.askopenfilename(filetypes=[("PNG/JPEG", "*.png *.jpg *.jpeg"), ("All files", "*.*")])
        if selected:
            self.source_var.set(selected)

    def _choose_output(self) -> None:
        selected = filedialog.askdirectory()
        if selected:
            self.output_var.set(selected)

    def _build_config(self) -> LineartConfig:
        return LineartConfig(
            preprocess=PreprocessConfig(
                threshold=int(self.threshold_var.get()),
                min_component_pixels=int(self.min_component_var.get()),
            ),
            size=SizeConfig(max_extent_mm=float(self.max_extent_var.get())),
            simplify_tolerance_mm=float(self.simplify_var.get()),
            smoothing_iterations=int(self.smooth_iterations_var.get()),
            smoothing_strength=float(self.smooth_strength_var.get()),
            enable_arc_fitting=bool(self.arc_fit_var.get()),
            compiler=ToolpathCompilerConfig(
                drawing_speed_mm_s=float(self.drawing_speed_var.get()),
                travel_speed_mm_s=float(self.travel_speed_var.get()),
            ),
        )

    def generate(self) -> None:
        source = Path(self.source_var.get().strip())
        if not source.is_file():
            self._show_error("Select an existing PNG or JPEG source image.")
            return
        try:
            config = self._build_config()
            self.status_var.set("Generating…")
            self.root.update_idletasks()
            result = lineart_to_trj2(source, config)
            artifacts = write_lineart_artifacts(result, config, self.output_var.get(), source=source)
        except Exception as exc:
            self.result = None
            self.config = None
            self.artifacts = None
            self.export_button.configure(state="disabled")
            self._show_error(str(exc))
            return
        self.result = result
        self.config = config
        self.artifacts = artifacts
        self._update_previews(artifacts)
        self.stats_var.set(
            f"{result.stroke_count} strokes  •  {result.line_count} LINE  •  "
            f"{result.circle_count} CIRCLE  •  {result.motion_count} motions  •  "
            f"PEN DOWN {sum(isinstance(r, PenDown) for r in result.toolpath.records)}  •  "
            f"PEN UP {sum(isinstance(r, PenUp) for r in result.toolpath.records)}  •  "
            f"{len(result.traj2_bytes)} bytes"
        )
        self.status_var.set(
            f"Generated successfully. Pen-state review is included in toolpath.png. "
            f"Files saved to {artifacts.output_dir}"
        )
        self.export_button.configure(state="normal")

    def export(self) -> None:
        if self.result is None or self.config is None:
            return
        try:
            self.artifacts = write_lineart_artifacts(
                self.result,
                self.config,
                self.output_var.get(),
                source=self.source_var.get(),
            )
            self.status_var.set(f"Exported {self.artifacts.trajectory.name} and verification previews.")
        except Exception as exc:
            self._show_error(str(exc))

    def _update_previews(self, artifacts: ArtifactPaths) -> None:
        paths = {
            "source": artifacts.stages.get("source"),
            "grayscale": artifacts.stages.get("grayscale"),
            "cleaned": artifacts.stages.get("cleaned"),
            "trace": artifacts.trace_preview,
            "toolpath": artifacts.toolpath_preview,
        }
        for key, label in self.preview_labels.items():
            path = paths.get(key)
            if path is None or not Path(path).is_file():
                label.configure(image="", text="Preview unavailable")
                continue
            image = Image.open(path).convert("RGB")
            image.thumbnail((760, 570), Image.Resampling.LANCZOS)
            photo = ImageTk.PhotoImage(image)
            self._photo_images[key] = photo
            label.configure(image=photo, text="")

    def _show_error(self, message: str) -> None:
        self.status_var.set(f"Error: {message}")
        messagebox.showerror("Lineart → TRJ2", message, parent=self.root)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="python -m pc_trajectory.ui")
    parser.add_argument("--source", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--headless", action="store_true", help="generate artifacts without opening Tkinter")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.headless:
        if args.source is None or args.output is None:
            raise SystemExit("--headless requires --source and --output")
        config = LineartConfig(
            preprocess=PreprocessConfig(threshold=127, min_component_pixels=80),
        )
        result = lineart_to_trj2(args.source, config)
        paths = write_lineart_artifacts(result, config, args.output, source=args.source)
        print(f"UI headless verification: {paths.report}")
        return 0
    root = tk.Tk()
    LineartApp(root, source=args.source, output=args.output)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["LineartApp", "main"]
