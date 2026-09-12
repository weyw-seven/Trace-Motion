"""First-stage visual-navigation calibration tool.

The tool is deliberately independent from the trajectory compiler UI. It can
load a local image for deterministic testing or connect to a webcam/phone
HTTP stream. Click paper corners in the order TL, TR, BR, BL, enter the real
paper dimensions, and save a JSON calibration for the later navigation UI.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from PIL import Image, ImageTk

from .demo.browser_camera import BrowserCameraError, BrowserCameraServer
from .demo.calibration import CalibrationError, PaperCalibration
from .demo.camera import CameraError, CameraFrame, CameraWorker


CORNER_NAMES = ("TL", "TR", "BR", "BL")


def _parse_points(text: str) -> tuple[tuple[float, float], ...]:
    points = []
    for item in text.split(";"):
        try:
            x_text, y_text = item.strip().split(",")
            points.append((float(x_text), float(y_text)))
        except (ValueError, TypeError) as exc:
            raise SystemExit("--points must look like x,y;x,y;x,y;x,y") from exc
    if len(points) != 4:
        raise SystemExit("--points must contain exactly four points")
    return tuple(points)


class CalibrationApp:
    def __init__(
        self,
        root: tk.Tk,
        *,
        source: str | None = None,
        output: str | Path | None = None,
        start_phone_browser: bool = False,
        browser_secure: bool = True,
        browser_port: int = 8765,
        browser_host: str | None = None,
    ) -> None:
        self.root = root
        self.root.title("Visual Navigation · Paper Calibration")
        self.root.geometry("1120x760")
        self.root.minsize(900, 620)

        self.source_var = tk.StringVar(value=source or "0")
        self.output_var = tk.StringVar(
            value=str(output or "test_results/demo_phase1/paper_calibration.json")
        )
        self.width_var = tk.DoubleVar(value=210.0)
        self.height_var = tk.DoubleVar(value=297.0)
        self.status_var = tk.StringVar(value="Load an image or connect a phone camera.")
        self.cursor_var = tk.StringVar(value="Camera pixel: —    Paper WORLD: —")
        self.browser_url_var = tk.StringVar(value="Phone browser: not running")

        self.canvas = tk.Canvas(root, background="#20242b", highlightthickness=0)
        self.canvas.grid(row=0, column=1, rowspan=3, sticky="nsew", padx=(0, 12), pady=12)
        root.columnconfigure(1, weight=1)
        root.rowconfigure(0, weight=1)

        self._photo: ImageTk.PhotoImage | None = None
        self._image: Image.Image | None = None
        self._display_scale = 1.0
        self._display_offset = (0.0, 0.0)
        self._corners: list[tuple[float, float]] = []
        self.calibration: PaperCalibration | None = None
        self.worker: CameraWorker | None = None
        self.browser_server: BrowserCameraServer | None = None
        self._browser_secure = browser_secure
        self._browser_port = browser_port
        self._browser_host = browser_host

        self._build_controls()
        self.canvas.bind("<Button-1>", self._on_canvas_click)
        self.canvas.bind("<Motion>", self._on_canvas_motion)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(100, self._poll_camera)
        if source:
            self.root.after(150, self.connect)
        elif start_phone_browser:
            self.root.after(150, self.start_phone_browser)

    def _build_controls(self) -> None:
        controls = ttk.Frame(self.root, padding=12)
        controls.grid(row=0, column=0, sticky="nsw")
        ttk.Label(
            controls,
            text="Visual Navigation",
            font=("Segoe UI", 16, "bold"),
        ).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 4))
        ttk.Label(
            controls,
            text="Phase 1 · paper calibration",
            foreground="#555",
        ).grid(row=1, column=0, columnspan=2, sticky="w", pady=(0, 14))

        row = 2
        ttk.Label(controls, text="Phone / camera source").grid(
            row=row, column=0, columnspan=2, sticky="w"
        )
        row += 1
        ttk.Entry(controls, textvariable=self.source_var, width=32).grid(
            row=row, column=0, sticky="ew", pady=(2, 4)
        )
        ttk.Button(controls, text="Connect", command=self.connect).grid(
            row=row, column=1, padx=(6, 0), pady=(2, 4)
        )
        row += 1
        ttk.Button(controls, text="Disconnect", command=self.disconnect).grid(
            row=row, column=0, columnspan=2, sticky="ew", pady=(0, 10)
        )
        row += 1
        ttk.Button(controls, text="Start phone browser camera", command=self.start_phone_browser).grid(
            row=row, column=0, columnspan=2, sticky="ew", pady=(0, 4)
        )
        row += 1
        ttk.Label(
            controls,
            textvariable=self.browser_url_var,
            wraplength=270,
            foreground="#555",
        ).grid(row=row, column=0, columnspan=2, sticky="w", pady=(0, 8))
        row += 1
        ttk.Button(controls, text="Load local image", command=self.load_image).grid(
            row=row, column=0, columnspan=2, sticky="ew", pady=(0, 12)
        )
        row += 1

        ttk.Separator(controls).grid(row=row, column=0, columnspan=2, sticky="ew", pady=8)
        row += 1
        ttk.Label(controls, text="Paper size (mm)", font=("Segoe UI", 10, "bold")).grid(
            row=row, column=0, columnspan=2, sticky="w", pady=(0, 6)
        )
        row += 1
        self._value_row(controls, row, "Width", self.width_var, 1.0, 2000.0)
        row += 1
        self._value_row(controls, row, "Height", self.height_var, 1.0, 3000.0)
        row += 1
        ttk.Label(
            controls,
            text="Click corners in order: TL → TR → BR → BL",
            wraplength=260,
            foreground="#555",
        ).grid(row=row, column=0, columnspan=2, sticky="w", pady=(8, 8))
        row += 1
        ttk.Button(controls, text="Clear corner points", command=self.clear_corners).grid(
            row=row, column=0, columnspan=2, sticky="ew", pady=(0, 6)
        )
        row += 1
        ttk.Button(controls, text="Calibrate and save", command=self.calibrate).grid(
            row=row, column=0, columnspan=2, sticky="ew", pady=(0, 12)
        )
        row += 1

        ttk.Label(controls, text="Calibration JSON").grid(
            row=row, column=0, columnspan=2, sticky="w"
        )
        row += 1
        ttk.Entry(controls, textvariable=self.output_var, width=32).grid(
            row=row, column=0, columnspan=2, sticky="ew", pady=(2, 8)
        )
        row += 1
        ttk.Label(controls, textvariable=self.cursor_var, wraplength=270).grid(
            row=row, column=0, columnspan=2, sticky="w", pady=(8, 4)
        )
        row += 1
        ttk.Label(
            controls,
            textvariable=self.status_var,
            wraplength=270,
            foreground="#555",
        ).grid(row=row, column=0, columnspan=2, sticky="w", pady=(4, 0))

    @staticmethod
    def _value_row(parent, row, label, variable, minimum, maximum) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=2)
        ttk.Spinbox(
            parent,
            textvariable=variable,
            from_=minimum,
            to=maximum,
            increment=1.0,
            width=12,
        ).grid(row=row, column=1, sticky="e", pady=2)

    def connect(self) -> None:
        self.disconnect()
        try:
            self.worker = CameraWorker(self.source_var.get().strip())
            self.worker.start()
        except CameraError as exc:
            self.worker = None
            self.status_var.set(f"Camera error: {exc}")
            return
        self.status_var.set(f"Connected to {self.source_var.get().strip()}; waiting for frames…")

    def start_phone_browser(self) -> None:
        self.disconnect()
        try:
            self.browser_server = BrowserCameraServer(
                port=self._browser_port,
                advertised_host=self._browser_host,
                secure=self._browser_secure,
            )
            url = self.browser_server.start()
        except BrowserCameraError as exc:
            self.browser_server = None
            self.browser_url_var.set("Phone browser: unavailable")
            self.status_var.set(f"Phone browser error: {exc}")
            return
        self.browser_url_var.set(f"Phone URL: {url}")
        security_hint = "accept the local certificate warning" if self._browser_secure else "HTTP mode"
        self.status_var.set(f"Open the URL on your phone and enable the camera ({security_hint}).")

    def disconnect(self) -> None:
        if self.worker is not None:
            self.worker.stop()
            self.worker = None
        if self.browser_server is not None:
            self.browser_server.stop()
            self.browser_server = None
            self.browser_url_var.set("Phone browser: not running")

    def load_image(self) -> None:
        selected = filedialog.askopenfilename(
            filetypes=[("Images", "*.png *.jpg *.jpeg *.bmp"), ("All files", "*.*")]
        )
        if not selected:
            return
        try:
            with Image.open(selected) as image:
                self._set_image(image.convert("RGB"))
        except OSError as exc:
            self.status_var.set(f"Image error: {exc}")
            return
        self.status_var.set(f"Loaded {selected}; click four paper corners.")

    def _set_image(self, image: Image.Image) -> None:
        self._image = image.copy().convert("RGB")
        self._render()

    def _poll_camera(self) -> None:
        if self.worker is not None:
            frame = self.worker.latest()
            if frame is not None:
                self._set_image(frame.image)
                self.status_var.set("Live frame received; click four paper corners.")
            elif self.worker.error:
                self.status_var.set(f"Camera error: {self.worker.error}")
        elif self.browser_server is not None:
            frame = self.browser_server.latest()
            if frame is not None:
                self._set_image(frame.image)
                self.status_var.set("Phone browser frame received; click four paper corners.")
        self.root.after(100, self._poll_camera)

    def _render(self) -> None:
        if self._image is None:
            self.canvas.delete("all")
            self.canvas.create_text(
                max(1, self.canvas.winfo_width() // 2),
                max(1, self.canvas.winfo_height() // 2),
                text="No image",
                fill="white",
                font=("Segoe UI", 16),
            )
            return
        canvas_width = max(1, self.canvas.winfo_width())
        canvas_height = max(1, self.canvas.winfo_height())
        image_width, image_height = self._image.size
        self._display_scale = min(canvas_width / image_width, canvas_height / image_height)
        display_width = max(1, round(image_width * self._display_scale))
        display_height = max(1, round(image_height * self._display_scale))
        offset_x = (canvas_width - display_width) / 2.0
        offset_y = (canvas_height - display_height) / 2.0
        self._display_offset = (offset_x, offset_y)
        display = self._image.resize((display_width, display_height), Image.Resampling.LANCZOS)
        self._photo = ImageTk.PhotoImage(display)
        self.canvas.delete("all")
        self.canvas.create_image(offset_x, offset_y, image=self._photo, anchor="nw")
        self._draw_corners()

    def _draw_corners(self) -> None:
        for index, (x, y) in enumerate(self._corners):
            canvas_x, canvas_y = self._image_to_canvas((x, y))
            self.canvas.create_oval(
                canvas_x - 7,
                canvas_y - 7,
                canvas_x + 7,
                canvas_y + 7,
                fill="#00e676",
                outline="black",
                width=2,
            )
            self.canvas.create_text(
                canvas_x + 14,
                canvas_y - 12,
                text=CORNER_NAMES[index],
                fill="#00e676",
                anchor="w",
                font=("Segoe UI", 11, "bold"),
            )

    def _canvas_to_image(self, point: tuple[float, float]) -> tuple[float, float] | None:
        if self._image is None:
            return None
        offset_x, offset_y = self._display_offset
        x = (point[0] - offset_x) / self._display_scale
        y = (point[1] - offset_y) / self._display_scale
        width, height = self._image.size
        if not 0.0 <= x <= width or not 0.0 <= y <= height:
            return None
        return x, y

    def _image_to_canvas(self, point: tuple[float, float]) -> tuple[float, float]:
        offset_x, offset_y = self._display_offset
        return (
            offset_x + point[0] * self._display_scale,
            offset_y + point[1] * self._display_scale,
        )

    def _on_canvas_click(self, event: tk.Event) -> None:
        point = self._canvas_to_image((float(event.x), float(event.y)))
        if point is None:
            return
        if len(self._corners) == 4:
            self._corners.clear()
        self._corners.append(point)
        self._draw_corners()
        self.status_var.set(f"Selected {len(self._corners)}/4 corners.")

    def _on_canvas_motion(self, event: tk.Event) -> None:
        point = self._canvas_to_image((float(event.x), float(event.y)))
        if point is None:
            return
        if self.calibration is None:
            self.cursor_var.set(f"Camera pixel: ({point[0]:.1f}, {point[1]:.1f})    Paper WORLD: —")
            return
        world = self.calibration.image_to_world(point)
        self.cursor_var.set(
            f"Camera pixel: ({point[0]:.1f}, {point[1]:.1f})    "
            f"Paper WORLD: ({world[0]:.1f}, {world[1]:.1f}) mm"
        )

    def clear_corners(self) -> None:
        self._corners.clear()
        self.calibration = None
        self._render()
        self.status_var.set("Corner points cleared.")

    def calibrate(self) -> None:
        if len(self._corners) != 4:
            self.status_var.set("Select exactly four corners first.")
            return
        try:
            calibration = PaperCalibration.from_points(
                self._corners,
                self.width_var.get(),
                self.height_var.get(),
            )
            output = calibration.save(self.output_var.get().strip())
        except (CalibrationError, OSError, tk.TclError, ValueError) as exc:
            self.status_var.set(f"Calibration error: {exc}")
            return
        self.calibration = calibration
        self.status_var.set(
            f"Saved {output}; max corner error "
            f"{calibration.max_reprojection_error_mm:.4f} mm."
        )

    def close(self) -> None:
        self.disconnect()
        self.root.destroy()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="python -m pc_trajectory.camera_calibration_ui")
    parser.add_argument("--source", help="webcam index or phone camera URL")
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--phone-browser",
        action="store_true",
        help="start the HTTPS phone-browser camera gateway on launch",
    )
    parser.add_argument(
        "--browser-http",
        action="store_true",
        help="use HTTP for the phone gateway (diagnostic only; browsers may block camera access)",
    )
    parser.add_argument("--browser-port", type=int, default=8765)
    parser.add_argument("--browser-host", help="LAN IP shown in the phone URL")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--image", type=Path, help="static image for headless calibration")
    parser.add_argument("--points", help="TL,TR,BR,BL as x,y;x,y;x,y;x,y")
    parser.add_argument("--width-mm", type=float, default=210.0)
    parser.add_argument("--height-mm", type=float, default=297.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.headless:
        if args.image is None or args.points is None or args.output is None:
            raise SystemExit("--headless requires --image, --points and --output")
        image_points = _parse_points(args.points)
        calibration = PaperCalibration.from_points(
            image_points,
            args.width_mm,
            args.height_mm,
        )
        calibration.save(args.output)
        print(
            f"Calibration: {args.output} "
            f"max_error_mm={calibration.max_reprojection_error_mm:.6f}"
        )
        return 0

    root = tk.Tk()
    CalibrationApp(
        root,
        source=args.source,
        output=args.output,
        start_phone_browser=args.phone_browser,
        browser_secure=not args.browser_http,
        browser_port=args.browser_port,
        browser_host=args.browser_host,
    )
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["CalibrationApp", "main"]
