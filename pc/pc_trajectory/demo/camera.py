"""Camera input adapters for the visual-navigation demo.

OpenCV is imported lazily, so the calibration math and the existing PC
pipeline remain usable in environments that have no camera package installed.
The source may be a webcam index (``0``), an HTTP/MJPEG URL, or an RTSP URL.
"""

from __future__ import annotations

from dataclasses import dataclass
import queue
import threading
import time
from typing import Any

from PIL import Image


class CameraError(RuntimeError):
    """Raised when a camera source cannot be opened or read."""


def parse_camera_source(source: str | int) -> str | int:
    if isinstance(source, bool):
        raise CameraError("camera source must be an index or URL")
    if isinstance(source, int):
        if source < 0:
            raise CameraError("camera index must be non-negative")
        return source
    text = str(source).strip()
    if not text:
        raise CameraError("camera source cannot be empty")
    if text.isdecimal():
        return int(text)
    return text


@dataclass(frozen=True)
class CameraFrame:
    image: Image.Image
    captured_at_s: float
    source: str | int


class CameraSource:
    """Synchronous adapter around OpenCV VideoCapture."""

    def __init__(self, source: str | int) -> None:
        self.source = parse_camera_source(source)
        self._capture: Any | None = None

    @property
    def is_open(self) -> bool:
        return self._capture is not None and bool(self._capture.isOpened())

    def open(self) -> None:
        try:
            import cv2
        except ImportError as exc:
            raise CameraError(
                "camera input requires OpenCV; install the optional demo dependencies"
            ) from exc
        self.close()
        capture = cv2.VideoCapture(self.source)
        if not capture.isOpened():
            capture.release()
            raise CameraError(f"cannot open camera source: {self.source}")
        self._capture = capture

    def read(self) -> CameraFrame | None:
        if not self.is_open:
            raise CameraError("camera source is not open")
        import cv2

        ok, frame = self._capture.read()
        if not ok or frame is None:
            return None
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return CameraFrame(
            image=Image.fromarray(rgb, mode="RGB"),
            captured_at_s=time.monotonic(),
            source=self.source,
        )

    def close(self) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None


class CameraWorker:
    """Background latest-frame worker suitable for a Tkinter UI."""

    def __init__(self, source: str | int) -> None:
        self.camera = CameraSource(source)
        self._frames: queue.Queue[CameraFrame] = queue.Queue(maxsize=1)
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self.error: str | None = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.running:
            return
        self.error = None
        self.camera.open()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="camera-worker", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                frame = self.camera.read()
                if frame is None:
                    time.sleep(0.05)
                    continue
                try:
                    self._frames.put_nowait(frame)
                except queue.Full:
                    try:
                        self._frames.get_nowait()
                    except queue.Empty:
                        pass
                    self._frames.put_nowait(frame)
        except Exception as exc:  # surfaced to the UI through ``error``
            self.error = str(exc)
        finally:
            self.camera.close()

    def latest(self) -> CameraFrame | None:
        latest: CameraFrame | None = None
        while True:
            try:
                latest = self._frames.get_nowait()
            except queue.Empty:
                return latest

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._thread = None
        self.camera.close()


__all__ = [
    "CameraError",
    "CameraFrame",
    "CameraSource",
    "CameraWorker",
    "parse_camera_source",
]
