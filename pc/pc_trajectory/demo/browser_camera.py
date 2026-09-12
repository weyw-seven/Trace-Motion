"""Phone-browser camera gateway for the visual-navigation demo.

The phone opens a small page served by the PC, grants the browser camera
permission, and periodically uploads JPEG frames to this process.  This keeps
the desktop side independent of a particular mobile camera application.

The default server is HTTPS because mobile browsers generally expose
``getUserMedia`` only from a secure context.  A self-signed certificate is
generated with the system ``openssl`` command on first use.  The phone user
must accept that local certificate warning once.
"""

from __future__ import annotations

from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
import ipaddress
import json
import os
from pathlib import Path
import shutil
import socket
import ssl
import subprocess
import threading
import time
from typing import Any
from urllib.parse import urlparse

from PIL import Image

from .camera import CameraFrame


class BrowserCameraError(RuntimeError):
    """Raised when the phone-browser camera server cannot start or receive data."""


_MAX_FRAME_BYTES = 8 * 1024 * 1024
_PAGE = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Visual Navigation Camera</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, sans-serif; }
    body { margin: 0; padding: 18px; background: #15181d; color: #f4f6f8; }
    main { max-width: 720px; margin: auto; }
    h1 { font-size: 1.35rem; margin: 0 0 8px; }
    p { color: #bdc5cf; line-height: 1.4; }
    button, select { font: inherit; padding: 10px 14px; margin: 4px 4px 12px 0; }
    button { color: #101318; background: #65e6a5; border: 0; border-radius: 6px; }
    video { display: block; width: 100%; max-height: 65vh; object-fit: contain;
            background: #080a0d; border-radius: 8px; }
    #status { min-height: 1.4em; color: #65e6a5; }
  </style>
</head>
<body>
<main>
  <h1>Visual Navigation · Phone Camera</h1>
  <p>Press <b>Enable camera</b>, allow camera access, and keep this page open.
     The PC will receive a low-latency preview for calibration and tracking.</p>
    <button id="start">Enable camera</button>
    <button id="stop" disabled>Stop</button>
    <select id="facing" aria-label="Camera direction">
    <option value="environment">Back camera</option>
    <option value="user">Front camera</option>
  </select>
  <select id="camera" aria-label="Camera device">
    <option value="">Auto-select camera</option>
  </select>
  <div id="status">Waiting for camera permission.</div>
  <video id="preview" autoplay playsinline muted></video>
  <canvas id="canvas" hidden></canvas>
</main>
<script>
(() => {
  const video = document.getElementById('preview');
  const canvas = document.getElementById('canvas');
  const startButton = document.getElementById('start');
  const stopButton = document.getElementById('stop');
  const facing = document.getElementById('facing');
  const camera = document.getElementById('camera');
  const status = document.getElementById('status');
  let stream = null;
  let timer = null;
  let uploading = false;
  let context = null;

  function setStatus(message, error = false) {
    status.textContent = message;
    status.style.color = error ? '#ff8d8d' : '#65e6a5';
  }

  async function uploadFrame() {
    if (!stream || uploading || video.readyState < 2 || !video.videoWidth) return;
    uploading = true;
    canvas.width = video.videoWidth;
    canvas.height = video.videoHeight;
    if (!context) context = canvas.getContext('2d', {alpha: false});
    // Resizing a canvas clears it.  Draw the current video frame before
    // encoding; otherwise toBlob() produces a valid but completely black JPEG.
    context.drawImage(video, 0, 0, canvas.width, canvas.height);
    const blob = await new Promise(resolve => canvas.toBlob(resolve, 'image/jpeg', 0.78));
    if (!blob) { uploading = false; return; }
    try {
      const response = await fetch('/frame', {
        method: 'POST',
        headers: {'Content-Type': 'image/jpeg'},
        body: blob,
        cache: 'no-store'
      });
      if (!response.ok) throw new Error(`PC returned ${response.status}`);
      setStatus(`Streaming ${video.videoWidth}×${video.videoHeight}`);
    } catch (error) {
      setStatus(`Upload error: ${error}`, true);
    } finally {
      uploading = false;
    }
  }

  async function refreshCameras(selectedId = '') {
    if (!navigator.mediaDevices.enumerateDevices) return;
    const devices = await navigator.mediaDevices.enumerateDevices();
    const cameras = devices.filter(device => device.kind === 'videoinput');
    const previous = selectedId || camera.value;
    camera.innerHTML = '<option value="">Auto-select camera</option>';
    cameras.forEach((device, index) => {
      const option = document.createElement('option');
      option.value = device.deviceId;
      option.textContent = device.label || `Camera ${index + 1}`;
      camera.appendChild(option);
    });
    if (previous && cameras.some(device => device.deviceId === previous)) {
      camera.value = previous;
    }
  }

  async function forceOneX(track) {
    const capabilities = track.getCapabilities ? track.getCapabilities() : {};
    if (!capabilities.zoom || capabilities.zoom.min > 1 || capabilities.zoom.max < 1) {
      return false;
    }
    try {
      await track.applyConstraints({advanced: [{zoom: 1}]});
      return true;
    } catch (error) {
      return false;
    }
  }

  async function start() {
    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
      setStatus('This browser does not expose camera access. Use HTTPS.', true);
      return;
    }
    if (stream) stream.getTracks().forEach(track => track.stop());
    try {
      const videoConstraints = {
        width: {ideal: 1280},
        height: {ideal: 960},
        aspectRatio: {ideal: 4 / 3}
      };
      if (camera.value) {
        videoConstraints.deviceId = {exact: camera.value};
      } else {
        videoConstraints.facingMode = {ideal: facing.value};
      }
      stream = await navigator.mediaDevices.getUserMedia({
        audio: false,
        video: videoConstraints
      });
      video.srcObject = stream;
      await video.play();
      const track = stream.getVideoTracks()[0];
      const oneX = await forceOneX(track);
      const settings = track.getSettings ? track.getSettings() : {};
      await refreshCameras(settings.deviceId || '');
      startButton.disabled = true;
      stopButton.disabled = false;
      facing.disabled = true;
      camera.disabled = true;
      const zoomText = oneX ? 'zoom 1x' : 'zoom control unavailable';
      setStatus(`Streaming ${settings.width || video.videoWidth}×${settings.height || video.videoHeight} · ${zoomText}`);
      timer = setInterval(uploadFrame, 200);
      await uploadFrame();
    } catch (error) {
      setStatus(`Camera error: ${error.name || error}`, true);
    }
  }

  function stop() {
    if (timer) clearInterval(timer);
    timer = null;
    if (stream) stream.getTracks().forEach(track => track.stop());
    stream = null;
    video.srcObject = null;
    startButton.disabled = false;
    stopButton.disabled = true;
    facing.disabled = false;
    camera.disabled = false;
    setStatus('Camera stopped.');
  }

  startButton.addEventListener('click', start);
  stopButton.addEventListener('click', stop);
  camera.addEventListener('change', () => {
    if (camera.value) setStatus('Selected camera. Press Enable camera to start.');
  });
  window.addEventListener('pagehide', stop);
})();
</script>
</body>
</html>
"""


def _local_ip() -> str:
    """Best-effort LAN address for the URL shown to a phone."""
    candidates: list[str] = []
    try:
        for item in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            address = item[4][0]
            if address not in candidates and not address.startswith("127."):
                candidates.append(address)
    except OSError:
        pass
    # Windows Mobile Hotspot normally exposes this fixed private gateway.  If
    # it is present, the phone connected to that hotspot must use it rather
    # than the PC's internet-facing Wi-Fi address.
    if "192.168.137.1" in candidates:
        return "192.168.137.1"
    for address in candidates:
        try:
            if ipaddress.ip_address(address).is_private:
                return address
        except ValueError:
            continue
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect(("192.0.2.1", 9))
            address = probe.getsockname()[0]
            if not address.startswith("127."):
                return address
    except OSError:
        pass
    try:
        for item in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            address = item[4][0]
            if not address.startswith("127."):
                return address
    except OSError:
        pass
    return "127.0.0.1"


def _certificate_paths(directory: Path, advertised_host: str) -> tuple[Path, Path]:
    directory.mkdir(parents=True, exist_ok=True)
    suffix = "".join(character if character.isalnum() else "_" for character in advertised_host)
    return (
        directory / f"phone_camera_cert_{suffix}.pem",
        directory / f"phone_camera_key_{suffix}.pem",
    )


def _ensure_certificate(directory: Path, advertised_host: str) -> tuple[Path, Path]:
    certificate, key = _certificate_paths(directory, advertised_host)
    openssl = shutil.which("openssl")
    if openssl is None:
        raise BrowserCameraError(
            "HTTPS camera mode needs the openssl command; use --browser-http "
            "for a diagnostic server or install OpenSSL"
        )
    # Regenerate when either file is absent.  The certificate is local-only and
    # deliberately short lived; it is not intended for public deployment.
    if certificate.exists() and key.exists():
        return certificate, key
    try:
        ipaddress.ip_address(advertised_host)
        subject_alt_name = f"IP:{advertised_host}"
    except ValueError:
        subject_alt_name = f"DNS:{advertised_host}"
    command = [
        openssl,
        "req",
        "-x509",
        "-newkey",
        "rsa:2048",
        "-nodes",
        "-sha256",
        "-days",
        "365",
        "-keyout",
        str(key),
        "-out",
        str(certificate),
        "-subj",
        f"/CN={advertised_host}",
        "-addext",
        f"subjectAltName={subject_alt_name},DNS:localhost",
    ]
    environment = os.environ.copy()
    # Some Windows Conda installations ship openssl.exe but leave
    # OPENSSL_CONF pointing at a non-existent system path.  Prefer the config
    # next to the selected executable when it exists.
    bundled_config = Path(openssl).resolve().parents[1] / "ssl" / "openssl.cnf"
    if bundled_config.exists():
        environment["OPENSSL_CONF"] = str(bundled_config)
    result = subprocess.run(
        command,
        capture_output=True,
        text=True,
        check=False,
        env=environment,
    )
    if result.returncode != 0:
        raise BrowserCameraError(
            "cannot create local HTTPS certificate: "
            + (result.stderr.strip() or "openssl failed")
        )
    return certificate, key


class _CameraRequestHandler(BaseHTTPRequestHandler):
    server_ref: "BrowserCameraServer"

    def log_message(self, format: str, *args: Any) -> None:
        # The Tkinter status area is the user-facing log.  Avoid noisy console
        # output for every 5 FPS frame upload.
        if self.path != "/frame":
            super().log_message(format, *args)

    def _send(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802 - stdlib handler API
        path = urlparse(self.path).path
        if path == "/":
            self._send(200, _PAGE.encode("utf-8"), "text/html; charset=utf-8")
            return
        if path == "/health":
            self._send(
                200,
                json.dumps(self.server_ref.health(), ensure_ascii=False).encode("utf-8"),
                "application/json; charset=utf-8",
            )
            return
        self._send(404, b"not found", "text/plain; charset=utf-8")

    def do_POST(self) -> None:  # noqa: N802 - stdlib handler API
        if urlparse(self.path).path != "/frame":
            self._send(404, b"not found", "text/plain; charset=utf-8")
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            length = 0
        if length <= 0 or length > _MAX_FRAME_BYTES:
            self._send(413, b"invalid frame size", "text/plain; charset=utf-8")
            return
        payload = self.rfile.read(length)
        try:
            self.server_ref.receive(payload)
        except (OSError, ValueError) as exc:
            self._send(400, str(exc).encode("utf-8"), "text/plain; charset=utf-8")
            return
        self._send(204, b"", "text/plain; charset=utf-8")


@dataclass
class BrowserCameraServer:
    """Threaded local server that exposes the phone-camera page and frames."""

    port: int = 8765
    advertised_host: str | None = None
    secure: bool = True
    certificate_directory: str | Path = "test_results/demo_phase1/browser_tls"

    def __post_init__(self) -> None:
        self._httpd: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._latest: CameraFrame | None = None
        self._frame_count = 0
        self._delivered_count = 0
        self._url = ""

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def url(self) -> str:
        if not self._url:
            raise BrowserCameraError("browser camera server has not started")
        return self._url

    def start(self) -> str:
        if self.running:
            return self.url
        host = self.advertised_host or _local_ip()
        try:
            httpd = ThreadingHTTPServer(("0.0.0.0", int(self.port)), _CameraRequestHandler)
        except OSError as exc:
            raise BrowserCameraError(f"cannot bind browser camera port {self.port}: {exc}") from exc
        httpd.daemon_threads = True
        httpd.RequestHandlerClass.server_ref = self  # type: ignore[attr-defined]
        try:
            if self.secure:
                certificate, key = _ensure_certificate(Path(self.certificate_directory), host)
                context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
                context.minimum_version = ssl.TLSVersion.TLSv1_2
                context.load_cert_chain(certificate, key)
                # Defer the handshake until the request is running in its own
                # ThreadingHTTPServer worker.  Some mobile browsers probe a
                # local HTTPS endpoint and leave the handshake half-open; a
                # handshake inside accept() would block every later client.
                httpd.socket = context.wrap_socket(
                    httpd.socket,
                    server_side=True,
                    do_handshake_on_connect=False,
                )
                scheme = "https"
            else:
                scheme = "http"
        except Exception:
            httpd.server_close()
            raise
        self._httpd = httpd
        actual_port = int(httpd.server_address[1])
        self._url = f"{scheme}://{host}:{actual_port}/"
        self._thread = threading.Thread(
            target=httpd.serve_forever,
            name="phone-browser-camera",
            daemon=True,
        )
        self._thread.start()
        return self._url

    def receive(self, payload: bytes) -> None:
        if not payload:
            raise ValueError("empty camera frame")
        try:
            with Image.open(BytesIO(payload)) as image:
                frame = image.convert("RGB")
                frame.load()
        except (OSError, ValueError) as exc:
            raise ValueError("frame is not a readable image") from exc
        with self._lock:
            self._latest = CameraFrame(
                image=frame,
                captured_at_s=time.monotonic(),
                source=self.url,
            )
            self._frame_count += 1

    def latest(self) -> CameraFrame | None:
        with self._lock:
            if self._frame_count == self._delivered_count or self._latest is None:
                return None
            self._delivered_count = self._frame_count
            return self._latest

    def health(self) -> dict[str, object]:
        with self._lock:
            return {
                "running": self.running,
                "frames": self._frame_count,
                "last_frame": self._latest is not None,
            }

    def stop(self) -> None:
        httpd = self._httpd
        self._httpd = None
        if httpd is not None:
            httpd.shutdown()
            httpd.server_close()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._thread = None
        self._url = ""


__all__ = ["BrowserCameraError", "BrowserCameraServer"]
