"""Bridge Kinect RGB/IR → virtual webcam (v4l2loopback) as normal YUYV for browsers."""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

from kinect_camera import (
    MODES,
    find_kinect_video,
    find_vcam,
    free_device,
    grab_ir_raw,
    nudge_pipewire_cameras,
    set_mode,
    unpack_y10bpack,
)

DIR = Path(__file__).resolve().parent
LABEL = os.environ.get("KINECT_VCAM_LABEL", "Kinect Webcam")
VCAM_W, VCAM_H = 640, 480
IR_FPS = 8
RGB_FPS = 15


def ensure_vcam() -> str:
    """Use an existing loopback. Never pkexec from the panel."""
    dev = find_vcam()
    if dev:
        return dev
    setup = DIR / "setup_loopback.sh"
    r = subprocess.run([str(setup), "--check"], capture_output=True, text=True)
    if r.returncode == 0:
        dev = (r.stdout or "").strip().splitlines()[-1] if r.stdout else ""
        if dev.startswith("/dev/video"):
            return dev
    dev = find_vcam()
    if dev:
        return dev
    raise RuntimeError(
        "No virtual webcam (v4l2loopback). "
        "Load once in a terminal: lab/kinect-panel/setup_loopback.sh --load"
    )


def _stretch_grey(grey: bytes) -> bytes:
    if not grey:
        return grey
    lo = min(grey)
    hi = max(grey)
    if hi <= lo:
        return grey
    scale = 255.0 / (hi - lo)
    return bytes(min(255, int((b - lo) * scale)) for b in grey)


def _pipewire_has_vcam_source(vcam: str) -> bool:
    try:
        r = subprocess.run(
            ["pw-cli", "ls", "Node"], capture_output=True, text=True, timeout=5
        )
    except Exception:
        return False
    text = r.stdout or ""
    # look for our path or card name as a Video/Source
    if vcam not in text and "Kinect Webcam" not in text:
        return False
    # crude: if object.path mentions videoN and nearby media.class Source
    lines = text.splitlines()
    for i, line in enumerate(lines):
        if vcam in line or "Kinect Webcam" in line or f"video{vcam.replace('/dev/video','')}" in line:
            chunk = "\n".join(lines[max(0, i - 8) : i + 12])
            if "Video/Source" in chunk or "media.role" in chunk:
                return True
    # also match object.path = "v4l2:/dev/video3"
    if f'v4l2:{vcam}' in text or f'v4l2:{vcam}' in text.replace(" ", ""):
        # check if any source node has this path
        if 'object.path = "v4l2:' + vcam + '"' in text:
            # need class source - search window
            idx = text.find(f'object.path = "v4l2:{vcam}"')
            if idx >= 0:
                window = text[max(0, idx - 400) : idx + 400]
                if "Video/Source" in window:
                    return True
    return False


class WebcamBridge:
    def __init__(self) -> None:
        self._proc: subprocess.Popen | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self.mode: str | None = None
        self.source: str | None = None
        self.vcam: str | None = None
        self.error: str | None = None
        self._frames: int = 0

    @property
    def running(self) -> bool:
        if self._stop.is_set():
            return False
        if self._thread and self._thread.is_alive():
            return True
        if self._proc and self._proc.poll() is None:
            return True
        return False

    def start(self, mode: str) -> None:
        self.stop()
        self.error = None
        self._frames = 0
        mode = mode.upper()
        if mode not in MODES:
            raise ValueError(mode)
        source = find_kinect_video()
        if not source:
            raise RuntimeError("No Kinect /dev/video* (gspca)")
        free_device(source)
        vcam = ensure_vcam()
        set_mode(source, mode, force=True)
        self.mode = mode
        self.source = source
        self.vcam = vcam
        self._stop.clear()
        if mode == "RGB":
            self._start_rgb(source, vcam)
        else:
            self._start_ir(source, vcam)
        # Browser (Firefox/Chrome) uses PipeWire — force rescan after producer is live
        self._publish_to_pipewire(vcam)

    def _publish_to_pipewire(self, vcam: str) -> None:
        # give exclusive_caps a moment to flip to CAPTURE
        time.sleep(0.4)
        try:
            nudge_pipewire_cameras()
        except Exception:
            pass
        time.sleep(0.6)
        if not _pipewire_has_vcam_source(vcam):
            # one more hard restart of wireplumber
            try:
                subprocess.run(
                    ["systemctl", "--user", "restart", "wireplumber"],
                    capture_output=True,
                    timeout=10,
                )
                time.sleep(1.0)
            except Exception:
                pass

    def _start_rgb(self, source: str, vcam: str) -> None:
        # Do NOT free_device after this point — it would kill ffmpeg
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "warning",
            "-fflags",
            "nobuffer",
            "-f",
            "v4l2",
            "-input_format",
            "uyvy422",
            "-video_size",
            "640x480",
            "-i",
            source,
            "-f",
            "v4l2",
            "-pix_fmt",
            "yuyv422",
            "-vf",
            f"scale={VCAM_W}:{VCAM_H},format=yuyv422,fps={RGB_FPS}",
            vcam,
        ]
        self._proc = subprocess.Popen(
            cmd,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            start_new_session=True,
        )
        time.sleep(0.9)
        if self._proc.poll() is not None:
            err = (self._proc.stderr.read() or b"").decode("utf-8", "replace")
            raise RuntimeError(f"RGB bridge died: {err.strip() or self._proc.returncode}")
        # verify capture side is live for browsers
        r = subprocess.run(
            ["v4l2-ctl", "-d", vcam, "--all"], capture_output=True, text=True, timeout=3
        )
        if "Video Capture" not in (r.stdout or ""):
            # still may work after wireplumber; warn only
            pass

    def _start_ir(self, source: str, vcam: str) -> None:
        self._thread = threading.Thread(
            target=self._ir_loop, args=(source, vcam), daemon=True, name="kinect-ir-bridge"
        )
        self._thread.start()
        for _ in range(40):
            if self.error:
                raise RuntimeError(self.error)
            if self._frames > 0:
                return
            if not self._thread.is_alive():
                raise RuntimeError(self.error or "IR bridge thread died")
            time.sleep(0.15)
        if self._frames == 0:
            raise RuntimeError(self.error or "IR bridge: no frames (device busy?)")

    def _ir_loop(self, source: str, vcam: str) -> None:
        spec = MODES["IR"]
        raw_path = Path(tempfile.gettempdir()) / f"kinect-ir-bridge-{os.getpid()}.raw"
        try:
            free_device(source)
            set_mode(source, "IR", force=True)
        except Exception as e:
            self.error = f"IR set_mode: {e}"
            return

        ff = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "warning",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "gray",
            "-s",
            f"{spec.width}x{spec.height}",
            "-framerate",
            str(IR_FPS),
            "-i",
            "pipe:0",
            "-f",
            "v4l2",
            "-pix_fmt",
            "yuyv422",
            "-vf",
            f"scale={VCAM_W}:{VCAM_H},eq=contrast=1.35:brightness=0.1,format=yuyv422,fps={IR_FPS}",
            vcam,
        ]
        try:
            self._proc = subprocess.Popen(
                ff,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                start_new_session=True,
            )
        except Exception as e:
            self.error = str(e)
            return

        assert self._proc.stdin is not None
        frame_interval = 1.0 / IR_FPS
        first = True
        try:
            while not self._stop.is_set():
                t0 = time.monotonic()
                try:
                    # set_fmt only once; never free_device while writing (would kill us)
                    data = grab_ir_raw(source, raw_path, set_fmt=first)
                    first = False
                except Exception as e:
                    try:
                        # careful free — skip if our ffmpeg is the only weird holder
                        free_device(source)
                        set_mode(source, "IR", force=True)
                        data = grab_ir_raw(source, raw_path, set_fmt=False)
                        first = False
                    except Exception as e2:
                        self.error = f"IR grab: {e2}"
                        time.sleep(0.3)
                        first = True
                        continue
                grey = unpack_y10bpack(data, spec.width, spec.height)
                grey = _stretch_grey(grey)
                try:
                    self._proc.stdin.write(grey)
                    self._proc.stdin.flush()
                    self._frames += 1
                except BrokenPipeError:
                    err = b""
                    if self._proc.stderr:
                        err = self._proc.stderr.read() or b""
                    self.error = f"IR pipe broken: {err.decode('utf-8', 'replace')}"
                    break
                if self._proc.poll() is not None:
                    err = (self._proc.stderr.read() or b"").decode("utf-8", "replace")
                    self.error = f"IR ffmpeg exit: {err.strip()}"
                    break
                dt = time.monotonic() - t0
                if dt < frame_interval:
                    time.sleep(frame_interval - dt)
        except Exception as e:
            self.error = str(e)
        finally:
            try:
                if self._proc and self._proc.stdin:
                    self._proc.stdin.close()
            except Exception:
                pass
            try:
                raw_path.unlink(missing_ok=True)
            except OSError:
                pass

    def stop(self) -> None:
        self._stop.set()
        if self._proc is not None:
            try:
                os.killpg(self._proc.pid, signal.SIGTERM)
            except (ProcessLookupError, PermissionError, OSError):
                try:
                    self._proc.terminate()
                except Exception:
                    pass
            try:
                self._proc.wait(timeout=2)
            except Exception:
                try:
                    os.killpg(self._proc.pid, signal.SIGKILL)
                except Exception:
                    pass
            self._proc = None
        if self._thread is not None:
            self._thread.join(timeout=4)
            self._thread = None
        if self.source:
            free_device(self.source)
        self.mode = None
        self._frames = 0


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] in ("-h", "--help"):
        print("Usage: kinect_bridge.py rgb|ir")
        return 0
    mode = argv[0].upper()
    if mode not in ("RGB", "IR"):
        print("mode must be rgb or ir", file=sys.stderr)
        return 2
    b = WebcamBridge()
    b.start(mode)
    print(f"streaming {mode} → {b.vcam}", flush=True)
    print("In browser pick: Kinect Webcam (keep this process / panel Camera ON)", flush=True)
    try:
        while b.running:
            if b.error:
                print(b.error, file=sys.stderr)
                return 1
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        b.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
