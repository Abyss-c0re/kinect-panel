"""Kinect RGB/IR via gspca + helpers for panel/bridge."""
from __future__ import annotations

import glob
import os
import signal
import subprocess
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from PIL import Image, ImageEnhance, ImageOps

DEFAULT_DEVICE = "/dev/video0"
VCAM_LABEL = os.environ.get("KINECT_VCAM_LABEL", "Kinect Webcam")


@dataclass
class ModeSpec:
    name: str
    pixelformat: str
    width: int
    height: int
    label: str


@dataclass
class V4LDevice:
    path: str
    name: str
    driver: str
    sys_path: str

    @property
    def virtual(self) -> bool:
        return "/devices/virtual/" in self.sys_path

    @property
    def loopback(self) -> bool:
        blob = f"{self.name} {self.driver}".lower()
        return (
            self.virtual
            or "loopback" in blob
            or self.driver.lower() in ("v4l2loopback", "v4l2 loopback")
        )


MODES = {
    "RGB": ModeSpec("RGB", "UYVY", 640, 480, "RGB color (UYVY 640×480)"),
    "IR": ModeSpec("IR", "Y10B", 640, 488, "Infrared (Y10B 640×488 greyscale)"),
}


def _v4l_devices() -> list[V4LDevice]:
    out: list[V4LDevice] = []
    for path in sorted(glob.glob("/dev/video*")):
        base = os.path.basename(path)
        sys_dir = Path("/sys/class/video4linux") / base
        try:
            name = (sys_dir / "name").read_text().strip()
        except OSError:
            name = ""
        driver = ""
        sys_path = ""
        try:
            sys_path = os.path.realpath(sys_dir)
        except OSError:
            sys_path = ""
        try:
            driver = os.path.basename(os.path.realpath(sys_dir / "device" / "driver"))
        except OSError:
            driver = ""
        out.append(V4LDevice(path=path, name=name, driver=driver, sys_path=sys_path))
    return out


def find_kinect_video() -> Optional[str]:
    """Real Xbox NUI / gspca capture node — never a v4l2loopback."""
    devices = _v4l_devices()
    hits: list[V4LDevice] = []
    for dev in devices:
        if dev.loopback:
            continue
        blob = f"{dev.name} {dev.driver}".lower()
        if "lenovo" in blob or "uvcvideo" in dev.driver.lower():
            continue
        if any(k in blob for k in ("nui", "kinect", "gspca", "xbox")):
            hits.append(dev)
    if hits:
        def _rank(dev: V4LDevice) -> tuple[int, str]:
            d = dev.driver.lower()
            if d in ("kinect", "gspca_kinect") or "gspca" in d:
                return (0, dev.path)
            return (1, dev.path)

        hits.sort(key=_rank)
        return hits[0].path
    for dev in devices:
        if dev.loopback:
            continue
        if "uvcvideo" in dev.driver.lower():
            continue
        return dev.path
    return None


def find_vcam() -> Optional[str]:
    """Virtual webcam node. Accepts exact label, truncated 'Kinect', or the only loopback."""
    loopbacks = [d for d in _v4l_devices() if d.loopback]
    if not loopbacks:
        return None
    want = VCAM_LABEL.strip()
    want_l = want.lower()
    for dev in loopbacks:
        if dev.name == want or dev.name.lower() == want_l:
            return dev.path
    # Unquoted card_label=Kinect Webcam becomes card type "Kinect"
    first = want.split()[0].lower() if want else ""
    if first:
        for dev in loopbacks:
            n = dev.name.lower()
            if n == first or n.startswith(first):
                return dev.path
    if len(loopbacks) == 1:
        return loopbacks[0].path
    return None


def _holders(device: str) -> list[tuple[int, str]]:
    out: list[tuple[int, str]] = []
    real = os.path.realpath(device)
    for fd in glob.glob("/proc/[0-9]*/fd/*"):
        try:
            dest = os.readlink(fd)
        except OSError:
            continue
        if dest == device or dest == real or dest.endswith("/" + os.path.basename(device)):
            pid = int(fd.split("/")[2])
            try:
                cmd = (
                    open(f"/proc/{pid}/cmdline", "rb")
                    .read()
                    .replace(b"\x00", b" ")
                    .decode("utf-8", "replace")[:200]
                )
            except OSError:
                cmd = "?"
            out.append((pid, cmd))
    return out


def _is_our_bridge(cmd: str) -> bool:
    """Never kill the Kinect→loopback publisher (browser depends on it)."""
    vcam = find_vcam() or ""
    if vcam and vcam in cmd:
        return True
    if "Kinect Webcam" in cmd or "kinect webcam" in cmd.lower():
        return True
    if "kinect_bridge" in cmd or "kinect-ir-bridge" in cmd:
        return True
    # Any ffmpeg writing into a loopback node is the publisher.
    if "ffmpeg" in cmd and "v4l2" in cmd:
        if "loopback" in cmd.lower():
            return True
        for dev in _v4l_devices():
            if dev.loopback and dev.path in cmd:
                return True
    return False


def free_device(device: str) -> None:
    """Kill stray grabbers on Kinect source — never the webcam bridge."""
    me = os.getpid()
    for pid, cmd in _holders(device):
        if pid == me:
            continue
        if "kinect_panel" in cmd:
            continue
        if _is_our_bridge(cmd):
            continue
        # kill known capture tools + await worker ffmpeg
        if not any(
            x in cmd
            for x in (
                "v4l2-ctl",
                "ffmpeg",
                "gst-launch",
                "await_restore",
                "camera_watch",
                "kinect_ir",
            )
        ):
            continue
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    time.sleep(0.12)
    for pid, cmd in _holders(device):
        if pid == me or "kinect_panel" in cmd or _is_our_bridge(cmd):
            continue
        if any(x in cmd for x in ("v4l2-ctl", "ffmpeg", "await_restore")):
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    time.sleep(0.08)


def _run(cmd: list[str], timeout: float | None = None) -> subprocess.CompletedProcess:
    try:
        return subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            check=False,
            timeout=timeout,
            start_new_session=True,
        )
    except subprocess.TimeoutExpired as e:
        if e.pid:
            try:
                os.killpg(e.pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError, OSError):
                try:
                    os.kill(e.pid, signal.SIGKILL)
                except OSError:
                    pass
        return subprocess.CompletedProcess(
            cmd, 124, e.stdout or "", (e.stderr or "") + "\nTIMEOUT"
        )


def set_mode(device: str, mode: str, *, force: bool = True) -> None:
    spec = MODES[mode]
    if force:
        free_device(device)
    r = _run(
        [
            "v4l2-ctl",
            "-d",
            device,
            f"--set-fmt-video=width={spec.width},height={spec.height},pixelformat={spec.pixelformat}",
        ],
        timeout=3,
    )
    if r.returncode != 0:
        free_device(device)
        r = _run(
            [
                "v4l2-ctl",
                "-d",
                device,
                f"--set-fmt-video=width={spec.width},height={spec.height},pixelformat={spec.pixelformat}",
            ],
            timeout=3,
        )
    if r.returncode != 0:
        err = (r.stderr or r.stdout or "v4l2-ctl S_FMT failed").strip()
        raise RuntimeError(err)
    g = _run(["v4l2-ctl", "-d", device, "--get-fmt-video"], timeout=2)
    out = (g.stdout or "") + (g.stderr or "")
    if g.returncode == 0:
        if mode == "IR" and "Y10B" not in out:
            raise RuntimeError(f"IR format not applied:\n{out}")
        if mode == "RGB" and "UYVY" not in out:
            raise RuntimeError(f"RGB format not applied:\n{out}")


def unpack_y10bpack(buf: bytes, width: int, height: int) -> bytes:
    npix = width * height
    out = bytearray(npix)
    bit = 0
    blen = len(buf) * 8
    for p in range(npix):
        if bit + 10 > blen:
            break
        v = 0
        for _ in range(10):
            byte = buf[bit // 8]
            v = (v << 1) | ((byte >> (7 - (bit % 8))) & 1)
            bit += 1
        out[p] = v >> 2
    return bytes(out)


def grab_ir_raw(device: str, raw_path: Path, *, set_fmt: bool = True) -> bytes:
    spec = MODES["IR"]
    if set_fmt:
        set_mode(device, "IR", force=True)
    expected = spec.width * spec.height * 10 // 8
    try:
        raw_path.unlink(missing_ok=True)
    except OSError:
        pass
    r = _run(
        [
            "v4l2-ctl",
            "-d",
            device,
            "--stream-mmap=2",
            "--stream-count=1",
            f"--stream-to={raw_path}",
        ],
        timeout=3.5,
    )
    if not raw_path.is_file() or raw_path.stat().st_size < expected // 2:
        free_device(device)
        if set_fmt:
            set_mode(device, "IR", force=True)
        r = _run(
            [
                "v4l2-ctl",
                "-d",
                device,
                "--stream-mmap=2",
                "--stream-count=1",
                f"--stream-to={raw_path}",
            ],
            timeout=3.5,
        )
    if not raw_path.is_file() or raw_path.stat().st_size < expected // 2:
        err = (r.stderr or r.stdout or "no IR raw data").strip()
        raise RuntimeError(f"IR capture failed: {err}")
    data = raw_path.read_bytes()
    if len(data) >= expected:
        data = data[-expected:]
    return data


def capture_frame(device: str, mode: str, out_path: str | Path) -> Path:
    out_path = Path(out_path)
    spec = MODES[mode]
    jpg = out_path.with_suffix(".jpg")

    if mode == "RGB":
        set_mode(device, "RGB", force=True)
        r = _run(
            [
                "ffmpeg",
                "-hide_banner",
                "-loglevel",
                "error",
                "-y",
                "-f",
                "v4l2",
                "-input_format",
                "uyvy422",
                "-video_size",
                f"{spec.width}x{spec.height}",
                "-i",
                device,
                "-frames:v",
                "1",
                "-update",
                "1",
                str(jpg),
            ],
            timeout=5,
        )
        if r.returncode != 0 or not jpg.is_file():
            free_device(device)
            raise RuntimeError((r.stderr or "ffmpeg RGB failed").strip() or "no RGB file")
        return jpg

    raw_path = Path(tempfile.gettempdir()) / f"kinect-ir-{os.getpid()}.raw"
    data = grab_ir_raw(device, raw_path, set_fmt=True)
    try:
        raw_path.unlink(missing_ok=True)
    except OSError:
        pass
    grey = unpack_y10bpack(data, spec.width, spec.height)
    im = Image.frombytes("L", (spec.width, spec.height), grey)
    im = ImageOps.autocontrast(im, cutoff=0.4)
    im = ImageEnhance.Brightness(im).enhance(1.4)
    im = ImageEnhance.Contrast(im).enhance(1.2)
    im.convert("RGB").save(jpg, quality=92)
    return jpg


def capture_preview_bytes(device: str, mode: str) -> Path:
    dest = Path(tempfile.gettempdir()) / "kinect-panel-snap"
    return capture_frame(device, mode, dest)


def capture_vcam_frame(device: str, out_path: str | Path) -> Path:
    out_path = Path(out_path).with_suffix(".jpg")
    r = _run(
        [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-f",
            "v4l2",
            "-i",
            device,
            "-frames:v",
            "1",
            "-update",
            "1",
            str(out_path),
        ],
        timeout=4,
    )
    if r.returncode != 0 or not out_path.is_file():
        raise RuntimeError((r.stderr or "vcam snap failed").strip())
    return out_path


def nudge_pipewire_cameras() -> None:
    """Ask WirePlumber to re-scan V4L2 so browsers see the loopback source."""
    # Soft: restart wireplumber (user session) — picks up exclusive_caps capture
    for cmd in (
        ["systemctl", "--user", "reload", "wireplumber"],
        ["systemctl", "--user", "restart", "wireplumber"],
    ):
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=8)
            if r.returncode == 0:
                time.sleep(0.8)
                return
        except Exception:
            continue
