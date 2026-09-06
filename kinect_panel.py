#!/usr/bin/env python3
"""Summonable Kinect control panel: tilt + RGB/IR webcam via virtual cam."""
from __future__ import annotations

import os
import signal
import socket
import sys
import tempfile
import threading
from pathlib import Path

import gi

gi.require_version("Gtk", "3.0")
gi.require_version("GdkPixbuf", "2.0")
from gi.repository import GLib, Gtk, Gdk, GdkPixbuf  # noqa: E402

from kinect_bridge import WebcamBridge  # noqa: E402
from kinect_camera import find_vcam  # noqa: E402
from kinect_camera import (  # noqa: E402
    MODES,
    capture_preview_bytes,
    capture_vcam_frame,
    find_kinect_video,
)
from kinect_motor import (  # noqa: E402
    LED_BLINK_GREEN,
    LED_GREEN,
    LED_OFF,
    LED_RED,
    KinectMotor,
)

SOCK_PATH = Path(os.environ.get("XDG_RUNTIME_DIR", "/tmp")) / "kinect-panel.sock"
STEP = 5.0
TILT_MIN, TILT_MAX = -30.0, 30.0


class SingleInstance:
    def __init__(self, on_activate) -> None:
        self.on_activate = on_activate
        self._sock: socket.socket | None = None

    def acquire(self) -> bool:
        if SOCK_PATH.exists():
            try:
                s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                s.settimeout(0.4)
                s.connect(str(SOCK_PATH))
                s.sendall(b"RAISE\n")
                s.close()
                return False
            except OSError:
                try:
                    SOCK_PATH.unlink(missing_ok=True)
                except OSError:
                    pass
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            self._sock.bind(str(SOCK_PATH))
        except OSError:
            return True
        self._sock.listen(3)
        self._sock.setblocking(False)
        GLib.io_add_watch(self._sock.fileno(), GLib.IO_IN, self._on_conn)
        return True

    def _on_conn(self, fd, cond):
        try:
            conn, _ = self._sock.accept()
            conn.settimeout(0.2)
            try:
                conn.recv(64)
            except OSError:
                pass
            conn.close()
        except OSError:
            pass
        self.on_activate()
        return True

    def close(self) -> None:
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
        try:
            SOCK_PATH.unlink(missing_ok=True)
        except OSError:
            pass


class KinectPanel(Gtk.Window):
    def __init__(self) -> None:
        super().__init__(title="Kinect Panel")
        self.set_default_size(460, 580)
        self.set_border_width(12)
        self.set_keep_above(True)
        self.set_type_hint(Gdk.WindowTypeHint.UTILITY)

        self.motor = KinectMotor()
        self.bridge = WebcamBridge()
        self.device = find_kinect_video()
        self.vcam = find_vcam()
        self.mode = "RGB"
        self.camera_on = False
        self.preview_on = False
        self._bridge_starting = False
        self._target_tilt = 0.0
        self._closing = False
        self._snap_busy = False

        root = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        self.add(root)

        self.status = Gtk.Label(label="Starting…", xalign=0)
        self.status.set_line_wrap(True)
        root.pack_start(self.status, False, False, 0)

        preview_frame = Gtk.Frame(label="Preview")
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        box.set_border_width(4)
        self.preview_image = Gtk.Image()
        self.preview_image.set_size_request(400, 240)
        box.pack_start(self.preview_image, True, True, 0)
        prev_row = Gtk.Box(spacing=8)
        self.btn_preview = Gtk.ToggleButton(label="Preview OFF")
        self.btn_preview.connect("toggled", self._on_preview_toggled)
        self.btn_snap = Gtk.Button(label="Snap once")
        self.btn_snap.connect("clicked", lambda *_: self._snap_async())
        prev_row.pack_start(self.btn_preview, True, True, 0)
        prev_row.pack_start(self.btn_snap, False, False, 0)
        box.pack_start(prev_row, False, False, 0)
        preview_frame.add(box)
        root.pack_start(preview_frame, True, True, 0)

        cam_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.btn_cam = Gtk.ToggleButton(label="Camera OFF")
        self.btn_cam.connect("toggled", self._on_cam_toggled)
        self.btn_cam.get_style_context().add_class("suggested-action")
        cam_box.pack_start(self.btn_cam, True, True, 0)

        self.mode_rgb = Gtk.RadioButton.new_with_label_from_widget(None, "RGB")
        self.mode_ir = Gtk.RadioButton.new_with_label_from_widget(self.mode_rgb, "IR")
        self.mode_rgb.connect("toggled", self._on_mode, "RGB")
        self.mode_ir.connect("toggled", self._on_mode, "IR")
        cam_box.pack_start(self.mode_rgb, False, False, 0)
        cam_box.pack_start(self.mode_ir, False, False, 0)
        root.pack_start(cam_box, False, False, 0)

        self.dev_label = Gtk.Label(xalign=0)
        self.dev_label.set_selectable(True)
        self.dev_label.set_line_wrap(True)
        root.pack_start(self.dev_label, False, False, 0)

        tip = Gtk.Label(
            label="Browser: Camera ON first, then pick “Kinect Webcam” (not Xbox NUI / Lenovo).",
            xalign=0,
        )
        tip.set_line_wrap(True)
        tip.get_style_context().add_class("dim-label")
        root.pack_start(tip, False, False, 0)

        tilt_frame = Gtk.Frame(label="Tilt")
        tilt = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        tilt.set_border_width(8)
        tilt_frame.add(tilt)

        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.btn_up = Gtk.Button(label="▲ Up")
        self.btn_down = Gtk.Button(label="▼ Down")
        self.btn_level = Gtk.Button(label="Level 0°")
        self.btn_up.connect("clicked", lambda *_: self._nudge(+STEP))
        self.btn_down.connect("clicked", lambda *_: self._nudge(-STEP))
        self.btn_level.connect("clicked", lambda *_: self._set_tilt(0.0))
        for b in (self.btn_up, self.btn_level, self.btn_down):
            row.pack_start(b, True, True, 0)
        tilt.pack_start(row, False, False, 0)

        self.tilt_scale = Gtk.Scale.new_with_range(
            Gtk.Orientation.HORIZONTAL, TILT_MIN, TILT_MAX, 1
        )
        self.tilt_scale.set_value(0)
        self.tilt_scale.set_digits(0)
        self.tilt_scale.set_draw_value(True)
        self.tilt_scale.connect("button-release-event", self._on_scale_release)
        self.tilt_scale.connect("key-release-event", self._on_scale_release)
        tilt.pack_start(self.tilt_scale, False, False, 0)

        self.tilt_label = Gtk.Label(label="Tilt: —")
        tilt.pack_start(self.tilt_label, False, False, 0)
        root.pack_start(tilt_frame, False, False, 0)

        foot = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.keep_above = Gtk.CheckButton(label="Always on top")
        self.keep_above.set_active(True)
        self.keep_above.connect(
            "toggled", lambda b: self.set_keep_above(b.get_active())
        )
        foot.pack_start(self.keep_above, True, True, 0)
        hide_btn = Gtk.Button(label="Hide")
        hide_btn.connect("clicked", lambda *_: self.hide())
        foot.pack_end(hide_btn, False, False, 0)
        root.pack_start(foot, False, False, 0)

        self.connect("delete-event", self._on_delete)
        self.connect("destroy", self._on_destroy)

        GLib.timeout_add(250, self._poll_tilt)
        GLib.timeout_add(500, self._preview_tick)
        GLib.idle_add(self._boot)

    def _boot(self):
        try:
            self.motor.open()
            self.motor.set_led(LED_OFF)
            st = self.motor.get_tilt()
            self._target_tilt = max(TILT_MIN, min(TILT_MAX, st.degrees))
            if abs(self._target_tilt) > 35:
                self._target_tilt = 0.0
            self.tilt_scale.set_value(self._target_tilt)
            self._set_status(
                "Ready. Pick RGB or IR → Camera ON → use “Kinect Webcam” in apps."
            )
        except Exception as e:
            self._set_status(f"Motor: {e}")
        self.device = find_kinect_video()
        self.vcam = find_vcam()
        self._refresh_dev_label()
        return False

    def _set_status(self, text: str) -> None:
        self.status.set_text(text)

    def _refresh_dev_label(self) -> None:
        src = self.device or find_kinect_video() or "—"
        self.vcam = find_vcam()
        if self.camera_on and self.bridge.running and self.vcam:
            self.dev_label.set_text(
                f"Apps → {self.vcam}  “Kinect Webcam”  ·  {self.mode}  (src {src})"
            )
        elif self.vcam:
            self.dev_label.set_text(
                f"Apps → {self.vcam}  “Kinect Webcam”  (Camera OFF)  ·  src {src}"
            )
        else:
            self.dev_label.set_text(
                f"Virtual cam not loaded · src {src} · run setup_loopback.sh --load once"
            )

    def raise_panel(self) -> None:
        self.show_all()
        self.present()
        self.set_keep_above(self.keep_above.get_active())

    def _on_delete(self, *args):
        self.hide()
        return True

    def _on_destroy(self, *args):
        self._closing = True
        try:
            self.bridge.stop()
        except Exception:
            pass
        try:
            self.motor.close()
        except Exception:
            pass

    def _nudge(self, delta: float) -> None:
        self._set_tilt(self._target_tilt + delta)

    def _on_scale_release(self, *args):
        self._set_tilt(self.tilt_scale.get_value())
        return False

    def _set_tilt(self, degrees: float) -> None:
        degrees = max(TILT_MIN, min(TILT_MAX, float(degrees)))
        self._target_tilt = degrees
        if abs(self.tilt_scale.get_value() - degrees) > 0.05:
            self.tilt_scale.set_value(degrees)

        def work():
            try:
                self.motor.set_tilt(degrees)
                GLib.idle_add(self._set_status, f"Tilt → {degrees:.0f}°")
            except Exception as e:
                GLib.idle_add(self._set_status, f"Tilt error: {e}")

        threading.Thread(target=work, daemon=True).start()

    def _poll_tilt(self):
        if self._closing:
            return False
        try:
            self.motor.pump()
            st = self.motor.get_tilt()
            deg = st.degrees
            if deg < -40:
                deg = self._target_tilt
            moving = "moving" if st.status == 4 else "ok"
            self.tilt_label.set_text(
                f"Tilt: {deg:.1f}°  (target {self._target_tilt:.0f}°)  [{moving}]"
            )
        except Exception as e:
            self.tilt_label.set_text(f"Tilt: error ({e})")
        return True

    def _on_mode(self, button, mode: str):
        if not button.get_active():
            return
        self.mode = mode
        if self.camera_on:
            self._arm_camera()
        if not self.preview_on:
            self.btn_preview.set_active(True)
        else:
            self._snap_async()
        if not self.camera_on:
            self._set_status(
                f"Mode {mode} · turn Camera ON to publish as “Kinect Webcam”"
            )

    def _on_cam_toggled(self, button: Gtk.ToggleButton):
        on = button.get_active()
        self.camera_on = on
        button.set_label("Camera ON" if on else "Camera OFF")
        if on:
            self._arm_camera()
        else:
            self._bridge_starting = False
            try:
                self.bridge.stop()
            except Exception:
                pass
            try:
                self.motor.set_led(LED_OFF)
            except Exception:
                pass
            self._set_status("Camera OFF — virtual webcam stopped")
            self._refresh_dev_label()

    def _on_preview_toggled(self, button: Gtk.ToggleButton):
        self.preview_on = button.get_active()
        button.set_label("Preview ON" if self.preview_on else "Preview OFF")
        if self.preview_on:
            self._snap_async()

    def _arm_camera(self) -> None:
        self.device = find_kinect_video()
        if not self.device:
            self._set_status("No Kinect video node. Plug Kinect.")
            self.btn_cam.set_active(False)
            self.camera_on = False
            return
        mode = self.mode
        self._bridge_starting = True
        self._set_status(
            f"Starting {mode} webcam…"
        )

        def work():
            try:
                self.bridge.start(mode)
                try:
                    self.motor.set_led(
                        LED_BLINK_GREEN if mode == "RGB" else LED_GREEN
                    )
                except Exception:
                    pass
                GLib.idle_add(self._on_bridge_ok, mode)
            except Exception as e:
                GLib.idle_add(self._on_bridge_fail, str(e))

        threading.Thread(target=work, daemon=True).start()

    def _on_bridge_ok(self, mode: str):
        self._bridge_starting = False
        self.vcam = self.bridge.vcam or find_vcam()
        self._refresh_dev_label()
        self._set_status(
            f"Webcam LIVE · {mode} · browser: “Kinect Webcam” ({self.vcam}) — leave Camera ON"
        )
        if not self.preview_on:
            self.btn_preview.set_active(True)
        else:
            self._snap_async()
        return False

    def _on_bridge_fail(self, err: str):
        self._bridge_starting = False
        self.camera_on = False
        self.btn_cam.set_active(False)
        try:
            self.motor.set_led(LED_RED)
        except Exception:
            pass
        self._set_status(f"Webcam failed: {err}")
        self._refresh_dev_label()
        return False

    def _preview_tick(self):
        if self._closing:
            return False
        if self.preview_on and not self._bridge_starting:
            self._snap_async()
        if self.camera_on and not self._bridge_starting:
            if self.bridge.error:
                self._set_status(f"Bridge error: {self.bridge.error}")
            elif not self.bridge.running:
                self._set_status("Bridge stopped — toggle Camera ON again")
                self.camera_on = False
                self.btn_cam.set_active(False)
                self._refresh_dev_label()
        return True

    def _snap_async(self) -> None:
        if self._snap_busy or self._bridge_starting:
            return
        self._snap_busy = True
        mode = self.mode
        use_vcam = bool(
            self.camera_on and self.bridge.running and (self.bridge.vcam or find_vcam())
        )
        vcam = self.bridge.vcam or find_vcam()
        source = self.device or find_kinect_video()

        def work():
            try:
                dest = Path(tempfile.gettempdir()) / "kinect-panel-snap"
                if use_vcam and vcam:
                    path = capture_vcam_frame(vcam, dest)
                else:
                    if not source:
                        raise RuntimeError("no Kinect source")
                    path = capture_preview_bytes(source, mode)
                GLib.idle_add(self._show_snap, str(path))
            except Exception as e:
                GLib.idle_add(self._set_status, f"Preview: {e}")
            finally:
                self._snap_busy = False

        threading.Thread(target=work, daemon=True).start()

    def _show_snap(self, path: str):
        try:
            if not path or not Path(path).is_file():
                self._set_status(f"Preview: missing file {path}")
                return False
            pb = GdkPixbuf.Pixbuf.new_from_file_at_scale(path, 400, 240, True)
            self.preview_image.set_from_pixbuf(pb)
            self.preview_image.queue_draw()
        except Exception as e:
            self._set_status(f"Preview show: {e}")
        return False


def main(argv=None) -> int:
    Gtk.init(argv)
    holder = {"win": None}

    def activate():
        win = holder["win"]
        if win is not None:
            win.raise_panel()

    inst = SingleInstance(on_activate=lambda: GLib.idle_add(activate))
    if not inst.acquire():
        print("Kinect panel already running — raised existing window.", file=sys.stderr)
        return 0

    win = KinectPanel()
    holder["win"] = win

    def _cleanup(*_):
        win._closing = True
        try:
            win.bridge.stop()
        except Exception:
            pass
        try:
            win.motor.close()
        except Exception:
            pass
        inst.close()
        Gtk.main_quit()

    win.connect("destroy", lambda *_: _cleanup())
    win.show_all()
    signal.signal(signal.SIGINT, lambda *_: _cleanup())
    signal.signal(signal.SIGTERM, lambda *_: _cleanup())
    try:
        Gtk.main()
    finally:
        inst.close()
        try:
            win.bridge.stop()
        except Exception:
            pass
        try:
            win.motor.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
