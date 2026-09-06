"""libfreenect motor/LED control via ctypes (Python 3.14 safe)."""
from __future__ import annotations

import ctypes
import threading
from dataclasses import dataclass
from typing import Optional

# freenect_device_flags
FREENECT_DEVICE_MOTOR = 0x01
FREENECT_DEVICE_CAMERA = 0x02

# freenect_led_options
LED_OFF = 0
LED_GREEN = 1
LED_RED = 2
LED_YELLOW = 3
LED_BLINK_GREEN = 4
LED_BLINK_RED_YELLOW = 6

# log level
FREENECT_LOG_WARNING = 1


class timeval(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_usec", ctypes.c_long)]


@dataclass
class TiltState:
    degrees: float
    status: int  # 0 stopped, 1 limit, 4 moving (typical)


class KinectMotor:
    """Keep motor handle open for low-latency tilt from the panel."""

    def __init__(self) -> None:
        self._lib = ctypes.CDLL("libfreenect.so.0")
        self._setup_prototypes()
        self._ctx = ctypes.c_void_p()
        self._dev = ctypes.c_void_p()
        self._lock = threading.RLock()
        self._open = False

    def _setup_prototypes(self) -> None:
        L = self._lib
        L.freenect_init.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_void_p]
        L.freenect_init.restype = ctypes.c_int
        L.freenect_shutdown.argtypes = [ctypes.c_void_p]
        L.freenect_set_log_level.argtypes = [ctypes.c_void_p, ctypes.c_int]
        L.freenect_select_subdevices.argtypes = [ctypes.c_void_p, ctypes.c_int]
        L.freenect_num_devices.argtypes = [ctypes.c_void_p]
        L.freenect_num_devices.restype = ctypes.c_int
        L.freenect_open_device.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_void_p),
            ctypes.c_int,
        ]
        L.freenect_open_device.restype = ctypes.c_int
        L.freenect_close_device.argtypes = [ctypes.c_void_p]
        L.freenect_close_device.restype = ctypes.c_int
        L.freenect_set_tilt_degs.argtypes = [ctypes.c_void_p, ctypes.c_double]
        L.freenect_set_tilt_degs.restype = ctypes.c_int
        L.freenect_set_led.argtypes = [ctypes.c_void_p, ctypes.c_int]
        L.freenect_set_led.restype = ctypes.c_int
        L.freenect_update_tilt_state.argtypes = [ctypes.c_void_p]
        L.freenect_update_tilt_state.restype = ctypes.c_int
        L.freenect_get_tilt_state.argtypes = [ctypes.c_void_p]
        L.freenect_get_tilt_state.restype = ctypes.c_void_p
        L.freenect_get_tilt_degs.argtypes = [ctypes.c_void_p]
        L.freenect_get_tilt_degs.restype = ctypes.c_double
        L.freenect_get_tilt_status.argtypes = [ctypes.c_void_p]  # freenect_raw_tilt_state*
        L.freenect_get_tilt_status.restype = ctypes.c_int
        L.freenect_process_events_timeout.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(timeval),
        ]
        L.freenect_process_events_timeout.restype = ctypes.c_int

    def open(self) -> None:
        with self._lock:
            if self._open:
                return
            if self._lib.freenect_init(ctypes.byref(self._ctx), None) < 0:
                raise RuntimeError("freenect_init failed")
            self._lib.freenect_set_log_level(self._ctx, FREENECT_LOG_WARNING)
            # Motor only — gspca owns the camera V4L2 node
            self._lib.freenect_select_subdevices(self._ctx, FREENECT_DEVICE_MOTOR)
            n = self._lib.freenect_num_devices(self._ctx)
            if n < 1:
                self._lib.freenect_shutdown(self._ctx)
                self._ctx = ctypes.c_void_p()
                raise RuntimeError("No Kinect found (motor)")
            if self._lib.freenect_open_device(self._ctx, ctypes.byref(self._dev), 0) < 0:
                self._lib.freenect_shutdown(self._ctx)
                self._ctx = ctypes.c_void_p()
                raise RuntimeError("Could not open Kinect motor (busy?)")
            self._open = True

    def close(self) -> None:
        with self._lock:
            if not self._open:
                return
            try:
                self._lib.freenect_set_led(self._dev, LED_OFF)
            except Exception:
                pass
            try:
                self._lib.freenect_close_device(self._dev)
            except Exception:
                pass
            try:
                self._lib.freenect_shutdown(self._ctx)
            except Exception:
                pass
            self._dev = ctypes.c_void_p()
            self._ctx = ctypes.c_void_p()
            self._open = False

    def ensure(self) -> None:
        if not self._open:
            self.open()

    def set_tilt(self, degrees: float) -> None:
        degrees = max(-30.0, min(30.0, float(degrees)))
        with self._lock:
            self.ensure()
            r = self._lib.freenect_set_tilt_degs(self._dev, degrees)
            if r < 0:
                raise RuntimeError(f"set_tilt failed ({r})")
            # pump a few events so the motor moves
            tv = timeval(0, 50000)
            for _ in range(4):
                self._lib.freenect_process_events_timeout(self._ctx, ctypes.byref(tv))

    def set_led(self, led: int) -> None:
        with self._lock:
            self.ensure()
            self._lib.freenect_set_led(self._dev, int(led))

    def get_tilt(self) -> TiltState:
        with self._lock:
            self.ensure()
            tv = timeval(0, 30000)
            self._lib.freenect_process_events_timeout(self._ctx, ctypes.byref(tv))
            self._lib.freenect_update_tilt_state(self._dev)
            st = self._lib.freenect_get_tilt_state(self._dev)
            if not st:
                return TiltState(degrees=0.0, status=-1)
            deg = float(self._lib.freenect_get_tilt_degs(st))
            status = int(self._lib.freenect_get_tilt_status(st))
            return TiltState(degrees=deg, status=status)

    def pump(self) -> None:
        with self._lock:
            if not self._open:
                return
            tv = timeval(0, 20000)
            self._lib.freenect_process_events_timeout(self._ctx, ctypes.byref(tv))

    def __enter__(self) -> "KinectMotor":
        self.open()
        return self

    def __exit__(self, *args) -> None:
        self.close()
