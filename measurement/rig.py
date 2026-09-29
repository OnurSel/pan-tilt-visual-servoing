"""
rig.py -- hardware layer for the tracking trials: threaded camera, colour
detector (identical to host/avci_taret.py), serial link, small helpers.
"""

import os
import threading
import time

import cv2
import numpy as np

try:
    import serial
except ImportError:
    serial = None

# ============================================================
#  Geometry and camera constants (identical to avci_taret.py)
# ============================================================
CAM_WIDTH, CAM_HEIGHT = 640, 480
FRAME_CX, FRAME_CY = CAM_WIDTH // 2, CAM_HEIGHT // 2

HFOV_DEG, VFOV_DEG = 60.0, 45.0
PX_PER_DEG_X = CAM_WIDTH / HFOV_DEG      # ~10.67 px/deg  (nominal, from lens spec)
PX_PER_DEG_Y = CAM_HEIGHT / VFOV_DEG     # E1 measures the TRUE value

PAN_MIN, PAN_MAX = 0.0, 180.0
TILT_MIN, TILT_MAX = 0.0, 180.0
PAN_INIT, TILT_INIT = 90.0, 90.0

SERIAL_PORT_CANDIDATES = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0']
SERIAL_BAUD = 115200

DETECT_SCALE = 0.5
MIN_CONTOUR_AREA = 500
MAX_CONTOUR_AREA_FRAC = 0.30

HSV_PRESETS = {
    'red':    [((0, 120, 70), (10, 255, 255)),
               ((170, 120, 70), (180, 255, 255))],
    'green':  [((36, 100, 70), (86, 255, 255))],
    'blue':   [((94, 80, 70), (126, 255, 255))],
    'yellow': [((20, 100, 100), (35, 255, 255))],
}

# ============================================================
#  Threaded camera with auto-reconnect
# ============================================================
class CameraThread:
    """Owns VideoCapture in a daemon thread and publishes the latest frame.

    Lossy on purpose: for tracking, the freshest frame always wins over a
    complete queue. Auto-reopens the device after repeated read failures.
    """

    def __init__(self, src=0, width=CAM_WIDTH, height=CAM_HEIGHT, fps=30):
        self.src, self.width, self.height = src, width, height
        self.fps = fps
        self.cap = None
        self._frame = None
        self._frame_t = None
        self._lock = threading.Lock()
        self._running = False
        self.connected = False
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._running = True
        self._open()
        self.thread.start()
        return self

    def _open(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None
        cap = cv2.VideoCapture(self.src)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            self.cap, self.connected = cap, True
            print(f"[CAMERA] OPEN src={self.src} "
                  f"fps={cap.get(cv2.CAP_PROP_FPS):.0f} (requested {self.fps})")
        else:
            self.cap, self.connected = None, False
            print(f"[CAMERA] FAIL src={self.src}")

    def _loop(self):
        misses = 0
        while self._running:
            if not self.connected or self.cap is None:
                time.sleep(0.5)
                self._open()
                continue
            ok, frame = self.cap.read()
            if not ok:
                misses += 1
                if misses > 5:
                    self.connected, misses = False, 0
                time.sleep(0.02)
                continue
            misses = 0
            with self._lock:
                self._frame, self._frame_t = frame, time.monotonic()

    def read(self):
        """Returns (frame, capture_timestamp) or (None, None)."""
        with self._lock:
            if self._frame is None:
                return None, None
            return self._frame.copy(), self._frame_t

    def stop(self):
        self._running = False
        time.sleep(0.1)
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass


# ============================================================
#  Colour detector (identical logic to avci_taret.py)
# ============================================================
_MORPH_K = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


def detect_color_largest(frame, ranges):
    """Largest in-range blob -> (cx, cy, x, y, w, h) or None."""
    if DETECT_SCALE != 1.0:
        work = cv2.resize(frame, (0, 0), fx=DETECT_SCALE, fy=DETECT_SCALE,
                          interpolation=cv2.INTER_AREA)
        min_area = MIN_CONTOUR_AREA * DETECT_SCALE * DETECT_SCALE
        max_area = MAX_CONTOUR_AREA_FRAC * (work.shape[1] * work.shape[0])
    else:
        work = frame
        min_area = MIN_CONTOUR_AREA
        max_area = MAX_CONTOUR_AREA_FRAC * (CAM_WIDTH * CAM_HEIGHT)

    hsv = cv2.cvtColor(work, cv2.COLOR_BGR2HSV)
    mask = None
    for lo, hi in ranges:
        m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _MORPH_K)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _MORPH_K)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < min_area or area > max_area:
        return None
    x, y, w, h = cv2.boundingRect(c)
    if DETECT_SCALE != 1.0:
        inv = 1.0 / DETECT_SCALE
        x, y, w, h = int(x * inv), int(y * inv), int(w * inv), int(h * inv)
    return (x + w // 2, y + h // 2, x, y, w, h)


# ============================================================
#  Serial link
# ============================================================
_serial_lock = threading.Lock()


def open_serial(port_hint=None):
    """Returns (Serial-or-None, port-or-None). port_hint='none' forces dry-run."""
    if port_hint == 'none' or serial is None:
        print("[ARDUINO] DRY-RUN (no serial)")
        return None, None
    candidates = [port_hint] if port_hint else SERIAL_PORT_CANDIDATES
    for port in candidates:
        if not port or not os.path.exists(port):
            continue
        try:
            ser = serial.Serial(port, SERIAL_BAUD, timeout=0.1, write_timeout=0.1)
            time.sleep(2.0)      # let the Arduino finish its auto-reset
            print(f"[ARDUINO] OPEN port={port}")
            return ser, port
        except Exception as e:
            print(f"[ARDUINO] FAIL {port}: {e}")
    print("[ARDUINO] NO PORT - DRY-RUN")
    return None, None


def safe_write(ser, line):
    """Thread-safe write; returns the (possibly reconnected) handle."""
    if ser is None:
        return None
    with _serial_lock:
        try:
            ser.write(line.encode('utf-8'))
            return ser
        except Exception as e:
            print(f"[ARDUINO] write error: {e} - reconnecting")
            try:
                ser.close()
            except Exception:
                pass
    new, _ = open_serial()
    return new


def send_turret(ser, pan_deg, tilt_deg, laser=0):
    """Emit "PAN_X10,TILT_X10,LASER\\n" -- tenths of a degree, 0.1 deg resolution."""
    p = int(round(max(PAN_MIN, min(PAN_MAX, pan_deg)) * 10))
    t = int(round(max(TILT_MIN, min(TILT_MAX, tilt_deg)) * 10))
    return safe_write(ser, f"{p},{t},{int(laser)}\n")


# ============================================================
#  Small helpers
# ============================================================
class MovingAverage:
    """Five-sample centroid filter, matching the original pipeline."""

    def __init__(self, size=5):
        self.size, self.buf = size, []

    def add(self, v):
        self.buf.append(v)
        if len(self.buf) > self.size:
            self.buf.pop(0)
        return sum(self.buf) / len(self.buf)

    def reset(self):
        self.buf = []


def park_head(ser, pan=PAN_INIT, tilt=TILT_INIT, settle=2.0):
    """Centre the head and wait for it to physically settle."""
    send_turret(ser, pan, tilt, 0)
    time.sleep(settle)


LEG_HOLD = {"FL": (50, 170), "FR": (100, 175), "BL": (80, 177), "BR": (80, 175)}


def hold_legs(ser):
    """Put all four legs in the standing pose; the robot stays still."""
    for leg, (yaw, lift) in LEG_HOLD.items():
        ser = safe_write(ser, f"{leg}_Y {yaw}\n")
        ser = safe_write(ser, f"{leg}_L {lift}\n")
    return ser
