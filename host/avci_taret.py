#!/usr/bin/env python3
"""
=====================================================================
  PREDATOR TRACKING TURRET - Raspberry Pi "Brain"
  EE 304 Embedded Systems - Capstone Project A
=====================================================================

  Two-axis (pan/tilt) servo turret that autonomously tracks either a
  face (Haar cascade) or a coloured object (HSV thresholding) and
  "fires" a laser when the target is locked in the centre deadband.

  Hardware pin map (mirror of turret_arduino.ino):
      Pan  servo            -> Arduino D10
      Tilt servo            -> Arduino D9
      Laser (NPN base)      -> Arduino D13

  Engineering features (mapped to the rubric):
      * Proportional control loop on each axis (pixel-error -> angle-delta)
      * Threaded camera capture, decoupled from vision processing
      * Moving-average filter on detection coordinates (anti-jitter)
      * Camera & serial auto-reconnect (passes the "unplug test")
      * Largest-target selection (multi-face / multi-blob robust)
      * Latency CSV log for the report's latency graph
      * Live HUD: FPS, latency, pan/tilt, mode, serial/cam status
      * Safe shutdown: centres servos and turns laser off on exit

  Hotkeys:
      q   quit (clean shutdown)
      m   toggle face <-> color mode
      f   force face mode
      c   force color mode
      r/g/b/y  set tracked colour (red / green / blue / yellow)
      [ / ]    decrease / increase lock-tolerance (deadband px)
      , / .    decrease / increase approach percentage (5..100 %)
      ; / '    decrease / increase max-step per frame (0.5..15 deg)
      h        home: servos back to (90, 90)

  Mouse:
      Left-click on the live feed to sample the colour under the cursor;
      the colour is stored as the 'custom' preset and immediately tracked.

  Usage:
      python3 avci_taret.py
      python3 avci_taret.py --mode color --color green
      python3 avci_taret.py --log      # write latency_log.csv
"""

import argparse
import csv
import os
import queue
import threading
import time
from collections import deque

import cv2
import numpy as np
import serial

# ============================================================
#  Configuration
# ============================================================
CAM_WIDTH      = 640
CAM_HEIGHT     = 480
FRAME_CX       = CAM_WIDTH  // 2
FRAME_CY       = CAM_HEIGHT // 2
LOCK_TOL       = 32           # pixels; deadband for "locked" state (yari-genis)
# Hysteresis: kilit EDINIRKEN dar bolge, KORURKEN genis bolge.
# 0.1 derece servo cozunurlugu ile (writeMicroseconds) bir komut ~1 px
# hareket veriyor; bu HOLD oranini gelecekte azaltma imkanini yaratir
# ama goresel rahatlik icin asagidaki orani 1.8'de tutuyoruz.
LOCK_HOLD_MULT = 1.8          # HOLD bolgesi = LOCK_TOL * 1.8 (~57 px)

# Ic (fine) kare: dis kutuda LOCKED olduktan sonra turret, hedefi bu kucuk
# ic kareye getirmek icin "Step-and-Wait" (vur ve bekle) algoritmasi ile
# yaklasir. Lazer DIS kutuya gore yanmaya devam eder; ic kareye tam varmak
# sart degil.
#
# NEDEN STEP-AND-WAIT?
#   Tipik analog servo deadband ~5-10 us pulse = 0.5-1.0 derece. Bunun
#   altinda komutlar yutulur; ust uste birikip tek seferde sicrar. Ayrica
#   her karede komut atmak servo+kamera gecikmesi ile "hunting" (sag-sol
#   salinim) yaratir.
#   Cozum: her FINE_WAIT_SEC'de BIR kez, deadband UZERINDE NET bir adim
#   (FINE_STEP_DEG) at; arada kamera olcumu otursun. "Tik... tik... tik..."
#   ritimli, sakin yaklasim.
FINE_LOCK_TOL       = 12      # pixels; ic kare yari-genis (giris esigi)
FINE_RELEASE_MULT   = 2.5     # ic kare LATCH cikis carpani: 12*2.5 = 30 px
                              # Bir kez ic kareye girince, hedef ~30 px disina
                              # cikana kadar SERVO HAREKETSIZ. Detection jitter
                              # ve servo overshoot ile olusan salinim biter.
FINE_STEP_DEG       = 0.5     # NET adim - servoyu kesin hareket ettirir (>deadband)
FINE_WAIT_SEC       = 0.3     # iki ince adim arasi bekleme (kamera oturmasi)

PAN_MIN,  PAN_MAX  = 0, 180
TILT_MIN, TILT_MAX = 0, 180
PAN_INIT, TILT_INIT = 90, 90

# --- Hareket hesabi: piksel hatasi -> derece donus ---
# Kamera goerus acisi (degrees). Tipik USB webcam ~60 deg yatay.
HFOV_DEG     = 60.0
VFOV_DEG     = 45.0
PX_PER_DEG_X = CAM_WIDTH  / HFOV_DEG   # ~10.67 px/derece
PX_PER_DEG_Y = CAM_HEIGHT / VFOV_DEG   # ~10.67 px/derece

# --- Yuzde-yaklasimli, kare-kilitli kontrol ---
# Her YENI karede gereken donus aci hesaplanir; bu acinin sadece
# APPROACH_PCT kadari uygulanir. Kalan kismi, bir sonraki kare
# geldiginde kamera taze pozisyonu olcer ve tekrar hesaplanir.
# Boylece "kamera donmeden komut gondermek" hatasi (servonun 180'e
# firlamasi) tamamen ortadan kalkar.
APPROACH_PCT  = 0.08   # 0..1 -- her karede gereken acinin %8'i (yavas)
MAX_STEP_DEG  = 0.5    # bir komutta atilabilecek max aci (~15 deg/sn)

# Soft-zone: hedef bu kadar dereceden yakinsa kazanc kuadratik kuculur.
# ESKIDEN 0.5'ti (~5 px). Bu COK dardi: taret hedefe bodoslama girip
# overshoot yapiyordu, dis kutuya hizla girip iceride salinim basliyordu.
# 3.0 derece (~32 px) yaparak frenlemeyi ERKEN baslatiyoruz; taret dis
# kutuya zaten yavaslamis olarak girer, "hunting" durur.
SOFT_ZONE_DEG = 3.0

# Velocity smoothing kaldirildi: davranis dogrudan, ongorulebilir.
# Float pan/tilt birikir; Arduino'ya yalniz int(round) degisince komut gider
# (komut hysteresis'i, bkz asagisi). Bu kombinasyon "yavas ama kesin
# yakinsayan" davranis verir; mikro adimlar bile birike birike sonunda
# int sinirini gecince servo bir derece doner.
#
# Bu sabitler runtime'da klavyeden de ayarlanir; bkz ',' '.' ';' '''.
# Eger servo gercekte komut edilenden daha cok donuyorsa, kameranin
# gercek FOV'u burada varsayilandan dardir; HFOV_DEG / VFOV_DEG
# degerlerini olcup duzeltmek "kalici" cozumdur.

# Yeni kare gelmedigi durumda her N saniyede bir "heartbeat" gonderilir
# (Arduino watchdog'u bizim hala oldugumuzu bilsin).
HEARTBEAT_S   = 0.20

FILTER_WINDOW  = 5            # moving-average length for x/y (Haar gurultusu ~5 -> 2 px)

SEND_PERIOD_S  = 0.050        # max 20 commands/sec to the Arduino (yavasletildi)
WATCHDOG_RESEND = 0.10        # re-send center+laser=0 if no detection
                              # in this long (helps Arduino watchdog hear us)

LATENCY_LOG    = 'latency_log.csv'

SERIAL_PORT_CANDIDATES = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0']
SERIAL_BAUD            = 115200

# Yuz/renk tespitini bu oranda kuculmus frame'de yap (hiz icin).
# 0.5 -> 320x240'da tespit, koordinatlar 2x ile geri buyutulur.
# Haar minSize de bu olcekte uygulanir (40 -> 20).
DETECT_SCALE           = 0.5

# Kamera bu kadar saniye baglanamazsa program cikis yapar.
# Sonsuz "WAITING FOR CAMERA" yerine net hata sinyali.
CAM_OPEN_TIMEOUT_S     = 10.0

# ============================================================
#  Walk (quadruped) konfigurasyonu  -- robot_arduino.ino ile birlikte
# ============================================================
# Sketch'teki per-leg HOME pozisyonlari ile bire bir ayni olmali.
WALK_YAW_HOME  = {"FL": 50,  "FR": 100, "BL": 80,  "BR": 80}
# LIFT_HOME: Arduino'nun robot_arduino.ino dosyasindaki LIFT_HOME[4] ile
# BIREBIR ayni olmali. Onceki degerler (162/170/180/172) eski kalibrasyon
# idi -> 500g yuk altinda bacaklar yeterince uzanmiyordu, robot cokuyordu.
# Bu degerler Arduino kalibrasyonu, yuklu robot ayakta durabiliyor.
WALK_LIFT_HOME = {"FL": 170, "FR": 175, "BL": 177, "BR": 175}

# Yon duzeltmesi (ters monte servolar icin). Yuruyus yanlis yone giderse cevir.
WALK_YAW_DIR   = {"FL": +1, "FR": -1, "BL": +1, "BR": -1}
WALK_LEG_SIDE  = {"FL": -1, "FR": +1, "BL": -1, "BR": +1}

# === Walk parametreleri: narin bacak + 500g yuk modu =====================
# Tasarim: SWING-and-PUSH gait. Her adim 3 evre:
#   1) Capraz cift FL+BR havada ileri +A'ya swing, yere konur
#   2) Capraz cift FR+BL havada ileri +A'ya swing, yere konur
#   3) 4 bacak yerde -> hepsi ESANLI 0'a (home yaw) doner -> govde ileri kayar
# Adim sonunda her bacak HOME pozisyonunda. State birikmiyor, bacak asla
# extreme aciya gitmiyor. Swing sirasinda 2 capraz bacak yerde = X destek
# (en kararli 2-leg paterni). Body-push sirasinda 4 bacak yerde = 500g yuk
# minimum gerilimde dagilir.
# Garanti yuruyus parametreleri (500g yuk + narin bacak):
#   Swing genlik buyuk + lift yuksek + faz yavas. Cok kucuk genlikte
#   4-bacak push'u zemin surtunmesini yenecek torque uretemiyordu;
#   genligi 2x artirdik, body push fazini 2x yavaslattik.
WALK_SWING_AMP = 14     # +/-14 derece: gorulebilir adim mesafesi (DEGISMEDI)
WALK_LIFT_UP   = 18     # 18 derece: ayak yere takilmadan kalkar (DEGISMEDI)
# ASIMETRIK FAZ SURELERI -- HER FAZ KENDI GERCEK SURESINI ALSIN.
# Eskiden hepsi WALK_PHASE_DT (0.08-0.15) idi; en yavas fazi tum digerlerine
# zorluyordu. Simdi:
#   LIFT  : 18 deg dikey hareket, bacak agirligi serbest -> 0.04 sn yeter
#   YAW   : 14 deg havada swing, bacak yuksuz -> 0.07 sn (yon icin KRITIK,
#           daha kisa olursa servo varmadan komut biter, yonler karisir)
#   PUSH  : 14 deg yatay, 4 bacak ESANLI ~500g yuk -> 0.07 sn min
#
# Adim suresi: 4*(LIFT+YAW+LIFT) + PUSH + HOLD = 4*0.15 + 0.07 + 0.04
#            = 0.71 sn/adim  (eski 1.14 sn'den ~%37 hizli)
#
# DAHA HIZLI ISTERSEN: yaw 0.06 dene; yon karisirsa 0.07'ye geri don.
# DAHA STABIL ISTERSEN: hepsini 0.01 sn arttir.
WALK_LIFT_DT   = 0.04   # lift up / lift down faz suresi
WALK_YAW_DT    = 0.07   # havada yaw swing (yon icin kritik min)
WALK_PUSH_DT   = 0.07   # 4-bacak yuklu body push
WALK_HOLD_DT   = 0.04   # adim sonu HOLD oturma suresi

# Donme icin ozel genlik. Donmede 4 bacak karsilikli (sol back, sag forward)
# kuvvet uretir; karsilikli surtunmeler birbirini kilitleyebildigi icin
# forward walk'tan daha buyuk genlik gerekir.
WALK_ROT_AMP   = 18     # donme swing genligi (forward walk'tan %30 buyuk)
WALK_STEPS_180 = 10     # 180 derece icin tahmini adim (rot_amp=18 ile, deneyerek ayarla)

HAAR_CANDIDATES = [
    '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml',
    '/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml',
]
# Some OpenCV builds ship the cascades inside the wheel; only add that
# path if it actually exists (older pip-installed cv2 has no .data).
_cv2_data = getattr(cv2, 'data', None)
if _cv2_data is not None:
    HAAR_CANDIDATES.append(
        os.path.join(_cv2_data.haarcascades,
                     'haarcascade_frontalface_default.xml'))

# HSV presets - red wraps the hue circle so it needs two ranges.
# 'custom' slot tikla-renk-ogret ozelligi tarafindan dolduruluyor.
HSV_PRESETS = {
    'red':    [((0,   120,  70), ( 10, 255, 255)),
               ((170, 120,  70), (180, 255, 255))],
    'green':  [(( 36, 100,  70), ( 86, 255, 255))],
    'blue':   [(( 94,  80,  70), (126, 255, 255))],
    'yellow': [(( 20, 100, 100), ( 35, 255, 255))],
    'custom': [(( 0,   60,  60), ( 10, 255, 255))],
}
MIN_CONTOUR_AREA      = 500    # gurultu hedef sanilmasin (px^2)
MAX_CONTOUR_AREA_FRAC = 0.30   # cerceveden buyuk -> hedef degil (duvar/gokyuzu)

# Lazer ancak bu kadar ardisik karede merkezde kalirsa yanar.
# (Tek karelik gurultude lazer cakacagi yerine sabit/anlamli kilit verir.)
LOCK_STABILITY_FRAMES = 2     # 3 -> 2: aksi takdirde sinirda flip eden
                              # gurultu hic kilit alamiyordu

# "Bad frame" toleransi: hedef hem KAYIP olsa hem de KUTUNUN DISINDA olsa
# bu kadar ardisik kotu kareye kadar lock_streak ve lazer yasamaya devam
# eder. Boylece:
#   - Haar tek-kare iskasi lazeri sondurmez
#   - Hedef lock_tol sinirinda gurultuden in/out flip ederse kilit
#     edinilebilir (her flip lock_streak'i sifirlamaz)
LOCK_BAD_TOLERANCE    = 5


# ============================================================
#  Kontrol: yuzde-yaklasimli oransal adim hesabi
# ============================================================
def compute_step(err_px, px_per_deg,
                 approach_pct=APPROACH_PCT, max_step=MAX_STEP_DEG):
    """Bir karede uygulanacak aci adimini hesaplar.

    Sira:
      1) Piksel hatasi -> derece (FOV kalibrasyonu ile)
      2) Bu acinin APPROACH_PCT * SOFT GAIN kadarini al
         (yakinda kazanc kuadratik kuculur -> overshoot yok)
      3) MAX_STEP_DEG ile kirp; yakinken max_step de orantili kuculur

    SOFT GAIN: gain = approach_pct * min(1, (|need|/SOFT_ZONE_DEG)^2)
      |need| >= SOFT_ZONE_DEG  -> full kazanc, hizli yaklasim
      |need| == SOFT_ZONE/2    -> %25 kazanc
      |need| -> 0              -> kazanc 0, servo hareketsiz
    Boylece uzakta hizli, yakinda yumusak ve overshoot'suz.

    Donus:
      (uygulanacak_step_deg, ihtiyac_duyulan_deg)
    """
    needed_deg = err_px / px_per_deg
    rel        = abs(needed_deg) / SOFT_ZONE_DEG
    soft       = rel * rel if rel < 1.0 else 1.0
    step       = needed_deg * approach_pct * soft
    eff_max    = max_step * (rel if rel < 1.0 else 1.0)
    if step >  eff_max: step =  eff_max
    if step < -eff_max: step = -eff_max
    return step, needed_deg


# ============================================================
#  Threaded camera with auto-reconnect
# ============================================================
class CameraThread:
    def __init__(self, src=0, width=CAM_WIDTH, height=CAM_HEIGHT):
        self.src        = src
        self.width      = width
        self.height     = height
        self.cap        = None
        self._frame     = None
        self._frame_t   = None
        self._lock      = threading.Lock()
        self._running   = False
        self.connected  = False
        self.thread     = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._running = True
        self._open()
        self.thread.start()
        return self

    def _open(self):
        if self.cap is not None:
            try: self.cap.release()
            except Exception: pass
            self.cap = None
        cap = cv2.VideoCapture(self.src)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
            cap.set(cv2.CAP_PROP_FPS,          30)   # iste, surucu duser
            self.cap = cap
            self.connected = True
            fps_now = cap.get(cv2.CAP_PROP_FPS)
            print(f"[CAMERA] OPEN  src={self.src}  fps={fps_now:.0f}")
        else:
            self.cap = None
            self.connected = False

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
                    print("[CAMERA] LOST  - attempting reconnect")
                    self.connected = False
                    misses = 0
                time.sleep(0.02)
                continue
            misses = 0
            with self._lock:
                self._frame   = frame
                self._frame_t = time.monotonic()

    def read(self):
        with self._lock:
            if self._frame is None:
                return None, None
            return self._frame.copy(), self._frame_t

    def stop(self):
        self._running = False
        self.thread.join(timeout=1.0)
        if self.cap: self.cap.release()


# ============================================================
#  Moving-average filter
# ============================================================
class MovingAverage:
    def __init__(self, size=5):
        self.size = size
        self.buf  = deque(maxlen=size)

    def add(self, v):
        self.buf.append(v)
        return sum(self.buf) / len(self.buf)

    def reset(self):
        self.buf.clear()


# ============================================================
#  Detection
# ============================================================
def detect_face_largest(gray, cascade):
    # Tespiti DETECT_SCALE oranindaki kucultulmus frame'de yap, koordinatlari
    # geri olcekle. minSize de orantili kucultulmeli yoksa hic yuz bulunmaz.
    if DETECT_SCALE != 1.0:
        small = cv2.resize(gray, (0, 0),
                           fx=DETECT_SCALE, fy=DETECT_SCALE,
                           interpolation=cv2.INTER_AREA)
        min_s = max(8, int(40 * DETECT_SCALE))
    else:
        small = gray
        min_s = 40
    faces = cascade.detectMultiScale(
        small, scaleFactor=1.2, minNeighbors=5, minSize=(min_s, min_s))
    if len(faces) == 0:
        return None
    x, y, w, h = max(faces, key=lambda f: f[2] * f[3])
    if DETECT_SCALE != 1.0:
        inv = 1.0 / DETECT_SCALE
        x = int(x * inv); y = int(y * inv)
        w = int(w * inv); h = int(h * inv)
    return (x + w // 2, y + h // 2, x, y, w, h)


_MORPH_K = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

def detect_color_largest(frame, ranges):
    # Renk tespiti de kucultulmus frame'de yapilir; HSV donusumu + maskeleme
    # + kontur arama hepsi DETECT_SCALE * DETECT_SCALE kadar daha az pikselde
    # calisir. Sonuctaki bbox geri olceklenir.
    if DETECT_SCALE != 1.0:
        work = cv2.resize(frame, (0, 0),
                          fx=DETECT_SCALE, fy=DETECT_SCALE,
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
        m = cv2.inRange(hsv, np.array(lo, dtype=np.uint8),
                              np.array(hi, dtype=np.uint8))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  _MORPH_K)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _MORPH_K)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                         cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < min_area:
        return None
    if area > max_area:
        return None   # cerceveye dolan blob -> arka plan, hedef sayma
    x, y, w, h = cv2.boundingRect(c)
    if DETECT_SCALE != 1.0:
        inv = 1.0 / DETECT_SCALE
        x = int(x * inv); y = int(y * inv)
        w = int(w * inv); h = int(h * inv)
    return (x + w // 2, y + h // 2, x, y, w, h)


# ============================================================
#  Serial helpers (thread-safe: turret ana loop + WalkController paylasir)
# ============================================================
# Tek seri port iki yazicidan kullanildigi icin tum yazmalar lock altinda.
_serial_lock = threading.Lock()


def open_serial():
    """Try each candidate port. Returns (Serial-or-None, port-or-None)."""
    for port in SERIAL_PORT_CANDIDATES:
        if not os.path.exists(port):
            continue
        try:
            ser = serial.Serial(port, SERIAL_BAUD,
                                timeout=0.1, write_timeout=0.1)
            time.sleep(2.0)   # let the Arduino finish its reset
            print(f"[ARDUINO] OPEN  port={port}")
            return ser, port
        except Exception as e:
            print(f"[ARDUINO] FAIL  {port}: {e}")
    print("[ARDUINO] NO PORT - running in DRY-RUN (vision only)")
    return None, None


def safe_write(ser, line):
    """Returns the (possibly-replaced) serial handle. Thread-safe."""
    if ser is None:
        return None
    with _serial_lock:
        try:
            ser.write(line.encode('utf-8'))
            return ser
        except (serial.SerialException, OSError) as e:
            print(f"[ARDUINO] write error: {e} - reconnecting")
            try: ser.close()
            except Exception: pass
    # reconnect lock disinda (open_serial uzun sleep yapiyor)
    new, _ = open_serial()
    return new


# ============================================================
#  WalkController: arka plan thread'i, queue uzerinden komut alir,
#  trot_step'i seri port lock'u ile uygular. Kamera ana loop'u
#  ile ayni seri portu paylasir.
# ============================================================
class WalkController:
    """Quadruped walk thread. command() ile 'forward'/'back'/'left'/
    'right'/'hold'/'rotate180' kuyrugu birakir; thread sirayla isler."""

    # Sirali (wave) gait: 4 bacak tek tek havalanir; her swing'de diger 3
    # bacak yerde -> 3-bacak destek ucgeni, mumkun olan en kararli quadruped.
    # Sira capraz alternatif: FL -> BR -> FR -> BL (her ardisik swing
    # capraz tarafa gecer, govde sallanmaz).
    SWING_ORDER = ("FL", "BR", "FR", "BL")
    ALL_LEGS    = ("FL", "FR", "BL", "BR")

    def __init__(self, get_serial):
        # get_serial: ana thread'in current ser objesini dondurur (callable).
        # Reconnect olunca ana thread ser'i degistirir, lambda taze ser'i okur.
        self.get_serial = get_serial
        self.q          = queue.Queue(maxsize=4)
        self.running    = False
        self.busy       = False
        self.last_cmd   = "idle"
        self.thread     = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self.running = True
        self.thread.start()
        return self

    def stop(self):
        self.running = False
        try: self.q.put_nowait(None)
        except queue.Full: pass
        self.thread.join(timeout=2.0)

    def command(self, name):
        """Kuyrukta yer varsa komut ekle, doluysa dussur (input yutma)."""
        if not self.running:
            return
        try:
            self.q.put_nowait(name)
        except queue.Full:
            pass

    def _send(self, line):
        ser = self.get_serial()
        if ser is None:
            return
        with _serial_lock:
            try:
                ser.write((line + "\n").encode('utf-8'))
            except Exception:
                pass
        time.sleep(0.005)   # Arduino'ya nefes

    def _set_yaw(self, leg, offset):
        angle = WALK_YAW_HOME[leg] + WALK_YAW_DIR[leg] * offset
        angle = max(0, min(180, int(round(angle))))
        self._send(f"{leg}_Y {angle}")

    def _set_lift(self, leg, up):
        angle = (WALK_LIFT_HOME[leg] - WALK_LIFT_UP) if up else WALK_LIFT_HOME[leg]
        self._send(f"{leg}_L {angle}")

    def _hold(self):
        """Tum bacaklari Python tarafindaki WALK_YAW_HOME + WALK_LIFT_HOME
        pozisyonuna yaz. Arduino'nun 'hold' komutunu KULLANMIYORUZ cunku
        onun LIFT_HOME degerleri walk gait'in kullandigi WALK_LIFT_HOME
        ile birebir ayni degil (FL: 162 vs 170, ...). Eger Arduino hold
        cagrilirsa her adimin sonunda 4 lift jolt yer, robot pozu sıçrar,
        ayak yer temasi bozulur ve adim ilerlemesi kayıt edilmez.
        Burada Python'un kendi degerlerini yazariz; gait icindeki lift-down
        komutlari ile birebir ayni -> sicrama yok, traksiyon korunur."""
        for leg in self.ALL_LEGS:
            self._set_yaw(leg, 0)              # WALK_YAW_HOME[leg]
            self._set_lift(leg, up=False)      # WALK_LIFT_HOME[leg]

    def _swing_leg(self, leg, gain, A):
        """Tek bacagi havaya kaldir, +A'ya swing yap, yere indir.
        Bu sirada diger 3 bacak yerde durur -> tripod destek.
        3 mikro-faz: lift-up -> yaw-forward -> lift-down.
        Asimetrik timing: lift kisa (kucuk dikey hareket, yuksuz);
                          yaw daha uzun (yon icin servo'nun varmasi gerek)."""
        # 1) Bacagi havaya kaldir
        self._set_lift(leg, up=True)
        time.sleep(WALK_LIFT_DT)

        # 2) Bacagi ileri yaw (+A * gain) -> havada, yere takilmaz.
        # YAW_DT KRITIK: kisaltirsan servo +A'ya varmadan bir sonraki
        # komut gelir, gait yon farkini kaybeder.
        self._set_yaw(leg, +A * gain[leg])
        time.sleep(WALK_YAW_DT)

        # 3) Bacagi yere indir
        self._set_lift(leg, up=False)
        time.sleep(WALK_LIFT_DT)

    def _body_push_home(self):
        """4 bacak ESANLI yaw 0'a (home) doner. Hepsi yerde olduğundan ayak
        temas noktalari sabit, govde ileri kayar (body translation).
        Adim sonunda her bacak HOME yaw pozisyonunda -> state birikmiyor."""
        for leg in self.ALL_LEGS:
            self._set_yaw(leg, 0)
        time.sleep(WALK_PUSH_DT)

    def _trot_step(self, vx, wz):
        """Tek 'adim': 4 bacak SIRAYLA swing (FL->BR->FR->BL) + 1 body push.
        Toplam 4*3 + 1 = 13 mikro-faz. Her swing'de 3 bacak yerde (tripod).
            vx: ileri-geri  (-1..+1, + = ileri)
            wz: donus       (-1..+1, + = sola / CCW)
        Hareket karisigi (omega: vx + side * wz) her bacaga uygulanir.
        Adim sonu: hold cagrilir, tum bacaklar Arduino HOME'unda."""
        gain = {}
        for leg in WALK_YAW_HOME:
            g = vx + WALK_LEG_SIDE[leg] * wz
            gain[leg] = max(-1.0, min(1.0, g))

        A = WALK_SWING_AMP
        for leg in self.SWING_ORDER:             # sirali tek bacak swing
            self._swing_leg(leg, gain, A)
        self._body_push_home()                   # 4 bacak yerde, govde ileri

        # Her adim sonunda HOLD: tum servolari Arduino'nun per-bacak HOME'una
        # zorla yaz. Adim icinde birikmis kucuk servo hatasi / mekanik bosluk
        # her adimda sifirlanir; bacaklar gittikce bozulmaz.
        self._hold()
        time.sleep(WALK_HOLD_DT)                 # hold oturma suresi

    def _rotate_step(self, direction):
        """Saf yerinde donme - bir adim. direction: +1 = sola (CCW), -1 = saga (CW).

        Forward walk'tan farklari:
        1) Daha buyuk swing genligi (WALK_ROT_AMP): donmede 4 bacak karsilikli
           kuvvet uretiyor (sol back, sag forward), surtunmeler birbirini
           kilitleyebiliyor -> buyuk genlik gerekli.
        2) IKI ASAMALI BODY PUSH: tek seferde A*gain'den 0'a degil, once
           A*gain*0.5'a (yarim), sonra 0'a. Tek hamledeki tork kilitlenmesi
           yerine kademeli kayma -> her yarida sürtünme oturuyor, gövde
           gercekten doneyor.
        3) Translation drift'i azaltir: simetrik moment uretildigi icin
           net translation kuvveti yok, ama tek hamle pushda asimetrik
           surtunme drift uretebiliyor; yarim hamlelerde drift yarilanir.

        Cikti: govde direction'da temiz dönüş, x-y translation yok (ideal).
        """
        # Donme gain'i: sol bacaklar -dir, sag bacaklar +dir. Karsilikli
        # moment = saf rotasyon, net translation 0.
        gain = {leg: WALK_LEG_SIDE[leg] * direction for leg in WALK_LEG_SIDE}
        A = WALK_ROT_AMP

        # Sirali tek bacak swing - cross alternatif sira (FL, BR, FR, BL)
        # her swing'de 3 bacak yerde, tripod denge korunur.
        for leg in self.SWING_ORDER:
            self._swing_leg(leg, gain, A)

        # IKI ASAMALI BODY PUSH (kilitlenme onlemi):
        # Asama 1: hepsi A*gain*0.5'e (yari yola). Servo oturma + surtunme settle.
        for leg in self.ALL_LEGS:
            self._set_yaw(leg, +A * gain[leg] * 0.5)
        time.sleep(WALK_PUSH_DT)
        # Asama 2: hepsi 0'a (home). Govde donmeyi tamamlar.
        for leg in self.ALL_LEGS:
            self._set_yaw(leg, 0)
        time.sleep(WALK_PUSH_DT)

        # Hold reset - lift'ler de Python HOME'a, bir sonraki adim temiz
        self._hold()
        time.sleep(WALK_HOLD_DT)

    def _loop(self):
        while self.running:
            try:
                cmd = self.q.get(timeout=0.2)
            except queue.Empty:
                continue
            if cmd is None or not self.running:
                break

            self.busy     = True
            self.last_cmd = cmd
            try:
                if   cmd == "forward":   self._trot_step(+1,  0)
                elif cmd == "back":      self._trot_step(-1,  0)
                elif cmd == "left":      self._rotate_step(+1)   # CCW saf donme
                elif cmd == "right":     self._rotate_step(-1)   # CW saf donme
                elif cmd == "hold":      self._hold()
                elif cmd == "rotate180":
                    for _ in range(WALK_STEPS_180):
                        if not self.running:
                            break
                        self._rotate_step(+1)                    # her adim saf donme
                    self._hold()
            finally:
                self.busy     = False
                self.last_cmd = "idle"


def find_haar_cascade():
    for p in HAAR_CANDIDATES:
        if os.path.exists(p):
            print(f"[HAAR] using {p}")
            return p
    raise FileNotFoundError("haarcascade_frontalface_default.xml not found")


# ============================================================
#  Main loop
# ============================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode',  choices=['face', 'color'], default='face')
    ap.add_argument('--color', choices=list(HSV_PRESETS.keys()), default='red')
    ap.add_argument('--cam',   type=int, default=0,
                    help='OpenCV camera index (USB webcam = 0)')
    ap.add_argument('--log',   action='store_true',
                    help='write latency / accuracy log to CSV')
    args = ap.parse_args()

    cascade = cv2.CascadeClassifier(find_haar_cascade())
    if cascade.empty():
        raise RuntimeError("Haar cascade loaded empty - file corrupt?")

    ser, port = open_serial()
    cam = CameraThread(src=args.cam).start()
    time.sleep(1.0)   # camera warm-up

    # Kamera timeout: bu kadar saniye icinde ilk frame gelmezse vazgec.
    # Aksi takdirde tum loop sonsuza dek "WAITING FOR CAMERA" ekraninda
    # asili kalir. Hata mesaji + temiz cikis daha kullanilabilir.
    cam_wait_t0 = time.monotonic()
    while True:
        _f, _t = cam.read()
        if _f is not None:
            break
        if (time.monotonic() - cam_wait_t0) > CAM_OPEN_TIMEOUT_S:
            print(f"[CAMERA] TIMEOUT  ilk frame {CAM_OPEN_TIMEOUT_S:.0f} sn "
                  f"icinde gelmedi -- cikis.")
            cam.stop()
            if ser is not None:
                try: ser.close()
                except Exception: pass
            return
        time.sleep(0.1)

    # Walk thread: ana thread "ser"i rebind ederse (reconnect) lambda
    # taze deger okur; serial_lock ile turret yazimi ile cakismaz.
    walker = WalkController(get_serial=lambda: ser).start()

    filt_x   = MovingAverage(FILTER_WINDOW)
    filt_y   = MovingAverage(FILTER_WINDOW)

    pan          = float(PAN_INIT)
    tilt         = float(TILT_INIT)
    mode         = args.mode
    color        = args.color
    lock_tol     = LOCK_TOL
    approach_pct = APPROACH_PCT   # runtime: ',' / '.' tuslari ayarlar
    max_step     = MAX_STEP_DEG   # runtime: ';' / "'" tuslari ayarlar
    prev_frame_t = None           # ayni kareyi tekrar islemeyi engeller
    target       = None           # son tespit (ekranda kalsin diye saklanir)
    last_step    = (0.0, 0.0)     # son uygulanan (pan, tilt) adimi
    last_need    = (0.0, 0.0)     # son ihtiyac (pan, tilt)
    lock_streak  = 0              # ardisik "merkezde + tespit var" kare sayisi
    bad_count    = 0              # ardisik kotu (miss VEYA out-of-tol) kare sayisi
                                  # LOCK_BAD_TOLERANCE altinda kilit korunur
    last_fine_t  = None           # locked ic-kare yavas hizalamasinda dt icin
                                  # son frame zaman damgasi (None -> reset)
    fine_locked_pan  = False      # ic kare LATCH (pan): bagimsiz eksen latch
    fine_locked_tilt = False      # ic kare LATCH (tilt): bagimsiz eksen latch
                                  # Her eksen kendi iç bandına girince donar;
                                  # diger eksenin durumundan etkilenmez.
                                  # GREEN gosterimi: ikisi de True iken.

    # Komut hysteresis: ayni acida tekrar tekrar komut gonderme.
    # YENI PROTOKOL: pan/tilt derecenin onda biri olarak gonderilir
    # (90.0 -> 900). 0.1 derece resolution. Arduino writeMicroseconds.
    last_sent_pan   = int(round(pan  * 10))
    last_sent_tilt  = int(round(tilt * 10))
    last_sent_laser = 0

    # Tikla-renk-ogret: sol tik ile o pikselin HSV'sini orneklenir.
    # Mouse callback baska bir thread'den firlatildigi icin sadece
    # koordinati birakir; orneklem ana dongude yapilir.
    click_target = {'pos': None}
    def _on_mouse(event, x, y, flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            click_target['pos'] = (x, y)
    cv2.namedWindow("Avci Modu")
    cv2.setMouseCallback("Avci Modu", _on_mouse)

    csv_file = csv_writer = None
    if args.log:
        csv_file = open(LATENCY_LOG, 'w', newline='')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(['t_ms', 'mode', 'detected',
                             'target_x', 'target_y',
                             'pan', 'tilt', 'laser', 'lat_ms'])

    print("=" * 60)
    print("  ROBOT (turret + walk) ready.")
    print("  TURRET : ESC=quit  m=mode toggle  f=face  c=color")
    print("           r/g/b/y=color  h=turret home")
    print("           [/]=tol  ,/.=approach%  ;/'=max-step")
    print("  WALK   : w=ileri  s=geri  a=sola  d=saga")
    print("           x=HOLD (dur+home)  q=180 donus")
    print("  MOUSE  : left-click -> sample colour as 'custom'")
    print(f"  mode={mode}  color={color}  port={port or 'DRY-RUN'}")
    print(f"  approach={int(approach_pct*100)}%  max-step={max_step} deg/frame"
          f"  lock-frames={LOCK_STABILITY_FRAMES}")
    print("=" * 60)

    last_send_t   = 0.0
    last_detect_t = time.monotonic()
    fps_t0, fps_count, fps = time.monotonic(), 0, 0.0
    last_lat_ms = 0.0
    blank = np.zeros((CAM_HEIGHT, CAM_WIDTH, 3), dtype=np.uint8)

    try:
        while True:
            frame, frame_t = cam.read()

            if frame is None:
                disp = blank.copy()
                cv2.putText(disp, "WAITING FOR CAMERA...",
                            (60, FRAME_CY),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                cv2.imshow("Avci Modu", disp)
                if (cv2.waitKey(40) & 0xFF) == ord('q'):
                    break
                continue

            # ----- Mouse tikla-renk-ogret -----
            if click_target['pos'] is not None:
                cx_c, cy_c = click_target['pos']
                click_target['pos'] = None
                if 0 <= cx_c < CAM_WIDTH and 0 <= cy_c < CAM_HEIGHT:
                    x0 = max(0, cx_c - 3); x1 = min(CAM_WIDTH,  cx_c + 4)
                    y0 = max(0, cy_c - 3); y1 = min(CAM_HEIGHT, cy_c + 4)
                    patch    = frame[y0:y1, x0:x1]
                    hsv_pa   = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
                    h_med    = int(np.median(hsv_pa[:, :, 0]))
                    s_med    = int(np.median(hsv_pa[:, :, 1]))
                    v_med    = int(np.median(hsv_pa[:, :, 2]))
                    sat_lo   = max(40, s_med - 70)
                    val_lo   = max(40, v_med - 70)
                    if h_med < 15 or h_med > 165:
                        # Kirmizi: hue circle wrap -> iki aralik
                        HSV_PRESETS['custom'] = [
                            ((0,   sat_lo, val_lo), (15,  255, 255)),
                            ((165, sat_lo, val_lo), (180, 255, 255)),
                        ]
                    else:
                        HSV_PRESETS['custom'] = [
                            ((max(0,   h_med - 12), sat_lo, val_lo),
                             (min(180, h_med + 12), 255, 255))
                        ]
                    color, mode = 'custom', 'color'
                    filt_x.reset(); filt_y.reset()
                    target, lock_streak = None, 0
                    print(f"[CLICK] HSV=({h_med:3d},{s_med:3d},{v_med:3d}) "
                          f"-> 'custom' (mode=color)")

            # ----- Yeni kare mi? -----
            # CameraThread baska bir thread'de uretiyor; ayni kareyi tekrar
            # islememek icin frame_t timestamp'i ile karsilastir. Boylece
            # control adimi yalniz YENI olcumlerle calisir; servolar
            # "kor sayim" yapmaz.
            is_new_frame = (frame_t != prev_frame_t)
            if is_new_frame:
                prev_frame_t = frame_t

                # Detect
                if mode == 'face':
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    target = detect_face_largest(gray, cascade)
                else:
                    target = detect_color_largest(frame, HSV_PRESETS[color])

            laser = 0

            # crosshair + deadband kutusu (her karede ciz).
            # KILITLIYKEN gen"s HOLD kutusunu goster (kilit kirilma sinirin),
            # KILITSIZ iken dar ACQUIRE kutusunu goster (kilit edinme sinir).
            _is_locked_now = (lock_streak >= LOCK_STABILITY_FRAMES)
            _box_tol = int(lock_tol * LOCK_HOLD_MULT) if _is_locked_now else lock_tol
            _box_col = (0, 0, 255) if _is_locked_now else (255, 255, 255)
            cv2.rectangle(frame,
                          (FRAME_CX - _box_tol, FRAME_CY - _box_tol),
                          (FRAME_CX + _box_tol, FRAME_CY + _box_tol),
                          _box_col, 1)
            # Ic kare: dis kutuda LOCKED iken hedefi cok yavas getirecegimiz
            # hassas merkez. Iki eksen de latch = YESIL (tam donmus),
            # locked iken sari, kilitsizken gri.
            if fine_locked_pan and fine_locked_tilt:
                _fine_col = (0, 255, 0)
            elif _is_locked_now:
                _fine_col = (0, 200, 255)
            else:
                _fine_col = (120, 120, 120)
            cv2.rectangle(frame,
                          (FRAME_CX - FINE_LOCK_TOL, FRAME_CY - FINE_LOCK_TOL),
                          (FRAME_CX + FINE_LOCK_TOL, FRAME_CY + FINE_LOCK_TOL),
                          _fine_col, 1)
            cv2.line(frame, (FRAME_CX, 0), (FRAME_CX, CAM_HEIGHT),
                     (90, 90, 90), 1)
            cv2.line(frame, (0, FRAME_CY), (CAM_WIDTH, FRAME_CY),
                     (90, 90, 90), 1)

            if target is not None:
                cx, cy, x, y, w, h = target

                # Hedef varken her zaman taze cizimi goster (last bilinen kutu)
                box_col = (0, 255, 0) if mode == 'face' else (255, 128, 0)
                cv2.rectangle(frame, (x, y), (x + w, y + h), box_col, 2)

                if is_new_frame:
                    # ---- YENI olcum: kontrol adimini SADECE simdi at ----
                    last_detect_t = time.monotonic()

                    fx = filt_x.add(cx)
                    fy = filt_y.add(cy)

                    # Pozitif hata => hedef sol/yukari taraf
                    err_pan  = FRAME_CX - fx
                    err_tilt = FRAME_CY - fy

                    already_locked = (lock_streak >= LOCK_STABILITY_FRAMES)
                    # Hysteresis: kilitliysek genis HOLD bolgesi kullan,
                    # degilsek dar ACQUIRE bolgesi. Boylece kilit bir kere
                    # alindiginda, kucuk salinimlar (1deg servo komutu ~10px
                    # + backlash) kilidi kirip durmaz.
                    hold_tol = int(lock_tol * LOCK_HOLD_MULT)
                    active_tol = hold_tol if already_locked else lock_tol
                    in_tol = (abs(err_pan)  < active_tol and
                              abs(err_tilt) < active_tol)

                    if already_locked and in_tol:
                        # KILITLI: lazer dis kutuda yaniyor. Ic kareye
                        # "Step-and-Wait" (vur ve bekle) ile yaklas:
                        #   - FINE_WAIT_SEC bekle (kamera oturmasi),
                        #   - sonra tek NET adim at (FINE_STEP_DEG = 0.5 deg),
                        #   - servo deadband uzerinde oldugu icin GERCEK hareket
                        # Boylece "tik... tik... tik..." ritimli, hunting yok.
                        need_pan  = err_pan  / PX_PER_DEG_X
                        need_tilt = err_tilt / PX_PER_DEG_Y

                        # ----- IC KARE HYSTERESIS (LATCH) - PER AXIS -----
                        # Her eksen BAGIMSIZ. Pan ic banda girince donar,
                        # tilt ic banda girince ayri donar. Birinin sinirinda
                        # salinim digerini etkilemez.
                        # Giris: dar FINE_LOCK_TOL (12 px) -> latch=True
                        # Cikis: genis RELEASE bandi (30 px) -> latch=False
                        fine_release_tol = FINE_LOCK_TOL * FINE_RELEASE_MULT

                        # Pan ekseni latch
                        if fine_locked_pan:
                            if abs(err_pan) > fine_release_tol:
                                fine_locked_pan = False
                        else:
                            if abs(err_pan) < FINE_LOCK_TOL:
                                fine_locked_pan = True

                        # Tilt ekseni latch (ayni mantik, bagimsiz)
                        if fine_locked_tilt:
                            if abs(err_tilt) > fine_release_tol:
                                fine_locked_tilt = False
                        else:
                            if abs(err_tilt) < FINE_LOCK_TOL:
                                fine_locked_tilt = True

                        if fine_locked_pan and fine_locked_tilt:
                            # Iki eksen de LATCH: tamamen dur. Mikro-titreme yok.
                            step_pan, step_tilt = 0.0, 0.0
                            last_fine_t = frame_t
                        else:
                            # En az bir eksen hala hizalanmali
                            if last_fine_t is None:
                                last_fine_t = frame_t - 1.0
                            dt_fine = frame_t - last_fine_t

                            if dt_fine > FINE_WAIT_SEC:
                                # Bekleme bitti: SADECE latch'lenmemis eksenler
                                # adim atar. Latch olan eksen 0 step alir.
                                sp = 0.0
                                if (not fine_locked_pan and
                                        abs(err_pan) >= FINE_LOCK_TOL):
                                    sp = FINE_STEP_DEG if need_pan > 0 else -FINE_STEP_DEG
                                st = 0.0
                                if (not fine_locked_tilt and
                                        abs(err_tilt) >= FINE_LOCK_TOL):
                                    st = FINE_STEP_DEG if need_tilt > 0 else -FINE_STEP_DEG
                                step_pan, step_tilt = sp, st
                                pan  += step_pan
                                tilt += step_tilt
                                pan   = max(PAN_MIN,  min(PAN_MAX,  pan))
                                tilt  = max(TILT_MIN, min(TILT_MAX, tilt))
                                last_fine_t = frame_t
                            else:
                                # Bekleme suresi dolmadi: hareket etme,
                                # kameranin yeni pozisyonu algilamasini bekle.
                                step_pan, step_tilt = 0.0, 0.0
                    else:
                        # Duz P-kontrol: ihtiyacin yuzdesi, max_step ile kirpilir.
                        # Velocity smoothing YOK -- her kare bagimsiz, ongorulebilir.
                        # Cok kucuk adimlar bile float pan/tilt'e birikir; servo
                        # int sinirini gecince doner. Boylece "yavas ama mutlaka
                        # hedefe gelir" davranisi cikar.
                        last_fine_t = None   # ince hizalama modundan cik
                        fine_locked_pan  = False
                        fine_locked_tilt = False
                        step_pan,  need_pan  = compute_step(
                            err_pan,  PX_PER_DEG_X, approach_pct, max_step)
                        step_tilt, need_tilt = compute_step(
                            err_tilt, PX_PER_DEG_Y, approach_pct, max_step)

                        pan  += step_pan
                        tilt += step_tilt
                        pan   = max(PAN_MIN,  min(PAN_MAX,  pan))
                        tilt  = max(TILT_MIN, min(TILT_MAX, tilt))

                    last_step = (step_pan,  step_tilt)
                    last_need = (need_pan, need_tilt)

                    # Lock streak / bad_count guncellemesi YALNIZ yeni karede.
                    # in_tol: iyi kare -> streak arttir, bad sayaci sifirla.
                    # out_of_tol: kotu kare. Ama LOCK_BAD_TOLERANCE'a kadar
                    # streak'i kirma (kenar gurultusu kilit edinimini bozmasin).
                    if in_tol:
                        lock_streak = min(LOCK_STABILITY_FRAMES,
                                          lock_streak + 1)
                        bad_count = 0
                    else:
                        bad_count += 1
                        if bad_count > LOCK_BAD_TOLERANCE:
                            lock_streak = 0
                            fine_locked_pan  = False
                            fine_locked_tilt = False
                else:
                    # Yeni kare yok: degerleri tasi ama yeni adim atma
                    fx = filt_x.buf[-1] if filt_x.buf else FRAME_CX
                    fy = filt_y.buf[-1] if filt_y.buf else FRAME_CY
                    err_pan  = FRAME_CX - fx
                    err_tilt = FRAME_CY - fy

                # Lazer mantigi asagida (target is None durumunu da kapsiyor),
                # burada artik atama YAPMA -- aksi takdirde bad_count
                # toleransinda iken laser bir karede 1 bir karede 0 olur.

                # ----- Overlay: hedef-merkez vektoru ve hesap detaylari -----
                cv2.line(frame, (FRAME_CX, FRAME_CY), (int(fx), int(fy)),
                         (0, 255, 255), 2)
                cv2.circle(frame, (int(fx), int(fy)), 5, (0, 0, 255), -1)

                need_pan, need_tilt = last_need
                step_pan, step_tilt = last_step
                # Kalan derece: ihtiyac - uygulanan (1 kare gecikmesi ile)
                rem_pan  = need_pan  - step_pan
                rem_tilt = need_tilt - step_tilt

                info_lines = [
                    (f"NEED  pan={need_pan:+6.2f}  tilt={need_tilt:+6.2f}  deg",
                     (0, 255, 255)),
                    (f"STEP  pan={step_pan:+6.2f}  tilt={step_tilt:+6.2f}  deg"
                     f"  ({int(approach_pct*100)}%)",
                     (0, 200, 255)),
                    (f"REM   pan={rem_pan:+6.2f}  tilt={rem_tilt:+6.2f}  deg",
                     (180, 180, 180)),
                ]
                for i, (txt, col) in enumerate(info_lines):
                    cv2.putText(frame, txt, (20, 50 + i * 24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 1,
                                cv2.LINE_AA)

            else:
                # Hedef yok: bad_count'u arttir. LOCK_BAD_TOLERANCE'a kadar
                # lock_streak yasamaya devam eder, lazer akmaya devam.
                # Bu sayede Haar'in tek/birkac kare iskasi lazeri sondurmez.
                if is_new_frame:
                    bad_count += 1
                    if bad_count > LOCK_BAD_TOLERANCE:
                        # Persistent kayip: artik kilidi tamamen birak
                        filt_x.reset()
                        filt_y.reset()
                        last_step   = (0.0, 0.0)
                        last_need   = (0.0, 0.0)
                        lock_streak = 0
                        fine_locked_pan  = False
                        fine_locked_tilt = False
                # else: kisa miss, lock_streak korunur -> laser asagida hala 1

            # ----- LAZER: kilit varsa VE kotu kare toleransi asilmadiysa yan.
            # Boylece Haar iskasi / kenar gurultusu lazeri sondurmez.
            is_locked_logical = (lock_streak >= LOCK_STABILITY_FRAMES and
                                 bad_count <= LOCK_BAD_TOLERANCE)
            if is_locked_logical:
                laser = 1

            # ----- Buyuk durum banneri (ust ortada) -----
            if target is None and bad_count > LOCK_BAD_TOLERANCE:
                status_text, status_col = "SEARCHING", (180, 180, 180)
            elif is_locked_logical:
                status_text, status_col = "LOCKED",    (0, 0, 255)
            elif target is None or bad_count > 0:
                # Kisa kayip veya kenar oynama: hala kilit hold mode
                status_text, status_col = "HOLD-LOCK", (0, 165, 255)
            else:
                status_text, status_col = "TRACKING",  (0, 255, 255)
            (tw, _), _ = cv2.getTextSize(
                status_text, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 3)
            cv2.putText(frame, status_text,
                        (FRAME_CX - tw // 2, 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, status_col, 3,
                        cv2.LINE_AA)
            # Lock confidence progress bar (sadece TRACKING/LOCKED'de)
            if target is not None:
                bar_w = 120
                fill  = int(bar_w * lock_streak / LOCK_STABILITY_FRAMES)
                cv2.rectangle(frame,
                              (FRAME_CX - bar_w // 2, 42),
                              (FRAME_CX + bar_w // 2, 50),
                              (80, 80, 80), 1)
                cv2.rectangle(frame,
                              (FRAME_CX - bar_w // 2, 42),
                              (FRAME_CX - bar_w // 2 + fill, 50),
                              status_col, -1)

            # ----- Komut gonderimi: state degisti VEYA heartbeat zamani -----
            # Format: "PAN_X10,TILT_X10,LASER\n"  (90.0 derece -> 900)
            # Hysteresis 0.1 derece cozunurlukte: ayni x10 deger tekrar
            # gonderilmez. Boylece sub-derece dengeleme mumkun, servo
            # micro-titreme yok.
            now = time.monotonic()
            sent = False
            new_pan_x10  = int(round(pan  * 10))
            new_tilt_x10 = int(round(tilt * 10))
            state_changed = (new_pan_x10  != last_sent_pan   or
                             new_tilt_x10 != last_sent_tilt  or
                             laser        != last_sent_laser)
            heartbeat_due = (now - last_send_t) >= HEARTBEAT_S
            if state_changed or heartbeat_due:
                cmd = f"{new_pan_x10},{new_tilt_x10},{laser}\n"
                ser = safe_write(ser, cmd)
                last_send_t     = now
                last_sent_pan   = new_pan_x10
                last_sent_tilt  = new_tilt_x10
                last_sent_laser = laser
                sent = True
                last_lat_ms = (now - frame_t) * 1000.0 if frame_t else 0.0

            # ----- FPS -----
            fps_count += 1
            if fps_count >= 30:
                tnow = time.monotonic()
                fps = fps_count / (tnow - fps_t0)
                fps_t0, fps_count = tnow, 0

            # ----- HUD overlay -----
            walk_busy = "[BUSY]" if walker.busy else "      "
            hud = [
                f"MODE: {mode.upper()}"
                + (f" ({color})" if mode == 'color' else ""),
                f"PAN: {pan:5.1f}  TILT: {tilt:5.1f}  LASER: {laser}",
                f"WALK: {walker.last_cmd.upper():<9} {walk_busy}",
                f"APPROACH: {int(approach_pct*100):2d}%   MAX/step: {max_step:.1f}deg",
                f"FPS: {fps:4.1f}  LAT: {last_lat_ms:5.1f} ms",
                f"PORT: {port or 'DRY-RUN'}"
                f"   CAM: {'OK' if cam.connected else 'LOST'}"
                f"   TOL: {lock_tol}px",
            ]
            for i, line in enumerate(hud):
                cv2.putText(frame, line,
                            (10, CAM_HEIGHT - 10 - (len(hud) - 1 - i) * 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (50, 255, 50), 1, cv2.LINE_AA)

            # ----- Latency log -----
            if csv_writer and sent:
                csv_writer.writerow([
                    int(time.monotonic() * 1000),
                    mode,
                    1 if target else 0,
                    int(target[0]) if target else -1,
                    int(target[1]) if target else -1,
                    int(round(pan)), int(round(tilt)), laser,
                    f"{last_lat_ms:.2f}",
                ])

            cv2.imshow("Avci Modu", frame)

            # ----- Keys -----
            key = cv2.waitKey(1) & 0xFF
            if key == 0xFF:
                continue
            # ESC = quit (q artik walk 180-donus icin)
            if   key == 27: break
            # ----- Walk komutlari (WalkController kuyruguna birakilir) -----
            elif key == ord('w'):
                walker.command("forward");   print("[WALK] ileri")
            elif key == ord('s'):
                walker.command("back");      print("[WALK] geri")
            elif key == ord('a'):
                walker.command("left");      print("[WALK] sola donus")
            elif key == ord('d'):
                walker.command("right");     print("[WALK] saga donus")
            elif key == ord('x'):
                walker.command("hold");      print("[WALK] HOLD (dur+home)")
            elif key == ord('q'):
                walker.command("rotate180"); print("[WALK] 180 donus basliyor")
            # ----- Turret komutlari -----
            elif key == ord('m'):
                mode = 'color' if mode == 'face' else 'face'
                filt_x.reset();  filt_y.reset()
                target = None
                last_step = (0.0, 0.0); last_need = (0.0, 0.0)
                print(f"[MODE] -> {mode}")
            elif key == ord('f'): mode = 'face';  print("[MODE] -> face")
            elif key == ord('c'): mode = 'color'; print("[MODE] -> color")
            elif key in (ord('r'),): color = 'red';    print("[COLOR] -> red")
            elif key in (ord('g'),): color = 'green';  print("[COLOR] -> green")
            elif key in (ord('b'),): color = 'blue';   print("[COLOR] -> blue")
            elif key in (ord('y'),): color = 'yellow'; print("[COLOR] -> yellow")
            elif key == ord('['):
                lock_tol = max(5, lock_tol - 5);  print(f"[TOL] {lock_tol}")
            elif key == ord(']'):
                lock_tol = min(150, lock_tol + 5); print(f"[TOL] {lock_tol}")
            elif key == ord(','):
                approach_pct = max(0.02, approach_pct - 0.02)
                print(f"[APPROACH] {approach_pct*100:.0f}%")
            elif key == ord('.'):
                approach_pct = min(1.00, approach_pct + 0.02)
                print(f"[APPROACH] {approach_pct*100:.0f}%")
            elif key == ord(';'):
                max_step = max(0.5, max_step - 0.5)
                print(f"[MAX_STEP] {max_step:.1f} deg/frame")
            elif key == ord("'"):
                max_step = min(15.0, max_step + 0.5)
                print(f"[MAX_STEP] {max_step:.1f} deg/frame")
            elif key == ord('h'):
                # acil "merkeze don" - kalibrasyon icin
                pan, tilt = float(PAN_INIT), float(TILT_INIT)
                last_step = (0.0, 0.0); last_need = (0.0, 0.0)
                print("[HOME] servolar merkeze")

    finally:
        # ----- Safe shutdown -----
        print("[SHUTDOWN] walker stop, centering servos, laser off, legs HOLD")
        try:
            walker.stop()
        except Exception:
            pass
        if ser is not None:
            try:
                with _serial_lock:
                    # Once turret merkeze + lazer off (x10 protokol: 90.0 -> 900)
                    ser.write(b"900,900,0\n")
                    ser.flush()
                    time.sleep(0.1)
                    # Sonra bacaklar per-leg HOME
                    ser.write(b"hold\n")
                    ser.flush()
                    time.sleep(0.3)
                ser.close()
            except Exception:
                pass
        cam.stop()
        cv2.destroyAllWindows()
        if csv_file:
            csv_file.close()
            print(f"[LOG] wrote {LATENCY_LOG}")


if __name__ == '__main__':
    main()
