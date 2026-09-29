"""
takip_denemesi.py -- Hardware tracking trial 
(hardware tracking performance, processed frame rate, law comparison).

Runs ONE control law on the real robot against a target shown on a monitor by
hedef_ekrani.py, logs every processed frame, and saves every Nth frame as a JPEG
so the detector can be checked against hand-annotated ground truth
(isaretle.py). The robot stays STATIONARY (legs holding) for every trial.

Laws (same constants as the manuscript, nominal kappa 10.67 px/deg, so the
paper's law is tested exactly as published):
    PID   discrete PID            (controllers.PIDController)
    FLAT  flat proportional gain  (defined below, Kflat = 0.3)
    SOFT  soft-zone P + step-and-wait + latch (controllers.SoftZoneController)

Conditions (must match the hedef_ekrani.py mode running on the monitor):
    S1  static target, started ~120 px off centre      -> hedef_ekrani --mod sabit
    S2  sinusoidal motion                              -> hedef_ekrani --mod sinus
    S3  constant-speed sweep, speed rising each pass   -> hedef_ekrani --mod tarama
    J   target jumps (for the high-speed-video latency test) -> --mod sicrama

USAGE (on the Pi, from this folder)
    python3 takip_denemesi.py --law SOFT --cond S1 --run 1 --port /dev/ttyACM0
    python3 takip_denemesi.py --law PID  --cond S2 --run 1 --freq 0.2

Output: veri_takip/<law>_<cond>_<freq>_run<k>.csv  and  veri_takip/kareler/...
The end-of-trial summary is printed AND appended to veri_takip/ozet.csv.
"""

import argparse
import csv
import os
import time

import cv2
import numpy as np

import rig
from controllers import PIDController, SoftZoneController, MAX_STEP_DEG

KAPPA = 10.67            # nominal, exactly as in the manuscript (Table 2)


class FlatController:
    """Flat proportional gain applied on every frame (ablation baseline)."""

    def __init__(self, k=0.3, px_per_deg=KAPPA):
        self.k, self.px_per_deg = k, px_per_deg

    def reset(self):
        pass

    def compute_step(self, ex, ey, dex=0, dey=0, phase_feat=None, dt=1 / 30.0):
        f = lambda e: float(np.clip(self.k * e / self.px_per_deg,
                                    -MAX_STEP_DEG, MAX_STEP_DEG))
        return f(ex), f(ey)


def make_law(name):
    if name == 'PID':
        return PIDController(px_per_deg=KAPPA)
    if name == 'SOFT':
        return SoftZoneController(px_per_deg=KAPPA)
    if name == 'FLAT':
        return FlatController()
    raise ValueError(name)


FIELDS = ['t_s', 'frame_t_s', 'frame_idx', 'detected', 'cx', 'cy',
          'err_x', 'err_y', 'cmd_pan', 'cmd_tilt', 'pan', 'tilt',
          'lat_ms', 'proc_ms', 'saved_jpg']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--law', required=True, choices=['PID', 'FLAT', 'SOFT'])
    ap.add_argument('--cond', required=True, choices=['S1', 'S2', 'S3', 'J'])
    ap.add_argument('--run', type=int, default=1)
    ap.add_argument('--freq', type=float, default=0.0,
                    help='S2 only: the frequency set on hedef_ekrani (Hz), for the file name')
    ap.add_argument('--duration', type=float, default=None)
    ap.add_argument('--color', default='red', choices=list(rig.HSV_PRESETS))
    ap.add_argument('--port', default=None)
    ap.add_argument('--cam', type=int, default=0)
    ap.add_argument('--save-every', type=int, default=15,
                    help='save every Nth frame as JPEG for annotation (0 = off)')
    ap.add_argument('--outdir', default='veri_takip')
    a = ap.parse_args()

    duration = a.duration or {'S1': 15.0, 'S2': 40.0, 'S3': 60.0, 'J': 60.0}[a.cond]
    tag = f"{a.law}_{a.cond}" + (f"_{a.freq:g}Hz" if a.cond == 'S2' else '') + f"_run{a.run}"
    os.makedirs(os.path.join(a.outdir, 'kareler', tag), exist_ok=True)

    ranges = rig.HSV_PRESETS[a.color]
    cam = rig.CameraThread(src=a.cam).start()
    time.sleep(2.0)
    if not cam.connected:
        print("[TAKIP] kamera acilmadi"); return
    ser, _ = rig.open_serial(a.port)
    ser = rig.hold_legs(ser)                        # legs hold, robot stationary
    rig.park_head(ser)

    ctrl = make_law(a.law)
    ctrl.reset()
    pan, tilt = rig.PAN_INIT, rig.TILT_INIT
    fx, fy = rig.MovingAverage(5), rig.MovingAverage(5)
    prev_ft, prev_ex, prev_ey = None, 0.0, 0.0
    last_t = time.monotonic()

    f = open(os.path.join(a.outdir, tag + '.csv'), 'w', newline='')
    w = csv.writer(f); w.writerow(FIELDS)
    print(f"[TAKIP] {tag}: {duration:.0f} s -- hedef_ekrani'nda ayni kosul acik olmali")
    time.sleep(1.0)

    t0 = time.monotonic()
    k = 0
    errs, cmds, n_det = [], [], 0
    while time.monotonic() - t0 < duration:
        frame, ft = cam.read()
        if frame is None or ft == prev_ft:
            time.sleep(0.002); continue
        prev_ft = ft
        k += 1
        now = time.monotonic()
        dt = min(max(now - last_t, 1e-3), 0.2); last_t = now

        p0 = time.perf_counter()
        det = rig.detect_color_largest(frame, ranges)
        sp = st = 0.0
        ex = ey = float('nan')
        if det is not None:
            n_det += 1
            cx, cy = fx.add(det[0]), fy.add(det[1])
            ex, ey = float(rig.FRAME_CX - cx), float(rig.FRAME_CY - cy)
            sp, st = ctrl.compute_step(ex, ey, ex - prev_ex, ey - prev_ey, None, dt)
            prev_ex, prev_ey = ex, ey
            pan = float(np.clip(pan + sp, rig.PAN_MIN, rig.PAN_MAX))
            tilt = float(np.clip(tilt + st, rig.TILT_MIN, rig.TILT_MAX))
            ser = rig.send_turret(ser, pan, tilt, 0)
            errs.append((ex, ey)); cmds.append((sp, st))
        proc_ms = (time.perf_counter() - p0) * 1000.0
        lat_ms = (time.monotonic() - ft) * 1000.0

        jpg = ''
        if a.save_every and k % a.save_every == 0:
            jpg = os.path.join('kareler', tag, f"{k:06d}.jpg")
            cv2.imwrite(os.path.join(a.outdir, jpg), frame,
                        [cv2.IMWRITE_JPEG_QUALITY, 92])
        w.writerow([f"{now - t0:.4f}", f"{ft:.4f}", k, int(det is not None),
                    det[0] if det else -1, det[1] if det else -1,
                    f"{ex:.2f}", f"{ey:.2f}", f"{sp:.3f}", f"{st:.3f}",
                    f"{pan:.2f}", f"{tilt:.2f}", f"{lat_ms:.2f}",
                    f"{proc_ms:.2f}", jpg])

    f.close()
    rig.park_head(ser, settle=0.5)
    cam.stop()

    # ---- quick summary (the full analysis is analiz_takip.py) ----
    fps = k / duration
    e = np.asarray(errs) if errs else np.zeros((0, 2))
    c = np.asarray(cmds) if cmds else np.zeros((0, 2))
    en = np.linalg.norm(e, axis=1) if len(e) else np.array([np.nan])
    summ = dict(tag=tag, fps=round(fps, 2),
                det_rate=round(100 * n_det / max(k, 1), 1),
                rms_px=round(float(np.sqrt(np.mean(en ** 2))), 2),
                p95_px=round(float(np.percentile(en, 95)), 2),
                in12=round(100 * float(np.mean(en < 12)), 1),
                in32=round(100 * float(np.mean(en < 32)), 1),
                duty=round(100 * float(np.mean(np.any(np.abs(c) > 1e-6, axis=1)))
                           if len(c) else 0, 1))
    print("[TAKIP] ozet:", summ)
    op = os.path.join(a.outdir, 'ozet.csv')
    new = not os.path.exists(op)
    with open(op, 'a', newline='') as g:
        ww = csv.DictWriter(g, fieldnames=list(summ))
        if new:
            ww.writeheader()
        ww.writerow(summ)


if __name__ == '__main__':
    main()
