"""
hedef_ekrani.py -- Repeatable moving red target on a laptop/monitor screen.

Runs on ANY computer with Python 3 (tkinter is in the standard library). Put
the screen in front of the robot, full screen, and give the robot-to-screen
distance and the visible screen width so the motion is set in DEGREES as seen
from the camera (that is what the paper reports).

MODES
    sabit    disc shown ~11 deg left of centre, so the robot starts ~120 px off
             (S1: step / static hold)
    sinus    horizontal sine, --genlik deg amplitude, --frekans Hz      (S2)
    tarama   constant-speed back-and-forth sweep; speed rises by --adim deg/s
             every full pass, starting at --hiz deg/s                    (S3)
    sicrama  disc jumps between two positions every 2-4 s, and a white square
             flashes in the lower-left corner at the same instant -- film the
             screen AND the servo horn together at 240 fps              (J)

EXAMPLES
    python hedef_ekrani.py --mod sinus --genlik 8 --frekans 0.2 --mesafe-cm 100 --ekran-cm 34.5
    python hedef_ekrani.py --mod tarama --hiz 5 --adim 5 --mesafe-cm 100 --ekran-cm 34.5

Every screen update is logged to hedef_log_<mod>.csv (time, x_px, deg) so the
exact motion used in each trial is on record. Esc quits.
"""

import argparse
import csv
import math
import random
import time
import tkinter as tk


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mod', required=True, choices=['sabit', 'sinus', 'tarama', 'sicrama'])
    ap.add_argument('--mesafe-cm', type=float, required=True, help='camera-to-screen distance')
    ap.add_argument('--ekran-cm', type=float, required=True, help='visible screen width')
    ap.add_argument('--cap-cm', type=float, default=6.0, help='disc diameter')
    ap.add_argument('--genlik', type=float, default=8.0, help='sinus amplitude, deg')
    ap.add_argument('--frekans', type=float, default=0.2, help='sinus frequency, Hz')
    ap.add_argument('--hiz', type=float, default=5.0, help='tarama start speed, deg/s')
    ap.add_argument('--adim', type=float, default=5.0, help='tarama speed increase per pass, deg/s')
    ap.add_argument('--sicrama-deg', type=float, default=8.0, help='sicrama jump size, deg')
    a = ap.parse_args()

    root = tk.Tk()
    root.attributes('-fullscreen', True)
    root.configure(bg='black')
    W, H = root.winfo_screenwidth(), root.winfo_screenheight()
    cv = tk.Canvas(root, width=W, height=H, bg='black', highlightthickness=0)
    cv.pack()
    px_per_cm = W / a.ekran_cm

    def deg_to_x(deg):
        return W / 2 + math.tan(math.radians(deg)) * a.mesafe_cm * px_per_cm

    r = a.cap_cm * px_per_cm / 2
    disc = cv.create_oval(0, 0, 1, 1, fill='#ff0000', outline='')
    flash = cv.create_rectangle(0, H - 120, 120, H, fill='black', outline='')
    info = cv.create_text(10, 10, anchor='nw', fill='#666666', font=('Arial', 12))
    half_range = min(20.0, math.degrees(math.atan((W / 2 - r) / px_per_cm / a.mesafe_cm)))

    log = open(f"hedef_log_{a.mod}.csv", 'w', newline='')
    lw = csv.writer(log); lw.writerow(['t_s', 'x_px', 'deg', 'speed_dps', 'flash'])
    st = dict(t0=time.perf_counter(), deg=0.0, dirn=1, speed=a.hiz,
              next_jump=time.perf_counter() + 3.0, side=-1, flash_until=0.0)

    def tick():
        t = time.perf_counter() - st['t0']
        fl = 0
        if a.mod == 'sabit':
            deg = -11.0
        elif a.mod == 'sinus':
            deg = a.genlik * math.sin(2 * math.pi * a.frekans * t)
        elif a.mod == 'tarama':
            deg = st['deg'] + st['dirn'] * st['speed'] / 60.0
            if abs(deg) > half_range:
                deg = math.copysign(half_range, deg)
                st['dirn'] *= -1
                if st['dirn'] > 0:                   # one full pass done
                    st['speed'] += a.adim
            st['deg'] = deg
        else:                                        # sicrama
            now = time.perf_counter()
            if now >= st['next_jump']:
                st['side'] *= -1
                st['next_jump'] = now + random.uniform(2.0, 4.0)
                st['flash_until'] = now + 0.10
            deg = st['side'] * a.sicrama_deg / 2
            fl = int(now < st['flash_until'])
            cv.itemconfigure(flash, fill='white' if fl else 'black')
        x = deg_to_x(deg)
        cv.coords(disc, x - r, H / 2 - r, x + r, H / 2 + r)
        cv.itemconfigure(info, text=f"{a.mod}  {deg:+6.2f} deg"
                         + (f"  {st['speed']:.0f} deg/s" if a.mod == 'tarama' else ''))
        lw.writerow([f"{t:.4f}", f"{x:.1f}", f"{deg:.3f}",
                     f"{st['speed']:.1f}" if a.mod == 'tarama' else '', fl])
        root.after(16, tick)

    root.bind('<Escape>', lambda e: (log.close(), root.destroy()))
    tick()
    root.mainloop()


if __name__ == '__main__':
    main()
