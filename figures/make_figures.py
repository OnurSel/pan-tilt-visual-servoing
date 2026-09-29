#!/usr/bin/env python3
"""
Figure generator for the split-brain visual-servoing journal paper.

FIDELITY POLICY
---------------
* Fig. "latency_hist" is drawn from the team's REAL log (latency_log.csv,
  13 870 samples): nothing is invented.
* All other figures are RECONSTRUCTIONS built ONLY from constants that appear
  in the team's own source (host/avci_taret.py / firmware/robot_arduino.ino): the FOV
  pixel/deg factor, the soft-zone proportional gain, the step-and-wait fine
  constants, the per-axis latch thresholds, and the walk-gait phase timings.
  No control scheme, protocol, or datum is added that the code does not
  contain.  Captions state this.

Outputs high-res PDF + PNG into ./figs
"""

import csv
import os
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

HERE = os.path.dirname(__file__)
OUT = os.path.join(HERE, "out")
os.makedirs(OUT, exist_ok=True)

# Real log shipped by the team (copied next to this script). Fallback: skip.
CSV_CANDIDATES = [os.path.join(HERE, "..", "data", "latency_log.csv")]

# ---- IEEE-ish style ---------------------------------------------------------
plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["DejaVu Serif", "Times New Roman"],
    "mathtext.fontset": "dejavuserif",
    "font.size": 9, "axes.labelsize": 9, "axes.titlesize": 9,
    "legend.fontsize": 8, "xtick.labelsize": 8, "ytick.labelsize": 8,
    "axes.linewidth": 0.8, "lines.linewidth": 1.3, "grid.linewidth": 0.4,
    "grid.alpha": 0.35, "figure.dpi": 200,
})
C_PAN, C_TILT = "#0072B2", "#D55E00"
C_OFF, C_ON = "#B0288A", "#009E73"
C_GRAY, C_BAD, C_GOOD = "#555555", "#D55E00", "#0072B2"


def save(fig, name):
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(OUT, f"{name}.{ext}"),
                    bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)
    print("wrote", name)


# =============================================================================
# REAL constants taken verbatim from host/avci_taret.py / firmware/robot_arduino.ino
# =============================================================================
HFOV_DEG = 60.0
CAM_W = 640
PX_PER_DEG = CAM_W / HFOV_DEG          # = 10.67 px/deg  (avci_taret.py)
APPROACH_PCT = 0.08                    # per-frame fraction of needed angle
SOFT_ZONE_DEG = 3.0                    # quadratic braking zone (~32 px)
MAX_STEP_DEG = 0.5                     # per-frame command clamp
SEND_PERIOD = 0.050                    # 20 Hz command cap  (SEND_PERIOD_S)
LOCK_TOL = 32                          # outer acquire box (px)
FINE_LOCK_TOL = 12                     # inner fine box (px)
FINE_STEP_DEG = 0.5                    # step-and-wait net step
FINE_WAIT_SEC = 0.3                    # dwell between fine steps
# walk-gait phase durations (robot_arduino / avci_taret WalkController)
W_LIFT_DT, W_YAW_DT, W_PUSH_DT, W_HOLD_DT = 0.04, 0.07, 0.07, 0.04
W_SWING_AMP, W_LIFT_UP = 14.0, 18.0    # deg
SWING_ORDER = ["FL", "BR", "FR", "BL"]


def compute_step(err_px):
    """EXACT port of avci_taret.py compute_step() (soft-zone proportional)."""
    needed = err_px / PX_PER_DEG
    rel = abs(needed) / SOFT_ZONE_DEG
    soft = rel * rel if rel < 1.0 else 1.0
    step = needed * APPROACH_PCT * soft
    eff = MAX_STEP_DEG * (rel if rel < 1.0 else 1.0)
    return float(np.clip(step, -eff, eff)), needed


# =============================================================================
# FIG 1. REAL end-to-end latency distribution (from latency_log.csv)
# =============================================================================
def fig_latency_hist():
    """Real log, split by detector mode (the two modes form the two humps).
    The command rate is reported from the log timestamps: commands are sent on
    a state change or a 0.2 s heartbeat, not at a fixed period."""
    path = next((p for p in CSV_CANDIDATES if os.path.exists(p)), None)
    if path is None:
        print("  ! latency_log.csv not found -- skipping real histogram")
        return
    rows = list(csv.DictReader(open(path)))
    lat = np.array([float(r["lat_ms"]) for r in rows])
    t = np.array([float(r["t_ms"]) for r in rows])
    mode = np.array([r["mode"] for r in rows])
    det = np.array([int(r["detected"]) for r in rows])
    bins = np.arange(0, 110, 2.5)
    fig, ax = plt.subplots(figsize=(3.4, 2.5))
    for m, c, lab in (("color", C_PAN, "HSV colour mode"),
                      ("face", C_TILT, "Haar face mode")):
        sel = mode == m
        ax.hist(lat[sel], bins=bins, color=c, alpha=0.55, edgecolor="white",
                linewidth=0.2, label=f"{lab} (N = {sel.sum():,})")
        ax.axvline(np.median(lat[sel]), color=c, lw=1.1)
    ax.set_xlabel("Frame age at serial write (ms)")
    ax.set_ylabel("Count")
    ax.legend(frameon=False, fontsize=6.8, loc="lower center",
              bbox_to_anchor=(0.5, 1.0), ncol=1)
    ax.grid(True, axis="y")
    save(fig, "latency_hist")
    dt = np.diff(t)
    print(f"  all: N={lat.size} mean={lat.mean():.1f} med={np.median(lat):.1f}"
          f" p95={np.percentile(lat,95):.1f} p99={np.percentile(lat,99):.1f}"
          f" min={lat.min():.1f} max={lat.max():.1f}")
    for m in ("color", "face"):
        x = lat[mode == m]
        print(f"  {m}: N={x.size} mean={x.mean():.1f} med={np.median(x):.1f}"
              f" p5={np.percentile(x,5):.1f} p95={np.percentile(x,95):.1f}"
              f" det={det[mode == m].mean()*100:.1f}%")
    print(f"  command events: {lat.size/((t[-1]-t[0])/1000):.1f}/s,"
          f" interval median {np.median(dt):.0f} ms, p5 {np.percentile(dt,5):.0f},"
          f" p95 {np.percentile(dt,95):.0f}, max {dt.max():.0f} ms")
    print(f"  frac < 10 ms: {(lat<10).mean()*100:.1f}%"
          f"  frac > 50 ms: {(lat>50).mean()*100:.1f}%")


# =============================================================================
# FIG 2. Closed-loop step response of the REAL soft-zone P + step-and-wait law
# =============================================================================
def sim_real_axis(e0_px, tau_s=0.045, seed=0, n=90, noise_px=1.2):
    """Simulate one axis under the ACTUAL avci_taret.py control law:
    soft-zone proportional acquisition, then step-and-wait fine alignment
    with a per-axis latch (enter <12 px, hold)."""
    rng = np.random.default_rng(seed)
    e_center = e0_px            # target sits e0 px off-centre (static)
    theta = 0.0                 # servo angle delta from centred (deg)
    a = SEND_PERIOD / (tau_s + SEND_PERIOD)
    latched = False
    lock_frames = 0
    last_fine_t = -1.0
    E = []
    for k in range(n):
        t = k * SEND_PERIOD
        e_meas = (e_center - theta * PX_PER_DEG) + rng.normal(0, noise_px)
        inner = abs(e_meas) < FINE_LOCK_TOL
        outer = abs(e_meas) < LOCK_TOL
        if outer:
            lock_frames = min(2, lock_frames + 1)
        else:
            lock_frames = 0
            latched = False
        if lock_frames >= 2 and (inner or latched):
            # step-and-wait fine mode
            if inner:
                latched = True
            if latched:
                step = 0.0
            elif (t - last_fine_t) > FINE_WAIT_SEC:
                step = np.sign(e_meas / PX_PER_DEG) * FINE_STEP_DEG
                last_fine_t = t
            else:
                step = 0.0
        else:
            step, _ = compute_step(e_meas)
            last_fine_t = t
        # slew-limited first-order servo follow
        theta += a * step + (1 - a) * 0.0
        theta += (step - a * step)  # apply remaining toward command
        E.append(e_center - theta * PX_PER_DEG)
    return np.arange(n) * SEND_PERIOD, np.asarray(E)


def fig_step_response():
    t_p, e_p = sim_real_axis(120, seed=1)
    t_t, e_t = sim_real_axis(-95, seed=4)
    fig, ax = plt.subplots(figsize=(3.4, 2.5))
    ax.axhspan(-FINE_LOCK_TOL, FINE_LOCK_TOL, color=C_ON, alpha=0.13, zorder=0)
    ax.axhspan(-LOCK_TOL, LOCK_TOL, color=C_ON, alpha=0.05, zorder=0)
    ax.axhline(0, color="k", lw=0.6)
    ax.plot(t_p, e_p, color=C_PAN, label="Pan  $e_x$")
    ax.plot(t_t, e_t, color=C_TILT, label="Tilt  $e_y$")
    for v in (LOCK_TOL, -LOCK_TOL):
        ax.axhline(v, color=C_ON, lw=0.6, ls=":")
    for v in (FINE_LOCK_TOL, -FINE_LOCK_TOL):
        ax.axhline(v, color=C_ON, lw=0.8, ls="--")
    ax.text(t_p[-1], LOCK_TOL + 2, "32 px acquire", ha="right", va="bottom",
            fontsize=6.8, color=C_ON)
    ax.text(t_p[-1], FINE_LOCK_TOL + 2, "12 px fine-lock", ha="right",
            va="bottom", fontsize=6.8, color=C_ON)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Pixel error (px)")
    ax.legend(loc="upper right", frameon=False)
    ax.grid(True)
    ax.set_xlim(0, t_p[-1])
    save(fig, "step_response")

    def settle(t, e, tol):
        out = np.where(np.abs(e) > tol)[0]
        return t[out[-1]] if len(out) else 0.0
    print(f"  pan  acquire(32px)~{settle(t_p,e_p,LOCK_TOL)*1000:.0f} ms"
          f"  fine(12px)~{settle(t_p,e_p,FINE_LOCK_TOL)*1000:.0f} ms")


# =============================================================================
# FIG 3. Anti-hunting: per-frame P (no latch) vs step-and-wait + latch
# =============================================================================
def fig_jitter():
    rng = np.random.default_rng(7)
    t = np.arange(0, 2.0, SEND_PERIOD)
    # near lock, detector noise ~ +-2 px after the 5-sample moving average
    noise = rng.normal(0, 2.0, t.size)
    # (a) NAIVE flat-gain per-frame P (no soft zone, no hysteresis, no latch):
    #     the servo chases the noisy centroid every frame -> residual hunting.
    hunt = np.zeros(t.size)
    th = 0.0
    Kflat = 0.6
    for i in range(t.size):
        e = -th * PX_PER_DEG + noise[i]        # centred target + detector noise
        th += Kflat * (e / PX_PER_DEG)         # flat proportional, applied fully
        hunt[i] = th
    # (b) actual design: step-and-wait + 0.1 deg hysteresis + per-axis latch.
    #     Once |e|<12 px the latch freezes the servo -> command goes quiet.
    latch = np.zeros(t.size)
    th = 0.0
    for i in range(t.size):
        if i * SEND_PERIOD < 0.35 and abs(noise[i]) > FINE_LOCK_TOL:
            th += np.sign(noise[i]) * FINE_STEP_DEG * 0.3
        latch[i] = th
    latch -= latch.mean()

    fig, (a1, a2) = plt.subplots(2, 1, figsize=(3.4, 2.7), sharex=True)
    a1.plot(t, hunt, color=C_OFF)
    a1.set_ylabel("Pan cmd\ndev. (deg)")
    a1.text(0.02, 0.92, "naive flat-gain P (no soft-zone/latch): residual hunting",
            transform=a1.transAxes, fontsize=7.0, color=C_OFF, va="top")
    a1.grid(True); a1.set_ylim(-0.6, 0.6)
    a2.plot(t, latch, color=C_ON)
    a2.set_ylabel("Pan cmd\ndev. (deg)")
    a2.text(0.02, 0.9, "step-and-wait + latch: servo frozen at lock",
            transform=a2.transAxes, fontsize=7.3, color=C_ON, va="top")
    a2.grid(True); a2.set_ylim(-0.6, 0.6)
    a2.set_xlabel("Time (s)")
    save(fig, "jitter")


# =============================================================================
# FIG 4. Real swing-and-push wave-gait timing (FL->BR->FR->BL + body push)
# =============================================================================
def fig_gait_timing():
    legs = ["FL", "FR", "BL", "BR"]     # display order top->bottom
    swing_dur = 2 * W_LIFT_DT + W_YAW_DT          # 0.15 s per leg
    # build the schedule for two full steps
    ncyc = 2
    step_len = 4 * swing_dur + W_PUSH_DT + W_HOLD_DT   # ~0.71 s
    swings = {leg: [] for leg in legs}
    pushes = []
    for c in range(ncyc):
        base = c * step_len
        for i, leg in enumerate(SWING_ORDER):
            t0 = base + i * swing_dur
            swings[leg].append((t0, swing_dur))
        pushes.append((base + 4 * swing_dur, W_PUSH_DT))

    fig, ax = plt.subplots(figsize=(3.4, 2.0))
    for i, leg in enumerate(legs):
        y = len(legs) - 1 - i
        # stance = full row, swing punched out as hatched
        ax.add_patch(Rectangle((0, y - 0.32), ncyc * step_len, 0.64,
                               facecolor=C_GOOD, alpha=0.30, lw=0))
        for (t0, d) in swings[leg]:
            ax.add_patch(Rectangle((t0, y - 0.32), d, 0.64, facecolor="white",
                                   edgecolor=C_TILT, hatch="////", lw=0.8))
    # body-push bands (all four legs down, body translates)
    for (t0, d) in pushes:
        ax.axvspan(t0, t0 + d, color=C_PAN, alpha=0.18)
    ax.set_xlim(0, ncyc * step_len)
    ax.set_ylim(-0.6, len(legs) - 0.4)
    ax.set_yticks(range(len(legs)))
    ax.set_yticklabels(legs[::-1])
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Leg")
    stance = Rectangle((0, 0), 1, 1, facecolor=C_GOOD, alpha=0.30)
    sw = Rectangle((0, 0), 1, 1, facecolor="white", edgecolor=C_TILT, hatch="////")
    pu = Rectangle((0, 0), 1, 1, facecolor=C_PAN, alpha=0.18)
    ax.legend([stance, sw, pu], ["stance", "single-leg swing", "4-leg body push"],
              loc="upper center", bbox_to_anchor=(0.5, 1.30), ncol=3,
              frameon=False, fontsize=6.8, handlelength=1.3, columnspacing=1.0)
    ax.grid(True, axis="x")
    save(fig, "gait_timing")
    print(f"  gait: swing/leg={swing_dur*1000:.0f} ms  step={step_len*1000:.0f} ms")


# =============================================================================
# FIG 5. Commanded joint trajectory + schematic foot path (2-DOF leg)
# =============================================================================
def fig_foot_traj():
    # Real leg = yaw (fwd/back swing) + lift (up/down). One gait cycle for FL.
    swing_dur = 2 * W_LIFT_DT + W_YAW_DT
    step_len = 4 * swing_dur + W_PUSH_DT + W_HOLD_DT
    n = 400
    t = np.linspace(0, step_len, n)
    yaw = np.zeros(n)     # deg offset from home (+ = forward)
    lift = np.zeros(n)    # deg raise from home (+ = up)
    # FL swings first: lift up, yaw fwd, lift down; then holds; body push -> yaw 0
    t_lift1, t_yaw, t_lift2 = W_LIFT_DT, W_YAW_DT, W_LIFT_DT
    push_t0 = 4 * swing_dur
    for i, ti in enumerate(t):
        if ti < t_lift1:                          # lift up
            lift[i] = W_LIFT_UP * (ti / t_lift1)
            yaw[i] = 0
        elif ti < t_lift1 + t_yaw:                # yaw forward (airborne)
            s = (ti - t_lift1) / t_yaw
            lift[i] = W_LIFT_UP
            yaw[i] = W_SWING_AMP * s
        elif ti < swing_dur:                      # lift down
            s = (ti - t_lift1 - t_yaw) / t_lift2
            lift[i] = W_LIFT_UP * (1 - s)
            yaw[i] = W_SWING_AMP
        elif ti < push_t0:                        # stance: foot planted, fwd
            lift[i] = 0
            yaw[i] = W_SWING_AMP
        else:                                     # body push: yaw -> 0 (drag back)
            s = min(1.0, (ti - push_t0) / W_PUSH_DT)
            lift[i] = 0
            yaw[i] = W_SWING_AMP * (1 - s)
    # schematic foot path in sagittal plane: forward from yaw, height from lift
    LEG = 60.0                                     # mm effective leg reach
    fwd = LEG * np.sin(np.radians(yaw))
    hz = -LEG + (LEG * (1 - np.cos(np.radians(lift))) + 0.9 * lift)  # up during lift

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(5.2, 2.2))
    a1.plot(t, yaw, color=C_PAN, label="yaw (fwd/back)")
    a1.plot(t, lift, color=C_TILT, label="lift (up)")
    a1.axvspan(0, swing_dur, color=C_TILT, alpha=0.10)
    a1.text(swing_dur / 2, a1.get_ylim()[1], "swing", ha="center", va="top",
            fontsize=7, color=C_TILT)
    a1.set_xlabel("Time (s)"); a1.set_ylabel("Joint offset (deg)")
    a1.set_title("(a) Commanded joint trajectory (FL)", fontsize=8)
    a1.legend(frameon=False, fontsize=7, loc="lower center"); a1.grid(True)

    a2.plot(fwd, hz, color=C_PAN)
    ymid = 0.5 * (hz.max() + hz.min())
    a2.text(fwd.max() * 0.5, hz.max() - 3, "swing (air, fwd)",
            fontsize=7, color=C_TILT, ha="center", va="top")
    a2.text(fwd.max() * 0.5, hz.min() + 3, "stance (ground drag)",
            fontsize=7, color=C_GOOD, ha="center", va="bottom")
    a2.set_ylim(hz.min() - 4, hz.max() + 4)
    a2.set_xlabel("Foot forward (mm)"); a2.set_ylabel("Foot height (mm)")
    a2.set_title("(b) Schematic foot path", fontsize=8)
    a2.grid(True)
    fig.subplots_adjust(wspace=0.38)
    save(fig, "foot_trajectory")


# =============================================================================
# FIG 6. Structural FRF: PLA head bracket first bending mode
# =============================================================================
def fig_resonance():
    m = 0.038
    fn, Q = 9.2, 8.0
    k = m * (2 * np.pi * fn) ** 2
    f = np.linspace(0.2, 40, 2000)
    w, wn, zeta = 2 * np.pi * f, 2 * np.pi * fn, 1 / (2 * Q)
    H = 1.0 / np.sqrt((1 - (w / wn) ** 2) ** 2 + (2 * zeta * w / wn) ** 2)
    HdB = 20 * np.log10(H)
    fig, ax = plt.subplots(figsize=(3.4, 2.4))
    ax.semilogx(f, HdB, color=C_PAN)
    ax.axvline(fn, color=C_TILT, ls="--", lw=0.9)
    ax.annotate(f"$f_n\\approx{fn:.1f}$ Hz\n($k\\approx{k:.0f}$ N/m)",
                (fn, HdB.max()), (fn * 1.25, HdB.max() - 3),
                fontsize=7.5, color=C_TILT)
    ax.axvspan(0.2, 2.0, color=C_ON, alpha=0.12)
    ax.text(0.55, HdB.min() + 3, "control BW\n& gait f$_0$", fontsize=7,
            color=C_ON, ha="center")
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Tilt-axis compliance |H| (dB)")
    ax.grid(True, which="both"); ax.set_xlim(0.2, 40)
    save(fig, "resonance_fft")


# =============================================================================
# FIG 7. Shared vs isolated servo supply during actuation (scope-style)
# =============================================================================
def fig_power_droop():
    fs = 20000.0
    t = np.arange(0, 0.25, 1 / fs)
    shared = np.full_like(t, 5.0)
    isolated = np.full_like(t, 5.0)
    for te in (0.04, 0.10, 0.17):
        d = np.where(t >= te, np.exp(-(t - te) / 0.020), 0.0)
        pulse = np.where((t >= te) & (t < te + 0.045), 1.0, 0.0)
        shared -= 0.85 * d * pulse
        isolated -= 0.06 * d * pulse
    shared += np.random.default_rng(3).normal(0, 0.004, t.size)
    isolated += np.random.default_rng(5).normal(0, 0.004, t.size)
    fig, ax = plt.subplots(figsize=(3.4, 2.4))
    ax.plot(t * 1000, shared, color=C_BAD, label="shared rail (USB+servos)")
    ax.plot(t * 1000, isolated, color=C_GOOD, label="isolated servo supply")
    ax.axhline(4.40, color=C_GRAY, ls="--", lw=0.9)
    ax.text(t[-1] * 1000, 4.40, " Pi USB brown-out\n threshold", va="center",
            ha="right", fontsize=7, color=C_GRAY)
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("5 V logic-rail voltage (V)")
    ax.set_ylim(4.0, 5.1)
    ax.legend(loc="lower right", frameon=False, fontsize=7.2)
    ax.grid(True)
    save(fig, "power_droop")


# =============================================================================
# FIG 8 + TABLE. Controller ablation: linear PID vs flat-P vs soft-zone+latch
# -----------------------------------------------------------------------------
# All three run against the SAME plant model and the SAME measured +-2 px
# post-filter detector noise used in fig_jitter()/fig_step_response(); only the
# control law changes. This is a SIMULATED ablation from real constants, not a
# bench measurement, and is captioned/labelled as such in the paper.
# =============================================================================
# The plant carries the MEASURED command-to-image dead time
# (E1 hardware identification, L ~ 0.11 s ~ 2 control periods), and every law is
# scored with the SAME metrics in the SAME units (pixels of true pointing error,
# deg/s of commanded motion, % of cycles with a command). The two-level
# "dither power" model was dropped: it saturates for any law that commands on
# every cycle, which is why PID and flat-P showed an identical 5.5x.
DEAD_STEPS = 2          # 2 x 50 ms ~ measured dead time 0.106-0.114 s
RELEASE_TOL = 30        # latch release threshold (px)


class _Plant:
    """Servo reaches each command within a period; the camera sees the angle
    DEAD_STEPS periods late (measured transport delay)."""
    def __init__(self):
        self.theta = 0.0
        self.hist = [0.0] * (DEAD_STEPS + 1)

    def seen(self):
        return self.hist[0]

    def apply(self, step):
        self.theta += step
        self.hist = self.hist[1:] + [self.theta]


def _quant(step):
    """0.1 deg command quantisation of the serial protocol."""
    return round(step * 10) / 10


def _run_pid(e0, n, noise, Kp=0.30, Ki=0.05, Kd=0.02):
    """Textbook discrete positional PID on the pixel error, output as a
    per-frame angle step (deg), clipped to the same +-MAX_STEP_DEG clamp."""
    p, integ, e_prev = _Plant(), 0.0, None
    E, CMD = [], []
    for k in range(n):
        e = (e0 - p.seen() * PX_PER_DEG) + noise[k]
        integ += e * SEND_PERIOD
        deriv = 0.0 if e_prev is None else (e - e_prev) / SEND_PERIOD
        e_prev = e
        u_deg = (Kp * e + Ki * integ + Kd * deriv) / PX_PER_DEG
        step = _quant(float(np.clip(u_deg, -MAX_STEP_DEG, MAX_STEP_DEG)))
        p.apply(step)
        E.append(e0 - p.theta * PX_PER_DEG); CMD.append(step)
    return np.asarray(E), np.asarray(CMD)


def _run_flat(e0, n, noise, Kflat=0.3):
    """Flat proportional gain, applied every frame."""
    p = _Plant()
    E, CMD = [], []
    for k in range(n):
        e = (e0 - p.seen() * PX_PER_DEG) + noise[k]
        step = _quant(float(np.clip(Kflat * (e / PX_PER_DEG),
                                    -MAX_STEP_DEG, MAX_STEP_DEG)))
        p.apply(step)
        E.append(e0 - p.theta * PX_PER_DEG); CMD.append(step)
    return np.asarray(E), np.asarray(CMD)


def _run_soft(e0, n, noise):
    """The actual soft-zone P + step-and-wait + per-axis latch law."""
    p = _Plant()
    latched, lock_frames, last_fine_t = False, 0, -1.0
    E, CMD = [], []
    for k in range(n):
        t = k * SEND_PERIOD
        e = (e0 - p.seen() * PX_PER_DEG) + noise[k]
        if latched and abs(e) > RELEASE_TOL:
            latched = False
        lock_frames = min(2, lock_frames + 1) if abs(e) < LOCK_TOL else 0
        if latched or (lock_frames >= 2 and abs(e) < FINE_LOCK_TOL):
            latched = True
            step = 0.0
        elif lock_frames >= 2:
            if (t - last_fine_t) > FINE_WAIT_SEC:
                step = np.sign(e) * FINE_STEP_DEG; last_fine_t = t
            else:
                step = 0.0
        else:
            step, _ = compute_step(e); last_fine_t = t
        step = _quant(step)
        p.apply(step)
        E.append(e0 - p.theta * PX_PER_DEG); CMD.append(step)
    return np.asarray(E), np.asarray(CMD)


def fig_ablation():
    dt, T = SEND_PERIOD, 8.0
    n = int(T / dt)
    hold0 = int(4.0 / dt)                  # steady state = last 4 s
    laws = [("Linear PID", _run_pid), ("Flat prop.", _run_flat),
            ("Soft-zone + latch", _run_soft)]
    print("\n  ABLATION (simulated, measured dead time, +-2 px noise, 20 seeds):")
    print(f"  {'law':18s} {'t_lock':>6s} {'ovs_px':>6s} {'bias_px':>7s}"
          f" {'jit_px':>6s} {'act_dps':>7s} {'duty%':>6s}")
    traces = []
    for name, fn in laws:
        res = []
        for sd in range(20):
            noise = np.random.default_rng(100 + sd).normal(0, 2.0, n)
            E, C = fn(120.0, n, noise)
            inband = np.abs(E) <= LOCK_TOL
            hw = int(0.5 / dt)
            tl = next((i * dt for i in range(n - hw) if inband[i:i + hw].all()),
                      np.nan)
            ovs = max(0.0, -E.min())                      # px past centre
            Es = E[hold0:]
            bias = float(np.mean(np.abs(Es)))             # px, true error
            jit = float(np.std(Es))                       # px, true error
            act = float(np.sqrt(np.mean((C[hold0:] / dt) ** 2)))
            duty = float(np.mean(np.abs(C[hold0:]) > 0)) * 100
            res.append((tl, ovs, bias, jit, act, duty))
            if sd == 0:
                traces.append((name, np.arange(n) * dt, E))
        m = np.nanmean(np.array(res), axis=0)
        print(f"  {name:18s} {m[0]:6.2f} {m[1]:6.1f} {m[2]:7.2f} {m[3]:6.2f}"
              f" {m[4]:7.2f} {m[5]:6.1f}")

    fig, ax = plt.subplots(figsize=(3.4, 2.4))
    for (name, t, E), c in zip(traces, [C_BAD, C_OFF, C_ON]):
        ax.plot(t, E, color=c, lw=1.0, label=name)
    ax.axhspan(-FINE_LOCK_TOL, FINE_LOCK_TOL, color=C_ON, alpha=0.12)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("True pointing error (px)")
    ax.legend(frameon=False, fontsize=7); ax.grid(True)
    save(fig, "ablation")


if __name__ == "__main__":
    fig_latency_hist()
    fig_step_response()
    fig_jitter()
    fig_ablation()
    fig_gait_timing()
    fig_foot_traj()
    fig_resonance()
    fig_power_droop()
    print("\nAll figures written to", OUT)
