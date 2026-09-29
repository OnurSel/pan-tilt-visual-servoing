"""
controllers.py -- the two classical laws compared in the tracking trials:
a tuned discrete PID and the soft-zone P + step-and-wait + latch law of the paper.
Interface: ctrl.reset(); pan_step, tilt_step = ctrl.compute_step(ex, ey, dex, dey, None, dt)
"""

import numpy as np

# ---- Constants from the submitted manuscript, Table II ----
PX_PER_DEG = 10.67       # nominal; E1 measures the true value
MAX_STEP_DEG = 0.5       # per-frame command clamp (servo safety)
SOFT_ZONE_DEG = 3.0      # quadratic braking zone (~32 px)
APPROACH_PCT = 0.08      # per-frame fraction of the needed angle
FINE_LOCK_PX = 12.0      # latch enter threshold
FINE_RELEASE_PX = 30.0   # latch release threshold
FINE_STEP_DEG = 0.5      # step-and-wait net step
FINE_WAIT_S = 0.3        # dwell between fine steps


# ============================================================
#  C1 -- Tuned discrete PID
# ============================================================
class PIDController:
    """Textbook discrete PID, implemented properly.

    Three things the naive version gets wrong, all fixed here:
      * the integral is scaled by dt, so the gain has physical units and does
        not silently change when the frame rate does;
      * conditional integration plus an integral clamp prevent the windup that
        a deadband otherwise guarantees;
      * the derivative is low-pass filtered, because differentiating +/-2 px of
        detector noise at 30 fps is otherwise pure amplification.

    Default gains are a starting point. Tune them against the plant identified
    in E1 -- see tune_pid() below -- and report the tuned values in the paper.
    """

    def __init__(self, kp=0.45, ki=0.25, kd=0.02, px_per_deg=PX_PER_DEG,
                 i_clamp_deg=1.5, d_tau=0.05):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.px_per_deg = px_per_deg
        self.i_clamp = i_clamp_deg
        self.d_tau = d_tau
        self.reset()

    def reset(self):
        self.ix = self.iy = 0.0
        self.prev_ex = self.prev_ey = 0.0
        self.dfx = self.dfy = 0.0

    def _axis(self, e_px, prev_e, integ, dfilt, dt):
        e_deg = e_px / self.px_per_deg

        # Derivative on the measurement, low-pass filtered
        raw_d = (e_deg - prev_e / self.px_per_deg) / max(dt, 1e-6)
        alpha = dt / (self.d_tau + dt)
        dfilt = dfilt + alpha * (raw_d - dfilt)

        # Provisional output with the integral frozen
        u_unsat = self.kp * e_deg + self.ki * integ + self.kd * dfilt

        # Conditional integration: only accumulate when not saturated, or when
        # the error would drive the output back out of saturation
        if abs(u_unsat) < MAX_STEP_DEG or (u_unsat * e_deg) < 0:
            integ += e_deg * dt
        integ = float(np.clip(integ, -self.i_clamp, self.i_clamp))

        u = self.kp * e_deg + self.ki * integ + self.kd * dfilt
        return float(np.clip(u, -MAX_STEP_DEG, MAX_STEP_DEG)), integ, dfilt

    def compute_step(self, ex, ey, dex, dey, phase_feat=None, dt=1 / 30.0):
        ux, self.ix, self.dfx = self._axis(ex, self.prev_ex, self.ix, self.dfx, dt)
        uy, self.iy, self.dfy = self._axis(ey, self.prev_ey, self.iy, self.dfy, dt)
        self.prev_ex, self.prev_ey = ex, ey
        return ux, uy


# ============================================================
#  C2 -- Soft-zone proportional + step-and-wait + latch
# ============================================================
class SoftZoneController:
    """The law from the submitted manuscript, reproduced exactly.

    Acquisition (eqs. 5-7 of the manuscript):
        need = e / kappa
        r    = |need| / Z
        g    = a * min(1, r^2)
        step = clip(need * g, +/- theta_max * min(1, r))

    Once inside the fine box the axis LATCHES: at most one net 0.5 deg step
    every 0.3 s, and the latch only releases past 30 px. This is what kills the
    hold-jitter limit cycle on a deadbanded analogue servo.
    """

    def __init__(self, px_per_deg=PX_PER_DEG, approach=APPROACH_PCT,
                 soft_zone=SOFT_ZONE_DEG, max_step=MAX_STEP_DEG):
        self.px_per_deg = px_per_deg
        self.approach = approach
        self.soft_zone = soft_zone
        self.max_step = max_step
        self.reset()

    def reset(self):
        self.latched = [False, False]      # [pan, tilt]
        self.t_since_fine = [0.0, 0.0]

    def _acquire(self, e_px):
        need = e_px / self.px_per_deg
        rel = abs(need) / self.soft_zone
        soft = rel * rel if rel < 1.0 else 1.0
        step = need * self.approach * soft
        eff = self.max_step * (rel if rel < 1.0 else 1.0)
        return float(np.clip(step, -eff, eff))

    def _axis(self, i, e_px, dt):
        # Latch hysteresis: enter at 12 px, release at 30 px
        if self.latched[i]:
            if abs(e_px) > FINE_RELEASE_PX:
                self.latched[i] = False
                self.t_since_fine[i] = 0.0
        elif abs(e_px) < FINE_LOCK_PX:
            self.latched[i] = True
            self.t_since_fine[i] = 0.0

        if not self.latched[i]:
            return self._acquire(e_px)

        # Latched -> step-and-wait fine alignment
        self.t_since_fine[i] += dt
        if abs(e_px) <= FINE_LOCK_PX:
            return 0.0                      # inside fine box: issue nothing
        if self.t_since_fine[i] >= FINE_WAIT_S:
            self.t_since_fine[i] = 0.0
            return float(np.sign(e_px) * FINE_STEP_DEG)
        return 0.0

    def compute_step(self, ex, ey, dex, dey, phase_feat=None, dt=1 / 30.0):
        return self._axis(0, ex, dt), self._axis(1, ey, dt)


