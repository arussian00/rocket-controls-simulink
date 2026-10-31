#!/usr/bin/env python3
"""RK4 reproduction of the Super Heavy trajectory in matlab-rocket-sim.

Mirrors rocket_starship_sim.m (stack ascent to hot staging) and
simulateBooster.m with the 'superheavy' preset (flip, boostback, coast,
brake, landing). The companion has no time-varying CG: the centre of mass
stays at mid-length and the gimbal arm is L/2. Centre of pressure is the
pitch-study assumption (static margin -0.5 caliber on the stack, 0 on the
booster alone), not a number from the companion.

Writes data/superheavy_booster_trajectory.csv. Regenerate the ode45 version
in MATLAB with make_booster_trajectory.m.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np

G0 = 9.80665
RE = 6371e3
RHO0 = 1.225
MU = G0 * RE**2
R_AIR = 287.053
GAM = 1.4
R0_GEO = 6356766.0

# rocket_params('superheavy') and rocket_starship_sim.m
M1_DRY, M1_PROP = 230e3, 3400e3
M2_DRY, M2_PROP, MPAY = 120e3, 1500e3, 10e3
T_ENG = 2.3e6
ISP_SL, ISP_VAC = 327.0, 350.0
DIAM = 9.0
L_BOOST, L_SHIP = 71.0, 50.0
L_STACK = L_BOOST + L_SHIP
A_REF = math.pi * (DIAM / 2) ** 2
N_ASCENT, N_BOOST, N_BRAKE, N_LAND = 33, 13, 13, 3
GIMBAL_MAX = math.radians(15.0)
TAU_RCS = 30e6
ATT_WN, ATT_ZETA = 1.0, 0.8
THR_MIN = 0.40
RESERVE1 = 400e3
H_LAND = 65.0
BRAKE_GMAX = 5.0
BRAKE_END_FRAC = 0.6
IGN_FRAC = 0.75
IGN_ALT = 10e3
V_TD = 1.0
H_FLOOR = 0.2
MAX_TILT = math.radians(30.0)
TILT_FADE = 20.0
WN_DIVERT = 0.3
DIVERT_FADE = 50.0
DIVERT_TAU_MIN, DIVERT_TAU_MAX = 2.5, 3.0
FIN_ALT, FIN_CLA, TAU_FIN = 40e3, 25.0, 3.0
FIN_AIM = 0.0
X_PAD = 0.0
CD_STACK = 0.4
CD_UP, CD_DOWN = 0.5, 1.0
STACK_THRUST = N_ASCENT * T_ENG
THR_BUCKET = 0.75
BUCKET_MACH = (0.6, 0.8, 1.5, 1.8)
T_KICK, KICK_DUR, KICK_DEG = 8.0, 5.0, 0.8
CNA = 2.0
SM_STACK = -0.5
SM_BOOSTER = 0.0  # landing-burn design point; grid fins are a later step

MT = np.array([0, 0.6, 0.8, 0.95, 1.05, 1.2, 1.5, 2.0, 3.0, 5.0])
FT = np.array([1.0, 1.0, 1.1, 1.45, 1.65, 1.7, 1.5, 1.3, 1.12, 1.0])


def us76(h: float):
    hc = max(h, 0.0)
    H = R0_GEO * hc / (R0_GEO + hc)
    if H >= 84852.0:
        # Above the scripted troposphere/stratosphere layers the pitch
        # schedule only needs a small density. Exponential tail.
        rho = 1.225 * math.exp(-hc / 8500.0)
        return rho, 295.0
    Hb = (0, 11e3, 20e3, 32e3, 47e3, 51e3, 71e3, 84852.0)
    Tb = (288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65, 186.946)
    Lb = (-6.5e-3, 0.0, 1.0e-3, 2.8e-3, 0.0, -2.8e-3, -2.0e-3)
    pb = (101325, 22632.06, 5474.889, 868.0187, 110.9063, 66.93887, 3.956420)
    k = 0
    for i in range(6, -1, -1):
        if H >= Hb[i]:
            k = i
            break
    dH = H - Hb[k]
    if Lb[k] == 0.0:
        T = Tb[k]
        p = pb[k] * math.exp(-G0 * dH / (R_AIR * T))
    else:
        T = Tb[k] + Lb[k] * dH
        p = pb[k] * (T / Tb[k]) ** (-G0 / (R_AIR * Lb[k]))
    return p / (R_AIR * T), math.sqrt(GAM * R_AIR * T)


def mach_drag(M: float) -> float:
    if M >= MT[-1]:
        return float(FT[-1])
    k = int(np.searchsorted(MT, M, side="right") - 1)
    k = max(0, min(k, len(MT) - 2))
    return float(FT[k] + (FT[k + 1] - FT[k]) * (M - MT[k]) / (MT[k + 1] - MT[k]))


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def gravity(h: float) -> float:
    return MU / (RE + h) ** 2


# ---------------------------------------------------------------------------
# Stack ascent (starshipStack, mode 'stack')
# ---------------------------------------------------------------------------

def ascent_deriv(t, Y):
    x, h, vx, vh, m = Y
    r = RE + h
    g = gravity(h)
    rho, a_sound = us76(h)
    v = math.hypot(vx, vh)
    M = v / a_sound if a_sound > 0 else 0.0
    Cd = CD_STACK * mach_drag(M)
    D = 0.5 * rho * v * v * Cd * A_REF
    b0, b1, b2, b3 = BUCKET_MACH
    w = min(max((M - b0) / (b1 - b0), 0.0), 1.0) * min(max((b3 - M) / (b3 - b2), 0.0), 1.0)
    F = STACK_THRUST * (1.0 - (1.0 - THR_BUCKET) * w)
    Isp = ISP_VAC - (ISP_VAC - ISP_SL) * rho / RHO0
    if t < T_KICK or v == 0.0:
        ux, uy = 0.0, 1.0
    elif t < T_KICK + KICK_DUR:
        ux, uy = math.sin(math.radians(KICK_DEG)), math.cos(math.radians(KICK_DEG))
    else:
        ux, uy = vx / v, vh / v
    if v > 0:
        ax = F * ux / m - D * vx / (v * m)
        ay = F * uy / m - D * vh / (v * m)
    else:
        ax, ay = 0.0, F / m
    return np.array([
        vx * RE / r,
        vh,
        ax - vx * vh / r,
        ay - g + vx * vx / r,
        -F / (Isp * G0),
    ]), dict(F=F, throttle=F / STACK_THRUST, q=0.5 * rho * v * v, M=M, v=v, rho=rho, a=a_sound)


def rk4_ascent(dt=0.05):
    m0 = M1_DRY + M1_PROP + M2_DRY + M2_PROP + MPAY
    m_meco = m0 - (M1_PROP - RESERVE1)
    Y = np.array([0.0, 0.0, 0.0, 0.0, m0])
    t = 0.0
    rows = []
    while t < 200.0 and Y[4] > m_meco:
        dY, aux = ascent_deriv(t, Y)
        rows.append((t, Y.copy(), aux))
        k1, _ = ascent_deriv(t, Y)
        k2, _ = ascent_deriv(t + dt / 2, Y + dt * k1 / 2)
        k3, _ = ascent_deriv(t + dt / 2, Y + dt * k2 / 2)
        k4, _ = ascent_deriv(t + dt, Y + dt * k3)
        Y = Y + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6
        t += dt
    return rows


# ---------------------------------------------------------------------------
# Booster return (simulateBooster, superheavy phases)
# ---------------------------------------------------------------------------

def engines(phase: str) -> int:
    return {"boostback": N_BOOST, "brake": N_BRAKE, "landing": N_LAND}.get(phase, 0)


def ballistic_acc(s, m):
    h, vx, vh = s[1], s[2], s[3]
    r = RE + h
    g = gravity(h)
    rho, a_sound = us76(h)
    v = math.hypot(vx, vh)
    Cd = CD_UP if vh >= 0 else CD_DOWN
    if v > 0:
        Cd *= mach_drag(v / a_sound)
        k = 0.5 * rho * v * Cd * A_REF / m
    else:
        k = 0.0
    return np.array([
        vx * RE / r,
        vh,
        -k * vx - vx * vh / r,
        -k * vh - g + vx * vx / r,
    ])


def predict_impact(X):
    s = np.array(X[:4], dtype=float)
    m = float(X[4])
    dt = 2.0
    for _ in range(2000):
        k1 = ballistic_acc(s, m)
        k2 = ballistic_acc(s + dt / 2 * k1, m)
        s_new = s + dt * k2
        if s_new[1] <= 0:
            frac = s[1] / (s[1] - s_new[1] + 1e-12)
            return s[0] + frac * (s_new[0] - s[0])
        s = s_new
    return float(s[0])


def landing_accel(X):
    x, h, vx, vh, m = X[:5]
    h_above = h - H_LAND
    h_eff = max(h_above, H_FLOOR)
    rho, _ = us76(h)
    g = gravity(h)
    v = math.hypot(vx, vh)
    aD = 0.5 * 0.5 * rho * v * v * CD_DOWN * A_REF / m
    if v > 0:
        a_drag = aD * np.array([-vx, -vh]) / v
    else:
        a_drag = np.zeros(2)
    ay = (vh ** 2 - V_TD ** 2) / (2 * h_eff) + g - a_drag[1]
    ay = max(ay, 0.0)
    tgo = max(2 * h_eff / max(-vh + V_TD, V_TD), 0.5)
    dx = X_PAD - x
    vx_want = math.copysign(min(WN_DIVERT * abs(dx), 2 * abs(dx) / tgo), dx) if dx != 0 else 0.0
    vx_want *= min(1.0, max(h_above, 0.0) / DIVERT_FADE)
    tau = min(max(tgo / 3, DIVERT_TAU_MIN), DIVERT_TAU_MAX)
    ax = (vx_want - vx) / tau - a_drag[0]
    tilt = MAX_TILT * min(1.0, max(h_above, 0.0) / TILT_FADE)
    ax_lim = ay * math.tan(tilt) if tilt > 0 else 0.0
    ax = min(max(ax, -ax_lim), ax_lim)
    return np.array([ax, ay])


def burn_capacity(phase, m):
    F = engines(phase) * T_ENG
    if phase == "brake":
        F = min(F, BRAKE_GMAX * G0 * m)
    return F


def attitude(X, theta_cmd, F):
    h, vx, m, theta, omega = X[1], X[2], X[4], X[5], X[6]
    r = RE + h
    I = m * L_BOOST ** 2 / 12.0
    arm = L_BOOST / 2.0
    err = wrap(theta_cmd - theta)
    theta_dot = omega - vx / r
    tau_cmd = I * (ATT_WN ** 2 * err - 2 * ATT_ZETA * ATT_WN * theta_dot)
    if F > 0:
        tau_max = F * arm * math.sin(GIMBAL_MAX)
        tau_g = min(max(tau_cmd, -tau_max), tau_max)
        delta = math.asin(max(min(tau_g / (F * arm), 1.0), -1.0))
    else:
        tau_g, delta = 0.0, 0.0
    tau_r = min(max(tau_cmd - tau_g, -TAU_RCS), TAU_RCS)
    return delta, tau_g, tau_r, I


def booster_deriv(t, X, phase, bb_dir):
    x, h, vx, vh, m, theta, omega = X
    r = RE + h
    g = gravity(h)
    rho, a_sound = us76(h)
    n_eng = engines(phase)
    if phase == "boostback":
        theta_cmd, thr_req = bb_dir * math.pi / 2, 1.0
    elif phase == "brake":
        theta_cmd = 0.0  # overwritten below from landing law
        a_cmd = landing_accel(X)
        theta_cmd = math.atan2(a_cmd[0], a_cmd[1]) if np.linalg.norm(a_cmd) > 0 else 0.0
        thr_req = burn_capacity("brake", m) / (N_BRAKE * T_ENG)
    elif phase == "landing":
        a_cmd = landing_accel(X)
        theta_cmd = math.atan2(a_cmd[0], a_cmd[1]) if np.linalg.norm(a_cmd) > 0 else 0.0
        thr_req = np.linalg.norm(a_cmd) * m / (N_LAND * T_ENG)
    elif phase == "flip":
        theta_cmd, thr_req = bb_dir * math.pi / 2, 0.0
    else:
        theta_cmd, thr_req = math.atan2(-vx, abs(vh)), 0.0
    if n_eng > 0 and m > M1_DRY:
        throttle = min(max(thr_req, THR_MIN), 1.0)
    else:
        throttle = 0.0
    F = throttle * n_eng * T_ENG
    delta, tau_g, tau_r, I = attitude(X, theta_cmd, F)
    u = np.array([math.sin(theta - delta), math.cos(theta - delta)])
    v_rel = np.array([vx, vh])
    vr = np.linalg.norm(v_rel)
    Cd = CD_UP if vh >= 0 else CD_DOWN
    if vr > 0:
        Cd *= mach_drag(vr / a_sound)
    q = 0.5 * rho * vr * vr
    D = q * Cd * A_REF
    drag = -D * v_rel / vr if vr > 1e-6 else np.zeros(2)
    Ffin = 0.0
    if phase == "coast" and vh < 0 and h < FIN_ALT:
        t_fall = max((h - FIN_AIM) / max(-vh, 1.0), 1.0)
        vx_want = (X_PAD - x) / t_fall
        Ffin_max = q * FIN_CLA
        Ffin = min(max(m * (vx_want - vx) / TAU_FIN, -Ffin_max), Ffin_max)
    a_ng = F * u / m + drag / m + np.array([Ffin / m, 0.0])
    Isp = ISP_VAC - (ISP_VAC - ISP_SL) * rho / RHO0
    dX = np.array([
        vx * RE / r,
        vh,
        a_ng[0] - vx * vh / r,
        a_ng[1] - g + vx * vx / r,
        -F / (Isp * G0) if F > 0 else 0.0,
        omega - vx / r,
        (tau_g + tau_r) / I,
    ])
    aux = dict(F=F, throttle=throttle, q=q, v=vr, M=vr / a_sound if a_sound else 0.0,
               n_eng=n_eng, rho=rho, a=a_sound)
    return dX, aux


def rk4_phase(X0, t0, phase, bb_dir, event_fn, dt=0.1, t_max=800.0):
    X = np.array(X0, dtype=float)
    t = t0
    rows = []
    prev = event_fn(t, X)
    while t < t0 + t_max:
        dX, aux = booster_deriv(t, X, phase, bb_dir)
        rows.append((t, X.copy(), aux, phase))
        k1 = booster_deriv(t, X, phase, bb_dir)[0]
        k2 = booster_deriv(t + dt / 2, X + dt * k1 / 2, phase, bb_dir)[0]
        k3 = booster_deriv(t + dt / 2, X + dt * k2 / 2, phase, bb_dir)[0]
        k4 = booster_deriv(t + dt, X + dt * k3, phase, bb_dir)[0]
        X = X + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
        t += dt
        val = event_fn(t, X)
        if prev is not None and prev < 0 <= val or (isinstance(val, tuple) and val[0]):
            rows.append((t, X.copy(), booster_deriv(t, X, phase, bb_dir)[1], phase))
            return rows, X, t
        if isinstance(val, tuple):
            prev = val[1]
            if val[0]:
                return rows, X, t
        else:
            # sign crossing from negative to positive, or value already tripped
            if val >= 0 and prev < 0:
                rows.append((t, X.copy(), booster_deriv(t, X, phase, bb_dir)[1], phase))
                return rows, X, t
            prev = val
    raise RuntimeError(f"phase {phase} timed out at t={t:.1f}")


def event_flip(bb_dir):
    def ev(t, X):
        err = abs(wrap(bb_dir * math.pi / 2 - X[5]))
        theta_dot = abs(X[6] - X[2] / (RE + X[1]))
        # done when BOTH small: value is positive while still working
        return max(err - math.radians(2.0), theta_dot - math.radians(0.5))
    return ev


def event_boost(bb_dir):
    last = {"t": -1e9, "x": 0.0}

    def ev(t, X):
        if t - last["t"] > 0.4:
            last["x"] = predict_impact(X)
            last["t"] = t
        # stop when (impact - pad)*bbDir crosses up through 0, or propellant reserve
        m_stop = M1_DRY + 40e3
        crossed = (last["x"] - X_PAD) * bb_dir
        if X[4] <= m_stop:
            return (True, crossed)
        return crossed
    return ev


def event_coast():
    def ev(t, X):
        h, vx, vh, m = X[1], X[2], X[3], X[4]
        if vh < 0 and h < IGN_ALT and h > H_LAND:
            a_cmd = landing_accel(X)
            v1 = m * np.linalg.norm(a_cmd) - IGN_FRAC * burn_capacity("brake", m)
        else:
            v1 = -1.0
        if h <= H_LAND:
            return (True, v1)
        return v1
    return ev


def event_brake():
    def ev(t, X):
        m = X[4]
        a_cmd = landing_accel(X)
        v1 = m * np.linalg.norm(a_cmd) - BRAKE_END_FRAC * N_LAND * T_ENG
        if X[1] <= H_LAND:
            return (True, v1)
        # end brake when v1 crosses down through 0 (landing engines can finish)
        return -v1 if v1 > 0 else v1  # positive while brake should continue? 
    return ev


def fly_return(X0, t0):
    # bbDir from the unpowered impact at separation
    x_imp = predict_impact(X0)
    bb = -math.copysign(1.0, x_imp - X_PAD) if x_imp != X_PAD else -1.0
    print(f"separation impact {x_imp/1e3:.1f} km, bbDir {bb:.0f}")

    def until_negative(ev):
        def wrapped(t, X):
            val = ev(t, X)
            if isinstance(val, tuple):
                return val
            # phase ends when the positive "not done" measure becomes <= 0
            return (val <= 0, val)
        return wrapped

    rows = []
    # flip: event value starts positive and falls through zero
    r, X, t = rk4_until(X0, t0, "flip", bb, lambda X: event_flip(bb)(0.0, X) <= 0, dt=0.05)
    rows += r
    print(f"flip done t={t:.1f} h={X[1]/1e3:.2f} km m={X[4]/1e3:.1f} t")
    r, X, t = rk4_until(X, t, "boostback", bb, lambda X: _boost_done(X, bb), dt=0.25)
    rows += r
    print(f"boostback done t={t:.1f} h={X[1]/1e3:.2f} km vx={X[2]:.0f} m={X[4]/1e3:.1f} t")
    r, X, t = rk4_until(X, t, "coast", bb, lambda X: _coast_done(X), dt=0.25)
    rows += r
    print(f"coast done t={t:.1f} h={X[1]/1e3:.2f} km v={math.hypot(X[2], X[3]):.0f} m={X[4]/1e3:.1f} t")
    r, X, t = rk4_until(X, t, "brake", bb, lambda X: _brake_done(X), dt=0.05)
    rows += r
    print(f"brake done t={t:.1f} h={X[1]:.0f} m v={math.hypot(X[2], X[3]):.0f} m={X[4]/1e3:.1f} t")
    r, X, t = rk4_until(X, t, "landing", bb, lambda X: X[1] <= H_LAND, dt=0.02)
    rows += r
    print(f"landing done t={t:.1f} h={X[1]:.1f} m m={X[4]/1e3:.1f} t")
    return rows


_boost_cache = {"t": -1e9, "x": 0.0}


def _boost_done(X, bb):
    # Use a fresh impact prediction; called once per accepted step.
    x_imp = predict_impact(X)
    if (x_imp - X_PAD) * bb >= 0:
        return True
    if X[4] <= M1_DRY + 40e3:
        return True
    return False


def _coast_done(X):
    h, vh, m = X[1], X[3], X[4]
    if h <= H_LAND:
        return True
    if vh < 0 and h < IGN_ALT:
        a_cmd = landing_accel(X)
        if m * np.linalg.norm(a_cmd) >= IGN_FRAC * burn_capacity("brake", m):
            return True
    return False


def _brake_done(X):
    if X[1] <= H_LAND:
        return True
    a_cmd = landing_accel(X)
    if X[4] * np.linalg.norm(a_cmd) <= BRAKE_END_FRAC * N_LAND * T_ENG:
        return True
    return False


def rk4_until(X0, t0, phase, bb, done, dt):
    X = np.array(X0, dtype=float)
    t = float(t0)
    rows = []
    n = 0
    while n < 200000:
        dX, aux = booster_deriv(t, X, phase, bb)
        rows.append((t, X.copy(), aux, phase))
        if done(X) and n > 0:
            return rows, X, t
        k1 = dX
        k2 = booster_deriv(t + dt / 2, X + dt * k1 / 2, phase, bb)[0]
        k3 = booster_deriv(t + dt / 2, X + dt * k2 / 2, phase, bb)[0]
        k4 = booster_deriv(t + dt, X + dt * k3, phase, bb)[0]
        Xn = X + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
        # don't fly through the ground
        if Xn[1] < H_LAND and phase in ("coast", "brake", "landing"):
            Xn[1] = H_LAND
        X = Xn
        t += dt
        n += 1
        if t > t0 + 2000:
            break
    raise RuntimeError(f"{phase} did not end (t={t:.1f}, h={X[1]:.0f})")


def plant_row(t, h, v, M, q, mass, throttle, n_eng, thrust, length, sm, phase, config):
    arm = length / 2.0
    cg = length / 2.0  # from the nose; companion uniform rod
    I = mass * length ** 2 / 12.0
    lcp = -sm * DIAM  # >0 means CP ahead of CG
    cp = cg - lcp
    mu = thrust * arm / I if I > 0 else 0.0
    lam = q * CNA * A_REF * lcp / I if I > 0 else 0.0
    tvc = 1 if thrust > 1.0 else 0
    return {
        "t_s": t,
        "phase": phase,
        "config": config,
        "altitude_m": h,
        "speed_mps": v,
        "mach": M,
        "q_Pa": q,
        "mass_kg": mass,
        "throttle": throttle,
        "n_eng": n_eng,
        "thrust_N": thrust,
        "length_m": length,
        "cg_from_nose_m": cg,
        "arm_m": arm,
        "cp_from_nose_m": cp,
        "lcp_m": lcp,
        "inertia_kgm2": I,
        "static_margin_cal": sm,
        "mu_rps2": mu,
        "lambda_rps2": lam,
        "tvc_active": tvc,
    }


def main():
    ascent = rk4_ascent()
    t_sep, Y_sep, _ = ascent[-1]
    m_ship = M2_DRY + M2_PROP + MPAY
    m_boost = Y_sep[4] - m_ship
    print(f"hot staging t={t_sep:.2f}s h={Y_sep[1]/1e3:.2f} km v={math.hypot(Y_sep[2], Y_sep[3]):.0f} "
          f"m_stack={Y_sep[4]/1e3:.1f} t m_boost={m_boost/1e3:.1f} t")
    i_q = int(np.argmax([a["q"] for _, _, a in ascent]))
    tq, Yq, aq = ascent[i_q]
    print(f"max-q t={tq:.2f}s h={Yq[1]/1e3:.2f} km q={aq['q']/1e3:.2f} kPa M={aq['M']:.2f} thr={aq['throttle']:.2f}")

    X0 = np.array([
        Y_sep[0], Y_sep[1], Y_sep[2], Y_sep[3], m_boost,
        math.atan2(Y_sep[2], Y_sep[3]),
        Y_sep[2] / (RE + Y_sep[1]),
    ])
    ret = fly_return(X0, t_sep)

    # 1 Hz samples, keeping phase boundaries
    samples = []
    next_t = 0.0
    last_phase = None
    for t, Y, aux in [(t, Y, a) for t, Y, a in [(r[0], r[1], r[2]) for r in []]]:
        pass
    series = []
    for t, Y, aux in [(r[0], r[1], r[2]) for r in ascent]:
        series.append(("ascent", "stack", t, Y[1], aux["v"], aux["M"], aux["q"], Y[4],
                       aux["throttle"], N_ASCENT, aux["F"], L_STACK, SM_STACK))
    for t, X, aux, phase in ret:
        series.append((phase, "booster", t, X[1], aux["v"], aux["M"], aux["q"], X[4],
                       aux["throttle"], aux["n_eng"], aux["F"], L_BOOST, SM_BOOSTER))

    kept = []
    next_emit = -1.0
    prev_phase = None
    for rec in series:
        phase = rec[0]
        t = rec[2]
        if phase != prev_phase or t + 1e-9 >= next_emit:
            kept.append(rec)
            next_emit = math.floor(t) + 1.0
            prev_phase = phase

    root = Path(__file__).resolve().parents[1]
    out = root / "data" / "superheavy_booster_trajectory.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    fields = list(plant_row(0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, "ascent", "stack").keys())
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for rec in kept:
            phase, config, t, h, v, M, q, mass, thr, n_eng, thrust, length, sm = rec
            w.writerow(plant_row(t, h, v, M, q, mass, thr, n_eng, thrust, length, sm, phase, config))
    print(f"wrote {out} ({len(kept)} rows)")


if __name__ == "__main__":
    main()
