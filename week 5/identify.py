"""
Lab 5 - motor parameter identification, offline analysis.

Reads motor_data/wheel_<N>_v_<PWM>.txt, fits the first-order model

    theta(t) = w_ss * (t - T_m + T_m * exp(-t / T_m))

for every wheel and PWM level, fits w_ss(U) to get k_e and the dead zone,
simulates the identified motor with odeint and writes report.md + figures.

Usage (locally or in Colab: upload the folder, then `!python identify.py`):

    python identify.py                              # reads ./motor_data
    python identify.py --data motor_data --step-wheel 1 --step-pwm 50
    python identify.py --load "wheels in the air"   # what you measured
    python identify.py --data motor_data.txt --battery 11.1   # log from the block diagram

The raw files are only read, never modified.
"""
import argparse
import glob
import os
import re

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import optimize
from scipy.integrate import odeint

MOTION_DEG = 15.0     # less total rotation than this in the run = "did not move"
TM_MIN_PWM = 30       # T_m summary uses |PWM| >= this (friction distorts low levels)


# ----------------------------------------------------------------------------- reading
def read_file(path, battery_fallback=None):
    """Return dict with wheel, pwm, battery [V], t [s], theta [rad], raw sample count."""
    head, rows = {}, []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                if ":" in line:
                    k, v = line[1:].split(":", 1)
                    head[k.strip().lower()] = v.strip()
                continue
            parts = line.replace(",", " ").split()
            if len(parts) >= 2:
                rows.append((float(parts[0]), float(parts[1])))

    m = re.search(r"wheel_(\d+)_v_(-?\d+)", os.path.basename(path))
    wheel = int(head.get("wheel_id", m.group(1)))
    pwm = int(float(head.get("pwm_percent", head.get("voltage_percent", m.group(2)))))

    bat = head.get("battery_voltage", head.get("battery_voltage_mv", battery_fallback))
    bat = float(bat) if bat is not None else np.nan
    if bat > 100:                       # value really in mV
        bat /= 1000.0

    # column order from the "# format:" line; lecture default is "time_ms angle_deg"
    fmt = head.get("format", "time_ms angle_deg").lower().split()
    arr = np.array(rows, dtype=float).reshape(-1, 2)
    if fmt and fmt[0].startswith(("position", "angle")):
        theta_deg, t_ms = arr[:, 0], arr[:, 1]
    else:
        t_ms, theta_deg = arr[:, 0], arr[:, 1]

    order = np.argsort(t_ms, kind="stable")
    t_ms, theta_deg = t_ms[order], theta_deg[order]
    t_ms, keep = np.unique(t_ms, return_index=True)       # drop repeated time stamps
    theta_deg = theta_deg[keep]

    return dict(wheel=wheel, pwm=pwm, battery=bat, n_raw=len(rows),
                t=t_ms / 1000.0, theta=np.radians(theta_deg), path=path)


def read_combined_log(path, wheel_cols=("E3", "E4")):
    """Log written by the block-diagram program: one file, one line per sample:
    pwm_percent time_ms angle_E3_deg angle_E4_deg battery_V. Returns runs like read_file."""
    cols, rows = None, []
    for line in open(path):
        line = line.strip()
        if not line:
            continue
        if line.startswith("#"):
            if "format" in line:
                cols = line.split(":", 1)[1].split()
            continue
        rows.append([float(x) for x in line.replace(",", " ").split()])
    if cols is None:
        cols = ["pwm_percent", "time_ms", "angle_E3_deg", "angle_E4_deg", "battery_V"]
    arr = np.array(rows)
    ci = {c: i for i, c in enumerate(cols)}
    # a new run starts whenever the pwm value changes (consecutive blocks)
    pw = arr[:, ci["pwm_percent"]]
    starts = [0] + [i for i in range(1, len(pw)) if pw[i] != pw[i - 1]] + [len(pw)]
    runs = []
    for a0, a1 in zip(starts[:-1], starts[1:]):
        blk = arr[a0:a1]
        t_ms = blk[:, ci["time_ms"]]
        t_ms, keep = np.unique(t_ms, return_index=True)
        for wid, enc in enumerate(wheel_cols, start=1):
            th = blk[:, ci["angle_%s_deg" % enc]][keep]
            runs.append(dict(wheel=wid, pwm=int(blk[0, ci["pwm_percent"]]),
                             battery=float(np.median(blk[:, ci["battery_V"]])) if "battery_V" in ci else np.nan,
                             n_raw=len(blk), t=t_ms / 1000.0, theta=np.radians(th), path=path))
    return runs


def load_all(data_dir):
    combined = data_dir if os.path.isfile(data_dir) else os.path.join(data_dir, "motor_data.txt")
    if os.path.isfile(combined):
        return read_combined_log(combined)
    bats = {}
    for p in glob.glob(os.path.join(data_dir, "wheel_*_battery.txt")):
        w = int(re.search(r"wheel_(\d+)_battery", p).group(1))
        txt = open(p).read()
        num = re.findall(r"[-+]?\d*\.\d+|\d+", txt.split(":")[-1])
        if num:
            bats[w] = num[0]
    runs = []
    for p in sorted(glob.glob(os.path.join(data_dir, "wheel_*_v_*.txt"))):
        w = int(re.search(r"wheel_(\d+)_v_", p).group(1))
        runs.append(read_file(p, bats.get(w)))
    return runs


# ----------------------------------------------------------------------------- model
def theta_model(t, w_ss, T_m):
    return w_ss * (t - T_m + T_m * np.exp(-t / T_m))


def motor_ode(state, t, u, T_m, k_e):
    w, _ = state
    return [(-w + u / k_e) / T_m, w]


def tail_slope(t, th, frac=0.3):
    sel = t >= t[-1] - frac * (t[-1] - t[0])
    if sel.sum() < 3:
        sel = slice(-3, None)
    return np.polyfit(t[sel], th[sel], 1)[0]


def fit_run(r):
    t, th = r["t"], r["theta"]
    r["dt_ms"] = float(np.median(np.diff(t)) * 1000) if len(t) > 1 else np.nan
    r["moving"] = len(t) > 5 and abs(np.degrees(th[-1] - th[0])) >= MOTION_DEG
    r["w_tail"] = tail_slope(t, th) if len(t) > 3 else 0.0
    if not r["moving"]:
        r.update(w_ss=0.0, T_m=np.nan, w_err=np.nan, T_err=np.nan, rms_deg=np.nan)
        return r
    th0 = th - th[0]
    try:
        popt, pcov = optimize.curve_fit(theta_model, t, th0, p0=[r["w_tail"], 0.1],
                                        bounds=([-np.inf, 1e-3], [np.inf, 2.0]),
                                        maxfev=20000)
        err = np.sqrt(np.diag(pcov))
        res = th0 - theta_model(t, *popt)
        r.update(w_ss=popt[0], T_m=popt[1], w_err=err[0], T_err=err[1],
                 rms_deg=float(np.degrees(np.sqrt(np.mean(res ** 2)))))
    except RuntimeError:
        r.update(w_ss=r["w_tail"], T_m=np.nan, w_err=np.nan, T_err=np.nan, rms_deg=np.nan)
    return r


def fit_wheel(runs):
    """k_e = slope of U against w_ss through the origin (lecture definition: w_ss = U/k_e),
    fitted on levels with |PWM| >= TM_MIN_PWM so static friction near zero does not bias it.
    Dead zone = the levels where the wheel did not turn at all."""
    runs = sorted(runs, key=lambda r: r["pwm"])
    for r in runs:
        r["U"] = r["pwm"] / 100.0 * r["battery"]
    mov = [r for r in runs if r["moving"]]
    s = np.sign(sum(r["w_ss"] * r["pwm"] for r in mov)) or 1.0   # + if +PWM gives growing angle
    out = dict(sign=s, runs=runs)
    out["battery"] = float(np.nanmean([r["battery"] for r in runs]))
    out["battery_range"] = (float(np.nanmin([r["battery"] for r in runs])),
                            float(np.nanmax([r["battery"] for r in runs])))
    still = [r for r in runs if not r["moving"]]
    out["dz_pwm_pos"] = max([r["pwm"] for r in still if r["pwm"] > 0], default=0)
    out["dz_pwm_neg"] = min([r["pwm"] for r in still if r["pwm"] < 0], default=0)
    out["dz_U_pos"] = max([r["U"] for r in still if r["pwm"] > 0], default=0.0)
    out["dz_U_neg"] = min([r["U"] for r in still if r["pwm"] < 0], default=0.0)
    out["T_m_spread"] = (np.nan, np.nan)
    out["T_m"] = np.nan
    if len(mov) < 3:
        return out

    def ke_origin(sel):
        U = np.array([r["U"] for r in sel]); W = s * np.array([r["w_ss"] for r in sel])
        return float(np.sum(U * U) / np.sum(U * W)) if len(sel) >= 2 else np.nan

    hi = [r for r in mov if abs(r["pwm"]) >= TM_MIN_PWM] or mov
    out["k_e"] = ke_origin(hi)
    out["k_e_fwd"] = ke_origin([r for r in hi if r["pwm"] > 0])
    out["k_e_rev"] = ke_origin([r for r in hi if r["pwm"] < 0])
    out["k_e_all"] = ke_origin(mov)                       # incl. low levels, for comparison
    U = np.array([r["U"] for r in hi]); W = s * np.array([r["w_ss"] for r in hi])
    out["r2"] = 1 - np.sum((W - U / out["k_e"]) ** 2) / np.sum((W - W.mean()) ** 2)
    # local slope dU/dw (ignoring the offset) per direction, shows the curvature
    for name, sel in (("fwd", U > 0), ("rev", U < 0)):
        if sel.sum() >= 2:
            out["k_e_slope_" + name] = 1.0 / np.polyfit(U[sel], W[sel], 1)[0]

    tm = [r["T_m"] for r in hi if np.isfinite(r["T_m"])]
    if tm:
        out["T_m"] = float(np.median(tm))
        out["T_m_spread"] = (float(np.min(tm)), float(np.max(tm)))
    return out


def simulate(t, U, T_m, k_e, U_dz=0.0):
    u_eff = 0.0 if abs(U) <= U_dz else U        # inside the dead zone nothing moves
    y = odeint(motor_ode, [0.0, 0.0], t, args=(u_eff, T_m, k_e))
    return y[:, 1], y[:, 0]


# ----------------------------------------------------------------------------- plots
COLORS = ["#1f6fb4", "#d1495b", "#2a9d5c", "#8a5cc2", "#e0a400"]


def plot_omega_u(wheels, path):
    fig, ax = plt.subplots(figsize=(9, 6), dpi=120)
    dz_lo = min(w.get("dz_U_neg", 0) for w in wheels.values())
    dz_hi = max(w.get("dz_U_pos", 0) for w in wheels.values())
    if dz_hi > 0 or dz_lo < 0:
        ax.axvspan(dz_lo, dz_hi, color="0.85", zorder=0, label="dead zone (no rotation)")
    for i, (wid, w) in enumerate(sorted(wheels.items())):
        c = COLORS[i % len(COLORS)]
        U = np.array([r["U"] for r in w["runs"]])
        W = w["sign"] * np.array([r["w_ss"] for r in w["runs"]])
        mv = np.array([r["moving"] for r in w["runs"]])
        ax.plot(U[mv], W[mv], "o", color=c, ms=5, label="wheel %d measured" % wid)
        ax.plot(U[~mv], W[~mv], "x", color=c, ms=6)
        if np.isfinite(w.get("k_e", np.nan)):
            for sgn, key in ((1, "k_e_fwd"), (-1, "k_e_rev")):
                ke = w.get(key, np.nan)
                if np.isfinite(ke):
                    uu = np.linspace(0, sgn * np.abs(U).max(), 50)
                    ax.plot(uu, uu / ke, "-", color=c, lw=1.4, alpha=0.9)
            ax.plot([], [], "-", color=c, label="wheel %d: ω = U/k_e, k_e fwd %.3f / rev %.3f V·s/rad"
                    % (wid, w.get("k_e_fwd", np.nan), w.get("k_e_rev", np.nan)))
    ax.axhline(0, color="0.5", lw=0.6)
    ax.axvline(0, color="0.5", lw=0.6)
    ax.set_xlabel("U = PWM/100 · U_battery  [V]")
    ax.set_ylabel("ω_ss  [rad/s]  (positive = direction of positive PWM)")
    ax.set_title("Steady-state speed vs voltage")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="upper left")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def plot_step(run, w, path):
    t, th = run["t"], run["theta"] - run["theta"][0]
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 8), dpi=120, sharex=True)
    a1.plot(t, th, "-", color="#d1495b", lw=1.3, label="measured θ(t)")
    tt = np.linspace(0, t[-1], 500)
    if np.isfinite(run["T_m"]):
        a1.plot(tt, theta_model(tt, run["w_ss"], run["T_m"]), ":", color="0.3", lw=1.5,
                label="fit of this run: ω_ss = %.2f rad/s, T_m = %.3f s" % (run["w_ss"], run["T_m"]))
    ke = w.get("k_e_fwd" if run["pwm"] > 0 else "k_e_rev", w["k_e"])
    dz = w["dz_U_pos"] if run["pwm"] > 0 else -w["dz_U_neg"]
    th_sim, w_sim = simulate(tt, run["U"], w["T_m"], ke, dz)
    th_sim, w_sim = w["sign"] * th_sim, w["sign"] * w_sim
    a1.plot(tt, th_sim, "--", color="#1f6fb4", lw=2,
            label="simulated (odeint), identified T_m = %.3f s, k_e = %.3f V·s/rad" % (w["T_m"], ke))
    a1.set_ylabel("θ  [rad]")
    a1.set_title("Wheel %d, PWM %d %% (U = %.2f V)" % (run["wheel"], run["pwm"], run["U"]))
    a1.grid(alpha=0.3)
    a1.legend(fontsize=8)

    # numeric derivative of the measurement vs simulated speed
    if len(t) > 4:
        wm = np.gradient(th, t)
        k = max(1, len(t) // 40)
        wm = np.convolve(wm, np.ones(k) / k, mode="same")
        a2.plot(t, wm, "-", color="#d1495b", lw=1, label="measured (dθ/dt, smoothed)")
    a2.plot(tt, w_sim, "--", color="#1f6fb4", lw=2, label="simulated ω(t)")
    a2.set_xlabel("t  [s]")
    a2.set_ylabel("ω  [rad/s]")
    a2.grid(alpha=0.3)
    a2.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def plot_tm(wheels, path):
    fig, ax = plt.subplots(figsize=(9, 4.5), dpi=120)
    for i, (wid, w) in enumerate(sorted(wheels.items())):
        P = [r["pwm"] for r in w["runs"] if r["moving"]]
        T = [r["T_m"] for r in w["runs"] if r["moving"]]
        ax.plot(P, T, "o-", color=COLORS[i % len(COLORS)], ms=4, label="wheel %d" % wid)
    ax.set_xlabel("PWM [%]")
    ax.set_ylabel("T_m [s]")
    ax.set_title("Fitted time constant per level")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def plot_all_fits(wheels, path):
    runs = [r for w in wheels.values() for r in w["runs"]]
    n = len(runs)
    cols = 8
    rows = int(np.ceil(n / cols))
    fig, axs = plt.subplots(rows, cols, figsize=(cols * 2.2, rows * 1.8), dpi=90)
    for ax, r in zip(np.ravel(axs), runs):
        t, th = r["t"], r["theta"] - r["theta"][0]
        ax.plot(t, th, color="#d1495b", lw=1)
        if np.isfinite(r["T_m"]):
            ax.plot(t, theta_model(t, r["w_ss"], r["T_m"]), "--", color="#1f6fb4", lw=1)
        ax.set_title("W%d %+d%%" % (r["wheel"], r["pwm"]), fontsize=7)
        ax.tick_params(labelsize=5)
    for ax in np.ravel(axs)[n:]:
        ax.axis("off")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


# ----------------------------------------------------------------------------- report
def f(x, d=3):
    return "—" if x is None or not np.isfinite(x) else ("%." + str(d) + "f") % x


def write_report(wheels, args, figs, checks, path):
    ids = sorted(wheels)
    L = []
    L.append("# Lab 5 — Motor parameter identification\n")
    L.append("Robot: TRIK, motors RK370. Measured on: **%s**. Program: `motor_identification.qrs`, "
             "raw data: `motor_data/`, analysis: `identify.py`.\n" % args.load)
    L.append("Model: `T_m·dω/dt + ω = U/k_e`, `U = PWM/100 · U_battery`. Each step response is fitted "
             "with `θ(t) = ω_ss·(t − T_m + T_m·e^(−t/T_m))` (`scipy.optimize.curve_fit`); `k_e` is the "
             "least-squares slope of `ω_ss = U/k_e` through the origin, per direction, over the levels with "
             "|PWM| ≥ %d %% (lower levels are distorted by friction). The dead zone is the set of levels where "
             "the wheel did not turn at all.\n" % TM_MIN_PWM)

    L.append("## Identified parameters\n")
    L.append("| Wheel | k_e [V·s/rad] | k_e fwd / rev | T_m [s] (median, \\|PWM\\| ≥ %d %%) | T_m range | dead zone PWM | battery [V] | encoder sign |"
             % TM_MIN_PWM)
    L.append("|---|---|---|---|---|---|---|---|")
    for wid in ids:
        w = wheels[wid]
        L.append("| %d | %s | %s / %s | %s | %s–%s | %s | %s–%s | %s |" % (
            wid, f(w.get("k_e")), f(w.get("k_e_fwd")), f(w.get("k_e_rev")),
            f(w.get("T_m")), f(w["T_m_spread"][0]), f(w["T_m_spread"][1]),
            dz_text(w),
            f(w["battery_range"][0], 2), f(w["battery_range"][1], 2),
            "+" if w["sign"] > 0 else "− (encoder counts opposite to PWM)"))
    L.append("")
    L.append("For comparison: including the low levels too gives k_e = " +
             ", ".join("%s (wheel %d)" % (f(wheels[w].get("k_e_all")), w) for w in ids) +
             "; the local slope dU/dω_ss above the friction region (fwd / rev) is " +
             ", ".join("%s / %s (wheel %d)" % (f(wheels[w].get("k_e_slope_fwd")), f(wheels[w].get("k_e_slope_rev")), w)
                       for w in ids) +
             ". The gap between these numbers is the nonlinearity of ω_ss(U).\n")

    L.append("## Step-response fits per PWM level\n")
    L.append("`ω_ss` is the raw fitted value (sign as the encoder reports it). “—” = the wheel did not "
             "turn (less than %d° in the run), i.e. dead zone.\n" % MOTION_DEG)
    hdr = "| PWM [%] |" + "".join(" U%d [V] | ω_ss%d [rad/s] | T_m%d [s] |" % (w, w, w) for w in ids)
    L.append(hdr)
    L.append("|---" * (1 + 3 * len(ids)) + "|")
    pwms = sorted({r["pwm"] for w in wheels.values() for r in w["runs"]})
    by = {(r["wheel"], r["pwm"]): r for w in wheels.values() for r in w["runs"]}
    for p in pwms:
        row = "| %d |" % p
        for wid in ids:
            r = by.get((wid, p))
            if r is None:
                row += " | | |"
            elif not r["moving"]:
                row += " %s | 0 | — |" % f(r["U"], 2)
            else:
                row += " %s | %s | %s |" % (f(r["U"], 2), f(r["w_ss"], 2), f(r["T_m"]))
        L.append(row)
    L.append("")

    L.append("## ω_ss against voltage\n")
    L.append("![omega vs U](%s)\n" % figs["omega"])
    L.append("## Step response: measurement vs simulation\n")
    L.append("![step response](%s)\n" % figs["step"])
    L.append("Time constant per level:\n\n![T_m](%s)\n" % figs["tm"])
    L.append("All fits (sanity check):\n\n![all fits](%s)\n" % figs["all"])

    L.append("## Discussion\n")
    L.append("<!-- DRAFT generated from the numbers above. Read it against your plots and rewrite in your own words. -->\n")
    L.append(draft_discussion(wheels, args) + "\n")

    L.append("## Data checks\n")
    for c in checks:
        L.append("- " + c)
    L.append("")
    open(path, "w").write("\n".join(L))


def dz_text(w):
    lo, hi = w.get("dz_pwm_neg", 0), w.get("dz_pwm_pos", 0)
    if lo == 0 and hi == 0:
        return "none at ±5 %"
    return "%d … %d %%" % (lo, hi)


def draft_discussion(wheels, args):
    ids = sorted(wheels)
    s = []
    dz = ", ".join("wheel %d: %s" % (w, dz_text(wheels[w])) for w in ids)
    s.append("The linear model fails first around zero: in the dead zone (%s) static friction holds the wheel, "
             "although the model predicts ω_ss = U/k_e for any U ≠ 0; just above it the speed jumps up quickly, "
             "so ω_ss(U) is not one straight line through the origin — its slope above the friction region "
             "differs from the through-origin k_e." % dz)
    tms = [wheels[w]["T_m_spread"] for w in ids if np.isfinite(wheels[w]["T_m_spread"][0])]
    if tms:
        s.append("T_m is not a constant either: for |PWM| ≥ %d %% it ranges from %.3f to %.3f s, and near the "
                 "dead zone the fits get worse because friction, not the electrical drive, dominates the start."
                 % (TM_MIN_PWM, min(a for a, b in tms), max(b for a, b in tms)))
    asym = []
    for w in ids:
        kf, kr = wheels[w].get("k_e_fwd"), wheels[w].get("k_e_rev")
        if kf and kr:
            asym.append("wheel %d: %.3f fwd vs %.3f rev (%.0f %%)" % (w, kf, kr, 100 * abs(kf - kr) / ((kf + kr) / 2)))
    if asym:
        s.append("Forward and reverse are not identical (k_e %s)." % "; ".join(asym))
    if len(ids) >= 2:
        a, b = wheels[ids[0]], wheels[ids[1]]
        ra = {r["pwm"]: abs(r["w_ss"]) for r in a["runs"] if r["moving"] and abs(r["pwm"]) >= TM_MIN_PWM}
        rb = {r["pwm"]: abs(r["w_ss"]) for r in b["runs"] if r["moving"] and abs(r["pwm"]) >= TM_MIN_PWM}
        common = sorted(set(ra) & set(rb))
        if common:
            d = np.array([(ra[p] - rb[p]) / ((ra[p] + rb[p]) / 2) for p in common]) * 100
            s.append("At the same PWM wheel %d turns on average %.1f %% %s than wheel %d (range %.1f to %.1f %%), "
                     "with k_e = %.3f vs %.3f V·s/rad." % (
                         ids[0], abs(d.mean()), "faster" if d.mean() > 0 else "slower", ids[1],
                         d.min(), d.max(), a.get("k_e", np.nan), b.get("k_e", np.nan)))
            txt = ("Next week this means that sending the same power to both motors will not drive straight — "
                   "the robot will curve toward the slower wheel — and if odometry assumes the commanded speed "
                   "instead of reading the encoders, the heading error grows with distance; we must use the "
                   "per-wheel k_e, the dead zone and the encoder readings, not one shared model.")
            if args.wheel_radius and args.track:
                wmean = np.mean([(ra[p] + rb[p]) / 2 for p in common])
                dw = np.mean([ra[p] - rb[p] for p in common])
                rate = np.degrees(args.wheel_radius * dw / args.track)
                txt += (" With r = %.3f m and track L = %.3f m, an open-loop straight command would turn at "
                        "about %.1f °/s (≈ %.0f° per metre driven)." % (
                            args.wheel_radius, args.track, rate,
                            rate / (args.wheel_radius * wmean) if wmean else np.nan))
            s.append(txt)
    return " ".join(s)


# ----------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="motor_data")
    ap.add_argument("--out", default=".")
    ap.add_argument("--wheels", default="", help="comma list of wheel ids to use, default all")
    ap.add_argument("--step-wheel", type=int, default=None)
    ap.add_argument("--step-pwm", type=int, default=50)
    ap.add_argument("--load", default="wheels in the air (robot on a stand)")
    ap.add_argument("--wheel-radius", type=float, default=None, help="m, optional, for odometry estimate")
    ap.add_argument("--battery", type=float, default=None,
                    help="V, used where the files have no battery value (block-diagram version)")
    ap.add_argument("--track", type=float, default=None, help="m, distance between wheels, optional")
    args = ap.parse_args()

    runs = load_all(args.data)
    if not runs:
        raise SystemExit("no data in %s (expected motor_data.txt or wheel_*_v_*.txt)" % args.data)
    bad = [r for r in runs if not (r["battery"] > 1)]
    if bad and args.battery is None:
        raise SystemExit("battery voltage missing or 0 in the data - pass it with --battery 11.1 "
                         "(the value you read on the robot before the run)")
    if args.battery is not None:
        for r in bad:
            r["battery"] = args.battery
    if args.wheels:
        keep = {int(x) for x in args.wheels.split(",")}
        runs = [r for r in runs if r["wheel"] in keep]

    checks = []
    for r in runs:
        fit_run(r)
        if r["moving"] and r["t"][-1] > 0 and np.isfinite(r["T_m"]) and r["T_m"] > r["t"][-1] / 3:
            checks.append("wheel %d PWM %d: T_m = %.2f s is large vs the %.1f s record — curve may not reach "
                          "steady state, consider longer RUN_TIME." % (r["wheel"], r["pwm"], r["T_m"], r["t"][-1]))
        if np.isnan(r["battery"]):
            checks.append("wheel %d PWM %d: no battery voltage in file." % (r["wheel"], r["pwm"]))

    wheels = {}
    for wid in sorted({r["wheel"] for r in runs}):
        wheels[wid] = fit_wheel([r for r in runs if r["wheel"] == wid])
        if wheels[wid]["sign"] < 0:
            checks.append("wheel %d: positive PWM gives a decreasing angle (encoder/motor direction differ). "
                          "Data left as recorded; the sign is corrected only for the ω(U) plot and k_e fit." % wid)
    nsamp = [r["n_raw"] for r in runs]
    dts = [r["dt_ms"] for r in runs if np.isfinite(r["dt_ms"])]
    checks.insert(0, "%d runs, %d–%d samples per run, median sample period %.1f ms%s."
                  % (len(runs), min(nsamp), max(nsamp), np.median(dts),
                     " — much slower than the 5 ms target, so the first ~%d ms of each step are poorly resolved "
                     "and T_m is uncertain" % (2 * np.median(dts)) if np.median(dts) > 10 else ""))

    os.makedirs(os.path.join(args.out, "figures"), exist_ok=True)
    figs = {k: "figures/%s.png" % k for k in ("omega", "step", "tm", "all")}
    plot_omega_u(wheels, os.path.join(args.out, figs["omega"]))
    plot_tm(wheels, os.path.join(args.out, figs["tm"]))
    plot_all_fits(wheels, os.path.join(args.out, figs["all"]))

    sw = args.step_wheel if args.step_wheel in wheels else sorted(wheels)[0]
    cand = [r for r in wheels[sw]["runs"] if r["moving"]]
    step_run = min(cand, key=lambda r: abs(r["pwm"] - args.step_pwm))
    plot_step(step_run, wheels[sw], os.path.join(args.out, figs["step"]))

    with open(os.path.join(args.out, "fit_results.csv"), "w") as fh:
        fh.write("wheel,pwm,battery_V,U_V,moving,w_ss_rad_s,w_ss_std,T_m_s,T_m_std,rms_deg,samples,dt_ms\n")
        for wid in sorted(wheels):
            for r in wheels[wid]["runs"]:
                fh.write("%d,%d,%.4f,%.4f,%d,%.5f,%.5f,%.5f,%.5f,%.3f,%d,%.2f\n" % (
                    wid, r["pwm"], r["battery"], r["U"], r["moving"], r["w_ss"], r["w_err"],
                    r["T_m"], r["T_err"], r["rms_deg"], r["n_raw"], r["dt_ms"]))

    write_report(wheels, args, figs, checks, os.path.join(args.out, "report.md"))

    for wid in sorted(wheels):
        w = wheels[wid]
        print("wheel %d: k_e = %s (fwd %s / rev %s) V·s/rad, T_m = %s s, R² = %s, sign %+d" % (
            wid, f(w.get("k_e")), f(w.get("k_e_fwd")), f(w.get("k_e_rev")), f(w.get("T_m")), f(w.get("r2")), w["sign"]))
    for c in checks[:12]:
        print(" *", c)
    print("wrote report.md, fit_results.csv, figures/")


if __name__ == "__main__":
    main()
