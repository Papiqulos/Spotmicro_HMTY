"""Steady-state metrics of one or more runs, for tuning and Chapter 7.

Usage (from the repo root):
    python -m log.metrics                         # latest log/pid/pid_*.csv
    python -m log.metrics log/pid/pid_A.csv ...   # given runs, plus mean/std across them
    python -m log.metrics --no-notes ...          # do not append to tuning_notes.md
"""
import argparse
import json
from pathlib import Path
import numpy as np
import pandas as pd
from tools.pid_controller import PIDController

_ROOT = Path(__file__).parent.parent
_PID_DIR = _ROOT / "log" / "pid"
_NOTES = _ROOT / "tuning_notes.md"
_MAX_INTEGRAL = PIDController(0, 0, 0).max_integral
_RAMP_DURATION = 0.5
_BANDS = [(0.0, 5.0), (5.0, 15.0), (15.0, np.inf)]


def load_meta(csv_path):
    path = Path(str(csv_path).replace(".csv", ".json"))
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}


def steady_mask(df, meta):
    params = meta.get("params", {})
    lin = params.get("desired_lin_vel", df["eff_lin"].iloc[df["eff_lin"].abs().argmax()])
    ang = params.get("desired_ang_vel", df["eff_ang"].iloc[df["eff_ang"].abs().argmax()])
    return (np.isclose(df["eff_lin"], lin, atol=1e-6) &
            np.isclose(df["eff_ang"], ang, atol=1e-6)), lin, ang


def stats(x):
    x = np.degrees(np.asarray(x, dtype=float))
    return {"mean": x.mean(), "std": x.std(), "rms": np.sqrt(np.mean(x**2)), "max": np.abs(x).max()}


def band_power(t, x, bands=_BANDS):
    """Variance (deg^2) of x per frequency band, after resampling to the median sample period.

    Linear resampling attenuates the upper bands (about -12% power at 10 Hz), so compare runs
    against each other rather than reading the values as absolute.
    """
    dt = np.median(np.diff(t))
    tu = np.arange(t[0], t[-1], dt)
    xu = np.degrees(np.interp(tu, t, x))
    xu -= xu.mean()
    n = len(xu)
    X = np.fft.rfft(xu)
    f = np.fft.rfftfreq(n, dt)
    P = np.abs(X)**2 / n**2
    P[1:(n + 1) // 2] *= 2
    return [P[(f > lo) & (f <= hi)].sum() for lo, hi in bands], 0.5 / dt


def analyse(csv_path):
    df = pd.read_csv(csv_path)
    meta = load_meta(csv_path)
    # PyBullet runs: wall clock is not simulated time, use the PID dt instead
    t = df["dt"].cumsum().to_numpy() if meta.get("source") == "sim" else df["t"].to_numpy()
    mask, lin, ang = steady_mask(df, meta)
    if not mask.any():
        raise ValueError(f"{csv_path}: no samples at the nominal velocity")
    s = df[mask]
    ts = t[mask]
    T_c = ts[-1] - ts[0]

    loop = np.diff(df["t"].to_numpy())
    r = {
        "run": Path(csv_path).stem,
        "tag": meta.get("tag"),
        "meta": meta,
        "lin": lin, "ang": ang,
        "T_c": T_c,
        "n": int(mask.sum()),
        "roll": stats(s["roll_meas"]),
        "pitch": stats(s["imu_pitch"]),
        "loop_median": np.median(loop) * 1000,
        "loop_max": loop.max() * 1000,
        "loop_gaps": int((loop > 0.05).sum()),
        "ik_clamped": int(df["ik_clamped"].iloc[-1] - df["ik_clamped"].iloc[0]),
        "dist": abs(lin) * (T_c + _RAMP_DURATION),
        "angle": np.degrees(abs(ang) * (T_c + _RAMP_DURATION)),
    }
    if (s["banked_roll"] != 0).any():
        r["roll_err"] = stats(s["imu_roll"])
    for axis in ("roll", "pitch"):
        r[f"terms_{axis}"] = {k: np.degrees(np.sqrt(np.mean(s[f"{k}_{axis}"]**2))) for k in "pid"}
        ki = meta.get(f"pid_{axis}", {}).get("ki", 0.0)
        r[f"i_sat_{axis}"] = (np.mean(np.abs(s[f"i_{axis}"]) >= ki * _MAX_INTEGRAL - 1e-6)
                              if ki > 0 else 0.0)

    imu_log = meta.get("imu_log")
    if imu_log and Path(imu_log).exists():
        imu = pd.read_csv(imu_log)
        imu = imu[(imu["t"] >= ts[0]) & (imu["t"] <= ts[-1])]
        if len(imu) > 16:
            ti = imu["t"].to_numpy()
            r["bands"] = {}
            for col in ("imu_roll", "lpf_roll", "imu_pitch", "lpf_pitch"):
                r["bands"][col], r["fs_half"] = band_power(ti, imu[col].to_numpy())
    return r


def fmt_stats(name, st):
    return (f"  {name:<9} mean {st['mean']:+6.2f}  std {st['std']:5.2f}  "
            f"RMS {st['rms']:5.2f}  max {st['max']:5.2f}  (deg)")


def report(r):
    m = r["meta"]
    print(f"\n{r['run']}" + (f"  [{r['tag']}]" if r["tag"] else ""))
    if m:
        p = m.get("params", {})
        print(f"  cmd       v={p.get('desired_lin_vel')} w={p.get('desired_ang_vel')} dir={p.get('dir')}"
              f"  steps={m.get('steps')}  fallen={m.get('fallen')}  commit={m.get('git_commit')}")
        for axis in ("roll", "pitch"):
            g = m.get(f"pid_{axis}", {})
            print(f"  {axis:<9} kp={g.get('kp')} ki={g.get('ki')} kd={g.get('kd')}")
    print(f"  steady    T_c {r['T_c']:.2f} s, {r['n']} samples")
    print(fmt_stats("roll", r["roll"]))
    if "roll_err" in r:
        print(fmt_stats("roll err", r["roll_err"]))
    print(fmt_stats("pitch", r["pitch"]))
    for axis in ("roll", "pitch"):
        tm = r[f"terms_{axis}"]
        print(f"  PID {axis:<5} RMS P {tm['p']:.2f}  I {tm['i']:.2f}  D {tm['d']:.2f} (deg)"
              f"  I saturated {100 * r[f'i_sat_{axis}']:.0f}%")
    print(f"  loop      median {r['loop_median']:.1f} ms  max {r['loop_max']:.1f} ms  gaps>50ms {r['loop_gaps']}")
    print(f"  IK clamp  {r['ik_clamped']}")
    if r["lin"]:
        print(f"  expected  distance {r['dist']:.3f} m")
    if r["ang"]:
        print(f"  expected  turn {r['angle']:.0f} deg")
    if "bands" in r:
        hdr = "  ".join(f"{lo:g}-{hi if np.isfinite(hi) else r['fs_half']:.0f} Hz" for lo, hi in _BANDS)
        print(f"  power     (deg^2)  {hdr}")
        for col, pw in r["bands"].items():
            print(f"    {col:<10} " + "  ".join(f"{v:9.4f}" for v in pw))


def summary(results):
    print(f"\nMean +/- std over {len(results)} runs")
    for key in ("roll", "pitch", "roll_err"):
        vals = [r[key] for r in results if key in r]
        if len(vals) != len(results):
            continue
        for k in ("mean", "rms", "max"):
            x = np.array([v[k] for v in vals])
            print(f"  {key:<9} {k:<4} {x.mean():6.2f} +/- {x.std(ddof=1):5.2f} deg")


def append_notes(r, path=_NOTES):
    text = path.read_text(encoding="utf-8") if path.exists() else ""
    if r["run"] in text:
        return
    if not text:
        text = ("| run | tag | cmd (v, w, dir) | roll kp/ki/kd | pitch kp/ki/kd | RMS roll | RMS pitch "
                "| max roll | max pitch | T_c (s) | fallen | observation |\n"
                "|---|---|---|---|---|---|---|---|---|---|---|---|\n")
    m = r["meta"]
    p = m.get("params", {})
    g = lambda a: "/".join(str(m.get(f"pid_{a}", {}).get(k, "?")) for k in ("kp", "ki", "kd"))
    text += (f"| {r['run']} | {r['tag'] or ''} | {p.get('desired_lin_vel')}, {p.get('desired_ang_vel')}, "
             f"{p.get('dir')} | {g('roll')} | {g('pitch')} | {r['roll']['rms']:.2f} | {r['pitch']['rms']:.2f} "
             f"| {r['roll']['max']:.2f} | {r['pitch']['max']:.2f} | {r['T_c']:.1f} | {m.get('fallen', '')} |  |\n")
    path.write_text(text, encoding="utf-8")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="*")
    ap.add_argument("--no-notes", action="store_true")
    args = ap.parse_args()

    files = args.csv or [max(_PID_DIR.glob("pid_*.csv"), key=lambda f: f.stat().st_mtime)]
    results = [analyse(f) for f in files]
    for r in results:
        report(r)
        if not args.no_notes:
            append_notes(r)
    if len(results) > 1:
        summary(results)
