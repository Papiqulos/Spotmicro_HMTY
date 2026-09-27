"""Collect the final runs into thesis_export/ (figures, LaTeX tables, summary, data).

    python -m log.export_thesis           # robot runs (hw/run_test.py)
    python -m log.export_thesis --sim     # simulation runs (sim/run_sim_test.py)

Final runs are found by their tag (F, F-OFF, B, R, L, TR, TW with _1.._3 on the robot, _1 in
simulation); if a tag was run more than once the latest run is used. Statistics cover the
constant-velocity segment: on the robot from the IMU angles before the low-pass filter, in
simulation from the base orientation logged by the controller.
"""
import argparse
import json
import shutil
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from log.metrics import steady_mask

_ROOT = Path(__file__).parent.parent
_PID_DIR = _ROOT / "log" / "pid"
_OUT = _ROOT / "thesis_export"
_RAMP = 0.5

# tag prefix -> (name in the thesis, figure suffix)
TESTS = {
    "F":     ("Ευθεία εμπρός",        "forward"),
    "B":     ("Ευθεία πίσω",          "backwards"),
    "R":     ("Πλάγια δεξιά",         "right"),
    "L":     ("Πλάγια αριστερά",      "left"),
    "TR":    ("Στροφή επί τόπου",     "turn"),
    "TW":    ("Στροφή εν κινήσει",    "combo"),
    "F-OFF": ("Εμπρός χωρίς ελεγκτή", "nothing"),
}
TAPE_TESTS = ("F", "F-OFF", "B", "R", "L", "TR")

SOURCES = {
    "robot": dict(reps=(1, 2, 3), fig="log_plot_real_{}.png", pitch_fig="pitch_off_vs_pi.png",
                  tables="final_results.tex", summary="FINAL_RESULTS.md", data="data/final", label="final",
                  title="Τελικές μετρήσεις", where="στις τελικές δοκιμές",
                  signal="από το σήμα της αδρανειακής μονάδας πριν από το βαθυπερατό φίλτρο",
                  measured="Μετρημένη"),
    "sim": dict(reps=(1,), fig="log_plot_sim_{}.png", pitch_fig="pitch_off_vs_pi_sim.png",
                tables="sim_results.tex", summary="FINAL_RESULTS_SIM.md", data="data/sim", label="sim",
                title="Προσομοίωση", where="στην προσομοίωση",
                signal="από τον προσανατολισμό της βάσης στην προσομοίωση",
                measured="Προσομοίωση"),
}
# the existing simulation figure is named log_plot_sim_backward.png in the thesis
FIG_NAME = {("sim", "backwards"): "backward"}


def latest_runs(source):
    """tag -> (csv path, meta) for the latest run of every tag."""
    runs = {}
    for js in sorted(_PID_DIR.glob("pid_*.json")):
        m = json.loads(js.read_text())
        if m.get("source") == source and m.get("tag"):
            runs[m["tag"]] = (js.with_suffix(".csv"), m)
    return runs


def angle_series(csv, m, source):
    """(t, roll, pitch) in s and degrees, over the whole run."""
    df = pd.read_csv(csv)
    if source == "sim":
        t = df["dt"].cumsum().to_numpy()
        return t, np.degrees(df.roll_meas.to_numpy()), np.degrees(df.imu_pitch.to_numpy())
    imu = pd.read_csv(m["imu_log"])
    return imu.t.to_numpy(), np.degrees(imu.imu_roll.to_numpy()), np.degrees(imu.imu_pitch.to_numpy())


def run_stats(csv, m, source):
    df = pd.read_csv(csv)
    mask, lin, ang = steady_mask(df, m)
    t = df["dt"].cumsum().to_numpy() if source == "sim" else df["t"].to_numpy()
    ts = t[mask]
    ta, r, p = angle_series(csv, m, source)
    sel = (ta >= ts[0]) & (ta <= ts[-1])
    r, p = r[sel], p[sel]
    st = lambda x: {"mean": x.mean(), "std": x.std(), "rms": np.sqrt(np.mean(x**2)), "max": np.abs(x).max()}
    T_c = ts[-1] - ts[0]
    loop = np.diff(df["t"].to_numpy()) * 1000
    return {"roll": st(r), "pitch": st(p), "T_c": T_c, "lin": lin, "ang": ang,
            "dist": abs(lin) * (T_c + _RAMP), "turn": np.degrees(abs(ang) * (T_c + _RAMP)),
            "loop_median": np.median(loop), "loop_max": loop.max(), "gaps": int((loop > 50).sum()),
            "fallen": bool(m.get("fallen")), "ik": m.get("ik_clamped", 0), "tape": m.get("tape", {})}


def plot_run(csv, out, source, ylim=(-9, 9)):
    # same layout as log/pid_plotter.py, wider scale so a PID OFF run is not clipped
    fig, axes = plt.subplots(2, 1, figsize=(10, 8))
    df = pd.read_csv(csv)
    df["t"] = df["dt"].cumsum() if source == "sim" else df["t"] - df["t"].iloc[0]
    for c in ("imu_roll", "pid_roll", "imu_pitch", "pid_pitch"):
        df[c] = np.degrees(df[c])
    df.plot(x="t", y=["imu_roll", "pid_roll"], ylim=ylim, ylabel="Angle (deg)", ax=axes[0], grid=True)
    df.plot(x="t", y=["imu_pitch", "pid_pitch"], ylim=ylim, ylabel="Angle (deg)", ax=axes[1], grid=True)
    for ax in axes:
        ax.set_xlabel("t (s)")
    fig.savefig(out)
    plt.close(fig)


def plot_pitch_on_off(on, off, source, out):
    BLUE, ORANGE, INK, INK2 = "#2a78d6", "#eb6834", "#0b0b0b", "#52514e"
    with plt.rc_context({"axes.spines.top": False, "axes.spines.right": False, "axes.grid": True,
                         "grid.color": "#e4e3df", "axes.edgecolor": INK2}):
        fig, ax = plt.subplots(figsize=(8, 3.4))
        t_end = 0
        for (csv, m), color, label in ((off, ORANGE, "χωρίς σταθεροποίηση"), (on, BLUE, "με σταθεροποίηση")):
            t, _, p = angle_series(csv, m, source)
            t = t - t[0]
            n = max(1, int(round(0.46 / np.median(np.diff(t)))))
            avg = pd.Series(p).rolling(n, center=True).mean()
            ax.plot(t, p, color=color, lw=0.8, alpha=0.35)
            ax.plot(t, avg, color=color, lw=2)
            ax.text(t[-1] + 0.15, avg.dropna().iloc[-1], label, color=INK, va="center", fontsize=9)
            t_end = max(t_end, t[-1])
        ax.axhline(0, color=INK2, lw=0.8)
        ax.set_xlabel("t (s)")
        ax.set_ylabel("pitch (°)")
        ax.set_xlim(0, t_end + 3.4)
        ax.set_xticks(range(0, int(t_end) + 1, 2))
        ax.spines["bottom"].set_bounds(0, t_end)
        fig.tight_layout()
        fig.savefig(out, dpi=200)
        plt.close(fig)


def num(x, d=2, sign=False):
    s = f"{x:+.{d}f}" if sign else f"{x:.{d}f}"
    return s.replace(".", "{,}")


def pm(vals, d=2, sign=False):
    v = np.array(vals, dtype=float)
    if len(v) > 1:
        return f"${num(v.mean(), d, sign)} \\pm {num(v.std(ddof=1), d)}$"
    return f"${num(v.mean(), d, sign)}$"


def latex_tables(stats, cfg):
    reps = "των τριών επαναλήψεων κάθε δοκιμής" if len(cfg["reps"]) > 1 else "μία εκτέλεση ανά δοκιμή"
    lines = []
    for axis in ("roll", "pitch"):
        lines += [r"\begin{table}[htbp]", r"    \centering", r"    \small",
                  r"    \begin{tabular}{@{}l c c c c@{}}", r"        \toprule",
                  r"        Δοκιμή & Μέση τιμή & Τυπική απόκλιση & RMS & Μέγιστη $|$τιμή$|$ \\",
                  r"        \midrule"]
        for pre, (label, _) in TESTS.items():
            S = stats.get(pre)
            if not S:
                continue
            cols = [pm([s[axis][k] for s in S], sign=(k == "mean")) for k in ("mean", "std", "rms", "max")]
            lines.append(f"        {label} & " + " & ".join(cols) + r" \\")
        lines += [r"        \bottomrule", r"    \end{tabular}",
                  f"    \\caption[Γωνία {axis} {cfg['where']}]{{Γωνία {axis} (σε μοίρες) {cfg['where']}, στο "
                  f"τμήμα σταθερής ταχύτητας, {reps}}}",
                  f"    \\label{{tab:{cfg['label']}-{axis}}}", r"\end{table}", ""]

    rows = []
    for pre in TAPE_TESTS:
        S = stats.get(pre) or []
        if pre == "TR":
            meas = [s["tape"]["turn_deg"] for s in S if "turn_deg" in s["tape"]]
            theo = np.mean([s["turn"] for s in S]) if S else 0
            if meas and theo > 0:
                rows.append(f"        {TESTS[pre][0]} & {pm(meas, 0)}$^\\circ$ & ${num(theo, 0)}^\\circ$ & "
                            f"${num(np.mean(meas) / theo * 100, 0)}\\%$ & & \\\\")
            continue
        meas = [s["tape"]["distance_m"] for s in S if "distance_m" in s["tape"]]
        theo = np.mean([s["dist"] for s in S]) if S else 0
        if not meas or theo <= 0:
            continue
        lat = [s["tape"]["lateral_cm"] for s in S if "lateral_cm" in s["tape"]]
        hdg = [s["tape"]["heading_deg"] for s in S if "heading_deg" in s["tape"]]
        rows.append(f"        {TESTS[pre][0]} & {pm(meas)}~m & ${num(theo)}$~m & "
                    f"${num(np.mean(meas) / theo * 100, 0)}\\%$ & "
                    f"{pm(lat, 1) + '~cm' if lat else ''} & {pm(hdg, 0, True) + '$^\\circ$' if hdg else ''} \\\\")
    if rows:
        lines += [r"\begin{table}[htbp]", r"    \centering", r"    \small",
                  r"    \begin{tabular}{@{}l c c c c c@{}}", r"        \toprule",
                  f"        Δοκιμή & {cfg['measured']} & Θεωρητική & Λόγος & Πλευρική απόκλιση & Αλλαγή κατεύθυνσης \\\\",
                  r"        \midrule", *rows, r"        \bottomrule", r"    \end{tabular}",
                  f"    \\caption[Διανυθείσα απόσταση και γωνία στροφής {cfg['where']}]{{Διανυθείσα και "
                  r"θεωρητική απόσταση, $d = v\,(T_c + 0{,}5)$, και γωνία στροφής, $\omega\,(T_c + 0{,}5)$, "
                  f"{cfg['where']}, {reps}}}",
                  f"    \\label{{tab:{cfg['label']}-distance}}", r"\end{table}", ""]
    return "\n".join(lines)


def representative(S):
    """Repetition whose combined RMS is the median of the runs."""
    key = [np.hypot(s["roll"]["rms"], s["pitch"]["rms"]) for s in S]
    return S[int(np.argsort(key)[len(key) // 2])]["rep"]


def export(source):
    cfg = SOURCES[source]
    runs = latest_runs(source)
    fig_dir, tab_dir, data_dir = _OUT / "figures", _OUT / "tables", _OUT / cfg["data"]
    for d in (fig_dir, tab_dir, data_dir / "pid", data_dir / "imu"):
        d.mkdir(parents=True, exist_ok=True)

    stats, chosen, missing = {}, {}, []
    for pre in TESTS:
        S = []
        for n in cfg["reps"]:
            tag = f"{pre}_{n}"
            if tag not in runs:
                missing.append(tag)
                continue
            csv, m = runs[tag]
            S.append(dict(run_stats(csv, m, source), rep=n))
            for f in (csv, csv.with_suffix(".json"), csv.with_suffix(".png")):
                if f.exists():
                    shutil.copy(f, data_dir / "pid")
            if m.get("imu_log") and Path(m["imu_log"]).exists():
                shutil.copy(m["imu_log"], data_dir / "imu")
        if not S:
            continue
        stats[pre] = S
        chosen[pre] = representative(S)
        suffix = FIG_NAME.get((source, TESTS[pre][1]), TESTS[pre][1])
        plot_run(runs[f"{pre}_{chosen[pre]}"][0], fig_dir / cfg["fig"].format(suffix), source)

    if "F" in chosen and "F-OFF" in chosen:
        plot_pitch_on_off(runs[f"F_{chosen['F']}"], runs[f"F-OFF_{chosen['F-OFF']}"], source,
                          fig_dir / cfg["pitch_fig"])

    (tab_dir / cfg["tables"]).write_text(latex_tables(stats, cfg))

    R = [f"# {cfg['title']}", "",
         f"Παράγεται από `python -m log.export_thesis{' --sim' if source == 'sim' else ''}`. Γωνίες σε "
         f"μοίρες, {cfg['signal']}, στο τμήμα σταθερής ταχύτητας."
         + (" Μέση τιμή ± τυπική απόκλιση των επαναλήψεων." if len(cfg["reps"]) > 1 else ""), ""]
    if missing:
        R += [f"**Λείπουν:** {', '.join(missing)}", ""]
    f2 = lambda v, sign=False: pm(v, sign=sign).strip("$").replace("{,}", ",").replace("\\pm", "±")
    R += ["| Δοκιμή | Εκτελέσεις | Μέση roll | Τ.α. roll | RMS roll | max roll | Μέση pitch | Τ.α. pitch | RMS pitch | max pitch |",
          "|---|---|---|---|---|---|---|---|---|---|"]
    for pre, S in stats.items():
        R.append(f"| {TESTS[pre][0]} | {len(S)} | " + " | ".join(
            f2([s[a][k] for s in S], sign=(k == "mean")) for a in ("roll", "pitch") for k in ("mean", "std", "rms", "max")) + " |")

    g = lambda v: "" if v is None else f"{round(v, 2):g}".replace(".", ",")
    R += ["", "## Απόσταση και γωνία", "",
          f"| Δοκιμή | Επανάληψη | {cfg['measured']} | Θεωρητική | Πλευρική απόκλιση (cm) | Αλλαγή κατεύθυνσης (deg) |",
          "|---|---|---|---|---|---|"]
    for pre in TAPE_TESTS:
        for s in stats.get(pre, []):
            tp = {k: g(v) for k, v in s["tape"].items()}
            if pre == "TR":
                R.append(f"| {TESTS[pre][0]} | {s['rep']} | {tp.get('turn_deg', '')} deg | {g(round(s['turn']))} deg | | |")
            else:
                R.append(f"| {TESTS[pre][0]} | {s['rep']} | {tp.get('distance_m', '')} m | {g(s['dist'])} m | "
                         f"{tp.get('lateral_cm', '')} | {tp.get('heading_deg', '')} |")
    if source == "sim":
        R += ["", "Στην προσομοίωση η αλλαγή κατεύθυνσης είναι θετική αριστερόστροφα (από πάνω)."]

    all_s = [s for S in stats.values() for s in S]
    checks = [("Καμία διακοπή ασφαλείας", not any(s["fallen"] for s in all_s)),
              ("Τουλάχιστον 10 s σταθερής ταχύτητας", all(s["T_c"] >= 10 for s in all_s)),
              ("Όλες οι εκτελέσεις υπάρχουν", not missing),
              ("Κανένας στόχος εκτός εμβέλειας", all(s["ik"] == 0 for s in all_s))]
    if source == "robot":
        checks = [("Διάμεσο βήμα βρόχου 13 έως 16 ms", all(13 <= s["loop_median"] <= 16 for s in all_s)),
                  ("Κανένα κενό > 50 ms", all(s["gaps"] == 0 for s in all_s))] + checks
    on, off = stats.get("F", []), stats.get("F-OFF", [])
    if on and off:
        rms_on = [s["pitch"]["rms"] for s in on]
        rms_off = [s["pitch"]["rms"] for s in off]
        checks.append(("PID ON: RMS pitch μικρότερο από PID OFF", max(rms_on) < min(rms_off)))
        if len(rms_on) > 1 and len(rms_off) > 1:
            checks.append(("Διασπορά RMS επαναλήψεων μικρότερη από τη διαφορά ON/OFF",
                           max(np.ptp(rms_on), np.ptp(rms_off)) < np.mean(rms_off) - np.mean(rms_on)))
    R += ["", "## Έλεγχος ποιότητας", ""] + [f"- [{'x' if ok else ' '}] {c}" for c, ok in checks]

    R += ["", "## Σχήματα", ""]
    if len(cfg["reps"]) > 1:
        R += ["Κάθε σχήμα είναι από την επανάληψη με τη διάμεση συνολική ενεργό τιμή (RMS):", ""]
    R += [f"- `figures/{cfg['fig'].format(FIG_NAME.get((source, TESTS[p][1]), TESTS[p][1]))}`: {p}_{n}"
          for p, n in chosen.items()]
    if "F" in chosen and "F-OFF" in chosen:
        R += [f"- `figures/{cfg['pitch_fig']}`: F_{chosen['F']} και F-OFF_{chosen['F-OFF']}"]
    if source == "robot":
        R += ["", "Στο Σχ. `fig:controller-response-comparison` το (β) γίνεται `images/log_plot_real_forward.png`."]
    R += [f"Οι πίνακες LaTeX είναι στο `tables/{cfg['tables']}` (`tab:{cfg['label']}-roll`, "
          f"`tab:{cfg['label']}-pitch`, `tab:{cfg['label']}-distance`)."]
    (_OUT / cfg["summary"]).write_text("\n".join(R) + "\n")
    print("\n".join(R))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", action="store_true", help="export the simulation runs")
    export("sim" if ap.parse_args().sim else "robot")
