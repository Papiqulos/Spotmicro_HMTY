import json, shutil
from pathlib import Path
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from log.metrics import steady_mask

OUT = Path("thesis_export/figures")
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"
plt.rcParams.update({"font.size": 10, "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2,
                     "ytick.color": INK2, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
                     "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False})

def run(stamp):
    f = f"log/pid/pid_2026_09_{stamp}.csv"
    m = json.load(open(f.replace(".csv", ".json")))
    df = pd.read_csv(f); mask, _, _ = steady_mask(df, m); t = df["t"].to_numpy()[mask]
    imu = pd.read_csv(m["imu_log"]); full = imu.copy()
    imu = imu[(imu.t >= t[0]) & (imu.t <= t[-1])]
    return m, imu, full, f

# 1. copies of the per-run plots
copies = {"log_plot_real_nothing.png": "28_00_29_47", "log_plot_real_pitch_pi.png": "28_00_30_46",
          "log_plot_real_weight_roll_off.png": "28_00_45_13", "log_plot_real_weight_roll_p.png": "28_00_46_15",
          "log_plot_real_weight_roll_pi.png": "28_00_48_00"}
def plot_run(csv, out, ylim=(-9, 9)):
    # same layout as log/pid_plotter.py, wider scale so the PID OFF pitch is not clipped
    with plt.rc_context(matplotlib.rcParamsDefault):
        fig, axes = plt.subplots(2, 1, figsize=(10, 8))
        df = pd.read_csv(csv)
        df["t"] = df["t"] - df["t"].iloc[0]
        for c in ("imu_roll", "pid_roll", "imu_pitch", "pid_pitch"):
            df[c] = np.degrees(df[c])
        df.plot(x="t", y=["imu_roll", "pid_roll"], ylim=ylim, ylabel="Angle (deg)", ax=axes[0], grid=True)
        df.plot(x="t", y=["imu_pitch", "pid_pitch"], ylim=ylim, ylabel="Angle (deg)", ax=axes[1], grid=True)
        for ax in axes:
            ax.set_xlabel("t (s)")
        fig.savefig(out); plt.close(fig)

for name, stamp in copies.items():
    plot_run(f"log/pid/pid_2026_09_{stamp}.csv", OUT / name)

# 2. pitch OFF vs pitch PI, raw and averaged over one gait cycle
fig, ax = plt.subplots(figsize=(8, 3.4))
for stamp, color, label in (("28_00_29_47", ORANGE, "χωρίς σταθεροποίηση"), ("28_00_30_46", BLUE, "PI στο pitch")):
    m, _, full, f = run(stamp)
    t = full.t.to_numpy() - full.t.iloc[0]
    p = np.degrees(full.imu_pitch.to_numpy())
    n = max(1, int(round(0.46 / np.median(np.diff(t)))))
    avg = pd.Series(p).rolling(n, center=True).mean()
    ax.plot(t, p, color=color, lw=0.8, alpha=0.35)
    ax.plot(t, avg, color=color, lw=2)
    ax.text(t[-1] + 0.15, avg.dropna().iloc[-1], label, color=INK, va="center", fontsize=9)
ax.axhline(0, color=INK2, lw=0.8)
ax.set_xlabel("t (s)"); ax.set_ylabel("pitch (°)")
ax.set_xlim(0, t[-1] + 3.2); ax.set_xticks(range(0, 12, 2)); ax.spines["bottom"].set_bounds(0, 11.2)
fig.tight_layout(); fig.savefig(OUT / "pitch_off_vs_pi.png", dpi=200); plt.close(fig)

# 3. tuning summary: mean pitch and oscillation per configuration
groups = {
    "OFF": ["27_22_48_36", "27_22_50_40", "28_00_29_47", "28_00_31_39"],
    "PD, μικρό $K_i$": ["27_22_44_39", "27_22_46_42", "27_22_55_09", "27_22_56_04", "27_22_58_14", "27_22_59_01",
                        "27_22_59_43", "27_23_00_37", "27_23_44_00", "27_23_45_05", "27_23_46_00", "27_23_47_28",
                        "27_23_49_06", "27_23_49_59", "27_23_51_07", "27_23_54_21", "27_23_56_10"],
    "PI στο pitch\n$K_i=1$, $K_d=0$": ["28_00_12_17", "28_00_13_03", "28_00_13_49", "28_00_14_21", "28_00_30_46", "28_00_32_31"],
}
stats = {g: [] for g in groups}
for g, stamps in groups.items():
    for s in stamps:
        _, imu, _, _ = run(s)
        stats[g].append((np.degrees(imu.imu_pitch).mean(), np.degrees(imu.imu_pitch).std(), np.degrees(imu.imu_roll).std()))
fig, (a1, a2) = plt.subplots(1, 2, figsize=(9, 3.6))
rng = np.random.default_rng(0)
x = np.arange(len(groups))
for i, g in enumerate(groups):
    v = np.array(stats[g])
    a1.scatter(i + rng.uniform(-0.12, 0.12, len(v)), v[:, 0], s=22, color=BLUE, alpha=0.8, zorder=3)
    a1.hlines(v[:, 0].mean(), i - 0.28, i + 0.28, color=INK, lw=2, zorder=4)
    a1.text(i + 0.32, v[:, 0].mean(), f"{v[:, 0].mean():.2f}".replace(".", ",").replace("-", "\u2212"), va="center", fontsize=9, color=INK)
    for k, (col, dx) in enumerate(((BLUE, -0.15), (ORANGE, 0.15))):
        yy = v[:, 1 + (1 - k)] if False else v[:, 2 - k]
        a2.scatter(i + dx + rng.uniform(-0.05, 0.05, len(v)), v[:, 2 - k], s=22, color=col, alpha=0.8, zorder=3,
                   label=("roll" if k == 0 else "pitch") if i == 0 else None)
        a2.hlines(v[:, 2 - k].mean(), i + dx - 0.1, i + dx + 0.1, color=INK, lw=2, zorder=4)
a1.axhline(0, color=INK2, lw=0.8)
for a in (a1, a2):
    a.set_xticks(x, list(groups), fontsize=9)
a1.set_ylabel("μέση τιμή pitch (°)"); a1.set_title("(α) σταθερή κλίση", fontsize=10, color=INK)
a2.set_ylabel("τυπική απόκλιση (°)"); a2.set_title("(β) ταλάντωση", fontsize=10, color=INK)
a2.legend(loc="upper left")
fig.tight_layout(); fig.savefig(OUT / "tuning_summary.png", dpi=200); plt.close(fig)

# 4. side load test: mean roll per roll controller
wg = {"roll off": ["28_00_45_13", "28_00_49_12"], "roll P\n$K_p=0{,}3$": ["28_00_46_15", "28_00_50_49"],
      "roll PI\n$K_p=0{,}3$, $K_i=1$": ["28_00_48_00", "28_00_52_48"]}
fig, ax = plt.subplots(figsize=(5.5, 3.4))
for i, (g, stamps) in enumerate(wg.items()):
    v = [np.degrees(run(s)[1].imu_roll).mean() for s in stamps]
    ax.scatter([i - 0.06, i + 0.06], v, s=30, color=BLUE, zorder=3)
    ax.hlines(np.mean(v), i - 0.25, i + 0.25, color=INK, lw=2, zorder=4)
    ax.text(i + 0.29, np.mean(v), f"{np.mean(v):.2f}".replace(".", ",").replace("-", "\u2212"), va="center", fontsize=9, color=INK)
ax.axhline(0, color=INK2, lw=0.8)
ax.set_xticks(range(len(wg)), list(wg), fontsize=9)
ax.set_ylabel("μέση τιμή roll (°)")
fig.tight_layout(); fig.savefig(OUT / "weight_test_roll.png", dpi=200); plt.close(fig)
print(sorted(p.name for p in OUT.iterdir()))
