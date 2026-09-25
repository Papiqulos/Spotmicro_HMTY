import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


def imu_log(file_name, in_deg=False):
    df = pd.read_csv(file_name)
    if df.empty:
        return

    if not in_deg:
        cols = ["imu_roll", "imu_pitch", "imu_yaw", "lpf_roll", "lpf_pitch", "lpf_yaw"]
        df[cols] = np.degrees(df[cols])

    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    df.plot(x="t", y=["imu_roll", "lpf_roll"],    ylabel="Roll (deg)",  ax=axes[0], grid=True)
    df.plot(x="t", y=["imu_pitch", "lpf_pitch"],  ylabel="Pitch (deg)", ax=axes[1], grid=True)
    df.plot(x="t", y=["imu_yaw", "lpf_yaw"],      ylabel="Yaw (deg)",   ax=axes[2], grid=True)
    for ax in axes:
        ax.legend(["EKF/Madgwick", "Low-pass"])
    axes[2].set_xlabel("t (s)")

    plt.savefig(file_name.replace(".csv", ".png"))
    plt.close(fig)
