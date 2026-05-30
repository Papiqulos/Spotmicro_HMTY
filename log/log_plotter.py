import pandas as pd
import matplotlib.pyplot as plt
import numpy as np



def plot_log(file_name):
    fig, axes = plt.subplots(2, 1, figsize=(10, 8))
    df = pd.read_csv(file_name)
    df["imu_roll"]  = df["imu_roll"]  * 180 / np.pi
    df["imu_pitch"] = df["imu_pitch"] * 180 / np.pi
    df.plot(x="t", y=["imu_roll"],  ylim=[-20, 20], ylabel="Roll (deg)",  ax=axes[0], grid=True)
    df.plot(x="t", y=["imu_pitch"], ylim=[-20, 20], ylabel="Pitch (deg)", ax=axes[1], grid=True)
    plt.tight_layout()
    plt.savefig(file_name.replace(".csv", ".png"))


def plot_log_dh(file_name):
    fig, axes = plt.subplots(2, 1, figsize=(10, 8))
    df = pd.read_csv(file_name)
    df.plot(x="t", y=["dh_roll",  "corr_roll"],  ylabel="Roll DH (mm)",  ax=axes[0], grid=True)
    df.plot(x="t", y=["dh_pitch", "corr_pitch"], ylabel="Pitch DH (mm)", ax=axes[1], grid=True)
    plt.tight_layout()
    plt.savefig(file_name.replace(".csv", ".png"))
