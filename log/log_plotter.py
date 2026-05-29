import pandas as pd
import matplotlib.pyplot as plt
import numpy as np



def plot_log(file_name):

    fig, axes = plt.subplots(2, 1, figsize=(10, 8))
    # file_name = "log/pid_20260518_193234.csv"
    df = pd.read_csv(file_name)
    df2 = pd.read_csv(file_name)
    # Convert the data from radians to degrees
    df["imu_roll"] = df["imu_roll"] * 180 / np.pi
    df["pid_roll"] = df["pid_roll"] * 180 / np.pi
    df2["imu_pitch"] = df2["imu_pitch"] * 180 / np.pi
    df2["pid_pitch"] = df2["pid_pitch"] * 180 / np.pi

    df.plot(x="t", y=["imu_roll", "pid_roll"], ylim=[-10, 10], ylabel="Angle (deg)", ax=axes[0], grid=True)
    df2.plot(x="t", y=["imu_pitch", "pid_pitch"], ylim=[-10, 10], ylabel="Angle (deg)", ax=axes[1], grid=True)

    image_name = file_name.replace(".csv", ".png")
    plt.savefig(image_name)
    # plt.show()
