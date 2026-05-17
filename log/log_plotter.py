import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
df = pd.read_csv("log/pid_20260517_171059.csv")
# Convert the data from radians to degrees
df["imu_roll"] = df["imu_roll"] * 180 / np.pi
df["pid_roll"] = df["pid_roll"] * 180 / np.pi
df.plot(x="t", y=["imu_roll", "pid_roll"])
plt.show()
