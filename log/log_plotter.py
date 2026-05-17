import pandas as pd
df = pd.read_csv("log/pid_20260517_171059.csv")
df.plot(x="t", y=["imu_roll", "pid_roll"])
