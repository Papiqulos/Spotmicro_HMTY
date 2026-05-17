clc;
clear;
close all;




T = readtable("log/pid_20260517_171059.csv");
% Convert the data from radians to degrees
T.imu_roll = T.imu_roll * 180 / pi;
T.pid_roll = T.pid_roll * 180 / pi;
plot(T.t, T.imu_roll, T.t, T.pid_roll)
legend("IMU", "PID")
xlabel("Time (s)")
ylabel("Angle (deg)")
title("Roll Angle")
grid on