clc;
clear;
close all;




T = readtable("log/pid_20260517_171059.csv");
% Convert the data from radians to degrees
T.imu_roll = T.imu_roll * 180 / pi;
T.pid_roll = T.pid_roll * 180 / pi;

T.imu_pitch = T.imu_pitch * 180 / pi;
T.pid_pitch = T.pid_pitch * 180 / pi;

subplot(2, 1, 1)
plot(T.t, T.imu_roll, T.t, T.pid_roll)
ylim([-10 10])
legend("IMU", "PID")
xlabel("Time (s)")
ylabel("Angle (deg)")
title("Roll Angle")
grid on

subplot(2, 1, 2)
plot(T.t, T.imu_pitch, T.t, T.pid_pitch)
ylim([-10 10])
legend("IMU", "PID")
xlabel("Time (s)")
ylabel("Angle (deg)")
title("Pitch Angle")
grid on