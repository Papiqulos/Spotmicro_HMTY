import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from ahrs.filters import Madgwick
import hw.imu as imu
import numpy as np


gyro_data = imu.get_sensor_readings()["gyro"]
# Convert the data from degrees/s to radians/s
gyro_data = np.array([gyro_data[0]*np.pi/180, gyro_data[1]*np.pi/180, gyro_data[2]*np.pi/180])

acc_data = imu.get_sensor_readings()["accelerometer"]["acceleration"]
# Convert the data from g to m/s^2
acc_data = np.array([acc_data[0]*9.81, acc_data[1]*9.81, acc_data[2]*9.81])

mag_data = imu.get_sensor_readings()["magnetometer"]["magnet"]
# Convert the data from uT to mT
mad_data = np.array([mag_data[0]*0.0001, mag_data[1]*0.0001, mag_data[2]*0.0001])

print(f"Data: {gyro_data}, {acc_data}, {mag_data}")
madgwick = Madgwick(gyro=gyro_data, acc=acc_data, mag=mag_data, q0=[0.7071, 0.0, 0.7071, 0.0])

print(madgwick.updateMARG())




