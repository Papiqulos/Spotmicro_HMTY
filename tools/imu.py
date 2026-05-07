import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


from tools import ITG_3200 as imu_gyro
from tools import ADXL435 as imu_accelerometer
# from tools import HMC5883L as imu_magnetometer
import time
import math
import numpy as np
from ahrs.filters import Madgwick
from ahrs.common.orientation import q2euler

# Roll angle (rotation around x-axis-FORWARD)
# Pitch angle (rotation around z-axis-LEFT)
# Yaw angle (rotation around y-axis-UP)


# X-imu-> Left, Y-imu-> Backwards, Z-imu-> Down
# X-robot-> Forward, Y-robot-> Left, Z-robot-> Up

# X-robot-> -Y-imu, Y-robot-> X-imu, Z-robot-> -Z-imu

class IMU:
    def __init__(self):
        self.gyro = imu_gyro.ITG_3200()
        self.accelerometer = imu_accelerometer.ADXL435()
        # self.magnetometer = imu_magnetometer.HMC5883L()

        self.madgwick = Madgwick(gain=0.02)

        self.quaternion = np.array([1, 0, 0, 0])

        self.last_time = time.time()
        print("IMU Ready")


    def update(self, raw_gyro, raw_acc, raw_mag=None):

        current_time = time.time()
        dt = current_time - self.last_time
        
        # Convert degrees/s to radians/s
        raw_gyro = np.radians(raw_gyro)
        
        # Convert acceleration to m/s^2
        raw_acc = raw_acc * 9.81
        
        if dt <= 0.0:
            dt = 0.001

        if raw_mag is None:
            self.quaternion = self.madgwick.updateIMU(q=self.quaternion,
                                                      gyr=raw_gyro,
                                                      acc=raw_acc,
                                                      dt=dt)
        else:
            self.quaternion = self.madgwick.updateMARG(q=self.quaternion,
                                                       gyr=raw_gyro,
                                                       acc=raw_acc,
                                                       mag=raw_mag,
                                                       dt=dt)

        self.last_time = current_time
        roll, pitch, yaw = q2euler(self.quaternion)

        # Convert to degrees
        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)

        return roll, pitch, yaw
            

if __name__ == "__main__":
    imu = IMU()
    while True:

        # Get the raw sensor readings
        raw_gyro = imu.gyro.get_xyzGyro()
        raw_acc = imu.accelerometer.get_acceleration()["acceleration"]
        # raw_mag = imu.magnetometer.get_magnetometer()["magnet"]

        # Update the orientation
        pitch, roll, yaw = imu.update(raw_gyro, raw_acc)
        print(f"Roll: {roll:.2f}°\tPitch: {pitch:.2f}°\tYaw: {yaw:.2f}°")
        time.sleep(0.01)


