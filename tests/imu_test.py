import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import time
import tools.imu as imu



while True:
    readings = imu.get_sensor_readings()
    print(f"Gyro: {readings['gyro']}\nAccelerometer: {readings['accelerometer']}\nMagnetometer: {readings['magnetometer']}")
    print("----------------------------------")
    time.sleep(0.5)


