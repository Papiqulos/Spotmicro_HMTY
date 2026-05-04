import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import time
import tools.imu as imu



while True:
    readings = imu.get_sensor_readings()
    # print(f"Gyro: {readings['gyro']}\nAccelerometer: {readings['accelerometer']}\nMagnetometer: {readings['magnetometer']}")
    acc = readings["accelerometer"]
    tap = acc["tap"]
    freefall = acc["freefall"]
    motion = acc["motion"]
    print(f"TAP: {tap}\nFREEFALL: {freefall}\nMOTION: {motion}")
    print("----------------------------------")
    time.sleep(0.5)


