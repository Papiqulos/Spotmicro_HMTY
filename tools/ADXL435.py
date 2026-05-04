# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
import board
import adafruit_adxl34x
import numpy as np


class ADXL435:
    def __init__(self):
        i2c = board.I2C()  # uses board.SCL and board.SDA
        # For ADXL345
        self.accelerometer = adafruit_adxl34x.ADXL345(i2c)

        self.accelerometer.enable_motion_detection()
        self.accelerometer.enable_tap_detection(tap_count=1, threshold=30)
        self.accelerometer.enable_freefall_detection(threshold=20, time=100)
        self.accelerometer.enable_motion_detection()

        self.tap = self.accelerometer.events["tap"]
        self.freefall = self.accelerometer.events["freefall"]
        self.motion = self.accelerometer.events["motion"]
        print("Accelerometer Ready")
        self.calibrate()

    def calibrate(self, samples=1000):
        self.zmax = 11.34
        self.zmin = -8.49

        self.xmax = 9.85
        self.xmin = -10.23

        self.ymax = 10.3
        self.ymin = -10.4
        
        
        # self.xmax = 9.9600
        # self.xmin = -10.1552
        # self.ymax = 10.4583
        # self.ymin = -10.0486
        # self.zmax = 11.3405
        # self.zmin = -8.4245
        
        

        self.z_offset = (self.zmax + self.zmin) / 2
        self.x_offset = (self.xmax + self.xmin) / 2
        self.y_offset = (self.ymax + self.ymin) / 2

        self.z_scale = (self.zmax - self.zmin) / 2
        self.x_scale = (self.xmax - self.xmin) / 2
        self.y_scale = (self.ymax - self.ymin) / 2
        print("Accelerometer Calibrated")

    def run_interactive_calibration(self, samples=2000):
        positions = [
            "Upright — bottom facing down (normal standing position)",
            "Upside down — top facing down",
            "Left side down — left flank facing down",
            "Right side down — right flank facing down",
            "Nose down — front facing down",
            "Nose up — rear facing down",
        ]

        all_readings = []

        for i, instruction in enumerate(positions):
            input(f"[{i+1}/6] {instruction}\n       Servos OFF. Press Enter when stable...")
            print(f"       Sampling {samples} readings...", end=" ", flush=True)
            data = []
            for _ in range(samples):
                data.append(self.accelerometer.acceleration)
            avg = np.mean(data, axis=0)
            all_readings.append(avg)
            print(f"X={avg[0]:.3f}  Y={avg[1]:.3f}  Z={avg[2]:.3f}")

        data = np.array(all_readings)
        xmax, ymax, zmax = data.max(axis=0)
        xmin, ymin, zmin = data.min(axis=0)

        print("\n--- Paste into calibrate() ---")
        print(f"        self.xmax = {xmax:.4f}")
        print(f"        self.xmin = {xmin:.4f}")
        print(f"        self.ymax = {ymax:.4f}")
        print(f"        self.ymin = {ymin:.4f}")
        print(f"        self.zmax = {zmax:.4f}")
        print(f"        self.zmin = {zmin:.4f}")

    def get_acceleration(self):
        "Returns (x, y, z) acceleration in g-force, tap detection, freefall detection, motion detection"
        calibrated_x = (self.accelerometer.acceleration[0] - self.x_offset) / self.x_scale
        calibrated_y = (self.accelerometer.acceleration[1] - self.y_offset) / self.y_scale
        calibrated_z = (self.accelerometer.acceleration[2] - self.z_offset) / self.z_scale

        readings = {"acceleration": np.array([calibrated_x, 
                                              calibrated_y, 
                                              calibrated_z]),
                    "tap": self.tap, 
                    "freefall": self.freefall, 
                    "motion": self.motion}

        return readings




if __name__ == "__main__":
    acc = ADXL435()
    acc.run_interactive_calibration(samples=2000)
