import math
import numpy as np
import time
import py_qmc5883l


class QMC5883L:
    def __init__(self):
        self.sensor = py_qmc5883l.QMC5883L()
        print("Magnetometer Ready")
        self.calibrate()
        print("Magnetometer Calibrated")

    def calibrate(self):
        self.soft_iron_correction = np.array([  [0.976468331640768,	0.091099238289340,	0.017881009067270],
                                                [0.091099238289340,	0.951059434012282,	-0.011378730331739],
                                                [0.017881009067270,	-0.01137873033173,	1.087015367841512]])
        self.hard_iron_correction = np.array([4.387984692383546e2,	1.745401690683345e3,	1.152495789367265e3])
        # with open("/home/papiqulos/quadruped/Spotmicro_HMTY/tools/raw_data.txt", "a") as f:
        #     while True:
        #         raw_data = self.sensor.get_magnet_raw()

        #         f.write(f"{raw_data[0]:.1f} {raw_data[1]:.1f} {raw_data[2]:.1f}\n")
        #         # print(f"{raw_data[0]:.1f} {raw_data[1]:.1f} {raw_data[2]:.1f}")

    def get_magnetometer(self):
        raw = self.sensor.get_magnet_raw()
        sub = np.subtract(raw, self.hard_iron_correction)
        calibrated = np.matmul(sub, self.soft_iron_correction)
        bearing = (math.degrees(math.atan2(-calibrated[0], calibrated[1])) + 4) % 360
        return {"bearing": bearing, "magnet": [calibrated[0], calibrated[1], calibrated[2]]}


if __name__ == "__main__":
    mag = QMC5883L()
    mag.calibrate()
    while True:
        data = mag.get_magnetometer()
        print(f"Bearing: {data['bearing']:6.1f}°   Magnet: {[f'{v:.1f}' for v in data['magnet']]}", end="\r")
        time.sleep(0.1)
