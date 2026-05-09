import math
import time
import py_qmc5883l


class QMC5883L:
    def __init__(self):
        self.sensor = py_qmc5883l.QMC5883L()
        self.sensor.set_declination(4)
        self.z_offset = 0.0
        self.z_scale = 1.0

    def calibrate(self, samples=5000, delay=0.01):
        print(f"Collecting {samples} samples — rotate sensor slowly in all directions...")
        min_vals = [float('inf')] * 3
        max_vals = [float('-inf')] * 3

        for i in range(samples):
            m = self.sensor.get_magnet_raw()
            if None in m:
                continue
            for j in range(3):
                if m[j] < min_vals[j]:
                    min_vals[j] = m[j]
                if m[j] > max_vals[j]:
                    max_vals[j] = m[j]
            time.sleep(delay)
            if (i + 1) % 500 == 0:
                print(f"  {i + 1}/{samples}")

        offset = [(min_vals[j] + max_vals[j]) / 2.0 for j in range(3)]
        chords = [(max_vals[j] - min_vals[j]) / 2.0 for j in range(3)]
        avg_chord = (chords[0] + chords[1]) / 2.0
        scale = [avg_chord / c if c > 0 else 1.0 for c in chords]

        # Encode hard-iron + soft-iron into the library's 3x3 calibration matrix
        # x1 = x*c[0][0] + y*c[0][1] + c[0][2]
        # y1 = x*c[1][0] + y*c[1][1] + c[1][2]
        self.sensor.set_calibration([
            [scale[0], 0.0, -offset[0] * scale[0]],
            [0.0, scale[1], -offset[1] * scale[1]],
            [0.0, 0.0, 1.0],
        ])
        self.z_offset = offset[2]
        self.z_scale = scale[2]

        print(f"Hard-iron offsets: x={offset[0]:.1f}  y={offset[1]:.1f}  z={offset[2]:.1f}")
        print(f"Soft-iron scales:  x={scale[0]:.4f}  y={scale[1]:.4f}  z={scale[2]:.4f}")

    def get_magnetometer(self):
        raw = self.sensor.get_magnet_raw()
        cal = self.sensor.get_magnet()
        z = (raw[2] - self.z_offset) * self.z_scale if raw[2] is not None else 0.0
        bearing = self.sensor.get_bearing()
        return {"bearing": bearing, "magnet": [cal[0], cal[1], z]}


if __name__ == "__main__":
    mag = QMC5883L()
    mag.calibrate()
    while True:
        data = mag.get_magnetometer()
        print(f"Bearing: {data['bearing']:6.1f}°   Magnet: {[f'{v:.1f}' for v in data['magnet']]}", end="\r")
        time.sleep(0.1)
