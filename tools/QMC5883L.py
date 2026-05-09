import math
import time
import py_qmc5883l


class QMC5883L:
    def __init__(self):
        self.sensor = py_qmc5883l.QMC5883L()
        self.offset = [0.0, 0.0, 0.0]

    def calibrate(self, samples=5000, delay=0.01):
        pass

    def get_magnetometer(self):
        m = self.sensor.get_magnet_raw()
        x = m[0] - self.offset[0]
        y = m[1] - self.offset[1]
        z = m[2] - self.offset[2]
        bearing = math.degrees(math.atan2(y, x))
        if bearing < 0:
            bearing += 360.0
        return {"bearing": bearing, "magnet": [x, y, z]}


if __name__ == "__main__":
    mag = QMC5883L()
    mag.calibrate()
    while True:
        data = mag.get_magnetometer()
        print(f"Bearing: {data['bearing']:.1f}°\nMagnet: {data['magnet']} G")
