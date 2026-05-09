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
    N = 100
    for i in range(N):
        with  open("mag.txt", "a") as f:
            data = mag.get_magnetometer()
            f.write(f"{data["magnet"][0]} {data['magnet'][1]} {data['magnet'][2]}\n")
    
