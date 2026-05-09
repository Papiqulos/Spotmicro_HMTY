import py_qmc5883l


class HMC5883L:
    def __init__(self):
        self.sensor = py_qmc5883l.QMC5883L()

    def calibrate(self, samples=1000):
        pass

    def get_magnetometer(self):
        """ Gets the sensor readings and returns them as a dictionary. """
        b = self.sensor.get_bearing()
        m = self.sensor.get_magnet_raw()    
        return {"bearing": b, "magnet": m}


if __name__ == "__main__":
    mag = HMC5883L()
    mag.calibrate(1000)
    print(f"Bearing: {mag.get_magnetometer()['bearing']}°\nMagnet: {mag.get_magnetometer()['magnet']}µT")
