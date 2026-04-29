import py_qmc5883l



def get_magnetometer():
    """ Gets the sensor readings and returns them as a dictionary. """
    sensor = py_qmc5883l.QMC5883L()
    b = sensor.get_bearing()
    m = sensor.get_magnet_raw()
    return {"bearing": b, "magnet": m}

if __name__ == "__main__":
    print(f"Bearing: {get_magnetometer()['bearing']}°\nMagnet: {get_magnetometer()['magnet']}µT")