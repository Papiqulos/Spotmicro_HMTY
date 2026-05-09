# Distributed with a free-will license.
# Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
# ITG-3200
# This code is designed to work with the ITG-3200_I2CS I2C Mini Module available from ControlEverything.com.
# https://www.controleverything.com/content/Gyro?sku=ITG-3200_I2CS#tabs-0-product_tabset-2

# Slightly altered
import smbus2
import time
import numpy as np



class ITG_3200:
    def __init__(self):
        # Get I2C bus
        self.bus = smbus2.SMBus(1)

        # ITG3200 address, 0x68(104)
        # Select Power management register 0x3E(62)
        #		0x01(01)	Power up, PLL with X-Gyro reference
        self.bus.write_byte_data(0x68, 0x3E, 0x01)
        # ITG3200 address, 0x68(104)
        # Select DLPF register, 0x16(22)
        #		0x18(24)	Gyro FSR of +/- 2000 dps
        self.bus.write_byte_data(0x68, 0x16, 0x18)

        self.x_offset = 0
        self.y_offset = 0
        self.z_offset = 0

        time.sleep(0.5)
        print("Gyro Ready")
        self.calibrate()

    def calibrate(self, samples=1000):
        values = []
        for i in range(samples):
            values.append(self.get_xyzGyro())
        # Average the values
        average = np.mean(values, axis=0)
        self.x_offset = average[0]
        self.y_offset = average[1]
        self.z_offset = average[2]
        print("Gyro Calibrated")

    def get_xyzGyro(self):
        """Returns angular velocity in 3-axis in degress/s. The axis are in the order of the sensor"""
        

        # ITG3200 address, 0x68(104)
        # Read data back from 0x1D(29), 6 bytes
        # X-Axis MSB, X-Axis LSB, Y-Axis MSB, Y-Axis LSB, Z-Axis MSB, Z-Axis LSB
        data = self.bus.read_i2c_block_data(0x68, 0x1D, 6)

        # Convert the data
        xGyro = data[0] * 256 + data[1]
        if xGyro > 32767 :
            xGyro -= 65536

        yGyro = data[2] * 256 + data[3]
        if yGyro > 32767 :
            yGyro -= 65536

        zGyro = data[4] * 256 + data[5]
        if zGyro > 32767 :
            zGyro -= 65536



        SENSITIVITY = 14.375
        return np.array([xGyro / SENSITIVITY - self.x_offset,
                         yGyro / SENSITIVITY - self.y_offset,
                         zGyro / SENSITIVITY - self.z_offset])


if __name__ == "__main__":
    gyro = ITG_3200()
    print(f"X: {gyro.get_xyzGyro()[0]}, Y: {gyro.get_xyzGyro()[2]}, Z: {gyro.get_xyzGyro()[1]}")
