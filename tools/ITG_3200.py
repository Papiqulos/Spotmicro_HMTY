# Distributed with a free-will license.
# Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
# ITG-3200
# This code is designed to work with the ITG-3200_I2CS I2C Mini Module available from ControlEverything.com.
# https://www.controleverything.com/content/Gyro?sku=ITG-3200_I2CS#tabs-0-product_tabset-2

# Slightly altered so that the code is in a method
import smbus
import time


	
# X-imu -> Z-robot, Y-imu -> X-robot, Z-imu -> Y-robot
def get_rpy():
    """Returns pitch, roll, yaw in that order"""
	# Get I2C bus
    bus = smbus.SMBus(1)

    # ITG3200 address, 0x68(104)
    # Select Power management register 0x3E(62)
    #		0x01(01)	Power up, PLL with X-Gyro reference
    bus.write_byte_data(0x68, 0x3E, 0x01)
    # ITG3200 address, 0x68(104)
    # Select DLPF register, 0x16(22)
    #		0x18(24)	Gyro FSR of +/- 2000 dps
    bus.write_byte_data(0x68, 0x16, 0x18)

    time.sleep(0.5)

    # ITG3200 address, 0x68(104)
    # Read data back from 0x1D(29), 6 bytes
    # X-Axis MSB, X-Axis LSB, Y-Axis MSB, Y-Axis LSB, Z-Axis MSB, Z-Axis LSB
    data = bus.read_i2c_block_data(0x68, 0x1D, 6)

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



    return xGyro, yGyro, zGyro


if __name__ == "__main__":
    print(f"X: {get_rpy()[2]}, Y: {get_rpy()[1]}, Z: {get_rpy()[0]}")
