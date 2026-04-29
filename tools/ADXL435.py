# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
import board
import adafruit_adxl34x



# X-imu -> Z-robot, Y-imu -> X-robot, Z-imu -> Y-robot
def get_acceleration():
    "Returns (x, y, z) acceleration, tap detection, freefall detection, motion detection"
    i2c = board.I2C()  # uses board.SCL and board.SDA

    # For ADXL345
    accelerometer = adafruit_adxl34x.ADXL345(i2c)

    accelerometer.enable_motion_detection()
    accelerometer.enable_tap_detection()
    accelerometer.enable_freefall_detection()
    accelerometer.enable_motion_detection()

    tap = accelerometer.events["tap"]
    freefall = accelerometer.events["freefall"]
    motion = accelerometer.events["motion"]

    return accelerometer.acceleration, tap, freefall, motion


if __name__ == "__main__":
    print(f"X: {get_acceleration()[0][2]}, Y: {get_acceleration()[0][1]}, Z: {get_acceleration()[0][0]}\nTAP: {get_acceleration()[1]}\nFREEFALL: {get_acceleration()[2]}\nMOTION: {get_acceleration()[3]}")