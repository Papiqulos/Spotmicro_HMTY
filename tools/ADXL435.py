# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
import board
import adafruit_adxl34x


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