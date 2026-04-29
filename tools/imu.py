import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


from tools import ITG_3200 as imu_gyro
from tools import ADXL435 as imu_accelerometer
from tools import HMC5883L as imu_magnetometer

# X-imu -> Z-robot, Y-imu -> X-robot, Z-imu -> Y-robot
def get_sensor_readings():
    
    """
    Gets the sensor readings and returns them as a dictionary.
    
    Returns:
        readings (dict): A dictionary containing the sensor readings.
    """
    ""
    # print("Getting sensor readings...")
    # GYRO
    pitch, roll, yaw = imu_gyro.get_rpy()
    # ACCELEROMETER
    acc, tap, freefall, motion = imu_accelerometer.get_acceleration()
    # MAGNETOMETER
    magnetometer = imu_magnetometer.get_magnetometer()

    readings = {"gyro": {"roll": roll, "pitch": pitch, "yaw": yaw}, 
                "accelerometer": {"x": acc[2], "y": acc[1], "z": acc[0], 
                                  "tap": tap, 
                                  "freefall": freefall, 
                                  "motion": motion},
                "magnetometer": magnetometer}
    return readings



if __name__ == "__main__":
    print(f"Gyro: {get_sensor_readings()['gyro']}\nAccelerometer: {get_sensor_readings()['accelerometer']}\nMagnetometer: {get_sensor_readings()['magnetometer']}")

