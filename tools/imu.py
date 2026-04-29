import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


from tools import ITG_3200 as imu_gyro
from tools import ADXL435 as imu_accelerometer

# X-imu -> Z-robot, Y-imu -> X-robot, Z-imu -> Y-robot
def get_sensor_readings():
    
    """
    Gets the sensor readings and returns them as a dictionary.
    
    Returns:
        readings (dict): A dictionary containing the sensor readings.
    """
    ""
    # print("Getting sensor readings...")
    pitch, roll, yaw = imu_gyro.get_rpy()
    acc, tap, freefall, motion = imu_accelerometer.get_acceleration()

    readings = {"gyro": {"roll": roll, "pitch": pitch, "yaw": yaw}, 
                "accelerometer": {"x": acc[1], "y": acc[2], "z": acc[0], 
                                  "tap": tap, 
                                  "freefall": freefall, 
                                  "motion": motion}}
    return readings



if __name__ == "__main__":
    print(get_sensor_readings())

