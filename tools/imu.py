import ITG_3200 as imu_gyro
import ADXL435 as imu_accelerometer




def get_sensor_readings():
    
    """
    Returns a dictionary of sensor readings
    """
    ""
    pitch, roll, yaw = imu_gyro.get_rpy()
    acc, tap, freefall, motion = imu_accelerometer.get_acceleration()

    readings = {"gyro": {"roll": roll, "pitch": pitch, "yaw": yaw}, 
                "accelerometer": {"x": acc[0], "y": acc[1], "z": acc[2], 
                                  "tap": tap, 
                                  "freefall": freefall, 
                                  "motion": motion}}
    return readings

