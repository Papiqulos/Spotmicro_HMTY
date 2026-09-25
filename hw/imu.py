import time
from pathlib import Path
import numpy as np
from ahrs.filters import Madgwick, EKF
from ahrs.common.orientation import q2rpy, acc2q
from hw import ITG_3200 as imu_gyro
from hw import ADXL435 as imu_accelerometer
from tools.utils import open_run_log

# Roll angle (rotation around x-axis-FORWARD)
# Pitch angle (rotation around z-axis-LEFT)
# Yaw angle (rotation around y-axis-UP)


# X-imu-> Left, Y-imu-> Backwards, Z-imu-> Down

# X-robot-> -Y-imu, Y-robot-> X-imu, Z-robot-> -Z-imu


_LOG_DIR = Path(__file__).parent.parent / "log" / "imu"

class IMU:
    def __init__(self, filter_type="Madgwick", log=False):
        self.gyro = imu_gyro.ITG_3200()
        self.accelerometer = imu_accelerometer.ADXL435()
        init_acc = self.accelerometer.read()["acceleration"]
        # Magnetometer (hw/QMC5883L.py) not used because of too much noise


        # Calculate initial orientation through the accelerometer
        self.q0 = acc2q(init_acc)
        # self.q0 = np.array([1, 0, 0, 0]) # static orientation
        self.initial_orientation_rad = q2rpy(self.q0)
        self.initial_orientation_deg = q2rpy(self.q0, in_deg=True)

        print(f"Initial Orientation\nRoll: {self.initial_orientation_deg[1]:.4f}°, Pitch: {self.initial_orientation_deg[0]:.4f}°, Yaw: {self.initial_orientation_deg[2]:.4f}")
        print(f"Roll: {self.initial_orientation_rad[1]:.4f}, Pitch: {self.initial_orientation_rad[0]:.4f}, Yaw: {self.initial_orientation_rad[2]:.4f}")



        self.filter_type = filter_type

        if filter_type == "Madgwick":
            self.filter = Madgwick(gain=0.045)
        elif filter_type == "EKF":
            self.filter = EKF()
        else:
            raise ValueError(f"Unknown filter: {filter_type}")


        # Low pass filter for roll and pitch
        self.s_roll = 0
        self.s_pitch = 0
        self.alpha = 0.3
        self.last_time = time.time()
        print("IMU Ready")

        self.log = log
        self.log_path = None
        if log:
            self._log_file, self._csv = open_run_log(
                _LOG_DIR, "imu", ["t", "imu_roll", "imu_pitch", "imu_yaw", "lpf_roll", "lpf_pitch", "lpf_yaw"])
            self.log_path = self._log_file.name
            self._t0 = time.time()

    def save_plot(self):
        if not self.log:
            return
        from log.imu_plotter import imu_log
        self._log_file.flush()
        imu_log(self.log_path)

    def update(self, raw_gyro, raw_acc, raw_mag=None, in_deg=False):
        """Update the orientation using raw IMU data.

        Args:
            raw_gyro (numpy.ndarray): raw gyroscope readings in degrees/s
            raw_acc (numpy.ndarray): raw accelerometer readings in g force
            raw_mag (numpy.ndarray, optional): raw magnetometer readings in nano tesla. Defaults to None.
            in_deg (bool, optional): Return the angles in degrees. Defaults to False.

        Returns:
            numpy.ndarray: orientation in roll, pitch, yaw in radians or degrees
        """

        current_time = time.time()
        dt = current_time - self.last_time

        # Convert degrees/s to radians/s
        raw_gyro = np.radians(raw_gyro)

        # Convert acceleration to m/s^2
        raw_acc = raw_acc * 9.81

        if raw_mag is not None:
            raw_mag = np.array([raw_mag[0] , -raw_mag[1], -raw_mag[2]])


        if dt <= 0.0:
            dt = 0.001

        if raw_mag is None:
            if self.filter_type == "Madgwick":
                self.q0 = self.filter.updateIMU(q=self.q0,
                                                      gyr=raw_gyro,
                                                      acc=raw_acc,
                                                      dt=dt)
            elif self.filter_type == "EKF":
                self.q0 = self.filter.update(q=self.q0,
                                                       gyr=raw_gyro,
                                                       acc=raw_acc,
                                                       dt=dt)
        else:
            if self.filter_type == "Madgwick":
                self.q0 = self.filter.updateMARG(q=self.q0,
                                                        gyr=raw_gyro,
                                                        acc=raw_acc,
                                                        mag=raw_mag,
                                                        dt=dt)
            elif self.filter_type == "EKF":
                self.q0 = self.filter.update(q=self.q0,
                                                       gyr=raw_gyro,
                                                       acc=raw_acc,
                                                       mag=raw_mag,
                                                       dt=dt)

        self.last_time = current_time
        # Pitch first then roll then yaw to match the coordinate system of the robot
        pitch, roll, yaw = q2rpy(self.q0, in_deg=in_deg)
        if in_deg:
            pitch = pitch - self.initial_orientation_deg[0]
            roll = roll   - self.initial_orientation_deg[1]
        else:
            pitch = pitch - self.initial_orientation_rad[0]
            roll = roll   - self.initial_orientation_rad[1]

        # Apply the low pass filter
        smoothed = self.alpha * self.s_roll + (1 - self.alpha) * roll
        self.s_roll = smoothed
        smoothed = self.alpha * self.s_pitch + (1 - self.alpha) * pitch
        self.s_pitch = smoothed

        self.smoothed_orientation = np.array([ self.s_roll, -self.s_pitch, yaw])

        if self.log:
            self._csv.writerow([f"{current_time - self._t0:.4f}",
                                f"{roll:.6f}", f"{-pitch:.6f}", f"{yaw:.6f}",
                                f"{self.smoothed_orientation[0]:.6f}",
                                f"{self.smoothed_orientation[1]:.6f}",
                                f"{self.smoothed_orientation[2]:.6f}"])

        return self.smoothed_orientation


if __name__ == "__main__":
    imu = IMU(filter_type="EKF", log=True)

    try:
        while True:
            raw_gyro = imu.gyro.read()
            raw_acc = imu.accelerometer.read()["acceleration"]
            # raw_mag = imu.magnetometer.read()

            roll, pitch, yaw = imu.update(raw_gyro, raw_acc)

            print(f"Roll: {np.degrees(roll):.4f}° Pitch: {np.degrees(pitch):.4f}° Yaw: {np.degrees(yaw):.4f}°", end='\r')
            # print(f"Roll: {roll:.6f} Pitch: {pitch:.6f} Yaw: {yaw:.6f}", end='\r')
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        imu.save_plot()
