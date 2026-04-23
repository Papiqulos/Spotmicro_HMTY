import time
import math
from smbus2 import SMBus

class GY85:
    def __init__(self, bus_num=1):
        self.bus = SMBus(bus_num)
        
        # Device I2C Addresses
        self.ACCEL_ADDR = 0x53 # ADXL345
        self.GYRO_ADDR = 0x68  # ITG3205
        
        # Initialize Sensors
        self._init_accel()
        self._init_gyro()
        
        # Complementary Filter Variables
        self.pitch = 0.0
        self.roll = 0.0
        self.last_time = time.time()
        
        # Filter tuning (0.98 trusts gyro for quick movement, 0.02 trusts accel to fix drift)
        self.alpha = 0.98 

    def _init_accel(self):
        # Wake up ADXL345 and set to +-2g range
        self.bus.write_byte_data(self.ACCEL_ADDR, 0x2D, 0x08)
        self.bus.write_byte_data(self.ACCEL_ADDR, 0x31, 0x0B)

    def _init_gyro(self):
        # Wake up ITG3205, set sample rate and Low Pass Filter
        self.bus.write_byte_data(self.GYRO_ADDR, 0x3E, 0x00)
        self.bus.write_byte_data(self.GYRO_ADDR, 0x15, 0x07)
        self.bus.write_byte_data(self.GYRO_ADDR, 0x16, 0x1E)

    def _read_word_2c(self, addr, reg):
        """Reads two bytes and converts to a signed 16-bit integer."""
        high = self.bus.read_byte_data(addr, reg)
        low = self.bus.read_byte_data(addr, reg + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        return val

    def get_angles(self):
        """Returns smoothed (Roll, Pitch) in Radians."""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # 1. Read Raw Accelerometer Data (g-force)
        acc_x = self._read_word_2c(self.ACCEL_ADDR, 0x32)
        acc_y = self._read_word_2c(self.ACCEL_ADDR, 0x34)
        acc_z = self._read_word_2c(self.ACCEL_ADDR, 0x36)

        # 2. Read Raw Gyroscope Data (Degrees per second)
        # ITG3205 scale factor is 14.375 LSB/(deg/s)
        gyro_x = self._read_word_2c(self.GYRO_ADDR, 0x1D) / 14.375
        gyro_y = self._read_word_2c(self.GYRO_ADDR, 0x1F) / 14.375

        # 3. Calculate absolute angles from Accelerometer (using atan2 for full 360 resolution)
        # Note: Math depends on your sensor's physical orientation on the robot
        acc_pitch = math.atan2(acc_y, math.sqrt(acc_x**2 + acc_z**2))
        acc_roll = math.atan2(-acc_x, acc_z)

        # 4. Integrate Gyroscope data (Convert deg/s to rad/s, then multiply by time)
        gyro_pitch_delta = math.radians(gyro_x) * dt
        gyro_roll_delta = math.radians(gyro_y) * dt

        # 5. The Complementary Filter
        # Combines the quick response of the gyro with the steady baseline of the accel
        self.pitch = self.alpha * (self.pitch + gyro_pitch_delta) + (1.0 - self.alpha) * acc_pitch
        self.roll = self.alpha * (self.roll + gyro_roll_delta) + (1.0 - self.alpha) * acc_roll

        return self.roll, self.pitch




if __name__ == "__main__":
    imu = GY85()

    while True:
        angles = imu.get_angles()
        print(f"roll:{angles[0]}\npitch:{angles[1]}")
        print("---------------------------------------")
        time.sleep(1)