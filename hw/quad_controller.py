from adafruit_servokit import ServoKit
import time
import core.kinematics as kinematics
from core.kinematics import LENGTH, WIDTH, L1, L2, L3, L4
import numpy as np
import math
from tools.utils import to_homogenous, rescale_number
from core.gait_controller import GaitController, _LOG_DIR
import yaml
from hw.imu import IMU
from log.log_plotter import plot_log
# import teleop
from pydualsense import *

with open("config/servo_calib.yaml") as f:
    calib = yaml.safe_load(f)

LEGS   = ["fl", "fr", "rl", "rr"]
JOINTS = ["shoulder", "leg", "foot"]

_LEG_SLICE = {"FL": slice(0, 3), "FR": slice(3, 6), "RL": slice(6, 9), "RR": slice(9, 12)}


class RobotController:

    def __init__(self, kin_solver, init_angles, init_center):
        self.kin_solver = kin_solver
        self.init_center = init_center
        self.init_angles = init_angles
        

        leg_names = ["FL", "FR", "RL", "RR"]
        

        self.zeros      = [calib[leg][joint]["zero_deg"]  for leg in LEGS for joint in JOINTS]
        self.indexes    = [calib[leg][joint]["channel"]   for leg in LEGS for joint in JOINTS]
        self.theta_dirs = [calib[leg][joint]["direction"] for leg in LEGS for joint in JOINTS]
        self.kits       = [calib[leg][joint]["kit"]       for leg in LEGS for joint in JOINTS]
      

        self.kit_front = ServoKit(channels=16)
        self.kit_rear  = ServoKit(channels=16, address=0x41)

        self.imu = IMU(filter_type="EKF")
        
        self.apply_angles_robot(self.init_angles)
        
        self.init_orientation_rad = np.array([0, 0, 0])
        self.init_orientation_deg = self.imu.initial_orientation_deg
        
        self.rad_offset = self.imu.initial_orientation_rad
        self.deg_offset = self.imu.initial_orientation_deg
        
        self.init_ef_positions = self.kin_solver.robot_FK(
            self.init_center, self.init_orientation_rad, self.init_angles, unit="degrees"
        )
        self.gait_controller = GaitController(
            initial_ef_positions=self.init_ef_positions,
            initial_theta=init_angles,
            initial_center=init_center,
            initial_orientation=self.init_orientation_rad,
        )
        
        
        print("--- Robot Initial Angles ---")
        for i, leg in enumerate(leg_names):
            shoulder, leg_joint, foot = self.init_angles[i*3 : i*3+3]
            print(f"{leg}: shoulder={shoulder:.2f}, leg={leg_joint:.2f}, foot={foot:.2f}")

        print("\n--- Robot Initial Positions ---")
        for i, leg in enumerate(leg_names):
            x, y, z = self.init_ef_positions[i][:3]
            print(f"{leg}: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        time.sleep(1)
        
    

    def _write_servo(self, i, angle):
        try:
            if self.kits[i] == 1:
                
                self.kit_front.servo[self.indexes[i]].angle = angle
            elif self.kits[i] == 2:
                
                self.kit_rear.servo[self.indexes[i]].angle = angle
        except ValueError as e:
            print(f"Servo {self.indexes[i]} out of range: {angle:.1f}° — {e}")

    def apply_angles_robot(self, angles, unit="deg"):
        if unit == "rad":
            angles = [math.degrees(a) for a in angles]
        for i, angle in enumerate(angles):
            angle = angle * self.theta_dirs[i]
            angle = rescale_number(angle, 0, 180, self.zeros[i], self.zeros[i] + 180)
            self._write_servo(i, angle)

    def apply_angles_leg(self, leg, angles, unit="deg"):
        if unit == "rad":
            angles = [math.degrees(a) for a in angles]
        s = _LEG_SLICE[leg]
        dirs  = self.theta_dirs[s]
        zeros = self.zeros[s]
        for i, angle in enumerate(angles):
            angle = angle * dirs[i]
            angle = rescale_number(angle, 0, 180, zeros[i], zeros[i] + 180)
            self._write_servo(s.start + i, angle)

    def drive_leg_to_position(self, leg, position):
        orientation = self.init_orientation_rad
        (T_fl, T_fr, T_rl, T_rr) = self.kin_solver.bodyIK(*orientation, *self.init_center)
        transforms = {"FL": T_fl, "FR": T_fr, "RL": T_rl, "RR": T_rr}
        Ix = self.kin_solver.Ix if leg in ("FR", "RR") else np.identity(4)
        target_pos_shoulder = Ix @ np.linalg.inv(transforms[leg]) @ to_homogenous(position)
        angles = self.kin_solver.legIK(target_pos_shoulder)
        self.apply_angles_leg(leg, angles, "rad")

    def change_orientation(self, new_orientation, unit="deg"):
        if unit == "deg":
            new_orientation = [math.radians(a) for a in new_orientation]
        angles = self.kin_solver.robot_IK(self.init_center, new_orientation, self.init_ef_positions)
        self.apply_angles_robot(angles, unit="rad")

    # Step based movement
    def move(self, velocity=0.1, T_cycle=0.2, duty_factor=0.5, swing_height=0.03, steps=150, dir="+x"):
        
        loop_period = 1.0 / 100
        start_time = time.time()
        self.gait_controller._set_pid(kp=0.4, ki=0.01, kd=0.005)
        
        for _ in range(steps):
            t0 = time.time()
            current_time = t0 - start_time
            
            raw_gyro = self.imu.gyro.read()
            raw_acc  = self.imu.accelerometer.read()["acceleration"]
            # raw_mag  = self.imu.magnetometer.get_magnetometer()["magnet"] # too noisy
            imu_data = self.imu.update(raw_gyro, raw_acc)
            # print(f"IMU: {imu_data}")
            
            
            ef_vel, log_file = self.gait_controller.trot(
                current_time, loop_period, T_cycle, duty_factor, velocity, swing_height, imu_data=imu_data, dir=dir,
                move_callback=self.apply_angles_leg,
            )
            elapsed = time.time() - t0
            remaining = loop_period - elapsed
            if remaining > 0:
                time.sleep(remaining)
                
        # After the given number steps or on button unpress, we decelerate into the default position
        print("\nDecelerating...")
        while True:
            
            t0 = time.time()
            current_time = t0 - start_time
            
            raw_gyro = self.imu.gyro.read()
            raw_acc  = self.imu.accelerometer.read()["acceleration"]
            # raw_mag  = self.imu.magnetometer.get_magnetometer()["magnet"] # too noisy
            imu_data = self.imu.update(raw_gyro, raw_acc)
            # print(f"IMU: {imu_data}")
            
            
            ef_vel, log_file = self.gait_controller.trot(
                current_time, loop_period, T_cycle, duty_factor, velocity, swing_height, imu_data=imu_data, dir=dir, deceleration_flag=True,
                move_callback=self.apply_angles_leg,
            )
            
            if ef_vel == 0.0:
                self.apply_angles_robot(self.init_angles)
                print("Deceleration complete!")
                break
        plot_log(log_file)

    def go_forwards(self):
        self.move(velocity=0.21, T_cycle=0.4, duty_factor=0.5, swing_height=0.03, steps=150, dir="+x")

    def go_backwards(self):
        self.move(velocity=0.15, T_cycle=0.4, duty_factor=0.5, swing_height=0.035, steps=80, dir="-x")
        
    def go_right(self):
        self.move(velocity=0.15, T_cycle=0.4, duty_factor=0.5, swing_height=0.035, steps=80, dir="+z")
    
    def go_left(self):
        self.move(velocity=0.15, T_cycle=0.4, duty_factor=0.5, swing_height=0.035, steps=80, dir="-z")  
          
    # Button based movement
    def button_go(self, state):
        

        if state:
            # self.go_forwards()
            print("Going")
            
        else:
            print("Stopped")
            
            

if __name__ == "__main__":
        orientation = [0, 0, 0] # static orientation
        center = [0, 270, 0]

        theta_default = [
                0, -30, 60,  # FL
                0, -30, 60,  # FR
                0, -30, 60,  # RL
                0, -30, 60,  # RR
        ]

        kin_solver = kinematics.Kinematics(LENGTH, WIDTH, L1, L2, L3, L4)
        robot_controller = RobotController(kin_solver, theta_default, center)
        # robot_controller.drive_leg_to_position("FL", [31.93, 69.69, 107.00])
        # robot_controller.apply_angles_leg("RR", [0, 0, 0])
        # robot_controller.change_orientation([0, -10, 0])
        robot_controller.go_forwards()
        # robot_controller.go_backwards()
        # robot_controller.go_right()
        # robot_controller.go_left()
     
        
        
        
