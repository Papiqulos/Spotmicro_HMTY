from adafruit_servokit import ServoKit
import time
import core.kinematics as kinematics
from core.kinematics import LENGTH, WIDTH, L1, L2, L3, L4
import numpy as np
import math
from tools.utils import to_homogenous, rescale_number, trans_inv
from core.gait_controller import GaitController
import yaml
from hw.imu import IMU
from log.log_plotter import plot_log
from hw.teleop import DualSenseController

with open("config/servo_calib.yaml") as f:
    calib = yaml.safe_load(f)

LEGS   = ["fl", "fr", "rl", "rr"]
JOINTS = ["shoulder", "leg", "foot"]

_LEG_SLICE = {"FL": slice(0, 3), "FR": slice(3, 6), "RL": slice(6, 9), "RR": slice(9, 12)}

_GAIT_PARAMS = {
    "+x": dict(velocity=0.25, T_cycle=0.38, duty_factor=0.5, swing_height=0.03),
    "-x": dict(velocity=0.25, T_cycle=0.38, duty_factor=0.5, swing_height=0.035),
    "+z": dict(velocity=0.25, T_cycle=0.38, duty_factor=0.5, swing_height=0.035),
    "-z": dict(velocity=0.25, T_cycle=0.38, duty_factor=0.5, swing_height=0.035),
}


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
        
        self.init_orientation_rad = self.init_orientation_deg = np.array([0, 0, 0])
         
        # self.apply_angles_rad = self.imu.initial_orientation_rad
        # self.init_orientation_deg = self.imu.initial_orientation_deg
        
        self.rad_offset = self.imu.initial_orientation_rad
        self.deg_offset = self.imu.initial_orientation_deg
        
        self.init_ef_positions = self.kin_solver.robot_FK(
            self.init_center, self.init_orientation_deg, self.init_angles, unit="degrees"
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
        target_pos_shoulder = Ix @ trans_inv(transforms[leg]) @ to_homogenous(position)
        angles = self.kin_solver.legIK(target_pos_shoulder)
        angles = np.array([math.degrees(a) for a in angles])
        # print(angles)
        self.apply_angles_leg(leg, angles, "deg")

    def change_orientation(self, new_orientation, unit="deg"):
        if unit == "deg":
            new_orientation = [math.radians(a) for a in new_orientation]
        angles = self.kin_solver.robot_IK(self.init_center, new_orientation, self.init_ef_positions)
        self.apply_angles_robot(angles, unit="rad")

    # Step based movement
    def move(self, velocity=0.1, T_cycle=0.2, duty_factor=0.5, swing_height=0.03, steps=150, dir="+x"):
        
        time_step = 1.0 / 100
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
                current_time, time_step, T_cycle, duty_factor, velocity, swing_height, imu_data=imu_data, dir=dir,
                move_callback=self.apply_angles_leg,
            )
            elapsed = time.time() - start_time
            print(f"Elapsed: {elapsed:.2f}s", end="\r")
            
                
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
                current_time, time_step, T_cycle, duty_factor, velocity, swing_height, imu_data=imu_data, dir=dir, deceleration_flag=True,
                move_callback=self.apply_angles_leg,
            )
            elapsed = time.time() - start_time
            print(f"Elapsed: {elapsed:.2f}s", end="\r")
            if ef_vel == 0.0:
                self.apply_angles_robot(self.init_angles)
                print("Deceleration complete!")
                break
        
        plot_log(log_file)

    def go_forwards(self):
        self.move(velocity=0.25, T_cycle=0.38, duty_factor=0.5, swing_height=0.03, steps=200, dir="+x")

    def go_backwards(self):
        self.move(velocity=0.15, T_cycle=0.4, duty_factor=0.5, swing_height=0.035, steps=80, dir="-x")
        
    def go_right(self):
        self.move(velocity=0.15, T_cycle=0.4, duty_factor=0.5, swing_height=0.035, steps=80, dir="+z")
    
    def go_left(self):
        self.move(velocity=0.15, T_cycle=0.4, duty_factor=0.5, swing_height=0.035, steps=80, dir="-z")

    # DualSense based movement
    def trot_step(self, current_time, time_step, dir="+x", deceleration_flag=False):
        raw_gyro = self.imu.gyro.read()
        raw_acc  = self.imu.accelerometer.read()["acceleration"]
        imu_data = self.imu.update(raw_gyro, raw_acc)
        p = _GAIT_PARAMS[dir]
        return self.gait_controller.trot(
            current_time, time_step, p["T_cycle"], p["duty_factor"],
            p["velocity"], p["swing_height"],
            imu_data=imu_data, dir=dir,
            deceleration_flag=deceleration_flag,
            move_callback=self.apply_angles_leg,
        )
            
            

if __name__ == "__main__":
    center = [0, 0, 0]
    theta_default = [
        0, -40, 61.86,  # FL
        0, -40, 61.86,  # FR
        0, -40, 61.86,  # RL
        0, -40, 61.86,  # RR
    ]

    kin_solver = kinematics.Kinematics(LENGTH, WIDTH, L1, L2, L3, L4)
    robot = RobotController(kin_solver, theta_default, center)
    robot.gait_controller._set_pid(kp=0.4, ki=0.01, kd=0.005)

    teleop = DualSenseController()

    state = "idle"
    current_dir = "+x"
    start_time = time.time()
    time_step = 1.0 / 100
    log_file = None

    try:
        while True:
            t0 = time.time()
            current_time = t0 - start_time

            dpad_up    = teleop.dualsense.state.DpadUp
            dpad_down  = teleop.dualsense.state.DpadDown
            dpad_right = teleop.dualsense.state.DpadRight
            dpad_left  = teleop.dualsense.state.DpadLeft
            circle     = teleop.dualsense.state.circle

            if circle:
                break

            if dpad_up:
                current_dir, state = "+x", "moving"
            elif dpad_down:
                current_dir, state = "-x", "moving"
            elif dpad_right:
                current_dir, state = "+z", "moving"
            elif dpad_left:
                current_dir, state = "-z", "moving"
            elif state == "moving":
                state = "decelerating"

            if state == "moving":
                ef_vel, log_file = robot.trot_step(current_time, time_step, current_dir)
                print(f"\033[2KMoving {current_dir}  vel={ef_vel:.3f}", end="\r", flush=True)
            elif state == "decelerating":
                ef_vel, log_file = robot.trot_step(current_time, time_step, current_dir, deceleration_flag=True)
                print(f"\033[2KDecelerating...  vel={ef_vel:.3f}", end="\r", flush=True)
                if ef_vel == 0.0:
                    robot.apply_angles_robot(robot.init_angles)
                    state = "idle"
            else:
                print("\033[2KWaiting for command. Press circle to exit...", end="\r", flush=True)

            elapsed = time.time() - t0
            rem = time_step - elapsed
            if rem > 0:
                time.sleep(rem)
    
    except:
        print("Plug in your DualSense controller")
    finally:
        if log_file:
            plot_log(log_file)
        teleop.dualsense.close()

