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
from core.robot_state import RobotState

with open("config/servo_calib.yaml") as f:
    calib = yaml.safe_load(f)

LEGS   = ["FL", "FR", "RL", "RR"]
JOINTS = ["shoulder", "leg", "foot"]
_LEG_SLICE = {"FL": slice(0, 3), "FR": slice(3, 6), "RL": slice(6, 9), "RR": slice(9, 12)}

class RobotController:

    def __init__(self, kin_solver, init_angles=None, init_ef_positions=None, init_center=[0, 0, 0]):
        """
        :param kin_solver: kinematics solver
        :param init_angles: initial joint angles in degrees
        :param init_ef_positions: initial foot positions in kinematics frame (mm, Y-up)
        :param init_center: body center in kinematics frame (mm)

        Given init_angles OR init_ef_positions NOT BOTH, the robot will be initialized with the given angles.
        If both are given, the ef_positions will be ignored.
        """
        self.kin_solver = kin_solver
        self.init_center = init_center
        self._live_status = ""
        self.zeros      = [calib[leg][joint]["zero_deg"]  for leg in LEGS for joint in JOINTS]
        self.indexes    = [calib[leg][joint]["channel"]   for leg in LEGS for joint in JOINTS]
        self.theta_dirs = [calib[leg][joint]["direction"] for leg in LEGS for joint in JOINTS]
        self.kit_index       = [calib[leg][joint]["kit"]       for leg in LEGS for joint in JOINTS]
        self.kit_front_obj = ServoKit(channels=16)
        self.kit_rear_obj  = ServoKit(channels=16, address=0x41)

        self.imu = IMU(filter_type="EKF")
        self.init_orientation_rad = self.init_orientation_deg = np.array([0, 0, 0])

        if init_angles is None:
            self.init_ef_positions = init_ef_positions
            self.init_angles = self.kin_solver.robot_IK(self.init_center, 
                                                        self.init_orientation_deg, 
                                                        self.init_ef_positions, 
                                                        unit="degrees")
            
        else:
            self.init_angles = init_angles
            self.init_ef_positions = self.kin_solver.robot_FK(self.init_center, 
                                                              self.init_orientation_deg, 
                                                              self.init_angles, 
                                                              unit="degrees"
            )
            
        if init_angles is not None and init_ef_positions is not None:
            print("WARNING: Both init_angles and init_ef_positions were given. init_ef_positions will be ignored.")

        init_angles_rad = np.array([math.radians(a) for a in self.init_angles])
        self.state = RobotState(
            init_angles=init_angles_rad,
            init_ef_positions=np.array(self.init_ef_positions, dtype=float),
            init_center=self.init_center,
            init_orientation=self.init_orientation_rad,
            kin_solver=self.kin_solver,
        )
        # self.state.angles = np.zeros(12)

        self.gait_controller = GaitController(self.state)

        # self.apply_angles_robot([0.0] * 12, unit="deg")
        
        self.gait_controller.smooth_to_target(
            np.array(self.init_angles, dtype=float),
            duration=1.0,
            move_callback=self.apply_angles_robot,
            unit="deg",
        )
        self.show_state()
        time.sleep(1)
        
    def _write_servo(self, servo_index, angle):
        try:
            if self.kit_index[servo_index] == 1:
                
                self.kit_front_obj.servo[self.indexes[servo_index]].angle = angle
            elif self.kit_index[servo_index] == 2:
                
                self.kit_rear_obj.servo[self.indexes[servo_index]].angle = angle

           
        except ValueError as e:
            print(f"Servo {self.indexes[servo_index]} out of range: {angle:.1f}° — {e}")

    def apply_angles_robot(self, angles, unit="deg"):
        if unit == "rad":
            angles_rad = list(angles)
            angles_deg = [math.degrees(a) for a in angles]
        else:
            angles_deg = list(angles)
            angles_rad = [math.radians(a) for a in angles]
        for i, angle in enumerate(angles_deg):
            angle = angle * self.theta_dirs[i]
            angle = rescale_number(angle, 0, 180, self.zeros[i], self.zeros[i] + 180)
            # TODO: Add threaded version
            self._write_servo(i, angle)
        self.state.angles = angles_rad

    def apply_angles_leg(self, leg, angles, unit="deg"):
        if unit == "rad":
            angles_rad = list(angles)
            angles_deg = [math.degrees(a) for a in angles]
        else:
            angles_deg = list(angles)
            angles_rad = [math.radians(a) for a in angles]
        s = _LEG_SLICE[leg]
        dirs  = self.theta_dirs[s]
        zeros = self.zeros[s]
        for i, angle in enumerate(angles_deg):
            angle = angle * dirs[i]
            angle = rescale_number(angle, 0, 180, zeros[i], zeros[i] + 180)
            self._write_servo(s.start + i, angle)
        current = list(self.state.angles) if self.state.angles is not None else [0.0] * 12
        for j, a in enumerate(angles_rad):
            current[s.start + j] = a
        self.state.angles = current  

    def drive_leg_to_position(self, leg, position):
        orientation = self.init_orientation_rad
        (T_fl, T_fr, T_rl, T_rr) = self.kin_solver.bodyIK(*orientation, *self.init_center)
        transforms = {"FL": T_fl, "FR": T_fr, "RL": T_rl, "RR": T_rr}
        Ix = self.kin_solver.Ix if leg in ("FR", "RR") else np.identity(4)
        target_pos_shoulder = Ix @ trans_inv(transforms[leg]) @ to_homogenous(position)
        angles = self.kin_solver.legIK(target_pos_shoulder)
        angles = np.array([math.degrees(a) for a in angles])
        self.apply_angles_leg(leg, angles, "deg")
        self.state.linear_vel  = 0.0
        self.state.angular_vel = 0.0

    def change_orientation(self, new_orientation, unit="deg"):
        """Change the robot orientation to new_orientation.
        
        :param new_orientation: new_orientation in given unit
        :param unit:            unit of given and new_orientation ("deg" or "rad")
        """
        if unit == "deg":
            new_orientation = np.radians(new_orientation)
        angles = self.kin_solver.robot_IK(self.init_center, new_orientation, self.state.ef_positions)
        print(angles)
        self.gait_controller.smooth_to_target(angles, move_callback=self.apply_angles_robot, unit="rad")

    def move(self, 
            params=None,
            steps=150):
        """Execute a certain number of steps in a given direction with given linear and angular velocity and log the results. Mainly used to tune gait parameters.
        
        
        :param desired_lin_vel: desired linear velocity in m/s
        :param desired_ang_vel: desired angular velocity in rad/s
        :param swing_height: peak foot clearance above nominal in meters
        :param stance_length: full step length in meters; Tstance = stance_length / velocity
        :param Tswing: fixed swing duration in seconds (servo-limited, ~0.2–0.25 s)
        :param dir: direction of motion "+x" / "-x" / "+z" / "-z" or a lateral_fraction float (rad)
        :param gait_type: trot / walk / bound / pace / pronk
        :param steps: number of steps to execute

        
        """
        time_step = 1.0 / 100
        start_time = time.time()
        self.imu._imu_window.clear()
        self.gait_controller.reset(kp=0.4, ki=0.025, kd=0.05)
        if not params:
            # Default parameters
            # params = dict(desired_lin_vel=0.2,
            #               desired_ang_vel=0.0,
            #               swing_height=0.040,
            #               stance_length=0.05,
            #               Tswing=0.25,
            #               dir="+x",
            #               gait_type="trot")
            params = dict(desired_lin_vel=0.3, 
                          desired_ang_vel=0.0, 
                          T_cycle=0.25, 
                          duty_factor=0.5,
                          swing_height=0.035,
                          dir="+x",  
                          gait_type="trot")
        
        self._start_live_display()
        self._live_status = "Running"
        for _ in range(steps):
            t0 = time.time()
            current_time = t0 - start_time

            ef_vel, log_file, imu_data = self.trot_step(current_time, time_step, params=params, deceleration_flag=False, verbose=True)

        self._live_status = "Decelerating"
        while True:

            t0 = time.time()
            current_time = t0 - start_time

            ef_vel, log_file, imu_data = self.trot_step(current_time, time_step, params=params, deceleration_flag=True, verbose=True)
            if ef_vel == 0.0:
                # Verbose output
                print(f"\033[7B", end='', flush=True)
                self._live_status = ""
                self.apply_angles_robot(self.init_angles)
                break
        
        # plot_log(log_file)

    def show_state(self, end='\n'):
        """Print verbose robot state to the terminal."""
        # HOLY VIBE CODING
        orn = np.degrees(self.state.orientation)
        if end == '\r':
            lines = []
            for i, leg in enumerate(LEGS):
                s, lj, f = np.degrees(self.state.angles[i*3 : i*3+3])
                x, y, z  = self.state.ef_positions[i][:3]
                lines.append(f"  {leg:<4} {s:>+7.1f}° {lj:>+7.1f}° {f:>+7.1f}°   {x:>8.2f} {y:>8.2f} {z:>8.2f}")
            d = self.state.direction
            dir_str = d if isinstance(d, str) else (f"{np.degrees(d):.0f}°" if d is not None else "?")
            lines.append(
                f"  dir={dir_str:<4}"
                f"  roll={orn[0]:>+6.2f}°  pitch={orn[1]:>+6.2f}°  yaw={orn[2]:>+6.2f}°"
            )
            lines.append(
                f"  vel={self.state.linear_vel:>+7.3f} m/s  ang={self.state.angular_vel:>+7.3f} r/s"
            )
            lines.append(f"  {self._live_status}")
            for line in lines:
                print(f"\033[2K{line}")
            print(f"\033[{len(lines)}A", end='', flush=True)
        else:
            print(f"\n{'':=<52}")
            print(f"  {'Leg':<6} {'Shoulder':>10} {'Leg':>10} {'Foot':>10}   {'X':>8} {'Y':>8} {'Z':>8}")
            print(f"  {'':-<6} {'(deg)':>10} {'(deg)':>10} {'(deg)':>10}   {'(mm)':>8} {'(mm)':>8} {'(mm)':>8}")
            for i, leg in enumerate(LEGS):
                s, lj, f = np.degrees(self.state.angles[i*3 : i*3+3])
                x, y, z  = self.state.ef_positions[i][:3]
                print(f"  {leg:<6} {s:>10.2f} {lj:>10.2f} {f:>10.2f}   {x:>8.2f} {y:>8.2f} {z:>8.2f}")
            print(f"  roll={orn[0]:.2f}°  pitch={orn[1]:.2f}°  yaw={orn[2]:.2f}°")
            print(f"  vel={self.state.linear_vel:.3f} m/s  ang={self.state.angular_vel:.3f} r/s")
            print(f"{'':=<52}\n")
    
    def _start_live_display(self):
        # Verbose output
        # HOLY VIBE CODING
        """Reserve 7 terminal lines for the live block so subsequent writes never trigger scroll."""
        N = 7
        print('\n' * N, end='', flush=True)
        print(f'\033[{N}A', end='', flush=True)

    # DualSense based movement
    def trot_step(self, current_time, time_step, params=None, deceleration_flag=False, verbose=False):
        raw_gyro = self.imu.gyro.read()
        raw_acc  = self.imu.accelerometer.read()["acceleration"]
        imu_data = self.imu.update(raw_gyro, raw_acc)
        imu_data_in_deg = np.degrees(imu_data)
        if abs(imu_data_in_deg[1]) > 45:
            print("Pitch > 45° — halting", flush=True)
            return None, None, imu_data
        if not params:
            # Default parameters
            params = dict(desired_lin_vel=0.2,
                          desired_ang_vel=0.0,
                          swing_height=0.040,
                          stance_length=0.05,
                          Tswing=0.25,
                          dir="+x",
                          gait_type="trot")
            # params = dict(desired_lin_vel=0.1, 
            #               desired_ang_vel=0.0, 
            #               T_cycle=0.25, 
            #               duty_factor=0.5,
            #               swing_height=0.035,
            #               dir="+x",  
            #               gait_type="trot")
        ef_vel, log_file = self.gait_controller.execute_gait_fixed_stance(
            current_time, time_step, imu_data=imu_data, deceleration_flag=deceleration_flag, move_callback=self.apply_angles_leg,
            desired_lin_vel=params["desired_lin_vel"],
            desired_ang_vel=params["desired_ang_vel"],
            swing_height=params["swing_height"],
            stance_length=params["stance_length"],
            Tswing=params["Tswing"],
            dir=params["dir"],
            gait_type=params["gait_type"])
        # ef_vel, log_file = self.gait_controller.execute_gait_fixed_stance_old(
        #     current_time, time_step, imu_data=imu_data, deceleration_flag=deceleration_flag, move_callback=self.apply_angles_leg,
        #     desired_lin_vel=params["desired_lin_vel"], 
        #     desired_ang_vel=params["desired_ang_vel"], 
        #     T_cycle=params["T_cycle"], 
        #     duty_factor=params["duty_factor"],
        #     swing_height=params["swing_height"],
        #     dir=params["dir"],
        #     gait_type=params["gait_type"])
        if verbose:
            self.show_state(end='\r')
        return ef_vel, log_file, imu_data
    
if __name__ == "__main__":
    center = [0, 0, 0]
    theta_default = np.array([
        0, -45, 60,  # FL
        0, -45, 60,  # FR
        0, -45, 60,  # RL
        0, -45, 60,  # RR
    ])

    ef_dafault = np.array([[92.25, -223.09,  93.94, 1],
                           [92.25, -223.09, -93.94, 1],
                           [-92.25, -223.09,  93.94, 1],
                           [-92.25, -223.09, -93.94, 1]])
    

    kin_solver = kinematics.Kinematics(LENGTH, WIDTH, L1, L2, L3, L4)
    robot = RobotController(kin_solver, init_angles=theta_default)


    robot.change_orientation([0, -10, 0])
    robot.show_state()
    


    # Test a gait
    params = dict(desired_lin_vel=0.12, 
            desired_ang_vel=0.0, 
            swing_height=0.035, 
            stance_length=0.05, 
            Tswing=0.2, 
            dir="+x",  
            gait_type="trot")
    # params = dict(desired_lin_vel=0.1, 
    #             desired_ang_vel=0.0, 
    #             T_cycle=0.25, 
    #             duty_factor=0.5,
    #             swing_height=0.035,
    #             dir="+x",  
    #             gait_type="trot")
    
    # robot.move(params=params, steps=80)

    # try:
    #     log_file = None
    #     teleop = DualSenseController()

    #     state = "Idle"
    #     current_dir = "+x"
    #     start_time = time.time()
    #     time_step = 1.0 / 100

        
        
    #     robot.gait_controller.reset(kp_r=0.5, ki_r=0.025, kd_r=0.06,
    #                                 kp_p=0.4, ki_p=0.05,  kd_p=0.03)
    #     robot._start_live_display()
    #     while not teleop.dualsense.state.circle:
    #         t0 = time.time()
    #         current_time = t0 - start_time

    #         # Joysticks
    #         left_joystick_motion = teleop._joystick_in_motion(joystick="l")
    #         right_joystick_motion = teleop._joystick_in_motion(joystick="r")

    #         left_joystick_angle, right_joystick_angle = teleop._get_joystick_angle()

    #         # D-pad
    #         dpad_up    = teleop.dualsense.state.DpadUp
    #         dpad_down  = teleop.dualsense.state.DpadDown
    #         dpad_right = teleop.dualsense.state.DpadRight
    #         dpad_left  = teleop.dualsense.state.DpadLeft

    #         # Right buttons
    #         square     = teleop.dualsense.state.square
    #         triangle   = teleop.dualsense.state.triangle
    #         circle     = teleop.dualsense.state.circle
    #         cross      = teleop.dualsense.state.cross

    #         # Bumpers
    #         r_bumper   = teleop.dualsense.state.R1
    #         l_bumper   = teleop.dualsense.state.L1

    #         # Triggers
    #         r_trigger  = teleop.dualsense.state.R2
    #         l_trigger  = teleop.dualsense.state.L2
            

    #         if dpad_up:
    #             params = dict(desired_lin_vel=0.12, 
    #                         desired_ang_vel=0.0, 
    #                         swing_height=0.035, 
    #                         stance_length=0.06, 
    #                         Tswing=0.2, 
    #                         dir="+x",  
    #                         gait_type="trot")
    #             # params = dict(desired_lin_vel=0.15, 
    #             #           desired_ang_vel=0.0, 
    #             #           T_cycle=0.4, 
    #             #           duty_factor=0.5,
    #             #           swing_height=0.035,
    #             #           dir="-x",  
    #             #           gait_type="trot")
    #             state = "Running"
    #         elif dpad_down:
    #             params = dict(desired_lin_vel=0.12, 
    #                         desired_ang_vel=0.0, 
    #                         swing_height=0.035, 
    #                         stance_length=0.06, 
    #                         Tswing=0.2, 
    #                         dir="-x",  
    #                         gait_type="trot")
    #             state = "Running"
    #         elif dpad_right:
    #             params = dict(desired_lin_vel=0.12, 
    #                         desired_ang_vel=0.0, 
    #                         swing_height=0.035, 
    #                         stance_length=0.06, 
    #                         Tswing=0.2, 
    #                         dir="+z",  
    #                         gait_type="trot")
    #             state = "Running"
    #         elif dpad_left:
    #             params = dict(desired_lin_vel=0.12, 
    #                     desired_ang_vel=0.0, 
    #                     swing_height=0.035, 
    #                     stance_length=0.06, 
    #                     Tswing=0.2, 
    #                     dir="-z",  
    #                     gait_type="trot")
    #             state = "Running"
    #         # elif left_joystick_motion:
    #         #     params = dict(desired_lin_vel=0.12, 
    #         #             desired_ang_vel=0.0, 
    #         #             swing_height=0.035, 
    #         #             stance_length=0.06, 
    #         #             Tswing=0.2, 
    #         #             dir=left_joystick_angle,  
    #         #             gait_type="trot")
    #         #     state = "Running"
    #         elif r_bumper:
    #             params = dict(desired_lin_vel=0, 
    #                         desired_ang_vel=0.1, 
    #                         swing_height=0.035, 
    #                         stance_length=0.06, 
    #                         Tswing=0.2, 
    #                         dir="+x",  
    #                         gait_type="trot")
    #             state = "Running"
    #         elif l_bumper:
    #             params = dict(desired_lin_vel=0, 
    #                         desired_ang_vel=-0.1, 
    #                         swing_height=0.035, 
    #                         stance_length=0.06, 
    #                         Tswing=0.2, 
    #                         dir="+x",  
    #                         gait_type="trot")
    #             state = "Running"
    #         elif state == "Running":
    #             state = "Decelerating"
            

    #         if state == "Running":
    #             robot._live_status = state
    #             ef_vel, log_file, imu_data = robot.trot_step(current_time, time_step, params=params, deceleration_flag=False, verbose=True)
    #         elif state == "Decelerating":
    #             robot._live_status = state
    #             ef_vel, log_file, imu_data = robot.trot_step(current_time, time_step, params=params, deceleration_flag=True, verbose=True)

    #             if ef_vel == 0.0:
    #                 robot.apply_angles_robot(robot.init_angles)
    #                 state = "Idle"
    #         else:
    #             robot._live_status = "Idle — waiting for command"
    #             robot.show_state("\r")

    #         elapsed = time.time() - t0
    #         rem = time_step - elapsed
    #         if rem > 0:
    #             time.sleep(rem)
    #     # Verbose output
    #     print(f"\033[7B", end='', flush=True)
    # except Exception as e:
    #     if e.args[0] == "No device detected":
    #         teleop = None
    #         print(f"No device detected\nPlug in your DualSense controller")
    #     else:
    #         print(f"Error: {e}")
    # finally:
    #     if log_file:
    #         plot_log(log_file)
    #     if teleop:
    #         teleop.dualsense.close()

