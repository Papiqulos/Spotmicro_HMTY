import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np
import core.bezier_curve_gen as bezier
import sim.kinematics_old as kinematics_old
from tools.utils import from_pybullet_orn, from_pybullet_pos, to_homogenous, normalize_angle
import time
from tools.pid_controller import PIDController, PIDControllerRP


# MIT Cheetah 12-point Bezier swing parameters (normalized, from Bledt et al.)
# _X: horizontal progression 0=liftoff, 1=touchdown
#     3 clustered at each end -> zero endpoint tangent velocity (smooth lift/land)
# _H: height factor (1.0 = swing_height, 1.1 = slight overshoot for natural arc)
_SWING_X_NORM = [0.00, 0.00, 0.00, 0.15, 0.30, 0.45, 0.55, 0.70, 0.85, 1.00, 1.00, 1.00]
_SWING_H_NORM = [0.00, 0.10, 0.80, 1.00, 1.10, 1.10, 1.10, 1.10, 1.00, 0.80, 0.10, 0.00]

L1 = kinematics_old.L1
L2 = kinematics_old.L2
L3 = kinematics_old.L3
L4 = kinematics_old.L4
LENGTH = kinematics_old.LENGTH
WIDTH = kinematics_old.WIDTH

class GaitController:
    
    def __init__(self, initial_ef_positions=None, initial_theta=None, initial_center=None, initial_orientation=None):
        """
        :param initial_ef_positions: in pybullet frame and not homogenous coordinates
        :param initial_theta: 
        :param initial_center: in kinematics frame
        :param initial_orientation: in kinematics frame
        """
        self.initial_ef_positions = initial_ef_positions 
        self.initial_theta = initial_theta
        self.initial_center = initial_center
        self.initial_orientation = initial_orientation
        self.theta_dirs = [[-1, 1, 1],
                            [1, 1, 1],
                            [-1, 1, 1], 
                            [1, 1, 1]]
        self.kin_solver = kinematics_old.Kinematics(length=LENGTH, width=WIDTH, l1=L1, l2=L2, l3=L3, l4=L4)

        self.gait_init = None
        self.deceleration_init = 0

        self.pid_roll = PIDController(kp=0.2, ki=0.025, kd=0.025)
        self.pid_pitch = PIDController(kp=0.2, ki=0.025, kd=0.025)
        self.pid_yaw = PIDController(kp=0.2, ki=0.0, kd=0.025)

        self.pid_rp = PIDControllerRP(kp=0.2, ki=0.025, kd=0.025)

    # NOT USED
    def generate_bezier_trajectory(self, control_points, num_points=100, leg="FL"):

        """
        
        :param control_points: in pybullet frame and not homogenous coordinates
        :param num_points: 
        :param leg: 
        """
        bezier_gen = bezier.BezierCurveGen(control_points)
        curve = bezier_gen.generate_curve(num_points=num_points)

        # Get the angles of the trajectory points
        joint_angles = []

        # T_shoulder_base for each leg
        (T_fl, T_fr, T_rl, T_rr) = self.kin_solver.bodyIK(*self.initial_orientation, *self.initial_center)

        # Placeholder for Inversion Matrix for right legs
        Ix = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

        if leg == "FL":
            T_shoulder_base = T_fl
        elif leg == "FR":
            T_shoulder_base = T_fr
            Ix = self.kin_solver.Ix
        elif leg == "RL":
            T_shoulder_base = T_rl
        elif leg == "RR":
            T_shoulder_base = T_rr
            Ix = self.kin_solver.Ix
            
        
        for point in curve:
            point = from_pybullet_pos(point)
            point = to_homogenous(point)  # Homogeneous coordinates

            # The point is given in world frame, we need to convert it to be relative to the shoulder joint
            point = Ix @ np.linalg.inv(T_shoulder_base) @ point  
            angles = self.kin_solver.legIK(point) 
            joint_angles.append(angles)

        return joint_angles, curve, control_points

    def swing_trajectory_control_points(self, initial_pos, sl_mm, sh_mm, dir="+x"):
        """12-point MIT Cheetah Bezier swing control points.
        Frame: kinematics (X-forward, Y-up, Z-lateral), units mm.

        3-point clusters at liftoff/touchdown force zero endpoint tangent velocity.
        1.1x height overshoot gives a natural parabolic crown.
        """
        pts = []
        if dir == "+x":
            x0 = initial_pos[0] - sl_mm / 2
            for xn, hn in zip(_SWING_X_NORM, _SWING_H_NORM):
                pts.append(np.array([x0 + xn * sl_mm, initial_pos[1] + hn * sh_mm, initial_pos[2]]))
        elif dir == "-x":
            x0 = initial_pos[0] + sl_mm / 2
            for xn, hn in zip(_SWING_X_NORM, _SWING_H_NORM):
                pts.append(np.array([x0 - xn * sl_mm, initial_pos[1] + hn * sh_mm, initial_pos[2]]))
        elif dir == "+y":
            z0 = initial_pos[2] - sl_mm / 2
            for xn, hn in zip(_SWING_X_NORM, _SWING_H_NORM):
                pts.append(np.array([initial_pos[0], initial_pos[1] + hn * sh_mm, z0 + xn * sl_mm]))
        elif dir == "-y":
            z0 = initial_pos[2] + sl_mm / 2
            for xn, hn in zip(_SWING_X_NORM, _SWING_H_NORM):
                pts.append(np.array([initial_pos[0], initial_pos[1] + hn * sh_mm, z0 - xn * sl_mm]))
        return pts

    def stance_sine_trajectory(self, initial_pos, sl_mm, stance_progress, delta, dir="+x"):
        """Stance foot position with MIT Cheetah complementary sine dip.

        Horizontal: linear sweep from +sl/2 (front) to -sl/2 (back).
        Vertical (Y-up): y0 - delta * sin(pi * t)

        As the body COM rises slightly at mid-stance during a trot, commanding
        the foot delta lower maintains consistent ground contact force.
        sin(pi*t) completes exactly one half-cycle over stance (0->peak->0).
        """
        t = stance_progress
        y = initial_pos[1] - delta * np.sin(np.pi * t)
        if dir == "+x":
            return np.array([initial_pos[0] + sl_mm / 2 - t * sl_mm, y, initial_pos[2]])
        elif dir == "-x":
            return np.array([initial_pos[0] - sl_mm / 2 + t * sl_mm, y, initial_pos[2]])
        elif dir == "+y":
            return np.array([initial_pos[0], y, initial_pos[2] + sl_mm / 2 - t * sl_mm])
        elif dir == "-y":
            return np.array([initial_pos[0], y, initial_pos[2] - sl_mm / 2 + t * sl_mm])


    def trot(self, current_time, T_cycle, duty_factor, desired_velocity, swing_height, p, robotId, imu_data=None, dir="+x", deceleration_flag=False):
        """
        Make the robot trot in the specified direction
        
        :param current_time: current simulation time
        :param T_cycle: cycle time in seconds
        :param duty_factor: duty factor
        :param desired_velocity: desired velocity in meters per second
        :param swing_height: swing height in meters
        :param p: pybullet client
        :param robotId: robot id
        :param imu_data: imu data
        :param dir: direction of the trot (pybullet frame)


        """
        # print(f"current_time : {current_time}")

        # Ramp up the velocity so that the robot doesn't start moving too fast and turns to the left
        
        if self.gait_init is None:
            self.gait_init = current_time
            # print(f"Gait Init : {self.gait_init}")

        ramp_duration = 0.5

        # Calculate time since initiating the trot
        time_since_start = current_time - self.gait_init
        # print(f"time since start : {time_since_start}")

        # Create a multiplier from 0.0 to 1.0 to slowly accelerate to the desired velocity 
        # THIS HAPPENS UPON PRESSING THE ARROW or WASD KEY COMMAND AND THE PROCESS LASTS ramp_duration secs
        if time_since_start < ramp_duration:
            ramp_factor = time_since_start / ramp_duration
            # print("accelerating")
        elif time_since_start >= ramp_duration and not deceleration_flag:
            # print("fully accelerated")
            ramp_factor = 1.0

        if deceleration_flag and self.deceleration_init == 0:
            self.deceleration_init = current_time
            # print(f"decceleration init : {self.deceleration_init}")

        # Calculate time since triggering the decceleration
        time_since_dec_trigger = current_time - self.deceleration_init
        # print(f"time since dec trigger : {time_since_dec_trigger}")


        # Create a multiplier from 1.0 to 0.0 to slowly decelerate to a halt
        # THIS HAPPENS UPON PRESSING THE Q COMMAND AND THE PROCESS LASTS ramp_duration secs
        if time_since_dec_trigger < ramp_duration and deceleration_flag:
            ramp_factor = 1.0 - (time_since_dec_trigger / ramp_duration)
            # print("decelerating")
        elif time_since_dec_trigger >= ramp_duration and deceleration_flag:
            # Reset init params for the next trot command
            self.gait_init = None
            self.deceleration_init = 0
            ramp_factor = 0.0
            # print("fully decelerated")


    
            

        # Apply ramp to velocity
        effective_velocity = desired_velocity * ramp_factor

        # Use effective_velocity instead of desired_velocity_x
        stance_length = effective_velocity * T_cycle * duty_factor

        # Global phase shows where we are in the cycle 
        # (0 means start of cycle, T_cycle means end of cycle)
        global_phase = (current_time % T_cycle) / T_cycle

        legs = ["FL", "FR", "RL", "RR"]
        # legs = ["FR"]
        
        # Offset for the legs
        # Where the legs are in the cycle          
        leg_offsets = [0, 0.5, 0.5, 0]
        # leg_offsets = [0.5]

        
        # With lidar
        # joint_indices = [
        #     [3, 4, 6], 
        #     [8, 9, 11], 
        #     [13, 14, 16], 
        #     [18, 19, 21]
        # ]
        # Without lidar
        joint_indices = [
            [2, 3, 5], 
            [7, 8, 10], 
            [12, 13, 15], 
            [17, 18, 20]
        ]
        
        # joint_indices = [
        #     [7, 8, 10],]
        
        
        # Directions for the motors (from pybullet_sim.py)
        theta_dirs = [-1, 1, 1,   # FL
                       1, 1, 1,   # FR
                      -1, 1, 1,   # RL
                       1, 1, 1]   # RR


        


        roll_correction = pitch_correction = yaw_correction = 0.0
        roll_error = pitch_error = yaw_error = 0.0

        if imu_data is not None:
            imu_data = from_pybullet_orn(imu_data)
            dt = 1. / 240.
            roll_error  = self.initial_orientation[0] - imu_data[0]
            pitch_error = self.initial_orientation[1] - imu_data[1]
            yaw_error   = normalize_angle(self.initial_orientation[2] - imu_data[2])
            roll_correction  = self.pid_roll.update(roll_error, dt)
            pitch_correction = self.pid_pitch.update(pitch_error, dt)
            yaw_correction   = self.pid_yaw.update(yaw_error, dt)

        corrected_orientation = (
            self.initial_orientation[0] + roll_correction, 
            self.initial_orientation[1] + pitch_correction, 
            self.initial_orientation[2] + yaw_correction)

        # Get Body IK transforms in Kinematics frame
        (T_fl, T_fr, T_rl, T_rr) = self.kin_solver.bodyIK(*corrected_orientation, *self.initial_center)
        transforms = [T_fl, T_fr, T_rl, T_rr]

        for i, leg in enumerate(legs):
            leg_phase = (global_phase + leg_offsets[i]) % 1
            
            # Global Frame Positions (Kinematics Frame: mm, Y-up)
            initial_pos = self.initial_ef_positions[i][:3]
            
            
            # Stance Length and Swing Height are given in meters
            # Convert to mm
            sl_mm = stance_length * 1000.0
            sh_mm = swing_height * 1000.0

            # dl = 20.0
            # L_span = sl_mm / 2
            # h1 = sh_mm - 20.0
            # h2 = sh_mm
            
            if leg_phase < duty_factor:
                stance_progress = leg_phase / duty_factor
                # 5% of swing height gives a ~1-2 mm dip: subtle but effective
                stance_delta = sh_mm * 0.05
                current_pos = self.stance_sine_trajectory(initial_pos, sl_mm, stance_progress, stance_delta, dir)

            else:
                swing_progress = (leg_phase - duty_factor) / (1 - duty_factor)
                control_points = self.swing_trajectory_control_points(initial_pos, sl_mm, sh_mm, dir)
                bezier_gen = bezier.BezierCurveGen(control_points)
                current_pos = bezier_gen.n_point_curve(control_points, swing_progress)


            # Target Position is already in Kinematics Frame 
            # Convert to homogenous coordinates
            target_pos = to_homogenous(current_pos)

            # Get the shoulder base transform
            shoulder_base_transform = transforms[i]
            Ix = np.identity(4)
            if leg == "FR" or leg == "RR":
                Ix = self.kin_solver.Ix
                

            # Now that the point is in the kinematics body frame, we convert it to shoulder frame
            # target_pos_shoulder = inv(T_shoulder_body) @ target_pos_body
            target_pos_shoulder = Ix @ np.linalg.inv(shoulder_base_transform) @ target_pos
            
            # Get the angles through IK
            angles = self.kin_solver.legIK(target_pos_shoulder)

            # Apply theta direction
            angle_dirs = theta_dirs[i*3 : (i+1)*3]
            angles = [angle * dir for angle, dir in zip(angles, angle_dirs)]

            # Move the leg
            p.setJointMotorControlArray(robotId, 
                joint_indices[i], 
                p.POSITION_CONTROL, 
                angles)

        p.stepSimulation()
        time.sleep(1./240.)

        return effective_velocity, roll_error, pitch_error, yaw_error


    def turn(self, current_time, T_cycle, duty_factor, desired_velocity, swing_height, p, robotId, imu_data=None, dir="+x"):
        
        pass

    def turn_in_place(self, current_time, T_cycle, duty_factor, desired_velocity, swing_height, p, robotId, imu_data=None, dir="+x"):
        
        pass

    def body_manipulation(self, map_angle, p, robotId):
        
        pass
                

if __name__ == "__main__":
    pass
    