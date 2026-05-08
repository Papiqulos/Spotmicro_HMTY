import numpy as np
import math
import core.bezier_curve_gen as bezier
import core.kinematics as kinematics
from tools.utils import normalize_angle, to_homogenous, from_pybullet_orn
import time
from tools.pid_controller import PIDController, PIDControllerRP


L1 = kinematics.L1
L2 = kinematics.L2
L3 = kinematics.L3
L4 = kinematics.L4
LENGTH = kinematics.LENGTH
WIDTH = kinematics.WIDTH

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
        self.kin_solver = kinematics.Kinematics(length=LENGTH, width=WIDTH, l1=L1, l2=L2, l3=L3, l4=L4)

        self.gait_init = None
        self.deceleration_init = 0

        self.pid_roll  = PIDController(kp=0.2, ki=0.025, kd=0.025)
        self.pid_pitch = PIDController(kp=0.2, ki=0.025, kd=0.025)
        self.pid_yaw   = PIDController(kp=0.2, ki=0.0,   kd=0.025)
        self.pid_rp    = PIDControllerRP(kp=0.2, ki=0.025, kd=0.025)

    def swing_trajectory_control_points(self, initial_pos, L_span, dl, h1, h2, dir="+x"):
        raise NotImplementedError

    def stance_sine_trajectory(self, initial_pos, L_span, delta, stance_progress, dir="+x"):
        raise NotImplementedError

    def trot(self,
             current_time,
             time_step,
             T_cycle,
             duty_factor,
             desired_velocity,
             swing_height,
             imu_data=None,
             dir="+x",
             deceleration_flag=False,
             move_callback=None):
        """
        Make the robot trot in the specified direction

        :param current_time: current simulation time
        :param T_cycle: cycle time in seconds
        :param duty_factor: duty factor
        :param desired_velocity: desired velocity in meters per second
        :param swing_height: swing height in meters
        :param imu_data: imu data
        :param dir: direction of the trot (pybullet frame)
        """
        if self.gait_init is None:
            self.gait_init = current_time

        ramp_duration = 0.5
        time_since_start = current_time - self.gait_init
        ramp_factor = 1.0

        if time_since_start < ramp_duration:
            ramp_factor = time_since_start / ramp_duration
        elif not deceleration_flag:
            ramp_factor = 1.0

        if deceleration_flag and self.deceleration_init == 0:
            self.deceleration_init = current_time

        time_since_dec_trigger = current_time - self.deceleration_init

        if time_since_dec_trigger < ramp_duration and deceleration_flag:
            ramp_factor = 1.0 - (time_since_dec_trigger / ramp_duration)
        elif time_since_dec_trigger >= ramp_duration and deceleration_flag:
            self.gait_init = None
            self.deceleration_init = 0
            ramp_factor = 0.0

        effective_velocity = desired_velocity * ramp_factor
        stance_length = effective_velocity * T_cycle * duty_factor
        global_phase = (current_time % T_cycle) / T_cycle

        legs = ["FL", "FR", "RL", "RR"]
        leg_offsets = [0, 0.5, 0.5, 0]

        roll_correction  = 0
        pitch_correction = 0
        yaw_correction   = 0
        roll_error  = 0
        pitch_error = 0
        yaw_error   = 0

        if imu_data is not None:
            imu_data = from_pybullet_orn(imu_data)
            dt = time_step
            roll_error  = self.initial_orientation[0] - imu_data[0]
            pitch_error = self.initial_orientation[1] - imu_data[1]
            yaw_error   = normalize_angle(self.initial_orientation[2] - imu_data[2])
            roll_correction  = self.pid_roll.update(roll_error, dt)
            pitch_correction = self.pid_pitch.update(pitch_error, dt)
            yaw_correction   = self.pid_yaw.update(yaw_error, dt)

        corrected_orientation = (
            self.initial_orientation[0] + roll_correction,
            self.initial_orientation[1] + pitch_correction,
            self.initial_orientation[2] + yaw_correction,
        )

        (T_fl, T_fr, T_rl, T_rr) = self.kin_solver.bodyIK(*corrected_orientation, *self.initial_center)
        transforms = [T_fl, T_fr, T_rl, T_rr]

        sl_mm = stance_length * 1000.0
        sh_mm = swing_height * 1000.0

        for i, leg in enumerate(legs):
            leg_phase = (global_phase + leg_offsets[i]) % 1
            initial_pos = self.initial_ef_positions[i][:3]

            if leg_phase < duty_factor:
                stance_progress = leg_phase / duty_factor
                if dir == "+x":
                    p0 = np.array([initial_pos[0] + (sl_mm / 2), initial_pos[1], initial_pos[2]])
                    p1 = np.array([initial_pos[0] - (sl_mm / 2), initial_pos[1], initial_pos[2]])
                elif dir == "-x":
                    p0 = np.array([initial_pos[0] - (sl_mm / 2), initial_pos[1], initial_pos[2]])
                    p1 = np.array([initial_pos[0] + (sl_mm / 2), initial_pos[1], initial_pos[2]])
                elif dir == "+y":
                    p0 = np.array([initial_pos[0], initial_pos[1], initial_pos[2] + (sl_mm / 2)])
                    p1 = np.array([initial_pos[0], initial_pos[1], initial_pos[2] - (sl_mm / 2)])
                elif dir == "-y":
                    p0 = np.array([initial_pos[0], initial_pos[1], initial_pos[2] - (sl_mm / 2)])
                    p1 = np.array([initial_pos[0], initial_pos[1], initial_pos[2] + (sl_mm / 2)])
                control_points = [p0, p1]
                bezier_gen = bezier.BezierCurveGen(control_points)
                current_pos = bezier_gen.n_point_curve(control_points, stance_progress)

            else:
                swing_progress = (leg_phase - duty_factor) / (1 - duty_factor)
                # Bezier control points: stance frame is kinematics Y-up, so height is +Y
                if dir == "+x":
                    p0 = np.array([initial_pos[0] - (sl_mm / 2), initial_pos[1],          initial_pos[2]])
                    p3 = np.array([initial_pos[0] + (sl_mm / 2), initial_pos[1],          initial_pos[2]])
                    p1 = np.array([initial_pos[0] - (sl_mm / 2), initial_pos[1] + sh_mm,  initial_pos[2]])
                    p2 = np.array([initial_pos[0] + (sl_mm / 2), initial_pos[1] + sh_mm,  initial_pos[2]])
                elif dir == "-x":
                    p0 = np.array([initial_pos[0] + (sl_mm / 2), initial_pos[1],          initial_pos[2]])
                    p3 = np.array([initial_pos[0] - (sl_mm / 2), initial_pos[1],          initial_pos[2]])
                    p1 = np.array([initial_pos[0] + (sl_mm / 2), initial_pos[1] + sh_mm,  initial_pos[2]])
                    p2 = np.array([initial_pos[0] - (sl_mm / 2), initial_pos[1] + sh_mm,  initial_pos[2]])
                elif dir == "+y":
                    p0 = np.array([initial_pos[0], initial_pos[1],          initial_pos[2] - (sl_mm / 2)])
                    p3 = np.array([initial_pos[0], initial_pos[1],          initial_pos[2] + (sl_mm / 2)])
                    p1 = np.array([initial_pos[0], initial_pos[1] + sh_mm,  initial_pos[2] - (sl_mm / 2)])
                    p2 = np.array([initial_pos[0], initial_pos[1] + sh_mm,  initial_pos[2] + (sl_mm / 2)])
                elif dir == "-y":
                    p0 = np.array([initial_pos[0], initial_pos[1],          initial_pos[2] + (sl_mm / 2)])
                    p3 = np.array([initial_pos[0], initial_pos[1],          initial_pos[2] - (sl_mm / 2)])
                    p1 = np.array([initial_pos[0], initial_pos[1] + sh_mm,  initial_pos[2] + (sl_mm / 2)])
                    p2 = np.array([initial_pos[0], initial_pos[1] + sh_mm,  initial_pos[2] - (sl_mm / 2)])
                control_points = [p0, p1, p2, p3]
                bezier_gen = bezier.BezierCurveGen(control_points)
                current_pos = bezier_gen.n_point_curve(control_points, swing_progress)

            target_pos = to_homogenous(current_pos)
            Ix = self.kin_solver.Ix if leg in ("FR", "RR") else np.identity(4)
            # target_pos_shoulder = inv(T_shoulder_body) @ target_pos_body
            target_pos_shoulder = Ix @ np.linalg.inv(transforms[i]) @ target_pos
            angles = self.kin_solver.legIK(target_pos_shoulder)

            if move_callback is not None:
                move_callback(leg, angles, unit="rad")

        time.sleep(time_step)
        return effective_velocity, roll_error, pitch_error, yaw_error

    def turn(self, current_time, T_cycle, duty_factor, desired_velocity, swing_height, p, robotId, imu_data=None, dir="+x"):
        raise NotImplementedError

    def turn_in_place(self, current_time, T_cycle, duty_factor, desired_velocity, swing_height, p, robotId, imu_data=None, dir="+x"):
        raise NotImplementedError

    def body_manipulation(self, map_angle, p, robotId):
        raise NotImplementedError


if __name__ == "__main__":
    pass
