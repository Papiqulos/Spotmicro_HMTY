import time
import csv
import os
import atexit
from pathlib import Path
import numpy as np
from tools.utils import to_homogenous
from tools.pid_controller import PIDControllerRP, PIDController
import core.kinematics as kinematics
import core.bezier_curve_gen as bezier
from collections import deque
from log.log_plotter import plot_log

_LOG_DIR = Path(__file__).parent.parent / "log" / "roll_pitch"

L1 = kinematics.L1
L2 = kinematics.L2
L3 = kinematics.L3
L4 = kinematics.L4
LENGTH = kinematics.LENGTH
WIDTH = kinematics.WIDTH

# 12-point Bezier swing parameters (normalized).
# Values from open-source community implementations (spot_mini_mini et al.),
# inspired by MIT Cheetah.
# _X: horizontal progression 0=liftoff, 1=touchdown
#     3 clustered at each end -> zero endpoint tangent velocity (smooth lift/land)
# _H: height factor relative to swing_height

# Claude's values based on the 12-point Bezier curve
_SWING_X_NORM = [0.00, 0.00, 0.00, 0.15, 0.30, 0.45, 0.55, 0.70, 0.85, 1.00, 1.00, 1.00]
# _SWING_H_NORM = [0.00, 0.10, 0.80, 1.00, 1.10, 1.10, 1.10, 1.10, 1.00, 0.80, 0.10, 0.00]

# Extracted normalized values from spot_mini_mini
# https://github.com/OpenQuadruped/spot_mini_mini/blob/spotmicroai/spotmicro/GaitGenerator/Bezier.py 
# _SWING_X_NORM = [0.00, 1.4, 1.5, 1.5, 1.5, 0.00, 0.00, 0.00, 1.5, 1.5, 1.40, 0.00]
_SWING_H_NORM = [0.00, 0.00, 0.00, 0.9, 0.9, 0.9, 0.9, 1.10, 1.10, 0.00, 0.00, 0.00]


class GaitController:

    def __init__(self, initial_ef_positions=None, initial_theta=None,
                 initial_center=None, initial_orientation=None):
        """
        :param initial_ef_positions: foot positions in kinematics frame (mm, Y-up)
        :param initial_theta:        initial joint angles
        :param initial_center:       body center in kinematics frame (mm)
        :param initial_orientation:  body RPY in kinematics frame (rad)
        """
        # Initial state
        self.initial_ef_positions = initial_ef_positions
        self.initial_theta = initial_theta
        self.initial_center = initial_center
        self.initial_orientation = initial_orientation

        # Kinematics
        self.kin_solver = kinematics.Kinematics(length=LENGTH, width=WIDTH,
                                                l1=L1, l2=L2, l3=L3, l4=L4)
        
        # Initializing variables for gait
        self.gait_init = None
        self.deceleration_init = 0

        # PID controllers
        self.pid = PIDControllerRP(kp=0.2, ki=0.025, kd=0.025)
        self.pid_h = PIDController(kp=0.2, ki=0.025, kd=0.025)
        self._pid_last_time = None
        self._imu_window = deque(maxlen=30)

        # CSV logging
        _LOG_DIR.mkdir(exist_ok=True)
        self._clear_log()
        _ts = time.strftime("%Y_%m_%d_%H_%M_%S")
        self._log_file = open(_LOG_DIR / f"pid_{_ts}.csv", "w", newline="")
        self._csv = csv.writer(self._log_file)
        self._csv.writerow(["t", "imu_roll", "imu_pitch", "pid_roll", "pid_pitch"])
        atexit.register(self._log_file.close)
    
    def _clear_log(self):
        # Delete any prexisting .csv and .png files
        for f in os.listdir(_LOG_DIR):
            if f.endswith(".csv") or f.endswith(".png"):
                os.remove(os.path.join(_LOG_DIR, f))

    def _set_pid(self, kp, ki, kd):
        """Set PID controller gains."""
        self.pid = PIDControllerRP(kp=kp, ki=ki, kd=kd)
        self._imu_window.clear()
        self._pid_last_time = None
        self.gait_init = None
        self.deceleration_init = 0

    def swing_trajectory_control_points(self, initial_pos, sl_mm, sh_mm, dir="+x"):
        """12-point Bezier swing control points.
        Frame: kinematics (X-forward, Y-up, Z-left), units mm.

        3-point clusters at liftoff/touchdown force zero endpoint tangent velocity.
        """
        # Touchdown at S/6 ahead of nominal (He 2020), liftoff at -5S/6.
        # Swing starts at liftoff (-5S/6) and ends at touchdown (+S/6).
        pts = []
        if dir == "+x":
            x0 = initial_pos[0] - 5 * sl_mm / 6
            for xn, hn in zip(_SWING_X_NORM, _SWING_H_NORM):
                pts.append(np.array([x0 + xn * sl_mm, initial_pos[1] + hn * sh_mm, initial_pos[2]]))
        elif dir == "-x":
            x0 = initial_pos[0] + 5 * sl_mm / 6
            for xn, hn in zip(_SWING_X_NORM, _SWING_H_NORM):
                pts.append(np.array([x0 - xn * sl_mm, initial_pos[1] + hn * sh_mm, initial_pos[2]]))
        elif dir == "+z":
            z0 = initial_pos[2] - 5 * sl_mm / 6
            for xn, hn in zip(_SWING_X_NORM, _SWING_H_NORM):
                pts.append(np.array([initial_pos[0], initial_pos[1] + hn * sh_mm, z0 + xn * sl_mm]))
        elif dir == "-z":
            z0 = initial_pos[2] + 5 * sl_mm / 6
            for xn, hn in zip(_SWING_X_NORM, _SWING_H_NORM):
                pts.append(np.array([initial_pos[0], initial_pos[1] + hn * sh_mm, z0 - xn * sl_mm]))
        return pts

    def stance_sine_trajectory(self, initial_pos, sl_mm, stance_progress, delta, dir="+x"):
        """Stance foot position with complementary sine dip.

        Horizontal: touchdown at +S/6 ahead of nominal, sweeps back by S to liftoff at -5S/6.
        Vertical (Y-up): y0 - delta * sin(pi * t)
        """
        t = stance_progress
        y = initial_pos[1] - delta * np.sin(np.pi * t)
        if dir == "+x":
            return np.array([initial_pos[0] + sl_mm / 6 - t * sl_mm, y, initial_pos[2]])
        elif dir == "-x":
            return np.array([initial_pos[0] - sl_mm / 6 + t * sl_mm, y, initial_pos[2]])
        elif dir == "+z":
            return np.array([initial_pos[0], y, initial_pos[2] + sl_mm / 6 - t * sl_mm])
        elif dir == "-z":
            return np.array([initial_pos[0], y, initial_pos[2] - sl_mm / 6 + t * sl_mm])

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
        Diagonal trot using 12-point Bezier swing and sinusoidal stance.

        :param current_time:      elapsed time in seconds
        :param time_step:         control loop period in seconds (used for PID dt)
        :param T_cycle:           gait cycle duration in seconds
        :param duty_factor:       stance fraction of cycle (0-1)
        :param desired_velocity:  target body speed in m/s
        :param swing_height:      peak foot clearance above nominal in meters
        :param imu_data:          orientation tuple in pybullet frame, or None
        :param dir:               motion direction: +x / -x / +y / -y
        :param deceleration_flag: when True, ramp velocity to zero over 0.5 s
        :param move_callback:     callable(leg, angles, unit="rad")
                                  handles hardware servo writes or pybullet joints
        """
        if self.gait_init is None:
            self.gait_init = current_time


        ramp_duration = 0.5
        time_since_start = current_time - self.gait_init
        ramp_factor = 1.0

        if time_since_start < ramp_duration:
            ramp_factor = ramp_factor = 0.5 * (1.0 - np.cos(np.pi * time_since_start / ramp_duration))
        elif not deceleration_flag:
            ramp_factor = 1.0

        if deceleration_flag and self.deceleration_init == 0:
            self.deceleration_init = current_time

        time_since_dec = current_time - self.deceleration_init
        if time_since_dec < ramp_duration and deceleration_flag:
            ramp_factor = ramp_factor = 0.5 * (1.0 + np.cos(np.pi * time_since_start / ramp_duration))
        elif time_since_dec >= ramp_duration and deceleration_flag:
            self.gait_init = None
            self.deceleration_init = 0
            ramp_factor = 0.0

        effective_velocity = desired_velocity * ramp_factor
        stance_length = effective_velocity * T_cycle * duty_factor
        global_phase = (current_time % T_cycle) / T_cycle

        legs = ["FL", "FR", "RL", "RR"]
        leg_offsets = [0.0, 0.5, 0.5, 0.0]

        corrected_orientation = self.initial_orientation

        if imu_data is not None:
            raw = np.array([imu_data[0], imu_data[1]])
            
            # Moving average filter for noisy IMU data
            self._imu_window.append(raw)
            filtered = np.mean(self._imu_window, axis=0)

            now = time.time()
            pid_dt = (now - self._pid_last_time) if self._pid_last_time is not None else time_step
            self._pid_last_time = now

            correction = self.pid.run(filtered[0], filtered[1], pid_dt)

            self._csv.writerow([f"{time.time():.4f}",
                                f"{filtered[0]:.6f}", f"{filtered[1]:.6f}",
                                f"{correction[0]:.6f}", f"{correction[1]:.6f}"])
            self._log_file.flush()

            corrected_orientation = np.array([
                correction[0],
                correction[1],
                self.initial_orientation[2],
            ])
            
        # Foot position correction based on corrected roll, pitch
        dy_fl = + (WIDTH/2)*np.tan(corrected_orientation[0]) + (LENGTH/4)*np.tan(corrected_orientation[1])
        dy_fr = - (WIDTH/2)*np.tan(corrected_orientation[0]) + (LENGTH/4)*np.tan(corrected_orientation[1])
        dy_rl = + (WIDTH/2)*np.tan(corrected_orientation[0]) - (LENGTH/4)*np.tan(corrected_orientation[1])
        dy_rr = - (WIDTH/2)*np.tan(corrected_orientation[0]) - (LENGTH/4)*np.tan(corrected_orientation[1])
        dy_dic = {0: dy_fl, 1: dy_fr, 2: dy_rl, 3: dy_rr}
        
        
        
        # Direct correction through IK
        # (T_fl, T_fr, T_rl, T_rr) = self.kin_solver.bodyIK(*corrected_orientation, *self.initial_center)
        (T_fl, T_fr, T_rl, T_rr) = self.kin_solver.bodyIK(*self.initial_orientation, *self.initial_center)
        transforms = [T_fl, T_fr, T_rl, T_rr]

        sl_mm = stance_length * 1000.0
        sh_mm = swing_height * 1000.0
        delta_base = sh_mm * 0.05

        for i, leg in enumerate(legs):
            leg_phase = (global_phase + leg_offsets[i]) % 1.0
            # Applying the pid height correction to the initial position and stance delta
            initial_pos = np.array([self.initial_ef_positions[i][0], self.initial_ef_positions[i][1] + dy_dic[i], self.initial_ef_positions[i][2]])
            stance_delta = delta_base + abs(dy_dic[i]) * 0.2

            if leg_phase < duty_factor:
                t = leg_phase / duty_factor
                current_pos = self.stance_sine_trajectory(initial_pos, sl_mm, t, stance_delta, dir)
            else:
                t = (leg_phase - duty_factor) / (1.0 - duty_factor)
                cps = self.swing_trajectory_control_points(initial_pos, sl_mm, sh_mm, dir)
                bezier_gen = bezier.BezierCurveGen(cps)
                current_pos = bezier_gen.n_point_curve(cps, t)

            target_pos = to_homogenous(current_pos)
            Ix = self.kin_solver.Ix if leg in ("FR", "RR") else np.identity(4)
            target_pos_shoulder = Ix @ np.linalg.inv(transforms[i]) @ target_pos
            angles = self.kin_solver.legIK(target_pos_shoulder)

            if move_callback is not None:
                move_callback(leg, angles, unit="rad")
            

        return effective_velocity, self._log_file.name

    def turn(self):
        raise NotImplementedError

    def turn_in_place(self):
        raise NotImplementedError

    def body_manipulation(self):
        raise NotImplementedError


if __name__ == "__main__":
    pass
