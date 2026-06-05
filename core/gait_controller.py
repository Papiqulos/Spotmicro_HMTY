import time
import csv
import os
import atexit
from pathlib import Path
import numpy as np
from tools.utils import to_homogenous, trans_inv
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

# Modified normalized values from combined spot_mini_mini and claude's values
# Tripled stacked points at beginning and end
_SWING_X_NORM = [0.00, 0.00, 0.00, 0.15, 0.30, 0.45, 0.55, 0.70, 0.85, 1.00, 1.00, 1.00]
_SWING_H_NORM = [0.00, 0.00, 0.00, 0.9, 0.9, 0.9, 0.9, 1.0, 1.1, 0.00, 0.00, 0.00]

# Phase offsets per leg [FL, FR, RL, RR] as fraction of cycle (0-1).
_GAIT_PHASES = {
    "trot":  [0.0, 0.5, 0.5, 0.0],   # diagonals: FL+RR, FR+RL
    "walk":  [0.0, 0.5, 0.25, 0.75], # one leg at a time, quarter-cycle apart
    "bound": [0.0, 0.0, 0.5, 0.5],   # front pair then rear pair
    "pace":  [0.0, 0.5, 0.0, 0.5],   # left pair then right pair
    "pronk": [0.0, 0.0, 0.0, 0.0],   # all legs together
}

# Maps direction strings to LateralFraction angles (radians).
_DIR_TO_LATERAL = {
    "+x":  0.0,
    "-x":  np.pi,
    "+z":  np.pi / 2.0,
    "-z": -np.pi / 2.0,
}


class GaitController:

    def __init__(self, initial_ef_positions=None, initial_theta=None,
                 initial_center=None, initial_orientation=None, imu=None):
        """
        :param initial_ef_positions: foot positions in kinematics frame (mm, Y-up)
        :param initial_theta:        initial joint angles
        :param initial_center:       body center in kinematics frame (mm)
        :param initial_orientation:  body RPY in kinematics frame (rad)
        :param imu:                  IMU object
        """
        self.initial_ef_positions = initial_ef_positions
        self.initial_theta = initial_theta
        self.initial_center = initial_center
        self.initial_orientation = initial_orientation
        self.imu = imu
        

        self.kin_solver = kinematics.Kinematics(length=LENGTH, width=WIDTH,
                                                l1=L1, l2=L2, l3=L3, l4=L4)
        
        # Cache body transforms
        self.transforms = list(self.kin_solver.bodyIK(*self.initial_orientation, *self.initial_center))

        self.gait_init = None
        self.deceleration_init = 0

        # TD-based phase clock state (execute_gait_fixed_swing_td only)
        self._td_time = None
        self._sw_ref  = 0.0
        self._td_flag = False

        if initial_ef_positions is not None:
            self._prev_foot_pos = [np.array(p[:3], dtype=float) for p in initial_ef_positions]
        else:
            self._prev_foot_pos = [np.zeros(3) for _ in range(4)]

        self.pid = PIDControllerRP(kp=0.4, ki=0.025, kd=0.05)
        self.pid_r = PIDController(kp=0.4, ki=0.025, kd=0.05)
        self.pid_p = PIDController(kp=0.4, ki=0.025, kd=0.05)
        self._pid_last_time = None

        _LOG_DIR.mkdir(exist_ok=True)
        self._clear_log()
        _ts = time.strftime("%Y_%m_%d_%H_%M_%S")
        self._log_file = open(_LOG_DIR / f"pid_{_ts}.csv", "w", newline="")
        self._csv = csv.writer(self._log_file)
        self._csv.writerow(["t", "imu_roll", "imu_pitch", "pid_roll", "pid_pitch"])
        atexit.register(self._log_file.close)

    def _clear_log(self):
        for f in os.listdir(_LOG_DIR):
            if f.endswith(".csv") or f.endswith(".png"):
                os.remove(os.path.join(_LOG_DIR, f))

    def reset(self, kp=0.4, ki=0.025, kd=0.05,
              kp_r=0.4, ki_r=0.025, kd_r=0.05,
              kp_p=0.4, ki_p=0.025, kd_p=0.05,):
        """Reset GaitController and set new PID gains."""
        self.pid = PIDControllerRP(kp=kp, ki=ki, kd=kd)
        self.pid_r = PIDController(kp=kp_r, ki=ki_r, kd=kd_r)
        self.pid_p = PIDController(kp=kp_p, ki=ki_p, kd=kd_p)
        self.imu._imu_window.clear()
        self._pid_last_time = None
        self.gait_init = None
        self.deceleration_init = 0
        self._td_time = None
        self._sw_ref  = 0.0
        self._td_flag = False
        if self.initial_ef_positions is not None:
            self._prev_foot_pos = [np.array(p[:3], dtype=float) for p in self.initial_ef_positions]
        else:
            self._prev_foot_pos = [np.zeros(3) for _ in range(4)]

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _compute_ramp(self, current_time, desired_lin_vel, desired_ang_vel, deceleration_flag):
        """Cosine velocity ramp.  Returns (eff_lin, eff_ang)."""
        if self.gait_init is None:
            self.gait_init = current_time

        ramp_duration = 0.5
        time_since_start = current_time - self.gait_init
        ramp_factor = 1.0

        if time_since_start < ramp_duration:
            ramp_factor = 0.5 * (1.0 - np.cos(np.pi * time_since_start / ramp_duration))
        elif not deceleration_flag:
            ramp_factor = 1.0

        if deceleration_flag and self.deceleration_init == 0:
            self.deceleration_init = current_time

        time_since_dec = current_time - self.deceleration_init
        if time_since_dec < ramp_duration and deceleration_flag:
            ramp_factor = 0.5 * (1.0 + np.cos(np.pi * time_since_dec / ramp_duration))
        elif time_since_dec >= ramp_duration and deceleration_flag:
            self.gait_init = None
            self.deceleration_init = 0
            ramp_factor = 0.0

        return desired_lin_vel * ramp_factor, desired_ang_vel * ramp_factor

    def _imu_correction(self, imu_data, time_step, banked_roll=0.0):
        """30-tap moving average + PID.  Returns corrected_orientation [roll, pitch, yaw]."""
        if imu_data is None:
            return np.array(self.initial_orientation, dtype=float)
        
        # Filtered roll and pitch(Low pass + 30-tap moving average)
        filtered = np.array([imu_data[0] + banked_roll,  imu_data[1]])

        now = time.time()
        pid_dt = (now - self._pid_last_time) if self._pid_last_time is not None else time_step
        self._pid_last_time = now

        correction = np.array([self.pid_r.update(filtered[0], pid_dt),
                               self.pid_p.update(filtered[1], pid_dt)])

        self._csv.writerow([f"{now:.4f}",
                            f"{filtered[0]:.6f}", f"{filtered[1]:.6f}",
                            f"{correction[0]:.6f}", f"{correction[1]:.6f}"])
        self._log_file.flush()

        return np.array([correction[0], correction[1], self.initial_orientation[2]])

    def _step_legs(self, global_phase, duty_factor, T_cycle, sl_mm, sh_mm,
                   lateral_fraction, eff_ang, corrected_orientation, gait_type, move_callback):
        """Compute and apply per-leg trajectories for one control step."""
        dy = [
            + (WIDTH/2)*np.tan(corrected_orientation[0]) + (LENGTH/4)*np.tan(corrected_orientation[1]),
            - (WIDTH/2)*np.tan(corrected_orientation[0]) + (LENGTH/4)*np.tan(corrected_orientation[1]),
            + (WIDTH/2)*np.tan(corrected_orientation[0]) - (LENGTH/4)*np.tan(corrected_orientation[1]),
            - (WIDTH/2)*np.tan(corrected_orientation[0]) - (LENGTH/4)*np.tan(corrected_orientation[1]),
        ]

        delta_base = sh_mm * 0.05
        leg_offset = _GAIT_PHASES[gait_type]
        legs = ["FL", "FR", "RL", "RR"]

        for i, leg in enumerate(legs):
            leg_phase = (global_phase + leg_offset[i]) % 1.0
            initial_pos = np.array([
                self.initial_ef_positions[i][0],
                self.initial_ef_positions[i][1] + dy[i],
                self.initial_ef_positions[i][2],
            ])
            stance_delta = delta_base + abs(dy[i]) * 0.2

            phi_arc, r = self._yaw_circle(i, leg)
            yaw_sl_mm = eff_ang * T_cycle * r
            step_x = sl_mm * np.cos(lateral_fraction) + yaw_sl_mm * np.cos(phi_arc)
            step_z = sl_mm * np.sin(lateral_fraction) + yaw_sl_mm * np.sin(phi_arc)
            combined_sl = np.sqrt(step_x**2 + step_z**2)
            combined_lf = np.arctan2(step_z, step_x) if combined_sl > 1e-6 else lateral_fraction

            if duty_factor > 0 and leg_phase < duty_factor:
                t = leg_phase / duty_factor
                current_pos = self.stance_sine_trajectory(
                    initial_pos, combined_sl, t, stance_delta, combined_lf)
            else:
                denom = 1.0 - duty_factor
                t = (leg_phase - duty_factor) / denom if denom > 0 else 0.0
                cps = self.swing_trajectory_control_points(
                    initial_pos, combined_sl, sh_mm, combined_lf)
                current_pos = bezier.BezierCurveGen(cps).n_point_curve(cps, t)

            self._prev_foot_pos[i] = current_pos.copy()

            target_pos = to_homogenous(current_pos)
            Ix = self.kin_solver.Ix if leg in ("FR", "RR") else np.identity(4)
            target_pos_shoulder = Ix @ trans_inv(self.transforms[i]) @ target_pos
            angles = self.kin_solver.legIK(target_pos_shoulder)

            if move_callback is not None:
                move_callback(leg, angles, unit="rad")

    # ------------------------------------------------------------------
    # Trajectory helpers
    # ------------------------------------------------------------------

    def _yaw_circle(self, i, leg):
        """Return (phi_arc, r) for leg i."""
        leg_x  = float(self.initial_ef_positions[i][0])
        leg_z  = float(self.initial_ef_positions[i][2])
        r      = np.sqrt(leg_x**2 + leg_z**2)
        hip_dir = np.arctan2(leg_z, leg_x)

        prev  = self._prev_foot_pos[i]
        g_mag = np.sqrt((prev[0] - leg_x)**2 + (prev[2] - leg_z)**2)
        th_mod = np.arctan2(g_mag, r)

        if leg in ("FR", "RL"):
            phi_arc = np.pi / 2.0 + hip_dir + th_mod
        else:
            phi_arc = np.pi / 2.0 - hip_dir + th_mod

        return phi_arc, r

    def swing_trajectory_control_points(self, initial_pos, sl_mm, sh_mm, lateral_fraction):
        """12-point Bezier swing control points in kinematics frame (mm)."""
        X_POLAR = np.cos(lateral_fraction)
        Z_POLAR = np.sin(lateral_fraction)
        pts = []
        for xn, hn in zip(_SWING_X_NORM, _SWING_H_NORM):
            d = (xn - 5.0 / 6.0) * sl_mm
            pts.append(np.array([
                initial_pos[0] + d * X_POLAR,
                initial_pos[1] + hn * sh_mm,
                initial_pos[2] + d * Z_POLAR,
            ]))
        return pts

    def stance_sine_trajectory(self, initial_pos, sl_mm, stance_progress, delta, lateral_fraction):
        """Stance foot position with sinusoidal dip."""
        t = stance_progress
        d = (sl_mm / 6.0 - t * sl_mm)
        y = initial_pos[1] - delta * np.sin(np.pi * t)
        return np.array([initial_pos[0] + d * np.cos(lateral_fraction),
                         y,
                         initial_pos[2] + d * np.sin(lateral_fraction)])

    # ------------------------------------------------------------------
    # Gait execution methods
    # All share the same signature. Internal difference: phase clock and
    # how T_cycle / duty_factor / sl_mm are derived.
    # ------------------------------------------------------------------

    def execute_gait_fixed_swing_td(self,
            # STANDARD PARAMETERS
            current_time, time_step, imu_data=None, deceleration_flag=False, move_callback=None, 
            # TUNABLE PARAMETERS
            desired_lin_vel=0.3, 
            desired_ang_vel=0.0, 
            swing_height=0.035, 
            stance_length=0.06, 
            Tswing=0.25, 
            dir="+x",  
            gait_type="trot"):
        """Fixed Tswing, physics-derived Tstance, TD-based phase clock.

        Tstance = stance_length / v.  Phase resets on FL touchdown detection.
        Most robust to timing drift; recommended for straight-line gaits.
        """
        lateral_fraction = _DIR_TO_LATERAL[dir] if isinstance(dir, str) else float(dir)
        eff_lin, eff_ang = self._compute_ramp(
            current_time, desired_lin_vel, desired_ang_vel, deceleration_flag)

        if abs(eff_lin) > 1e-3:
            Tstance = min(abs(stance_length) / abs(eff_lin), 1.3 * Tswing)
        else:
            Tstance = 0.0
            stance_length = 0.0
            self._td_time = current_time
            self._td_flag = False
            self._sw_ref  = 0.0
        T_cycle = Tswing + Tstance
        duty_factor = Tstance / T_cycle

        # TD phase clock
        if self._td_time is None:
            self._td_time = current_time
        if self._sw_ref >= 0.9 and self._td_flag:
            self._td_time = current_time
            self._td_flag = False
            self._sw_ref  = 0.0
        global_phase = min(current_time - self._td_time, T_cycle) / T_cycle
        fl_phase = global_phase
        if fl_phase >= duty_factor:
            self._sw_ref = (fl_phase - duty_factor) / (1.0 - duty_factor) if duty_factor < 1.0 else 0.0
            if self._sw_ref >= 0.999:
                self._td_flag = True
        else:
            self._sw_ref = 0.0

        R_yaw = abs(eff_lin) / abs(eff_ang) if abs(eff_ang) > 1e-6 else np.inf
        banked_roll = np.sign(eff_ang) * np.arctan2(eff_lin**2, 9.81 * R_yaw)
        corrected_orn = self._imu_correction(imu_data, time_step, banked_roll)
        self._step_legs(global_phase, duty_factor, T_cycle,
                        stance_length * 1000.0, swing_height * 1000.0,
                        lateral_fraction, eff_ang, corrected_orn, gait_type, move_callback)
        return eff_lin, self._log_file.name

    def execute_gait_fixed_swing(self,
            # STANDARD PARAMETERS
            current_time, time_step, imu_data=None, deceleration_flag=False, move_callback=None, 
            # TUNABLE PARAMETERS
            desired_lin_vel=0.3, 
            desired_ang_vel=0.0, 
            swing_height=0.035, 
            stance_length=0.06, 
            Tswing=0.25, 
            dir="+x",  
            gait_type="trot"):
        """Fixed Tswing, physics-derived Tstance, modulo phase clock.

        Tstance = stance_length / v.  Phase is (t % T_cycle) / T_cycle.
        T_cycle changes with velocity, so phase can jump at transitions.
        Recommended when TD detection is unreliable and yaw/lateral is needed.
        """
        lateral_fraction = _DIR_TO_LATERAL[dir] if isinstance(dir, str) else float(dir)
        eff_lin, eff_ang = self._compute_ramp(
            current_time, desired_lin_vel, desired_ang_vel, deceleration_flag)

        if abs(eff_lin) > 1e-3:
            Tstance = min(abs(stance_length) / abs(eff_lin), 1.3 * Tswing)
        else:
            Tstance = 0.0
            stance_length = 0.0
        T_cycle = Tswing + Tstance
        duty_factor = Tstance / T_cycle
        global_phase = (current_time % T_cycle) / T_cycle

        R_yaw = abs(eff_lin) / abs(eff_ang) if abs(eff_ang) > 1e-6 else np.inf
        banked_roll = np.sign(eff_ang) * np.arctan2(eff_lin**2, 9.81 * R_yaw)
        corrected_orn = self._imu_correction(imu_data, time_step, banked_roll)
        self._step_legs(global_phase, duty_factor, T_cycle,
                        stance_length * 1000.0, swing_height * 1000.0,
                        lateral_fraction, eff_ang, corrected_orn, gait_type, move_callback)
        return eff_lin, self._log_file.name

    def execute_gait_fixed_stance(self,
            # STANDARD PARAMETERS
            current_time, time_step, imu_data=None, deceleration_flag=False, move_callback=None, 
            # TUNABLE PARAMETERS
            desired_lin_vel=0.3, 
            desired_ang_vel=0.0, 
            swing_height=0.035, 
            stance_length=0.06, 
            Tswing=0.25, 
            dir="+x",  
            gait_type="trot"):
        """Fixed T_cycle and duty_factor derived from nominal (unramped) velocity.

        T_cycle = Tswing + stance_length / desired_lin_vel (constant).
        sl_mm scales with ramped velocity -> smooth ramp with stable phase clock.
        Recommended for omnidirectional and turning gaits.
        """
        lateral_fraction = _DIR_TO_LATERAL[dir] if isinstance(dir, str) else float(dir)
        eff_lin, eff_ang = self._compute_ramp(
            current_time, desired_lin_vel, desired_ang_vel, deceleration_flag)

        # T_cycle fixed from unramped target velocity -> phase clock never jumps
        if abs(desired_lin_vel) > 1e-3:
            Tstance_nom = min(abs(stance_length) / abs(desired_lin_vel), 1.3 * Tswing)
        else:
            Tstance_nom = 1.3 * Tswing
        T_cycle = Tswing + Tstance_nom
        duty_factor = Tstance_nom / T_cycle
        global_phase = (current_time % T_cycle) / T_cycle

        R_yaw = abs(eff_lin) / abs(eff_ang) if abs(eff_ang) > 1e-6 else np.inf
        banked_roll = np.sign(eff_ang) * np.arctan2(eff_lin**2, 9.81 * R_yaw)
        corrected_orn = self._imu_correction(imu_data, time_step, banked_roll)
        sl_mm = eff_lin * T_cycle * duty_factor * 1000.0  # proportional to ramped velocity
        self._step_legs(global_phase, duty_factor, T_cycle,
                        sl_mm, swing_height * 1000.0,
                        lateral_fraction, eff_ang, corrected_orn, gait_type, move_callback)
        return eff_lin, self._log_file.name

    def body_manipulation(self):
        raise NotImplementedError


if __name__ == "__main__":
    pass
