import numpy as np
import time

class PIDController:

    def __init__(self, kp, ki, kd, max_integral=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral

        self.previous_error = 0
        self.integral_sum = 0
        self.first_run = True

    def update(self, error, dt):
        if self.first_run:
            self.previous_error = error
            self.first_run = False

        proportional = self.kp * error

        self.integral_sum += error * dt
        if self.max_integral is not None:
            self.integral_sum = max(-self.max_integral, min(self.max_integral, self.integral_sum))
        integral = self.ki * self.integral_sum

        derivative = self.kd * (error - self.previous_error) / dt if dt > 0 else 0.0

        self.previous_error = error

        return proportional + derivative + integral

    def reset(self):
        self.integral_sum = 0
        self.previous_error = 0
        self.first_run = True


# From https://www.youtube.com/watch?v=O_2swSMecB4&t=24s
class PIDControllerRP:

    def __init__(self, kp, ki, kd, max_I=0.2):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.desired_roll_pitch = np.array([0.0, 0.0])
        self.I_term = np.array([0.0, 0.0])
        self.D_term = np.array([0.0, 0.0])
        self.max_I = max_I
        self.last_error = np.array([0.0, 0.0])
        self.first_run = True

    def run(self, roll, pitch, dt):
        error = self.desired_roll_pitch - np.array([roll, pitch])

        if self.first_run:
            self.last_error = error.copy()
            self.first_run = False

        self.I_term = self.I_term + error * dt
        for i in range(2):
            if self.I_term[i] < -self.max_I:
                self.I_term[i] = -self.max_I
            elif self.I_term[i] > self.max_I:
                self.I_term[i] = self.max_I

        self.D_term = (error - self.last_error) / dt if dt > 0 else np.zeros(2)
        self.last_error = error

        return self.kp * error + self.I_term * self.ki + self.D_term * self.kd

    def reset(self):
        self.I_term = np.array([0.0, 0.0])
        self.D_term = np.array([0.0, 0.0])
        self.last_error = np.array([0.0, 0.0])
        self.first_run = True

    def desired_RP_angles(self, des_roll, des_pitch):
        self.desired_roll_pitch = np.array([des_roll, des_pitch])
