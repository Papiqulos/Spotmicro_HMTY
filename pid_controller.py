import numpy as np
import time

class PIDController:

    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # State variables
        self.previous_error = 0
        self.integral_sum = 0  # This stores the accumulated history
        self.first_run = True  # Flag to prevent derivative spike on start

    def update(self, error, dt):
        # 1. Handle Initialization (prevents massive kick on startup)
        if self.first_run:
            self.previous_error = error
            self.first_run = False

        # 2. Proportional Term
        proportional = self.kp * error
        
        # 3. Integral Term (Accumulate the error over time)
        self.integral_sum += error * dt
        
        integral = self.ki * self.integral_sum

        # 4. Derivative Term
        derivative = self.kd * (error - self.previous_error) / dt
        
        # 5. Save state for next loop
        self.previous_error = error
        
        return proportional + derivative + integral
    
    def reset(self):
        """Call this if you reset the simulation"""
        self.integral_sum = 0
        self.previous_error = 0
        self.first_run = True



class PIDControllerRP:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # desired roll and pitch angles
        self.desired_roll_pitch = np.array([0.0,0.0])

        self.I_term = np.array([0.0,0.0])
        self.D_term = np.array([0.0,0.0])

        self.max_I = 0.2
        self.last_error = np.array([0.0,0.0])

    def run(self, roll, pitch, dt):
        # determine error
        error = self.desired_roll_pitch - np.array([roll, pitch])

        # I term update
        self.I_term = self.I_term + error * dt

        # anti-windup
        for i in range(2):
            if(self.I_term[i] < -self.max_I):
                self.I_term[i] = -self.max_I
            elif(self.I_term[i] > self.max_I):
                self.I_term[i] = self.max_I

        # approximate first derivate
        self.D_term = (error - self.last_error) / dt

        # update last values 
        self.last_error = error

        # compute return values
        P_ret = self.kp * error
        I_ret = self.I_term * self.ki
        D_ret = self.D_term * self.kd

        return P_ret + I_ret + D_ret

    def reset(self):
        self.I_term = np.array([0.0,0.0])
        self.D_term = np.array([0.0,0.0])
        self.last_error = np.array([0.0,0.0])

    def desired_RP_angles(self, des_roll, des_pitch):
        # set desired roll and pitch angles
        self.desired_roll_pitch = np.array([des_roll, des_pitch])