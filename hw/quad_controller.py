import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from adafruit_servokit import ServoKit
import time
import core.kinematics as kinematics
from core.kinematics import LENGTH, WIDTH, L1, L2, L3, L4
import numpy as np
import math
from tools.utils import to_homogenous, rescale_number
from hw.gait_controller_hw import GaitController
import yaml

with open("config/servo_calib.yaml") as f:
    calib = yaml.safe_load(f)


kit = ServoKit(channels=16)

LEGS    = ["fl", "fr", "rl", "rr"]
JOINTS  = ["shoulder", "leg", "foot"]



class RobotController:

        def __init__(self, kin_solver, init_angles, init_center, init_orientation):
                
                self.kin_solver = kin_solver
                self.init_center = init_center
                self.init_orientation = init_orientation
                self.init_angles = init_angles
                self.init_ef_positions = self.kin_solver.robot_FK(self.init_center, self.init_orientation, self.init_angles, unit="degrees") 
                leg_names = ["FL", "FR", "RL", "RR"]

                print("--- Robot Initial Angles ---")
                for i, leg in enumerate(leg_names):
                        shoulder, leg_joint, foot = self.init_angles[i*3 : i*3+3]
                        print(f"{leg}: shoulder={shoulder:.2f}, leg={leg_joint:.2f}, foot={foot:.2f}")

                print("\n--- Robot Initial Positions ---")
                for i, leg in enumerate(leg_names):
                        x, y, z = self.init_ef_positions[i][:3]
                        print(f"{leg}: x={x:.2f}, y={y:.2f}, z={z:.2f}")



                


                # Shoulder. leg, foot
                self.zeros = [calib[leg][joint]["zero_deg"]  for leg in LEGS for joint in JOINTS] # FL, FR, RL, RR

                self.indexes = [calib[leg][joint]["channel"]   for leg in LEGS for joint in JOINTS]# FL, FR, RL, RR

                self.theta_dirs = [calib[leg][joint]["direction"] for leg in LEGS for joint in JOINTS] # FL, FR, RL, RR


                self.gait_controller = GaitController(initial_ef_positions=self.init_ef_positions, 
                                        initial_theta=None, 
                                        initial_center=init_center, 
                                        initial_orientation=init_orientation)
                self.apply_angles_robot(self.init_angles)
                time.sleep(1)

        def apply_angles_robot(self, angles, unit="deg"):
                """Applies the angles to the real robot
                
                :param: angles [FL, FR, RL, RR] shoulder, leg, foot in degrees in real world
                """
                if unit == "rad":
                        angles = [math.degrees(angle) for angle in angles]

                for i, angle in enumerate(angles):
                        # Convert the angle to the corresponding servo angle
                        angle = angle * self.theta_dirs[i] # Apply the direction
                        angle = rescale_number(angle, 0, 180, self.zeros[i], self.zeros[i]+180) # Scale it
                        try:
                                kit.servo[self.indexes[i]].angle = angle

                        except ValueError as e:
                                print(f"Servo {self.indexes[i]} out of range: {angle:.1f}° — {e}")

        def apply_angles_leg(self, leg, angles, unit="deg"):
                leg_indices = []
                leg_dirs = []
                leg_zeros = []
                if unit == "rad":
                        angles = [math.degrees(angle) for angle in angles]
                        
                if leg == "FL":
                        leg_indices = self.indexes[0:3]
                        leg_dirs = self.theta_dirs[0:3]
                        leg_zeros = self.zeros[0:3]
                        
                elif leg == "FR":
                        leg_indices = self.indexes[3:6]
                        leg_dirs = self.theta_dirs[3:6]
                        leg_zeros = self.zeros[3:6]
                        
                elif leg == "RL":
                        leg_indices = self.indexes[6:9]
                        leg_dirs = self.theta_dirs[6:9]
                        leg_zeros = self.zeros[6:9]
                elif leg == "RR":
                        leg_indices = self.indexes[9:12]
                        leg_dirs = self.theta_dirs[9:12]
                        leg_zeros = self.zeros[9:12]
                        
                for i, angle in enumerate(angles):
                        # Convert the angle to the corresponding servo angle
                        angle = angle * leg_dirs[i] # Apply the direction
                        angle = rescale_number(angle, 0, 180, leg_zeros[i], leg_zeros[i]+180) # Scale it

                        try:
                                kit.servo[leg_indices[i]].angle = angle

                        except ValueError as e:
                                print(f"Servo {leg_indices[i]} out of range: {angle:.1f}° — {e}")
                        # kit.servo[leg_indices[i]].angle = angle # THIS NEEDS DEGREES
                                  
        def drive_leg_to_position(self, leg, position):
                (T_fl, T_fr, T_rl, T_rr) = self.kin_solver.bodyIK(*orientation, *center)
                Ix = np.identity(4) 
                if leg == "FL":
                        transform = T_fl
                elif leg == "FR":
                        transform = T_fr
                        Ix = self.kin_solver.Ix
                elif leg == "RL":
                        transform = T_rl
                elif leg == "RR":
                        transform = T_rr
                        Ix = self.kin_solver.Ix
                position = to_homogenous(position)
                target_pos_shoulder = Ix @ np.linalg.inv(transform) @ position
                angles = self.kin_solver.legIK(target_pos_shoulder)
                # angles_deg = [math.degrees(angle) for angle in angles]
                # print(f"Angles from IK: {angles_deg}")
                self.apply_angles_leg(leg, angles, "rad")

        def change_orientation(self, new_orientation, unit="deg"):
                if unit == "deg":
                        new_orientation = [math.radians(angle) for angle in new_orientation]
                angles = self.kin_solver.robot_IK(self.init_center, 
                                                new_orientation, 
                                                self.init_ef_positions)
                self.apply_angles_robot(angles, unit="rad")

        def go_forwards(self, velocity):
                current_time = 0
                time_step = 1./240
                T_cycle = 0.2
                duty_factor = 0.5
                swing_height = 0.03

                for _ in range(150):        
                        current_time += time_step
                        ef_vel, _, _ , _,  = self.gait_controller.trot(current_time, 
                                        time_step,
                                        T_cycle, 
                                        duty_factor, 
                                        velocity, 
                                        swing_height,
                                        move_callback=self.apply_angles_leg )                 
        
        def go_backwards(self):
                pass

        def go_right(self):
                pass

        def go_left(self):
                pass

if __name__ == "__main__":
        

        orientation = [0, 0, 0]  # Roll, Pitch, Yaw in radians
        center = [0, 250, 0]  # X, Y, Z in mm
        theta0 = [0] * 12
        
        theta_default = [0, -30, 60, # FL
                0, -30, 60, # FR
                0, -30, 60, # RL
                0, -30, 60 ] # RR
        theta_test = [0, -32.46, 37.07, # FL
                0, -45, 60, # FR
                0, -45, 60, # RL
                0, -45, 60 ] # RR
        
        ef_positions = np.array([
        [ 95, 48.13,  105, 1], # FL
        [ 95, 48.13,  -105, 1], # FR
        [-45, 48.13, 105, 1], # RL
        [-45, 48.13, -105, 1] # RR
        ])

        ef_positions2 = np.array([
        [67.29, 46.12, 107, 1],
        [67.29, 46.12, -107, 1],
        [-72.21, 46.12, 107, 1],
        [-72.21, 46.12, -107, 1]
        ])
        
        
        control_points = np.array([
        [-7.29, 56.12, -87],
        [12.29, 96.12, -87],
        [77.29, 56.12, -87]])


        kin_solver = kinematics.Kinematics(LENGTH, WIDTH, L1, L2, L3, L4)
        robot_controller = RobotController(kin_solver, theta_default, center, orientation)
        
        
        # robot_controller.apply_angles_leg("FL", [0, 0, 0])
        # robot_controller.drive_leg_to_position("FR", [14.62, 53.77, -107.00])
        # robot_controller.change_orientation([0, 10, 0])
        # robot_controller.apply_angles_robot(theta0)
        robot_controller.go_forwards(0.2)
        # kit.servo[10].angle = rescale_number(-10, 0, 180, 120, 120+180) # THIS NEEDS DEGREES

        


    
    


