import numpy as np
import bezier_curve_gen as bezier
import kinematics_new as kinematics
from robot_mania_code import *
from utils import *
import time
from pid_controller import PIDController, PIDControllerRP


L1 = kinematics.L1
L2 = kinematics.L2
L3 = kinematics.L3
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
        self.theta_dirs = [[-1, 1, 1],
                            [1, 1, 1],
                            [-1, 1, 1], 
                            [1, 1, 1]]
        self.kin_solver = kinematics.Kinematics(length=LENGTH, width=WIDTH, l1=L1, l2=L2, l3=L3)

        self.gait_init = None
        self.deceleration_init = 0

        self.pid_roll = PIDController(kp=0.2, ki=0.025, kd=0.025)
        self.pid_pitch = PIDController(kp=0.2, ki=0.025, kd=0.025)
        self.pid_yaw = PIDController(kp=0.5, ki=0.0, kd=0.025)

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

    # NOT USED
    def swing_trajectory_control_points(self, initial_pos, L_span, dl, h1, h2, dir="+x"):

        """Generates a swing trajectory for a single leg using an 12-point Bezier curve

        :param initial_pos: 
        :param L_span: in milimeters
        :param dl: in milimeters
        :param h1: in milimeters
        :param h2: in milimeters
        :param dir: direction of the trajectory
        """

        

        if dir=="+x":

            p0 = np.array([initial_pos[0] - L_span, initial_pos[1], initial_pos[2]])
            p1 = np.array([initial_pos[0] - L_span - dl, initial_pos[1], initial_pos[2]])

            p2 = np.array([initial_pos[0] - L_span - dl - 0.05, initial_pos[1] , initial_pos[2]+ h1])
            p3 = np.array([initial_pos[0] - L_span - dl - 0.05, initial_pos[1] , initial_pos[2]+ h1])
            p4 = np.array([initial_pos[0] - L_span - dl - 0.05, initial_pos[1] , initial_pos[2]+ h1])

            p5 = np.array([initial_pos[0], initial_pos[1] , initial_pos[2]+ h1])
            p6 = np.array([initial_pos[0], initial_pos[1] , initial_pos[2]+ h1])

            p7 = np.array([initial_pos[0], initial_pos[1] , initial_pos[2]+ h2])

            p8 = np.array([initial_pos[0] + L_span + dl + 0.05, initial_pos[1] , initial_pos[2]+ h2])
            p9 = np.array([initial_pos[0] + L_span + dl + 0.05, initial_pos[1] , initial_pos[2]+ h2])

            p10 = np.array([initial_pos[0] + L_span + dl, initial_pos[1], initial_pos[2]])
            p11 = np.array([initial_pos[0] + L_span, initial_pos[1], initial_pos[2]])
        elif dir=="-x":

            p0 = np.array([initial_pos[0] + L_span, initial_pos[1], initial_pos[2]])
            p1 = np.array([initial_pos[0] + L_span + dl, initial_pos[1], initial_pos[2]])

            p2 = np.array([initial_pos[0] + L_span + dl + 0.05, initial_pos[1] , initial_pos[2]+ h1])
            p3 = np.array([initial_pos[0] + L_span + dl + 0.05, initial_pos[1] , initial_pos[2]+ h1])
            p4 = np.array([initial_pos[0] + L_span + dl + 0.05, initial_pos[1] , initial_pos[2]+ h1])

            p5 = np.array([initial_pos[0], initial_pos[1] , initial_pos[2]+ h1])
            p6 = np.array([initial_pos[0], initial_pos[1] , initial_pos[2]+ h1])

            p7 = np.array([initial_pos[0], initial_pos[1] , initial_pos[2]+ h2])
            p8 = np.array([initial_pos[0] - L_span - dl - 0.05, initial_pos[1] , initial_pos[2]+ h2])
            p9 = np.array([initial_pos[0] - L_span - dl - 0.05, initial_pos[1] , initial_pos[2]+ h2])

            p10 = np.array([initial_pos[0] - L_span - dl, initial_pos[1], initial_pos[2]])
            p11 = np.array([initial_pos[0] - L_span, initial_pos[1], initial_pos[2]])
        elif dir=="+y":

            p0 = np.array([initial_pos[0], initial_pos[1] - L_span , initial_pos[2]])
            p1 = np.array([initial_pos[0], initial_pos[1] - L_span - dl, initial_pos[2]])

            p2 = np.array([initial_pos[0], initial_pos[1] - L_span - dl - 0.05 , initial_pos[2]+ h1])
            p3 = np.array([initial_pos[0], initial_pos[1] - L_span - dl - 0.05 , initial_pos[2]+ h1])
            p4 = np.array([initial_pos[0], initial_pos[1] - L_span - dl - 0.05 , initial_pos[2]+ h1])

            p5 = np.array([initial_pos[0], initial_pos[1], initial_pos[2]+ h1])
            p6 = np.array([initial_pos[0], initial_pos[1], initial_pos[2]+ h1])

            p7 = np.array([initial_pos[0], initial_pos[1], initial_pos[2]+ h2])
            p8 = np.array([initial_pos[0], initial_pos[1] + L_span + dl + 0.05, initial_pos[2]+ h2])
            p9 = np.array([initial_pos[0], initial_pos[1] + L_span + dl + 0.05, initial_pos[2]+ h2])

            p10 = np.array([initial_pos[0], initial_pos[1] + L_span + dl, initial_pos[2]])
            p11 = np.array([initial_pos[0], initial_pos[1] + L_span, initial_pos[2]])
        elif dir=="-y":

            p0 = np.array([initial_pos[0], initial_pos[1] + L_span , initial_pos[2]])
            p1 = np.array([initial_pos[0], initial_pos[1] + L_span + dl, initial_pos[2]])
            
            p2 = np.array([initial_pos[0], initial_pos[1] + L_span + dl + 0.05 , initial_pos[2]+ h1])
            p3 = np.array([initial_pos[0], initial_pos[1] + L_span + dl + 0.05 , initial_pos[2]+ h1])
            p4 = np.array([initial_pos[0], initial_pos[1] + L_span + dl + 0.05 , initial_pos[2]+ h1])

            p5 = np.array([initial_pos[0], initial_pos[1], initial_pos[2]+ h1])
            p6 = np.array([initial_pos[0], initial_pos[1], initial_pos[2]+ h1])

            p7 = np.array([initial_pos[0], initial_pos[1], initial_pos[2]+ h2])
            p8 = np.array([initial_pos[0], initial_pos[1] - L_span - dl - 0.05, initial_pos[2]+ h2])
            p9 = np.array([initial_pos[0], initial_pos[1] - L_span - dl - 0.05, initial_pos[2]+ h2])

            p10 = np.array([initial_pos[0], initial_pos[1] - L_span - dl, initial_pos[2]])
            p11 = np.array([initial_pos[0], initial_pos[1] - L_span, initial_pos[2]])



        control_points = [p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11]


        return control_points
    
    # NOT USED 
    def stance_sine_trajectory(self, initial_pos, L_span, delta, stance_progress, dir="+x", ):
        """
        Generates a stance trajectory for a single leg using a sine wave

        :param initial_pos: 
        :param L_span: in milimeters
        :param delta: in milimeters
        :param dir: direction of the trajectory
        """

        if dir == "+x":
            p0 = np.array([initial_pos[0] - L_span, initial_pos[1], initial_pos[2]])
            p11 = np.array([initial_pos[0] + L_span, initial_pos[1], initial_pos[2]])
        elif dir == "-x":
            p0 = np.array([initial_pos[0] + L_span, initial_pos[1], initial_pos[2]])
            p11 = np.array([initial_pos[0] - L_span, initial_pos[1], initial_pos[2]])
        elif dir == "+y":
            p0 = np.array([initial_pos[0], initial_pos[1] + L_span, initial_pos[2]])
            p11 = np.array([initial_pos[0] , initial_pos[1] - L_span, initial_pos[2]])
        elif dir == "-y":
            p0 = np.array([initial_pos[0] , initial_pos[1] - L_span, initial_pos[2]])
            p11 = np.array([initial_pos[0] , initial_pos[1] + L_span, initial_pos[2]])

        point  = p0 + (p11 - p0) * stance_progress + delta*np.sin(2*np.pi*stance_progress)

        return point


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

        legs_side = ["l", "r", "l", "r"]
        legs = ["FL", "FR", "RL", "RR"]
        joint_indices = [
            [3, 4, 6], 
            [8, 9, 11], 
            [13, 14, 16], 
            [18, 19, 21]
        ]
        
        # Directions for the motors (from pybullet_sim.py)
        theta_dirs = [-1, 1, 1,   # FL
                       1, 1, 1,   # FR
                      -1, 1, 1,   # RL
                       1, 1, 1]   # RR


        # Offset for the legs
        # Where the legs are in the cycle          
        leg_offsets = [0, 0.5, 0.5, 0]


        # PID CONTROLLER for roll, pitch, yaw
        if imu_data is not None:
            # Convert imu data to kinematics frame
            imu_data = from_pybullet_orn(imu_data)

            # Default time step
            dt = 1./240.

            # Calculate errors
            roll_error = self.initial_orientation[0] - imu_data[0] 
            pitch_error = self.initial_orientation[1] - imu_data[1]
            yaw_error = normalize_angle(self.initial_orientation[2] - imu_data[2])

            # print("---------------------------------")
            # print(f"target_roll : {self.initial_orientation[0]}")
            # print(f"target_pitch : {self.initial_orientation[1]}")
            # print(f"target_yaw : {self.initial_orientation[2]}")
            # print("---------------------------------")
            # print(f"imu_roll : {imu_data[0]}")
            # print(f"imu_pitch : {imu_data[1]}")
            # print(f"imu_yaw : {imu_data[2]}")
            # print("---------------------------------")
            # print(f"roll_error : {roll_error}")
            # print(f"pitch_error : {pitch_error}")
            # print(f"yaw_error : {yaw_error}")
            # print("---------------------------------")


            
            # Correct orientation
            roll_correction = self.pid_roll.update(roll_error, dt)
            pitch_correction = self.pid_pitch.update(pitch_error, dt)
            yaw_correction = self.pid_yaw.update(yaw_error, dt)
            
            # Different PID controller
            # compensation = self.pid_rp.run(imu_data[0], imu_data[1], dt)
            # roll_correction = compensation[0]
            # pitch_correction = compensation[1]

        corrected_orientation = (
            self.initial_orientation[0] + roll_correction, 
            self.initial_orientation[1] + pitch_correction, 
            self.initial_orientation[2] + yaw_correction)

        # Get Body IK transforms in Kinematics frame
        (T_fl, T_fr, T_rl, T_rr) = self.kin_solver.bodyIK(*corrected_orientation, *self.initial_center)
        transforms = [T_fl, T_fr, T_rl, T_rr]

        for i, leg in enumerate(legs):
            leg_phase = (global_phase + leg_offsets[i]) % 1
            
            # Global Frame Positions (Kinematics Frame: m, Z-up)
            initial_pos = self.initial_ef_positions[i][:3]
            
            
            # Stance Length and Swing Height are given in meters
            sl = stance_length
            sh = swing_height

            dl = 0.02
            L_span = sl / 2
            h1 = sh - 0.02
            h2 = sh
            
            if leg_phase < duty_factor:
                # Stance phase
                stance_progress = leg_phase / duty_factor

                # Stance moves backwards 
                # So foot moves from SL/2 to -SL/2
                # start_x = sl_mm / 2
                # end_x = -sl_mm / 2
                
                if dir == "+x":
                    p0 = np.array([initial_pos[0] + (sl / 2), initial_pos[1], initial_pos[2]])
                    p1 = np.array([initial_pos[0] - (sl / 2), initial_pos[1], initial_pos[2]])
                elif dir == "-x":
                    p0 = np.array([initial_pos[0] - (sl / 2), initial_pos[1], initial_pos[2]])
                    p1 = np.array([initial_pos[0] + (sl / 2), initial_pos[1], initial_pos[2]])
                # we change the z axis because we are using the kinematics frame
                elif dir == "+y":
                    p0 = np.array([initial_pos[0], initial_pos[1] + (sl / 2) , initial_pos[2] ]) 
                    p1 = np.array([initial_pos[0], initial_pos[1] - (sl / 2) , initial_pos[2] ])
                elif dir == "-y":
                    p0 = np.array([initial_pos[0], initial_pos[1] - (sl / 2) , initial_pos[2] ])
                    p1 = np.array([initial_pos[0], initial_pos[1] + (sl / 2) , initial_pos[2] ])

                control_points = [p0, p1]
                bezier_gen = bezier.BezierCurveGen(control_points)
                current_pos = bezier_gen.n_point_curve(control_points, stance_progress)
            
                # Unsuccesful sine wave stance
                # current_pos = self.stance_trajectory(initial_pos, L_span, dl, stance_progress, dir)

            else:
                # Swing phase
                swing_progress = (leg_phase - duty_factor) / (1 - duty_factor)
                
                # start_x = -sl_mm / 2
                # end_x = sl_mm / 2
                # Bezier Control Points apply the swing in the y because we are using the kinematics frame
                # 12 point bezier unsuccesful
                if dir == "+x":
                    p0 = np.array([initial_pos[0] - (sl / 2), initial_pos[1], initial_pos[2]])
                    p3 = np.array([initial_pos[0] + (sl / 2),   initial_pos[1], initial_pos[2]])
                    p1 = np.array([initial_pos[0] - (sl / 2), initial_pos[1], initial_pos[2] + sh ])  
                    p2 = np.array([initial_pos[0] + (sl / 2),   initial_pos[1], initial_pos[2] + sh ])
                    # control_points = self.swing_trajectory_control_points(initial_pos, L_span, dl, h1, h2, dir)
                                      
                elif dir == "-x":
                    p0 = np.array([initial_pos[0] + (sl / 2), initial_pos[1], initial_pos[2]])
                    p3 = np.array([initial_pos[0] - (sl / 2), initial_pos[1], initial_pos[2]])
                    p1 = np.array([initial_pos[0] + (sl / 2), initial_pos[1], initial_pos[2] + sh ]) 
                    p2 = np.array([initial_pos[0] - (sl / 2), initial_pos[1], initial_pos[2] + sh ])
                    # control_points = self.swing_trajectory_control_points(initial_pos, L_span, dl, h1, h2, dir)
                elif dir == "+y":
                    p0 = np.array([initial_pos[0], initial_pos[1] - (sl / 2) , initial_pos[2]   ])
                    p3 = np.array([initial_pos[0], initial_pos[1] + (sl / 2) ,   initial_pos[2] ])
                    p1 = np.array([initial_pos[0], initial_pos[1] - (sl / 2) , initial_pos[2]    + sh ]) 
                    p2 = np.array([initial_pos[0], initial_pos[1] + (sl / 2) ,   initial_pos[2]  + sh])
                    # control_points = self.swing_trajectory_control_points(initial_pos, L_span, dl, h1, h2, dir)
                elif dir == "-y":
                    p0 = np.array([initial_pos[0], initial_pos[1] + (sl / 2) , initial_pos[2]   ])
                    p3 = np.array([initial_pos[0], initial_pos[1] - (sl / 2) ,   initial_pos[2] ])
                    p1 = np.array([initial_pos[0], initial_pos[1] + (sl / 2) , initial_pos[2]    + sh ]) 
                    p2 = np.array([initial_pos[0], initial_pos[1] - (sl / 2) ,   initial_pos[2]  + sh])
                    # control_points = self.swing_trajectory_control_points(initial_pos, L_span, dl, h1, h2, dir)



                control_points = [p0, p1, p2, p3]
                bezier_gen = bezier.BezierCurveGen(control_points)
                current_pos = bezier_gen.n_point_curve(control_points, swing_progress)


            # Target Position is already in Kinematics Frame 
            # Convert to homogenous coordinates
            target_pos = to_homogenous(current_pos)

            # Get the shoulder base transform
            shoulder_base_transform = transforms[i]

            # Now that the point is in the kinematics body frame, we convert it to shoulder frame
            # target_pos_shoulder = inv(T_shoulder_body) @ target_pos_body
            target_pos_shoulder = np.linalg.inv(shoulder_base_transform) @ target_pos
            
            # Get the angles through IK
            angles = self.kin_solver.legIK(target_pos_shoulder, side=legs_side[i])

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
    