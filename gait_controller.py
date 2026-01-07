import numpy as np
import bezier_curve_gen as bezier
import kinematics
import matplotlib.pyplot as plt
from robot_mania_code import *
from utils import *
import time
from pid_controller import PIDController, PIDControllerRP


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
        self.theta_dirs = [[-1, 1, 1],
                            [1, 1, 1],
                            [-1, 1, 1], 
                            [1, 1, 1]]
        self.kin_solver = kinematics.Kinematics(length=LENGTH, width=WIDTH, l1=L1, l2=L2, l3=L3, l4=L4)
        self.gait_init = None
        self.pid_pitch = PIDController(kp=0.2, ki=0.025, kd=0.025)
        self.pid_roll = PIDController(kp=0.2, ki=0.025, kd=0.025)
        self.pid_yaw = PIDController(kp=0.2, ki=0.025, kd=0.025)

        self.pid_rp = PIDControllerRP(kp=0.2, ki=0.025, kd=0.025)

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

    def swing_trajectory(self, start_pos, swing_height, leg="FL", stride_len=0.1, stride_orn="+x"):

        """Generates a swing trajectory for a single leg using a cubic Bezier curve moving 
        disp_length meters in the specified direction 
        with a specified swing_height in the +Z direction.

        :param start_pos: in pybullet frame and not homogenous coordinates
        :param swing_height: in pybullet frame
        :param leg: "FL", "FR", "RL", "RR"
        :param stride_len: stride length in meters
        :param stride_orn: stride orientation in pybullet frame
        """

        if stride_orn == "+x":
            end_pos = np.array([start_pos[0] + stride_len, start_pos[1], start_pos[2]])
        elif stride_orn == "-x":
            end_pos = np.array([start_pos[0] - stride_len, start_pos[1], start_pos[2]])
        elif stride_orn == "+y":
            end_pos = np.array([start_pos[0], start_pos[1] + stride_len, start_pos[2]])
        elif stride_orn == "-y":
            end_pos = np.array([start_pos[0], start_pos[1] - stride_len, start_pos[2]])



        middle_pos1 = np.array([start_pos[0], start_pos[1], start_pos[2] + swing_height]) # Directly above start position
        middle_pos2 = np.array([end_pos[0], end_pos[1], end_pos[2] + swing_height]) # Directly above end position

        control_points = [start_pos, middle_pos1, middle_pos2, end_pos]

        # 12 point bezier curve based on the MIT Cheetah paper (in the future)
        # control_points = np.array([[]
        # ])

        joint_angles, curve_points, control_points = self.generate_bezier_trajectory(control_points, num_points=100, leg=leg)

        return joint_angles, curve_points, control_points
    
    def stance_trajectory(self, start_pos, leg="FL", stride_len=0.1, stride_orn="-x"):
        """
        Generates a stance trajectory for a single leg moving disp_length meters in the specified direction
        
        :param start_pos: in pybullet frame and not homogenous coordinates
        :param leg: "FL", "FR", "RL", "RR"
        :param stride_len: stride length in meters
        :param stride_orn: stride orientation in pybullet frame
        """
        if stride_orn == "+x":
            end_pos = np.array([start_pos[0] + stride_len, start_pos[1], start_pos[2]])
        elif stride_orn == "-x":
            end_pos = np.array([start_pos[0] - stride_len, start_pos[1], start_pos[2]])
        elif stride_orn == "+y":
            end_pos = np.array([start_pos[0], start_pos[1] + stride_len, start_pos[2]])
        elif stride_orn == "-y":
            end_pos = np.array([start_pos[0], start_pos[1] - stride_len, start_pos[2]])

        control_points = [start_pos, end_pos]

        joint_angles, curve_points, control_points = self.generate_bezier_trajectory(control_points, num_points=100, leg=leg)

        return joint_angles, curve_points, control_points
    
    def trot(self, current_time, T_cycle, duty_factor, desired_velocity, swing_height, p, robotId, imu_data=None, dir="+x"):
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


        # Ramp up the velocity so that the robot doesn't start moving too fast and turns to the left
        if self.gait_init is None:
            self.gait_init = current_time

        ramp_duration = 0.5

        # Calculate time since start
        time_since_start = current_time - self.gait_init
        
        # Create a multiplier from 0.0 to 1.0
        if time_since_start < ramp_duration:
            ramp_factor = time_since_start / ramp_duration
        else:
            ramp_factor = 1.0
            
        # Apply ramp to velocity
        effective_velocity = desired_velocity * ramp_factor

        # Use effective_velocity instead of desired_velocity_x
        stance_length = effective_velocity * T_cycle * duty_factor

        # Global phase shows where we are in the cycle 
        # (0 means start of cycle, T_cycle means end of cycle)
        global_phase = (current_time % T_cycle) / T_cycle

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

        roll_correction = 0
        pitch_correction = 0
        yaw_correction = 0
        if imu_data is not None:
            # Convert imu data to kinematics frame
            imu_data = from_pybullet_orn(imu_data)

            # Default time step
            dt = 1./240.

            # # Calculate errors
            # roll_error = imu_data[0] - self.initial_orientation[0]
            # pitch_error = imu_data[1] - self.initial_orientation[1]
            # yaw_error = imu_data[2] - self.initial_orientation[2]

            
            # # Correct orientation
            # roll_correction = self.pid_roll.update(roll_error, dt)
            # pitch_correction = self.pid_pitch.update(pitch_error, dt)
            # yaw_correction = self.pid_yaw.update(yaw_error, dt)


            

            compensation = self.pid_rp.run(imu_data[0], imu_data[1], dt)
            roll_correction = compensation[0]
            pitch_correction = compensation[1]

        corrected_orientation = (
            self.initial_orientation[0] + roll_correction, 
            self.initial_orientation[1] + pitch_correction, 
            self.initial_orientation[2] )

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
            
            if leg_phase < duty_factor:
                # Stance phase
                stance_progress = leg_phase / duty_factor

                # Stance moves backwards in Kinematics Frame (Forward is +X)
                # So foot moves from -SL/2 to +SL/2
                start_x = sl_mm / 2
                end_x = -sl_mm / 2
                
                if dir == "+x":
                    p0 = np.array([initial_pos[0] + start_x, initial_pos[1], initial_pos[2]])
                    p1 = np.array([initial_pos[0] + end_x, initial_pos[1], initial_pos[2]])
                elif dir == "-x":
                    p0 = np.array([initial_pos[0] - start_x, initial_pos[1], initial_pos[2]])
                    p1 = np.array([initial_pos[0] - end_x, initial_pos[1], initial_pos[2]])
                elif dir == "+y":
                    p0 = np.array([initial_pos[0], initial_pos[1] , initial_pos[2] + start_x]) # we change the z axis because we are using the kinematics frame
                    p1 = np.array([initial_pos[0], initial_pos[1] , initial_pos[2] + end_x])
                elif dir == "-y":
                    p0 = np.array([initial_pos[0], initial_pos[1] , initial_pos[2] - start_x])
                    p1 = np.array([initial_pos[0], initial_pos[1] , initial_pos[2] - end_x])

                control_points = [p0, p1]
                bezier_gen = bezier.BezierCurveGen(control_points)
                current_pos = bezier_gen.n_point_curve(control_points, stance_progress)
            
                # current_pos = np.array([initial_pos[0] + delta_x, initial_pos[1] + delta_y, initial_pos[2]])
            else:
                # Swing phase
                swing_progress = (leg_phase - duty_factor) / (1 - duty_factor)
                
                start_x = -sl_mm / 2
                end_x = sl_mm / 2
                
                # Bezier Control Points apply the swing in the y because we are using the kinematics frame
                if dir == "+x":
                    p0 = np.array([initial_pos[0] + start_x, initial_pos[1], initial_pos[2]])
                    p3 = np.array([initial_pos[0] + end_x,   initial_pos[1], initial_pos[2]])
                    p1 = np.array([initial_pos[0] + start_x, initial_pos[1] + sh_mm, initial_pos[2] ])  
                    p2 = np.array([initial_pos[0] + end_x,   initial_pos[1] + sh_mm, initial_pos[2] ])
                elif dir == "-x":
                    p0 = np.array([initial_pos[0] - start_x, initial_pos[1], initial_pos[2]])
                    p3 = np.array([initial_pos[0] - end_x,   initial_pos[1], initial_pos[2]])
                    p1 = np.array([initial_pos[0] - start_x, initial_pos[1] + sh_mm, initial_pos[2] ]) 
                    p2 = np.array([initial_pos[0] - end_x,   initial_pos[1] + sh_mm, initial_pos[2] ])
                elif dir == "+y":
                    p0 = np.array([initial_pos[0], initial_pos[1] , initial_pos[2] + start_x])
                    p3 = np.array([initial_pos[0], initial_pos[1] ,   initial_pos[2] + end_x])
                    p1 = np.array([initial_pos[0], initial_pos[1] + sh_mm , initial_pos[2] + start_x ]) 
                    p2 = np.array([initial_pos[0], initial_pos[1] + sh_mm ,   initial_pos[2] + end_x ])
                elif dir == "-y":
                    p0 = np.array([initial_pos[0], initial_pos[1] , initial_pos[2] - start_x])
                    p3 = np.array([initial_pos[0], initial_pos[1] ,   initial_pos[2] - end_x])
                    p1 = np.array([initial_pos[0], initial_pos[1] + sh_mm , initial_pos[2] - start_x ]) 
                    p2 = np.array([initial_pos[0], initial_pos[1] + sh_mm ,   initial_pos[2] - end_x ])
                
                control_points = [p0, p1, p2, p3]
                bezier_gen = bezier.BezierCurveGen(control_points)
                current_pos = bezier_gen.n_point_curve(control_points, swing_progress)


            # Target Position is already in Kinematics Frame (Local Body Frame)
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
                


       
        
        



if __name__ == "__main__":
    gait = GaitController()
    point_x = np.array([95 /1000, 105 /1000, 48 /1000])
    point_y = np.array([170 /1000, 105 /1000, 48 /1000])
    center = np.array([0,0,0.25])
    orientation = [0,0,0]
    _, curve_points, control_points = gait.swing_trajectory(
                    start_pos=point_x,
                    swing_height=0.05,
                    center=center,
                    orientation=orientation
                )
    

     # Plotting
    fig = plt.figure()
    
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X (Forward)')
    ax.set_ylabel('Y (Left)')
    ax.set_zlabel('Z (Up)')

    # Plot control points
    cp = np.array(control_points)
    ax.plot(cp[:, 0], cp[:, 1], cp[:, 2], 'ro--', label='Control Points')

    # Plot Bezier curve
    ax.plot(curve_points[:, 0], curve_points[:, 1], curve_points[:, 2], 'b-', label='Bezier Curve')

    ax.set_title('Cubic Bezier Curve')
    ax.legend()
    plt.show()
    