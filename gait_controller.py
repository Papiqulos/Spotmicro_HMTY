import numpy as np
import bezier_curve_gen as bezier
import kinematics
import matplotlib.pyplot as plt
from robot_mania_code import *
from utils import *


L1 = kinematics.L1
L2 = kinematics.L2
L3 = kinematics.L3
L4 = kinematics.L4
LENGTH = kinematics.LENGTH
WIDTH = kinematics.WIDTH

class GaitController:
    
    def __init__(self, initial_ef_positions=None, initial_theta=None, initial_center=None, initial_orientation=None):
        """
        :param initial_ef_positions: in kinematics frame and homogenous coordinates
        :param initial_theta: 
        :param initial_center: in kinematics frame
        :param initial_orientation: in kinematics frame
        """
        self.initial_ef_positions = initial_ef_positions 
        self.initial_theta = initial_theta
        self.initial_center = initial_center
        self.initial_orientation = initial_orientation
        self.kin_solver = kinematics.Kinematics(length=LENGTH, width=WIDTH, l1=L1, l2=L2, l3=L3, l4=L4)

    def generate_bezier_trajectory(self, control_points, num_points=100, leg="FL"):
        
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
    
    def trot_trajectory(self, start_pos, end_pos, starting_center, starting_orientation):
        pass
        



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
    