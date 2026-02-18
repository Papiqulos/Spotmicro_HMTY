import numpy as np
from math import *
import math
from utils import *

#--- Useful constants ---
pi = math.pi

#--- Robot Dimensions ---
LENGTH = 14.0 # Length of Robot's base in cm
WIDTH = 11.0  # Width of Robot's base in cm

# Lengths of leg segments
L1 = 5    # Horizontal offset from shoulder to leg in cm
L2 = 11    # Upper Leg Length in cm
L3 = 13  # Lower Leg Length in cm

#--- Real world parameters and connectivity ---
# pca pin - joint name - zero angle - direction of rotation
# 1 - Left front foot - 125 - -1
# 2 - Left front leg - 95 - -1
# 3 - Left front shoulder - 100 -1

# 13 - Left rear foot - 130 - -1
# 14 - Left rear leg - 95 - -1
# 15 - Left rear shoulder - 95 - -1

# 4 - Right front foot - 70 - 1
# 5 - Right front leg - 92 - 1
# 6 - Right front shoulder - 85 - -1

#  8 - Right rear foot-  70 - 1
#  9 - Right rear leg - 92 - 1
# 10 - Right rear shoulder - 70 - -1

DIRECTIONS = np.array([[1, 1, -1],    # Right Front
                       [-1, -1, -1],  # Left Front
                       [-1, -1, -1],  # Left Rear
                       [1, 1, -1]])   # Right Rear

ZEROES = np.array([[70, 92, 85],    # Right Front
                   [125, 95, 100],  # Left Front
                   [130, 95, 95],   # Left Rear
                   [70, 92, 70]])   # Right Rear





class Kinematics:

    def __init__(self, length=LENGTH, width=WIDTH, l1=L1, l2=L2, l3=L3):
        
        self.length = length  # Length of Robot's base in cm
        self.width = width   # Width of Robot's base in cm
        self.l1 = l1  # Horizontal offset from shoulder to leg in cm
        self.l2 = l2  # Upper Leg Length in cm
        self.l3 = l3  # Lower Leg Length in cm

        self.theta_dirs = [-1, 1, 1,
                        1, 1, 1,
                        -1, 1, 1, 
                        1, 1, 1] 

        # Inversion matrix for right legs
        self.Ix = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])    
    
    ## DH FK and ROS2 FRAME X(Forward), Y(Left), Z(Up) Frame
    def bodyIK(self, roll, pitch, yaw, xm, ym, zm):
        """
        Calculates the 4x4 Transformation Matrices for the 4 shoulders.
        Frame: ROS 2 (X-Forward, Y-Left, Z-Up)
        
        :param roll:  Rotation around X-axis
        :param pitch: Rotation around Y-axis
        :param yaw:   Rotation around Z-axis
        :param xm, ym, zm: Body center coordinates
        """
        
        # Rotation Matrices 
        c_r, s_r = np.cos(roll),  np.sin(roll)
        c_p, s_p = np.cos(pitch), np.sin(pitch)
        c_y, s_y = np.cos(yaw),   np.sin(yaw)

        Rx = np.array([
            [1,   0,    0, 0],
            [0, c_r, -s_r, 0],
            [0, s_r,  c_r, 0],
            [0,   0,    0, 1]
        ])
        
        Ry = np.array([
            [ c_p, 0, s_p, 0],
            [   0, 1,   0, 0],
            [-s_p, 0, c_p, 0],
            [   0, 0,   0, 1]
        ])
        
        Rz = np.array([
            [c_y, -s_y, 0, 0],
            [s_y,  c_y, 0, 0],
            [  0,    0, 1, 0],
            [  0,    0, 0, 1]
        ])

        # Body Rotation
        R_body = Rz @ Ry @ Rx

        # Body Translation
        T_trans = np.array([
            [0, 0, 0, xm],
            [0, 0, 0, ym],
            [0, 0, 0, zm],
            [0, 0, 0,  0]
        ])
        
        # Combined Body Transformation
        Tm = T_trans + R_body

        # Shoulder Offsets
        # We define where the shoulders are attached relative to the center.
        # X = Length, Y = Width, Z = 0
        l = self.length / 2.0
        w = self.width / 2.0

        # Create translation matrices for each shoulder
        # Format: [[1,0,0, x], [0,1,0, y], [0,0,1, z], [0,0,0,1]]
        
        # Front Left (+X, +Y)
        T_FL = np.array([[1,0,0, l], [0,1,0, w], [0,0,1, 0], [0,0,0,1]])
        
        # Front Right (+X, -Y)
        T_FR = np.array([[1,0,0, l], [0,1,0,-w], [0,0,1, 0], [0,0,0,1]])
        
        # Rear Left (-X, +Y)
        T_RL = np.array([[1,0,0,-l], [0,1,0, w], [0,0,1, 0], [0,0,0,1]])
        
        # Rear Right (-X, -Y)
        T_RR = np.array([[1,0,0,-l], [0,1,0,-w], [0,0,1, 0], [0,0,0,1]])

        # 4. Return Transformed Shoulders
        return [
            Tm @ T_FL,
            Tm @ T_FR,
            Tm @ T_RL,
            Tm @ T_RR
        ]

    def legIK(self, point):
        """ Inverse Kinematics for a single leg 
        
        :param point: Target foot position (x, y, z) relative to shoulder
        
        :returns: Tuple of joint angles (theta1, theta2, theta3)"""
        (x, y, z) = (point[0], point[1], point[2])
        
        A = math.sqrt(y**2 + z**2)

        a1 = math.atan2(y, z)
        a3 = pi/2 - math.asin(self.l1 / A)

        theta1 = a1 + a3 - pi/2
        z1 = z + self.l1*math.sin(theta1)
        z_ = z1*math.cos(theta1)

        A2 = math.sqrt(x**2 + z_**2)
        R1 = math.atan(x / z)

        R2 = math.acos((A2**2 +self.l2**2 - self.l3**2) / (2*A2*self.l2))

        theta2 = R1 - R2

        R3 = math.acos((self.l2**2 + self.l3**2 - A2**2) / (2*self.l2*self.l3))

        theta3 = pi/2 - R3




        return [theta1, theta2, theta3]

    def dh_params(self, theta1=0, theta2=0, theta3=0, side="r"):
        # degrees to radians
        theta1 = np.deg2rad(theta1)
        theta2 = np.deg2rad(theta2)
        theta3 = np.deg2rad(theta3)
        # cm and radians
        foot_sign = 1
        if side == "r":
            foot_sign = 1
        else:
            foot_sign = -1
        
        return [
            [0,      pi/2,   0,      theta1],  # Shoulder joint
            [self.l2,     0,      foot_sign*self.l1,     theta2],  # Leg joint
            [self.l3,     0,      0,     theta3]   # Foot joint
        ]

    def legFK(self, dh_params):
        '''
        Forward kinematics using Denavit-Hartenberg parameters for one leg.
        Starting from the shoulder joint to the end effector (foot).

        :dh_params: List of DH parameters [a, alpha, d, theta] for each joint.

        Returns the transformation matrix from shoulder joint to end effector and the kinematic chain.
        '''
        T = np.eye(4)
        # Frame so coordinates match the ros2 standard X(Forward), Y(Left), Z(Up)
        T_BaseFrame = np.array([
                       [ 0, 0, 1, 0],
                       [ 0, 1, 0, 0],
                       [-1, 0, 0, 0],
                       [ 0, 0, 0, 1]])
        
        kinematic_chain = []
        for a, alpha, d, theta in dh_params:
            T_i = np.array([
                            [math.cos(theta), -math.sin(theta)*math.cos(alpha),  math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
                            [math.sin(theta),  math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
                            [0,                math.sin(alpha),                  math.cos(alpha),                 d],
                            [0,                0,                                0,                               1]])
            T = T @ T_i
            kinematic_chain.append(T.copy())
        T_final = T_BaseFrame @ T
        kinematic_chain = [T_BaseFrame @ x for x in kinematic_chain]
        return T_final, kinematic_chain

    def robot_IK(self, center, orientation, ef_positions):
        """
        :center: X, Y, Z in cm
        :orientation: Roll, Pitch, Yaw in radians
        :ef_postions: X, Y, Z in cm
        
        
        
        Returns all joint angles for each leg in radians
        """
        # T_shoulder_base for each leg
        T_shoulder_base = self.bodyIK(*orientation, *center)

        fl = ef_positions[0]
        fr = ef_positions[1]
        rl = ef_positions[2]
        rr = ef_positions[3]

        angles = []

        # Front Left Leg
        fl_angles = self.legIK(np.linalg.inv(T_shoulder_base[0]) @ to_homogenous(fl)) # Passing foot position relative to left shoulder
        for angle in fl_angles:
            angles.append(angle)

        # Front Right Leg
        fr_angles = self.legIK( np.linalg.inv(T_shoulder_base[1]) @ to_homogenous(fr)) # Passing foot position relative to right shoulder
        for angle in fr_angles:
            angles.append(angle)
            
        # Rear Left Leg
        rl_angles = self.legIK(np.linalg.inv(T_shoulder_base[2]) @ to_homogenous(rl)) # Passing foot position relative to left shoulder
        for angle in rl_angles:
            angles.append(angle)

        # Rear Right Leg
        rr_angles = self.legIK( np.linalg.inv(T_shoulder_base[3]) @ to_homogenous(rr)) # Passing foot position relative to right shoulder
        for angle in rr_angles:
            angles.append(angle)

        return angles # [FL angles, FR angles, RL angles, RR angles]

    def robot_FK(self, center, orientation, joint_angles, unit='radians'):
        """Returns X(Forward) Y(Up) Z(Left) in cm"""

        if unit == "degrees":
            joint_angles = [math.radians(angle) for angle in joint_angles]

        fl_angles = joint_angles[0:3]
        fr_angles = joint_angles[3:6]
        rl_angles = joint_angles[6:9]
        rr_angles = joint_angles[9:12]

        # Kinematic chain for every joint relative to shoulder
        _, fl_chain = self.legFK(self.dh_params(*fl_angles, "l"))
        _, fr_chain = self.legFK(self.dh_params(*fr_angles, "r"))
        _, rl_chain = self.legFK(self.dh_params(*rl_angles, "l"))
        _, rr_chain = self.legFK(self.dh_params(*rr_angles, "r"))


        # T_shoulder_base for each leg
        fl_shoulder_base, fr_shoulder_base, rl_shoulder_base, rr_shoulder_base = self.bodyIK(*orientation, *center)

        # Kinematic chain for every joint relative to base
        fl_chain_ = [fl_shoulder_base @ x for x in fl_chain]
        fr_chain_ = [fr_shoulder_base @ x  for x in fr_chain]
        rl_chain_ = [rl_shoulder_base @ x for x in rl_chain]
        rr_chain_ = [rr_shoulder_base @ x  for x in rr_chain]

        ef_positions = [fl_chain_[-1][:3, 3], fr_chain_[-1][:3, 3], rl_chain_[-1][:3, 3], rr_chain_[-1][:3, 3]]


        return ef_positions
    


if __name__ == "__main__":

    print("Kinematics module loaded.")
    kinematics = Kinematics(LENGTH, WIDTH, L1, L2, L3)

    theta = [0, -30, 60, # FL
             0, -30, 60, # FR
             0, -30, 60, # RL
             0, -30, 60 ] # RR
    
    rads = [0, -pi/6,  pi/3, # FL
             0, -pi/6, pi/3, # FR
             0, -pi/6, pi/3, # RL
             0, -pi/6, pi/3 ] # RR

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
    

    
    orientation = [0, 0, 0]  # Roll, Pitch, Yaw in radians
    center = [0, 0, 25]  # X, Y, Z in cm


    leg_points = kinematics.robot_FK(center, orientation, theta, unit='degrees')
    print(f"Front Left leg:x={leg_points[0][0]:.2f}, y={leg_points[0][1]:.2f}, z={leg_points[0][2]:.2f}")
    print(f"Front Right leg:x={leg_points[1][0]:.2f}, y={leg_points[1][1]:.2f}, z={leg_points[1][2]:.2f}")
    print(f"Rear Left leg:x={leg_points[2][0]:.2f}, y={leg_points[2][1]:.2f}, z={leg_points[2][2]:.2f}")
    print(f"Rear Right leg:x={leg_points[3][0]:.2f}, y={leg_points[3][1]:.2f}, z={leg_points[3][2]:.2f}")

    # Verify IK
    ef_positions = np.array([
        leg_points[0],
        leg_points[1],
        leg_points[2],
        leg_points[3]
    ])
    angles = kinematics.robot_IK(center, orientation, ef_positions)
    # Convert radians to degrees for better readability
    angles_deg = [np.degrees(angle) for angle in angles]
    print(f"Front Left Leg Angles (rad): theta1={(angles_deg[0]):.2f}, theta2={(angles_deg[1]):.2f}, theta3={(angles_deg[2]):.2f}")
    print(f"Front Right Leg Angles (rad): theta1={(angles_deg[3]):.2f}, theta2={(angles_deg[4]):.2f}, theta3={(angles_deg[5]):.2f}")
    print(f"Rear Left Leg Angles (rad): theta1={(angles_deg[6]):.2f}, theta2={(angles_deg[7]):.2f}, theta3={(angles_deg[8]):.2f}")
    print(f"Rear Right Leg Angles (rad): theta1={(angles_deg[9]):.2f}, theta2={(angles_deg[10]):.2f}, theta3={(angles_deg[11]):.2f}")


        

        
        


