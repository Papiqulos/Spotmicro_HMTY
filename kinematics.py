import numpy as np
from math import *
import math
from utils import *

#--- Useful constants ---
sHp=np.sin(pi/2)
cHp=np.cos(pi/2)
pi = math.pi

# X(FORWARD), Y(UP), Z(LEFT), IDENTITY
# Legend Origin (Identity)
Lo = np.array([0, 0, 0, 1])


#--- Robot Dimensions ---
LENGTH = 140 # Length of Robot's base in mm
WIDTH = 110  # Width of Robot's base in mm

# Lengths of leg segments
L1 = 52    # Horizontal offset from shoulder to leg in mm
L2 = 0    # Vertical offset from shoulder to leg in mm
L3 = 120.416  # Upper Leg Length in mm
L4 = 115   # Lower Leg Length in mm


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

    def __init__(self, length=LENGTH, width=WIDTH, l1=L1, l2=L2, l3=L3, l4=L4):
        
        self.length = length  # Length of Robot's base in mm
        self.width = width   # Width of Robot's base in mm
        self.l1 = l1  # Horizontal offset from shoulder to leg in mm
        self.l2 = l2  # Vertical offset from shoulder to leg in mm
        self.l3 = l3  # Upper Leg Length in mm
        self.l4 = l4  # Lower Leg Length in mm

        self.theta_dirs = [-1, 1, 1,
                        1, 1, 1,
                        -1, 1, 1, 
                        1, 1, 1]
        
        # Inversion matrix for right legs
        self.Ix = np.array([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    
    # NOT USED
    def r_legs(self, theta1=0, theta2=0, theta3=0):
        # degrees to radians
        theta1 = np.deg2rad(theta1)
        theta2 = np.deg2rad(theta2)
        theta3 = np.deg2rad(theta3)
        return [
            [0,      pi/2,  0,    theta1],  # Shoulder joint
            [self.l3+self.l2,     0,      self.l1,     theta2],  # Leg joint
            [self.l4,     0,      0,     theta3]   # Foot joint
        ]

    # NOT USED
    def l_legs(self, theta1=0, theta2=0, theta3=0):
        # degrees to radians
        theta1 = np.deg2rad(theta1)
        theta2 = np.deg2rad(theta2)
        theta3 = np.deg2rad(theta3)
        return [
            [0,      pi/2,  0,    -theta1],  # Shoulder joint
            [self.l3+self.l2,     0,      -self.l1,     -theta2],  # Leg joint
            [self.l4,     0,      0,     -theta3]   # Foot joint
        ]

    # NEEDS FIXING
    def legFK(self, dh_params):
        '''
        Forward kinematics using Denavit-Hartenberg parameters for one leg.
        Starting from the shoulder joint to the end effector (foot).

        :dh_params: List of DH parameters [a, alpha, d, theta] for each joint.

        Returns the transformation matrix from shoulder joint to end effector and the kinematic chain.
        '''
        T = np.eye(4)
        # Rotation to adjust coordinate system
        T_ = Ry(-pi/2) @ Rz(-pi/2)
        
        kinematic_chain = []
        for a, alpha, d, theta in dh_params:
            T_i = np.array([[math.cos(theta), -math.sin(theta)*math.cos(alpha),  math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
                            [math.sin(theta),  math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
                            [0,                math.sin(alpha),                  math.cos(alpha),                 d],
                            [0,                0,                                0,                               1]])
            T = np.dot(T, T_i)
            kinematic_chain.append(T.copy())
        T = T_ @ T
        kinematic_chain = [T_ @ x for x in kinematic_chain]
        return T, kinematic_chain
    
    # From https://spotmicroai.readthedocs.io/en/latest/kinematic/
    def legFK_hard_coded(self, angles):
        '''
        Hard coded Forward kinematics for one leg.
        Everything relative to shoulder joint

        :angles: Tuple of joint angles (theta1, theta2, theta3)

        Returns the kinematic chain of the homogenous vectors 
        '''
        (theta1, theta2, theta3) = angles
        theta23 = theta2 + theta3

        T0 = Lo
        T1 = T0 + np.array([-self.l1 * cos(theta1), self.l1 * sin(theta1), 0, 0])
        T2 = T1 + np.array([-self.l2 * sin(theta1), -self.l2 * cos(theta1), 0, 0])
        T3 = T2 + np.array([-self.l3 * sin(theta1) * cos(theta2), -self.l3 * cos(theta1) * cos(theta2), self.l3 * sin(theta2), 0])
        T4 = T3 + np.array([-self.l4 * sin(theta1) * cos(theta23), -self.l4 * cos(theta1) * cos(theta23), self.l4 * sin(theta23), 0])
        return np.array([T0, T1, T2, T3, T4])
    
    # From https://spotmicroai.readthedocs.io/en/latest/kinematic/
    def bodyIK(self, omega, phi, psi, xm, ym, zm):
        """
        Return the transformation matrices for each leg shoulder relative to body given body center and orientation.
        
        :param omega: Roll angle (rotation around x-axis)
        :param phi: Pitch angle (rotation around y-axis)
        :param psi: Yaw angle (rotation around z-axis)
        :param xm: X-coordinate of body center
        :param ym: Y-coordinate of body center
        :param zm: Z-coordinate of body center
        """
        Rx = np.array([[1,0,0,0],
                    [0,np.cos(omega),-np.sin(omega),0],
                    [0,np.sin(omega),np.cos(omega),0],[0,0,0,1]])
        Ry = np.array([[np.cos(phi),0,np.sin(phi),0],
                    [0,1,0,0],
                    [-np.sin(phi),0,np.cos(phi),0],[0,0,0,1]])
        Rz = np.array([[np.cos(psi),-np.sin(psi),0,0],
                    [np.sin(psi),np.cos(psi),0,0],[0,0,1,0],[0,0,0,1]])
        Rxyz=Rx @ Ry @ Rz

        T = np.array([[0,0,0,xm],[0,0,0,ym],[0,0,0,zm],[0,0,0,0]])
        Tm = T+Rxyz

        return([Tm @ np.array([[cHp,0,sHp,self.length/2],[0,1,0,0],[-sHp,0,cHp,self.width/2],[0,0,0,1]]),
            Tm @ np.array([[cHp,0,sHp,self.length/2],[0,1,0,0],[-sHp,0,cHp,-self.width/2],[0,0,0,1]]),
            Tm @ np.array([[cHp,0,sHp,-self.length/2],[0,1,0,0],[-sHp,0,cHp,self.width/2],[0,0,0,1]]),
            Tm @ np.array([[cHp,0,sHp,-self.length/2],[0,1,0,0],[-sHp,0,cHp,-self.width/2],[0,0,0,1]])
            ])

    # From https://spotmicroai.readthedocs.io/en/latest/kinematic/
    def legIK(self, point):
        """ Inverse Kinematics for a single leg 
        
        :param point: Target foot position (x, y, z) relative to shoulder
        
        :returns: Tuple of joint angles (theta1, theta2, theta3)"""
        (x, y, z) = (point[0], point[1], point[2])
        
        # Check if target is reachable (simple validity check)
        if x**2 + y**2 - self.l1**2 < 0:
            return (0, 0, 0) # Error safety

        F = sqrt(x**2 + y**2 - self.l1**2)
        G = F - self.l2  
        H = sqrt(G**2 + z**2)
        
        theta1 = -atan2(y, x) - atan2(F, - self.l1)

        D = (H**2 - self.l3**2 - self.l4**2) / (2 * self.l3 * self.l4)
        
        # Domain check for acos
        if D > 1: D = 1
        if D < -1: D = -1
            
        theta3 = acos(D)

        theta2 = atan2(z, G) - atan2(self.l4 * sin(theta3), self.l3 + self.l4 * cos(theta3))

        return [theta1, theta2, theta3]
    
    def robot_IK(self, center, orientation, ef_positions):
        """Returns radians"""
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
        fr_angles = self.legIK(self.Ix @ np.linalg.inv(T_shoulder_base[1]) @ to_homogenous(fr)) # Passing foot position relative to right shoulder
        for angle in fr_angles:
            angles.append(angle)
            
        # Rear Left Leg
        rl_angles = self.legIK(np.linalg.inv(T_shoulder_base[2]) @ to_homogenous(rl)) # Passing foot position relative to left shoulder
        for angle in rl_angles:
            angles.append(angle)

        # Rear Right Leg
        rr_angles = self.legIK(self.Ix @ np.linalg.inv(T_shoulder_base[3]) @ to_homogenous(rr)) # Passing foot position relative to right shoulder
        for angle in rr_angles:
            angles.append(angle)

        return angles # [ [FL angles], [FR angles], [RL angles], [RR angles] ]
    
    def robot_FK(self, center, orientation, joint_angles, unit='radians'):
        """Returns mm and X(forward) Y(up) Z(left) 1(identity)"""
        # Convert angles from degrees to radians if necessary
        if unit == 'degrees':
            joint_angles = [math.radians(angle) for angle in joint_angles]


        fl_angles = joint_angles[0:3]
        fr_angles = joint_angles[3:6]
        rl_angles = joint_angles[6:9]
        rr_angles = joint_angles[9:12]

        # Kinematic chain for every joint relative to shoulder
        fl_chain = self.legFK_hard_coded(fl_angles)
        fr_chain = self.legFK_hard_coded(fr_angles)
        rl_chain = self.legFK_hard_coded(rl_angles)
        rr_chain = self.legFK_hard_coded(rr_angles)

        # T_shoulder_base for each leg
        fl_shoulder_base, fr_shoulder_base, rl_shoulder_base, rr_shoulder_base = self.bodyIK(*orientation, *center)


        # Kinematic chain for every joint relative to base
        fl_chain_ = [fl_shoulder_base @ x for x in fl_chain]
        fr_chain_ = [fr_shoulder_base @ self.Ix @ x for x in fr_chain]
        rl_chain_ = [rl_shoulder_base @ x for x in rl_chain]
        rr_chain_ = [rr_shoulder_base @ self.Ix @ x for x in rr_chain]

        ef_positions = [fl_chain_[-1], fr_chain_[-1], rl_chain_[-1], rr_chain_[-1]]


        return ef_positions





if __name__ == "__main__":

    print("Kinematics module loaded.")
    kinematics = Kinematics(LENGTH, WIDTH, L1, L2, L3, L4)

    theta = [0, -30, 60, # FL
             0, -30, 60, # FR
             0, -30, 60, # RL
             0, -30, 60 ] # RR
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
    center = [0, 250, 0]  # X, Y, Z in mm


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
    angles_deg = [math.degrees(angle) for angle in angles]
    print(f"Front Left Leg Angles (rad): theta1={(angles_deg[0]):.2f}, theta2={(angles_deg[1]):.2f}, theta3={(angles_deg[2]):.2f}")
    print(f"Front Right Leg Angles (rad): theta1={(angles_deg[3]):.2f}, theta2={(angles_deg[4]):.2f}, theta3={(angles_deg[5]):.2f}")
    print(f"Rear Left Leg Angles (rad): theta1={(angles_deg[6]):.2f}, theta2={(angles_deg[7]):.2f}, theta3={(angles_deg[8]):.2f}")
    print(f"Rear Right Leg Angles (rad): theta1={(angles_deg[9]):.2f}, theta2={(angles_deg[10]):.2f}, theta3={(angles_deg[11]):.2f}")