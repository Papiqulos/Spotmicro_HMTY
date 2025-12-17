import numpy as np
from math import *
import math

sHp=np.sin(pi/2)
cHp=np.cos(pi/2)
pi = math.pi
LENGTH = 125 # Length of Robot's base in mm
WIDTH = 108  # Width of Robot's base in mm

# Inverse Kinematics Constants
sHp = np.sin(pi/2)
cHp = np.cos(pi/2)

# Initial Foot Positions (Lp)
# [Front Left, Front Right, Back Left, Back Right]
Lp = np.array([
    [100, -100, 100, 1],
    [100, -100, -100, 1],
    [-100, -100, 100, 1],
    [-100, -100, -100, 1]
])

# Legend Origin (Identity)
Lo = np.array([0, 0, 0, 1])


# Lengths of leg segments
L1 = 50    # Horizontal offset from hip to knee in mm
Z_OFFSET = 0  # Height difference from hip joint to base in mm
L2 = 20    # Vertical offset from hip to knee in mm
L3 = 80   # Upper Leg Length in mm
L4 = 130   # Lower Leg Length in mm

# 1 - Left front ankle 125 -1
# 2 - Left front knee 95 -1
# 3 - Left front shoulder 100 -1

# 13 - Left back ankle 130 -1
# 14 - Left back knee 95 -1
# 15 - Left back shoulder 95 -1

# 4 - Right front ankle 70 1
# 5 - Right front knee 92 1
# 6 - Right front shoulder 85 -1

#  8 - Right back ankle 70 1
#  9 - Right back knee 92 1
# 10 - Right back shoulder 70 -1



DIRECTIONS = np.array([[1, 1, -1],    # Right Front
                       [-1, -1, -1],  # Left Front
                       [-1, -1, -1],  # Left Back
                       [1, 1, -1]])   # Right Back

ZEROES = np.array([[70, 92, 85],    # Right Front
                   [125, 95, 100],  # Left Front
                   [130, 95, 95],   # Left Back
                   [70, 92, 70]])   # Right Back

HIP_OFFSETS = {
            'FL': (LENGTH / 2, WIDTH / 2, Z_OFFSET),  # Front Left (+X, +Y, -Z)
            'FR': (LENGTH / 2, -WIDTH / 2, Z_OFFSET), # Front Right (+X, -Y, -Z)
            'RL': (-LENGTH / 2, WIDTH / 2, Z_OFFSET), # Rear Left (-X, +Y, -Z)
            'RR': (-LENGTH / 2, -WIDTH / 2, Z_OFFSET),# Rear Right (-X, -Y, -Z)
        }


def Rx(theta):
    return np.array([[1, 0, 0, 0],
                     [0, math.cos(theta), -math.sin(theta), 0],
                     [0, math.sin(theta), math.cos(theta), 0],
                     [0, 0, 0, 1]])

def Ry(theta):
    return np.array([[math.cos(theta), 0, math.sin(theta), 0],
                     [0, 1, 0, 0],
                     [-math.sin(theta), 0, math.cos(theta), 0],
                     [0, 0, 0, 1]])

def Rz(theta):
    return np.array([[math.cos(theta), -math.sin(theta), 0, 0],
                     [math.sin(theta), math.cos(theta), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])


class Kinematics:

    def __init__(self, l1=L1, l2=L2, l3=L3, l4=L4):
        
        self.origin = np.array([0, 0, 0, 1])  # Homogeneous coordinates of origin
        self.l1 = l1  # Horizontal offset from hip to knee in mm
        self.l2 = l2  # Vertical offset from hip to knee in mm
        self.l3 = l3  # Upper Leg Length in mm
        self.l4 = l4  # Lower Leg Length in mm
        
    def r_legs(self, theta1=0, theta2=0, theta3=0):
        # degrees to radians
        theta1 = np.deg2rad(theta1)
        theta2 = np.deg2rad(theta2)
        theta3 = np.deg2rad(theta3)
        return [
            [0,      pi/2,  0,    theta1],  # Hip joint
            [self.l3+self.l2,     0,      self.l1,     theta2],  # Knee joint
            [self.l4,     0,      0,     theta3]   # Ankle joint
        ]

    def l_legs(self, theta1=0, theta2=0, theta3=0):
        # degrees to radians
        theta1 = np.deg2rad(theta1)
        theta2 = np.deg2rad(theta2)
        theta3 = np.deg2rad(theta3)
        return [
            [0,      pi/2,  0,    -theta1],  # Hip joint
            [self.l3+self.l2,     0,      -self.l1,     -theta2],  # Knee joint
            [self.l4,     0,      0,     -theta3]   # Ankle joint
        ]

    # NEEDS FIXING
    def legFK(self, dh_params: list|np.ndarray):
        '''
        Forward kinematics using Denavit-Hartenberg parameters for one leg.
        Starting from the hip joint to the end effector (foot).

        :dh_params: List of DH parameters [a, alpha, d, theta] for each joint.

        Returns the transformation matrix from hip joint to end effector and the kinematic chain.
        '''
        T = np.eye(4)
        # Rotation to adjust coordinate system
        T_ = Ry(-pi/2) @ Rz(pi/2)
        
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
        Everything relative to hip joint

        :dh_params: List of DH parameters [a, alpha, d, theta] for each joint.

        Returns the kinematic chain of the homogenous vectors 
        '''
        (theta1, theta2, theta3) = angles
        theta23 = theta2 + theta3

        T0 = Lo
        T1 = T0 + np.array([-L1 * cos(theta1), L1 * sin(theta1), 0, 0])
        T2 = T1 + np.array([-L2 * sin(theta1), -L2 * cos(theta1), 0, 0])
        T3 = T2 + np.array([-L3 * sin(theta1) * cos(theta2), -L3 * cos(theta1) * cos(theta2), L3 * sin(theta2), 0])
        T4 = T3 + np.array([-L4 * sin(theta1) * cos(theta23), -L4 * cos(theta1) * cos(theta23), L4 * sin(theta23), 0])
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

        return([Tm @ np.array([[cHp,0,sHp,LENGTH/2],[0,1,0,0],[-sHp,0,cHp,WIDTH/2],[0,0,0,1]]),
            Tm @ np.array([[cHp,0,sHp,LENGTH/2],[0,1,0,0],[-sHp,0,cHp,-WIDTH/2],[0,0,0,1]]),
            Tm @ np.array([[cHp,0,sHp,-LENGTH/2],[0,1,0,0],[-sHp,0,cHp,WIDTH/2],[0,0,0,1]]),
            Tm @ np.array([[cHp,0,sHp,-LENGTH/2],[0,1,0,0],[-sHp,0,cHp,-WIDTH/2],[0,0,0,1]])
            ])

    # From https://spotmicroai.readthedocs.io/en/latest/kinematic/
    def legIK(self, point):
        """ Inverse Kinematics for a single leg 
        
        :param point: Target foot position (x, y, z) relative to hip
        
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

        return (theta1, theta2, theta3)

    def differential_kinematics(self, joint_angles, joint_velocities):
        # Placeholder for differential kinematics implementation
        pass




if __name__ == "__main__":

    print("Kinematics module loaded.")
    kinematics = Kinematics(L1, L2, L3)

    target_pos = [100,-200,100]
    
    theta1, theta2, theta3 = kinematics.legIK(target_pos)
    print(f"Inverse Kinematics for target position {target_pos}:\nTheta1: {math.degrees(theta1):.2f}°, Theta2: {math.degrees(theta2):.2f}°, Theta3: {math.degrees(theta3):.2f}°")

    # Verify with forward kinematics
    kinematic_chain = kinematics.legFK_hard_coded([theta1, theta2, theta3])
    print(f"Verifying with Forward Kinematics x={kinematic_chain[-1][0]:.2f}, y={kinematic_chain[-1][1]:.2f}, z={kinematic_chain[-1][2]:.2f}")
    
    

    