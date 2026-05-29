import numpy as np
from math import *
import math
from tools.utils import *
import yaml

with open("config/robot_config.yaml") as f:
    robot_cfg = yaml.safe_load(f)

#--- Useful constants ---
pi = math.pi
sHp=np.sin(pi/2)
cHp=np.cos(pi/2)


# X(FORWARD), Y(UP), Z(LEFT), IDENTITY
# Legend Origin (Identity)
Lo = np.array([0, 0, 0, 1])


#--- Robot Dimensions ---
LENGTH = robot_cfg["body"]["length"]
WIDTH = robot_cfg["body"]["width"] 

# Lengths of leg segments
L1 = robot_cfg["leg_segments"]["L1"]    # Horizontal offset from shoulder to leg in mm
L2 = robot_cfg["leg_segments"]["L2"]    # Vertical offset from shoulder to leg in mm
L3 = robot_cfg["leg_segments"]["L3"]    # Upper Leg Length in mm
L4 = robot_cfg["leg_segments"]["L4"]    # Lower Leg Length in mm







# AXIS X forward Y up Z left 
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
        
    # From https://spotmicroai.readthedocs.io/en/latest/kinematic/
    def legFK(self, angles):
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
    def bodyIK(self, omega, psi, phi, xm, ym, zm):
        """
        Return the transformation matrices for each leg shoulder relative to body given body center and orientation.
        
        :param omega: Roll angle (rotation around x-axis)
        :param psi: Pitch angle (rotation around z-axis)
        :param phi: Yaw angle (rotation around y-axis)
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

        return angles # [FL angles, FR angles, RL angles, RR angles]
    
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
        fl_chain = self.legFK(fl_angles)
        fr_chain = self.legFK(fr_angles)
        rl_chain = self.legFK(rl_angles)
        rr_chain = self.legFK(rr_angles)

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
        [14.62, 33.77, 107.00, 1],
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