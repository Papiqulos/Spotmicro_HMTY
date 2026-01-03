import numpy as np
import matplotlib.pyplot as plt
from kinematics import Kinematics, WIDTH, LENGTH, L1, L2, L3, L4
from math import *


class RobotVisualizer:

    def __init__(self, kinematics_solver):
        self.kin = kinematics_solver
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_title('Quadruped Kinematic Model')
        self._setup_axes()

    def _setup_axes(self):
        limit = 250
        self.ax.set_xlim([-limit, limit])
        self.ax.set_ylim([-limit, limit])
        self.ax.set_zlim([-limit, limit])
        self.ax.set_xlabel('X (Forward)')
        self.ax.set_ylabel('Z (Left)')
        self.ax.set_zlabel('Y (Up)')
        self.ax.view_init(elev=20, azim=45)

    def _get_p(self, T):
        
        return T[:3, 3]

    def _draw_frame(self, T, label, scale=20, color='k'):
        origin = self._get_p(T)
        x_axis = T[:3, 0]
        y_axis = T[:3, 1]
        z_axis = T[:3, 2]
        
        self.ax.scatter(*origin, color=color, s=50)
        # self.ax.text(*origin, label, fontsize=8) # Optional: add labels to frames

        self.ax.quiver(*origin, *x_axis, color='r', length=scale, normalize=True)
        self.ax.quiver(*origin, *y_axis, color='g', length=scale, normalize=True)
        self.ax.quiver(*origin, *z_axis, color='b', length=scale, normalize=True)

    def _draw_body(self, hips):

        fl, fr, rr, rl = [hips[name] for name in ['FL', 'FR', 'RR', 'RL']]
        
        # Extract positions
        pos = [T[:3, 3] for T in [fl, fr, rr, rl, fl]]
        xs, ys, zs = zip(*pos)
        
        self.ax.plot(xs, zs, ys, linewidth=3, label='Chassis', color='blue')

    def draw_pose_FK(self, theta, orientation, center):
        
        theta = [radians(angle) for angle in theta]

        Ix = np.array([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

        fl_angles = theta[0:3]
        fr_angles = theta[3:6]
        bl_angles = theta[6:9]
        br_angles = theta[9:12]

        
        
        # Kinematic chain for every joint relative to hip
        fl_chain = self.kin.legFK_hard_coded(fl_angles)
        fr_chain = self.kin.legFK_hard_coded(fr_angles)
        bl_chain = self.kin.legFK_hard_coded(bl_angles)
        br_chain = self.kin.legFK_hard_coded(br_angles)


        # T_hip_base for each leg
        fl_hip_base, fr_hip_base, bl_hip_base, br_hip_base = self.kin.bodyIK(*orientation, *center)

        # Kinematic chain for every joint relative to base
        fl_chain_ = [fl_hip_base @ x for x in fl_chain]
        fr_chain_ = [fr_hip_base @ Ix @ x for x in fr_chain]
        bl_chain_ = [bl_hip_base @ x for x in bl_chain]
        br_chain_ = [br_hip_base @ Ix @ x for x in br_chain]

        print(f"Front Left leg:x={fl_chain_[-1][0]:.2f}, y={fl_chain_[-1][1]:.2f}, z={fl_chain_[-1][2]:.2f}")
        print(f"Front Right leg:x={fr_chain_[-1][0]:.2f}, y={fr_chain_[-1][1]:.2f}, z={fr_chain_[-1][2]:.2f}")
        print(f"Back Left leg:x={bl_chain_[-1][0]:.2f}, y={bl_chain_[-1][1]:.2f}, z={bl_chain_[-1][2]:.2f}")
        print(f"Back Right leg:x={br_chain_[-1][0]:.2f}, y={br_chain_[-1][1]:.2f}, z={br_chain_[-1][2]:.2f}")

        # Draw Body Chassis (Blue box)
        self._draw_body({'FL': fl_hip_base, 'FR': fr_hip_base, 'RL': bl_hip_base, 'RR': br_hip_base})

        # Draw Legs
        self.drawLegPoints(fl_chain_)
        self.drawLegPoints(fr_chain_)
        self.drawLegPoints(bl_chain_)
        self.drawLegPoints(br_chain_)

    # From https://spotmicroai.readthedocs.io/en/latest/kinematic/
    def drawLegPoints(self, p):
        """ Draws the lines and joints for a leg 

        :p: List of the kinematic chain for each leg. The kinematic chain is given in homogenous vector form"""
        
        # Plot links (Black line)
        self.ax.plot([x[0] for x in p], [x[2] for x in p], [x[1] for x in p], 'k-', lw=3)
        # Plot joints (Blue and Red dots)
        self.ax.plot([p[0][0]], [p[0][2]], [p[0][1]], 'bo', lw=2)
        self.ax.plot([p[-1][0]], [p[-1][2]], [p[-1][1]], 'ro', lw=2)    
    
    # From https://spotmicroai.readthedocs.io/en/latest/kinematic/
    def drawLegPair(self, Tl, Tr, Ll, Lr, tag):
        """ Calculates and draws a pair of legs (Left and Right) 
        
        :param Tl: Transformation matrix for left leg shoulder relative to body
        :param Tr: Transformation matrix for right leg shoulder relative to body
        :param Ll: Target foot position for left leg relative to body
        :param Lr: Target foot position for right leg relative to body
        """
        Ix = np.array([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        
        # Left Leg
        left_angles = self.kin.legIK(np.linalg.inv(Tl) @ Ll) # Passing foot position relative to left shoulder
        print(f"{tag} Left Leg Angles (rad): theta1={degrees(left_angles[0]):.2f}, theta2={degrees(left_angles[1]):.2f}, theta3={degrees(left_angles[2]):.2f}", )
        left_points = [Tl @ x for x in self.kin.legFK_hard_coded(left_angles)]
        self.drawLegPoints(left_points)
        
        # Right Leg
        right_angles = self.kin.legIK(Ix @ np.linalg.inv(Tr) @ Lr) # Passing foot position relative to right shoulder
        print(f"{tag} Right Leg Angles (rad): theta1={degrees(right_angles[0]):.2f}, theta2={degrees(right_angles[1]):.2f}, theta3={degrees(right_angles[2]):.2f}", )
        right_points = [Tr @ Ix @ x for x in self.kin.legFK_hard_coded(right_angles)]
        self.drawLegPoints(right_points)

    def draw_pose_IK(self, eof_positions, orientation, center):

        # T_hip_base for each leg
        (T_fl, T_fr, T_bl, T_br) = self.kin.bodyIK(*orientation, *center) 
        
        # Draw Body Chassis (Blue box)
        self._draw_body({'FL': T_fl, 'FR': T_fr, 'RL': T_bl, 'RR': T_br})

        # Draw Legs 
        self.drawLegPair(T_fl, T_fr, eof_positions[0], eof_positions[1], "Front") # Front Legs
        self.drawLegPair(T_bl, T_br, eof_positions[2], eof_positions[3], "Back") # Back Legs

    def draw_robot_pose(self, orientation = [0, 0, 0], center= [0, 0, 0], theta = [], eof_positions= [], mode='FK'):
        '''
        Draws the entire robot given certain orientation and joint angles or end effector positions
        
        '''
        orientation = [radians(angle) for angle in orientation]
        if mode == 'FK' and len(theta) == 12:
            self.draw_pose_FK(theta, orientation, center)
        elif mode == 'IK' and len(eof_positions) == 4:
            self.draw_pose_IK(eof_positions, orientation, center)


if __name__ == "__main__":
    kin_solver = Kinematics(LENGTH, WIDTH, L1, L2, L3, L4 )
    viz = RobotVisualizer(kin_solver)

    theta = [0, -30, 60, # FL
             0, -30, 60, # FR
             0, -30, 60, # RL
             0, -30, 60 ] # RR
    eof_positions = np.array([
        [ 95, 48.13,  105, 1], # FL
        [ 95, 48.13,  -105, 1], # FR
        [-45, 48.13, 105, 1], # RL
        [-45, 48.13, -105, 1] # RR
        ])
    # angles = (Roll, Pitch, Yaw) in radians
    # center = (x, y, z) in mm
    current_angles = [0, 0, 0]
    current_center = [0, 250, 0]

    # viz.draw_robot_pose(orientation=current_angles, center=current_center, theta=theta)
    viz.draw_robot_pose(orientation=current_angles, center=current_center, theta=theta, mode='FK')
    plt.show()