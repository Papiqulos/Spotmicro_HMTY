import numpy as np
import matplotlib.pyplot as plt
from kinematics import Kinematics, L1, L2, L3, L4
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
        pos = [self._get_p(T) for T in [fl, fr, rr, rl, fl]]
        xs, ys, zs = zip(*pos)
        
        self.ax.plot(xs, zs, ys, linewidth=3, label='Chassis', color='blue')

    def drawLegPoints(self, p):
        """ Draws the lines and joints for a leg 

        :p: List of the kinematic chain for each leg. The kinematic chain is given in homogenous vector form"""
        print(p[0])
        # Plot links (Black line)
        self.ax.plot([x[0] for x in p], [x[2] for x in p], [x[1] for x in p], 'k-', lw=3)
        # Plot joints (Blue and Red dots)
        self.ax.plot([p[0][0]], [p[0][2]], [p[0][1]], 'bo', lw=2)
        self.ax.plot([p[4][0]], [p[4][2]], [p[4][1]], 'ro', lw=2)    

    def drawLegPair(self, Tl, Tr, Ll, Lr):
        """ Calculates and draws a pair of legs (Left and Right) 
        
        :param Tl: Transformation matrix for left leg shoulder relative to body
        :param Tr: Transformation matrix for right leg shoulder relative to body
        :param Ll: Target foot position for left leg relative to body
        :param Lr: Target foot position for right leg relative to body
        """
        Ix = np.array([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        
        # Left Leg
        left_angles = self.kin.legIK(np.linalg.inv(Tl) @ Ll) # Passing foot position relative to left shoulder
        print(f"Left Leg Angles (rad): theta1={degrees(left_angles[0]):.2f}, theta2={degrees(left_angles[1]):.2f}, theta3={degrees(left_angles[1]):.2f}", )
        left_points = [Tl @ x for x in self.kin.legFK_hard_coded(left_angles)]
        self.drawLegPoints(left_points)
        
        # Right Leg
        right_angles = self.kin.legIK(Ix @ np.linalg.inv(Tr) @ Lr) # Passing foot position relative to right shoulder
        print(f"Right Leg Angles (rad): theta1={degrees(left_angles[0]):.2f}, theta2={degrees(left_angles[1]):.2f}, theta3={degrees(left_angles[1]):.2f}", )
        right_points = [Tr @ Ix @ x for x in self.kin.legFK_hard_coded(right_angles)]
        self.drawLegPoints(right_points)

    def draw_pose_FK(self, theta, orientation, center):
        # Compute DH parameters for each leg
        theta = [radians(angle) for angle in theta]
        fl_angles = theta[0:3]
        fr_angles = theta[3:6]
        bl_angles = theta[6:9]
        br_angles = theta[9:12]

        Ix = np.array([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        
        # Kinematic chain for every joint relative to hip
        fl_chain = self.kin.legFK_hard_coded(fl_angles)
        fr_chain = self.kin.legFK_hard_coded(fr_angles)
        bl_chain = self.kin.legFK_hard_coded(bl_angles)
        br_chain = self.kin.legFK_hard_coded(br_angles)

        # fl_dh = self.kin.l_legs(*fl_angles)
        # fr_dh = self.kin.r_legs(*fr_angles)
        # bl_dh = self.kin.l_legs(*bl_angles)
        # br_dh = self.kin.r_legs(*br_angles)

        # _, chain_mfl = self.kin.legFK(fl_dh)
        # _, chain_mfr = self.kin.legFK(fr_dh)
        # _, chain_mbl = self.kin.legFK(bl_dh)
        # _, chain_mbr = self.kin.legFK(br_dh)

        # fl_chain = [M[:4, 3] for M in chain_mfl]
        # fr_chain = [M[:4, 3] for M in chain_mfr]
        # bl_chain = [M[:4, 3] for M in chain_mbl]
        # br_chain = [M[:4, 3] for M in chain_mbr]

        # T_hip_base for each leg
        fl_hip_base, fr_hip_base, bl_hip_base, br_hip_base = self.kin.bodyIK(*orientation, *center)

        # Kinematic chain for every joint relative to base
        fl_chain_ = [fl_hip_base @ x for x in fl_chain]
        fr_chain_ = [fr_hip_base @ Ix @ x for x in fr_chain]
        bl_chain_ = [bl_hip_base @ x for x in bl_chain]
        br_chain_ = [br_hip_base @ Ix @ x for x in br_chain]


        self._draw_body({'FL': fl_hip_base, 'FR': fr_hip_base, 'RL': bl_hip_base, 'RR': br_hip_base})

        self.drawLegPoints(fl_chain_)
        self.drawLegPoints(fr_chain_)
        self.drawLegPoints(bl_chain_)
        self.drawLegPoints(br_chain_)

    def draw_pose_IK(self, eof_positions, orientation, center):
        (omega, phi, psi) = orientation
        (xm, ym, zm) = center

        # 1. Calculate Body Position
        FP = [0, 0, 0, 1]
        (Tlf, Trf, Tlb, Trb) = self.kin.bodyIK(omega, phi, psi, xm, ym, zm) # Shoulder Positions of each leg
        
        # 2. Draw Body Chassis (Blue box)
        self._draw_body({'FL': Tlf, 'FR': Trf, 'RL': Tlb, 'RR': Trb})

        # 3. Draw Legs 
        self.drawLegPair(Tlf, Trf, eof_positions[0], eof_positions[1]) # Front Legs
        self.drawLegPair(Tlb, Trb, eof_positions[2], eof_positions[3]) # Back Legs

    def draw_robot_pose(self, orientation = [0, 0, 0], center= [0, 0, 0], theta = [], eof_positions= [], mode='FK'):
        '''
        Draws the entire robot given certain orientation and joint angles or end effector positions

        :theta: List of 12 joint angles [FL_hip, FL_knee, FL_ankle,
                                        FR_hip, FR_knee, FR_ankle,
                                        RL_hip, RL_knee, RL_ankle,
                                        RR_hip, RR_knee, RR_ankle]
        
        '''
        orientation = [radians(angle) for angle in orientation]
        if mode == 'FK' and len(theta) == 12:
            self.draw_pose_FK(theta, orientation, center)
        elif mode == 'IK' and len(eof_positions) == 4:
            self.draw_pose_IK(eof_positions, orientation, center)


if __name__ == "__main__":
    kin_solver = Kinematics()
    viz = RobotVisualizer(kin_solver)

    theta = [0, -30, 60, # FL
             0, -30, 60, # FR
             0, -30, 60, # RL
             0, -30, 60 ] # RR
    eof_positions = np.array([
        [ 100, -200,  100, 1], # Front Left
        [ 100, -200, -100, 1], # Front Right
        [-100, -200,  100, 1], # Back Left
        [-100, -200, -100, 1] # Back Right
        ])
    # angles = (Roll, Pitch, Yaw)
    # center = (x, y, z)
    current_angles = [20, 0, 0]
    current_center = [0, 0, 0]

    # viz.draw_robot_pose(orientation=current_angles, center=current_center, theta=theta)
    viz.draw_robot_pose(orientation=current_angles, center=current_center, eof_positions=eof_positions, mode='IK')

    plt.show()