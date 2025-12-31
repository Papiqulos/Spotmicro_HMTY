import numpy as np
import bezier_curve_gen as bezier
import kinematics
import matplotlib.pyplot as plt
from utils import *

class GaitController:
    def __init__(self, stance_time=0, swing_time=0, time_step=0, contact_phases=None, default_stance=None):
        self.stance_time = stance_time
        self.swing_time = swing_time
        self.time_step = time_step
        self.contact_phases = contact_phases
        self.default_stance = default_stance

    def swing_trajectory(self, start_pos, end_pos, swing_height, phase):
        middle_pos = (start_pos + end_pos) / 2
        middle_pos[2] += swing_height  # Raise Z for swing height

        control_points = [start_pos, middle_pos, end_pos]
        bezier_gen = bezier.BezierCurveGen(control_points)
        curve = bezier_gen.generate_curve(num_points=1000)

        # Get the angles of the trajectory points
        kinematics_solver = kinematics.Kinematics(kinematics.L1, kinematics.L2, kinematics.L3)
        joint_angles = []
        for point in curve:
            point = from_pybullet(point)
            angles = kinematics_solver.legIK(point)
            joint_angles.append(angles)

        return joint_angles, curve, control_points


    def stance_trajectory(self, start_pos, end_pos, phase):
        pass








if __name__ == "__main__":
    gait = GaitController()
    point_x = np.array([95 /1000, 105 /1000, 48 /1000])
    point_y = np.array([170 /1000, 105 /1000, 48 /1000])
    _, curve_points, control_points = gait.swing_trajectory(
                    start_pos=point_x,
                    end_pos=point_y,
                    swing_height=0.1,
                    phase=0
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
    