import numpy as np
from hw.quad_controller import RobotController, THETA_RESTING
from core.kinematics import LENGTH, WIDTH, L1, L2, L3, L4
import core.kinematics as kinematics


if __name__ == "__main__":

    kin_solver = kinematics.Kinematics(LENGTH, WIDTH, L1, L2, L3, L4)
    robot = RobotController(kin_solver, init_angles=THETA_RESTING, skip_rest=False)