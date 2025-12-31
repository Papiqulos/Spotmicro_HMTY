import numpy as np



pi = np.pi




def to_pybullet(vec):
    """ Converts Kinematics (Y-Up) to PyBullet (Z-Up) """
    return np.array([vec[0], vec[2], vec[1]]) / 1000.0  # Convert mm to meters

def from_pybullet(vec):
    """ Converts PyBullet (Z-Up) to Kinematics (Y-Up) """
    return np.array([vec[0], vec[2], vec[1]]) * 1000.0  # Convert meters to mm