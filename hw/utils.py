import numpy as np
import math



pi = np.pi

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

def rescale_number(value, original_min, original_max, new_min, new_max):
        """
        Rescale a number from one range to another
        """
        return ((value - original_min) / (original_max - original_min)) * \
                (new_max - new_min) + new_min


def to_homogenous(vec):
    """ Converts a 3D vector to homogeneous coordinates """
    return np.array([vec[0], vec[1], vec[2], 1.0])

def from_homogenous(vec):
    """ Converts homogeneous coordinates to a 3D vector """
    return vec[:3]

def normalize_angle(angle):
    """ Normalizes angle to [-pi, pi] """
    return (angle + pi) % (2 * pi) - pi

def to_pybullet_pos(vec):
    """ Converts Kinematics (Y-Up) to PyBullet (Z-Up) """
    return np.array([vec[0], vec[2], vec[1]]) / 1000.0  # Convert mm to meters

def from_pybullet_pos(vec):
    """ Converts PyBullet (Z-Up) to Kinematics (Y-Up) """
    return np.array([vec[0], vec[2], vec[1]]) * 1000.0  # Convert meters to mm

def to_pybullet_orn(vec):
    return np.array([vec[0], vec[1], vec[2]+pi])

def from_pybullet_orn(vec):
    return np.array([vec[0], vec[1], normalize_angle(vec[2]-pi)])