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


def to_homogenous(vec):
    """ Converts a 3D vector to homogeneous coordinates """
    return np.array([vec[0], vec[1], vec[2], 1.0])

def to_pybullet(vec):
    """ Converts Kinematics (Y-Up) to PyBullet (Z-Up) """
    return np.array([vec[0], vec[2], vec[1]]) / 1000.0  # Convert mm to meters

def from_pybullet(vec):
    """ Converts PyBullet (Z-Up) to Kinematics (Y-Up) """
    return np.array([vec[0], vec[2], vec[1]]) * 1000.0  # Convert meters to mm