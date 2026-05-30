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
    return ((value - original_min) / (original_max - original_min)) * (new_max - new_min) + new_min


def trans_inv(T):
    """Inverts a 4x4 SE(3) homogeneous transform using R^T instead of np.linalg.inv."""
    R = T[:3, :3]
    p = T[:3, 3]
    Rt = R.T
    T_inv = np.eye(4)
    T_inv[:3, :3] = Rt
    T_inv[:3, 3] = -Rt @ p
    return T_inv

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


def euler2q(roll, pitch , yaw):
    """
    euler angles to quaternion
    """
    
    
    # Convert Euler angles to Quaternion [w, x, y, z]
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [w, x, y, z]