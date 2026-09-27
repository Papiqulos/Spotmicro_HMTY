import atexit
import csv
import json
import math
import subprocess
import time
import numpy as np


pi = np.pi


def open_run_log(log_dir, prefix, header):
    """Open a timestamped CSV in log_dir and write its header. Old logs are kept.

    Returns (file, csv_writer). The file is closed at interpreter exit.
    """
    log_dir.mkdir(exist_ok=True)
    stamp = time.strftime('%Y_%m_%d_%H_%M_%S')
    path = log_dir / f"{prefix}_{stamp}.csv"
    n = 1
    while path.exists():
        path = log_dir / f"{prefix}_{stamp}_{n}.csv"
        n += 1
    log_file = open(path, "w", newline="")
    writer = csv.writer(log_file)
    writer.writerow(header)
    atexit.register(log_file.close)
    return log_file, writer


def git_commit():
    """Short HEAD hash, suffixed with '-dirty' when the working tree has changes. None outside git."""
    try:
        commit = subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                                capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "status", "--porcelain", "--untracked-files=no"],
                               capture_output=True, text=True, check=True).stdout.strip()
        return commit + ("-dirty" if dirty else "")
    except (OSError, subprocess.CalledProcessError):
        return None


def write_run_meta(csv_path, meta):
    """Write meta (plus the git commit) as JSON next to csv_path, same name with .json."""
    meta = dict(meta, git_commit=git_commit())
    path = str(csv_path).replace(".csv", ".json")
    with open(path, "w") as f:
        json.dump(meta, f, indent=2, default=float)
    return path

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