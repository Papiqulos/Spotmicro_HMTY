import numpy as np


class RobotState:
    """
    Robot state container.
    Units: radians, meters, seconds, radians/second, meters/second
    """
    def __init__(self,
                 init_angles=None,
                 init_ef_positions=None,
                 init_center=None,
                 init_orientation=None,
                 kin_solver=None):

        self.kin_solver        = kin_solver
        self.init_angles       = init_angles
        self.init_ef_positions = init_ef_positions
        self.init_center       = np.array(init_center,      dtype=float) if init_center      is not None else np.zeros(3)
        self.init_orientation  = np.array(init_orientation, dtype=float) if init_orientation is not None else np.zeros(3)

        self._angles       = init_angles.copy()
        self._ef_positions = init_ef_positions.copy()
        self._orientation  = self.init_orientation.copy()
        self.linear_vel    = 0.0
        self.angular_vel   = 0.0
        self.direction     = None

    # ------------------------------------------------------------------
    # Properties: writing angles triggers FK; writing ef_positions triggers IK.
    # Cross-writes go to the private attribute directly to prevent loops.
    # ------------------------------------------------------------------

    @property
    def orientation(self):
        return self._orientation

    @orientation.setter
    def orientation(self, value):
        self._orientation = np.array(value, dtype=float)

    @property
    def angles(self):
        return self._angles

    @angles.setter
    def angles(self, value):
        self._angles = np.array(value, dtype=float)
        if self.kin_solver is not None:
            self._ef_positions = np.array(
                self.kin_solver.robot_FK(self.init_center, self._orientation, self._angles),
                dtype=float)

    @property
    def ef_positions(self):
        return self._ef_positions

    @ef_positions.setter
    def ef_positions(self, value):
        self._ef_positions = np.array(value, dtype=float)
        if self.kin_solver is not None:
            self._angles = np.array(
                self.kin_solver.robot_IK(self.init_center, self._orientation, self._ef_positions),
                dtype=float)

    # ------------------------------------------------------------------

    def reset(self, angles=None, linear_vel=0.0, angular_vel=0.0, orientation=None):
        # Direct private writes: init values may not be in radians, skip FK/IK.
        self._angles       = self.init_angles.copy() if angles is None else np.array(angles, dtype=float)
        self._ef_positions = self.init_ef_positions.copy() if self.init_ef_positions is not None else None
        self._orientation  = self.init_orientation.copy() if orientation is None else np.array(orientation, dtype=float)
        self.linear_vel    = float(linear_vel)
        self.angular_vel   = float(angular_vel)
        self.direction     = None

    def update(self, angles, linear_vel, angular_vel, orientation, ef_positions=None):
        self.angles      = angles            # setter: FK updates _ef_positions
        self.linear_vel  = float(linear_vel)
        self.angular_vel = float(angular_vel)
        self._orientation = np.array(orientation, dtype=float)  # direct: avoids redundant second FK
        if ef_positions is not None:
            self._ef_positions = np.array(ef_positions, dtype=float)  # direct override, no IK
