import math
import numpy as np


class BezierCurveGen:

    """
    General Bezier Curve Generator using Bernstein polynomials
    """

    def __init__(self, control_points):
        self.control_points = np.array(control_points)

    @staticmethod
    def n_point_curve(points, t):
        n = len(points)
        interpolation = 0
        for i, point in enumerate(points):
            interpolation += math.comb(n-1, i) * point * (1 - t)**( (n-1 - i) )* t**i
        return interpolation
    
    def generate_curve(self, num_points=100):
        curve = []
        for i in range(num_points + 1):
            t = i / num_points
            point = self.n_point_curve(self.control_points, t)
            curve.append(point)
        return np.array(curve)


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import yaml

    with open("config/robot_config.yaml") as f:
        gait_cfg = yaml.safe_load(f)["gait"]

    x_norm = gait_cfg["swing_x_norm"]
    h_norm = gait_cfg["swing_h_norm"]
    control_points = np.array([[x, h, 0.0] for x, h in zip(x_norm, h_norm)])

    curve_points = BezierCurveGen(control_points).generate_curve(num_points=1000)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(control_points[:, 0], control_points[:, 2], control_points[:, 1], 'ro--', label='Control Points')
    ax.plot(curve_points[:, 0], curve_points[:, 2], curve_points[:, 1], 'b-', label='Bezier Curve')
    ax.set_title('Bezier Curve')
    ax.legend()
    ax.get_xaxis().set_visible(False)
    ax.get_yaxis().set_visible(False)
    ax.get_zaxis().set_visible(False)
    plt.savefig("bezier.png")
