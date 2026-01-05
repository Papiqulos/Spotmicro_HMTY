import matplotlib.pyplot as plt
import numpy as np



class BezierCurveGen:

    def __init__(self, control_points):
        self.control_points = np.array(control_points)

    def linear_interpolate(self, p0, p1, t):
        return (1 - t) * p0 + t * p1
    
    def quadratic_interpolate(self, p0, p1, p2, t):
        return (1 - t)**2 * p0 + 2 * (1 - t) * t * p1 + t**2 * p2
    
    def cubic_interpolate(self, p0, p1, p2, p3, t):
        return (1 - t)**3 * p0 + 3 * (1 - t)**2 * t * p1 + 3 * (1 - t) * t**2 * p2 + t**3 * p3
    
    def generate_curve(self, num_points=100):
        n = len(self.control_points)
        curve = []
        for i in range(num_points + 1):
            t = i / num_points
            if n == 2:
                point = self.linear_interpolate(self.control_points[0], self.control_points[1], t)
            elif n == 3:
                point = self.quadratic_interpolate(self.control_points[0], self.control_points[1], self.control_points[2], t)
            elif n == 4:
                point = self.cubic_interpolate(self.control_points[0], self.control_points[1], self.control_points[2], self.control_points[3], t)
            else:
                raise ValueError("Invalid number of control points")
            curve.append(point)
        return np.array(curve)
    


if __name__ == "__main__":
    
    # Define control points for a cubic Bezier curve
    start = np.array([0, 0, 0])
    end = np.array([1, 1, 0])
    middle1 = start + end / 2 + np.array([0, 0, 1])  # Elevated middle point
    middle2 = end + start / 2 + np.array([0, 1, 1])  # Elevated middle point


    control_points = [start, middle1, middle2, end]

    # Create Bezier curve generator
    bezier_gen = BezierCurveGen(control_points)

    # Generate curve points
    curve_points = bezier_gen.generate_curve(num_points=1000)

    # Plotting
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot control points
    cp = np.array(control_points)
    ax.plot(cp[:, 0], cp[:, 1], cp[:, 2], 'ro--', label='Control Points')

    # Plot Bezier curve
    ax.plot(curve_points[:, 0], curve_points[:, 1], curve_points[:, 2], 'b-', label='Bezier Curve')

    ax.set_title('Cubic Bezier Curve')
    ax.legend()
    plt.show()
    


    