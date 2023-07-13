import numpy as np
import matplotlib.pyplot as plt
from scipy.special import comb

def bezier_curve_length(control_points, num_samples=10):
    t = np.linspace(0, 1, num_samples)

    dx_dt = 2 * (1 - t) * (control_points[1][0] - control_points[0][0]) + 2 * t * (control_points[2][0] - control_points[1][0])
    dy_dt = 2 * (1 - t) * (control_points[1][1] - control_points[0][1]) + 2 * t * (control_points[2][1] - control_points[1][1])
    integrand = np.sqrt(dx_dt**2 + dy_dt**2)
    length = np.sum(integrand) / num_samples
    return length

class BezierCurveGenerator:
    def __init__(self, interestPT1, interestPT2, control_PT, curvity, frequency = 10):
        self.interestPT1 = np.array(interestPT1)
        self.interestPT2 = np.array(interestPT2)
        self.control_PT = control_PT
        self.curvity = curvity
        self.MID_PT = None
        self.var = 0
        self.frequency = frequency

    def bezier_curve(self, t, control_points):
        n = len(control_points) - 1
        curve_point = np.zeros(2)
        for i, point in enumerate(control_points):
            curve_point += comb(n, i) * (1 - t)**(n - i) * t**i * point
        return curve_point

    def generate_ctr_point(self):
        midpoint = (self.interestPT1 + self.interestPT2) / 2.0
        self.var = np.random.normal(loc=0, scale=self.curvity, size=2)
        displacement = (self.var+0.1*self.control_PT) * np.linalg.norm(self.interestPT2 - self.interestPT1)
        self.MID_PT = midpoint + displacement

    def generate_random_bezier_curve(self):
        self.generate_ctr_point()
        control_points = [self.interestPT1, self.MID_PT, self.interestPT2]

        # length = (1.2)**(np.linalg.norm(10*self.var + self.control_PT)) * np.linalg.norm(self.interestPT2 - self.interestPT1)
        length = bezier_curve_length(control_points)
        t = np.linspace(0, 1, int(length*self.frequency))
        curve_points = np.array([self.bezier_curve(ti, control_points) for ti in t]) 
        return curve_points

    def plot_curve(self, curve_points):
        plt.plot(curve_points[:, 0], curve_points[:, 1], 'b-')
        plt.plot(self.interestPT1[0], self.interestPT1[1], 'ro', label='Interest Point 1')
        plt.plot(self.interestPT2[0], self.interestPT2[1], 'ro', label='Interest Point 2')
        plt.plot(self.MID_PT[0], self.MID_PT[1], 'bo', label='Interest Point 3')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Random Bezier Curve')
        plt.legend()
        plt.grid(True)
        plt.show()

    def generate_curves(self, num_curves):
        for i in range(num_curves):
            curve_points = self.generate_random_bezier_curve()
            self.plot_curve(curve_points)

if __name__ == '__main__':
    interestPT1 = [3000, 1000]
    interestPT2 = [0, 1500]
    curvity = 0.075 # recommend 0.05 - 0.1
    generator = BezierCurveGenerator(interestPT1, interestPT2, curvity)
    generator.generate_curves(10)
