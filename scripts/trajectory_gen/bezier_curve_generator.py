import numpy as np
import matplotlib.pyplot as plt
from scipy.special import comb

class BezierCurveGenerator:
    def __init__(self, interestPT1, interestPT2, curvity, frequency = 10):
        self.interestPT1 = np.array(interestPT1)
        self.interestPT2 = np.array(interestPT2)
        
        self.curvity = curvity
        self.MID_PT = None
        self.frequency = frequency
        

    def bezier_curve(self, t, control_points):
        n = len(control_points) - 1
        curve_point = np.zeros(2)
        for i, point in enumerate(control_points):
            curve_point += comb(n, i) * (1 - t)**(n - i) * t**i * point
        return curve_point

    def generate_random_point(self):
        midpoint = (self.interestPT1 + self.interestPT2) / 2.0
        var = np.random.normal(loc=0, scale=self.curvity, size=2)
        displacement = var * np.linalg.norm(self.interestPT2 - self.interestPT1)
        self.MID_PT = midpoint + displacement

    def generate_random_bezier_curve(self):
        self.generate_random_point()
        control_points = [self.interestPT1, self.MID_PT, self.interestPT2]

        length = np.linalg.norm(self.interestPT2 - self.interestPT1)
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
