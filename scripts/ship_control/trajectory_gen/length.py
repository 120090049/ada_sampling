import numpy as np
import matplotlib.pyplot as plt

def bezier_curve_length(control_points, num_samples=10):
    t = np.linspace(0, 1, num_samples)

    dx_dt = 2 * (1 - t) * (control_points[1][0] - control_points[0][0]) + 2 * t * (control_points[2][0] - control_points[1][0])
    dy_dt = 2 * (1 - t) * (control_points[1][1] - control_points[0][1]) + 2 * t * (control_points[2][1] - control_points[1][1])
    integrand = np.sqrt(dx_dt**2 + dy_dt**2)
    length = np.sum(integrand) / num_samples
    return length

# 控制点坐标
control_points = [[0, 0], [2, 2], [4, 3]]

# 计算贝塞尔曲线长度
curve_length = bezier_curve_length(control_points)
print("曲线长度:", curve_length)

# 生成贝塞尔曲线
t = np.linspace(0, 1, 100)
x = (1 - t)**2 * control_points[0][0] + 2 * (1 - t) * t * control_points[1][0] + t**2 * control_points[2][0]
y = (1 - t)**2 * control_points[0][1] + 2 * (1 - t) * t * control_points[1][1] + t**2 * control_points[2][1]

# 绘制贝塞尔曲线
plt.plot(x, y, 'b-', label='Bézier Curve')
plt.scatter([point[0] for point in control_points], [point[1] for point in control_points], color='r', label='Control Points')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Bézier Curve Visualization')
plt.legend()
plt.grid(True)
plt.show()
