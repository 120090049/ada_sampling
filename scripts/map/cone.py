import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 创建圆锥的数据
height = 1  # 圆锥的高度
radius = 3  # 圆锥底面半径
angle_resolution = 0.1  # 角度分辨率

theta = np.arange(0, 2*np.pi + angle_resolution, angle_resolution)  # 角度范围
h = np.linspace(0, height, 100)  # 高度范围

Theta, H = np.meshgrid(theta, h)  # 构建网格坐标

X = H / height * radius * np.cos(Theta)  # 计算 X 坐标
Y = H / height * radius * np.sin(Theta)  # 计算 Y 坐标
Z = H  # 计算 Z 坐标

# 绘制圆锥斜面
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(X, Y, Z, cmap='viridis')

# 设置坐标轴标签
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# 设置图形标题
ax.set_title('The kernel surface')

# 显示图形
plt.show()
