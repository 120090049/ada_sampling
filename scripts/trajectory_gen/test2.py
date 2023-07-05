import matplotlib.pyplot as plt
import numpy as np

# 创建一个空的图形窗口
fig, ax = plt.subplots()

# 设置坐标轴的范围
ax.set_xlim(-1.1, 1.1)
ax.set_ylim(-1.1, 1.1)

# 设置初始点的坐标
x = 1.0
y = 0.0

# 设置时间步长和总时间
dt = 0.01
total_time = 2.0

# 循环绘制圆
for t in np.arange(0, total_time, dt):
    # 根据时间计算圆上的点的坐标
    theta = 2 * np.pi * t / total_time
    new_x = np.cos(theta)
    new_y = np.sin(theta)
    
    # 绘制从前一个点到当前点的线段
    ax.plot([x, new_x], [y, new_y], 'b')
    
    # 更新当前点的坐标
    x = new_x
    y = new_y
    
    # 刷新图形窗口
    plt.pause(dt)

# 显示绘制结果
plt.show()
