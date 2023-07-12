import numpy as np
import matplotlib.pyplot as plt

# 设置起始点 A 和目标点 B 的坐标
start_point = np.array([0, 0])
target_point = np.array([10, 2])

# 定义风浪参数
wind_speed = 10  # 风速
wave_amplitude = 1  # 浪高
wave_period = 5  # 浪周期

# 定义船只的动力学特性
mass = 1000  # 船只的质量
inertia = np.array([[1000, 0], [0, 200]])  # 船只的惯性矩阵

# 定义模拟参数
delta_t = 0.01  # 时间步长
total_time = 10  # 总模拟时间

# 计算模拟步数
num_steps = int(total_time / delta_t)

# 初始化船只状态
ship_position = [start_point]
ship_velocity = np.array([0.0, 0.0])  # 将初始速度设为浮点数类型

# 模拟船只的运动轨迹
while True:
    # 计算船只的加速度
    delta_x = np.random.normal(loc=0, scale=wind_speed)  # 风速的随机扰动
    delta_y = np.random.normal(loc=0, scale=wave_amplitude)  # 浪高的随机扰动

    acceleration = np.dot(np.linalg.inv(inertia), np.array([delta_x, delta_y])).astype(float)  # 将加速度转换为浮点数类型

    # 更新船只的速度和位置
    ship_velocity += acceleration * delta_t
    ship_displacement = ship_velocity * delta_t

    new_position = ship_position[-1] + ship_displacement

    # 如果船只接近终点，调整航向朝向目标点
    # if np.linalg.norm(new_position - target_point) < 1:
    direction = target_point - new_position
    direction /= np.linalg.norm(direction)
    ship_velocity = direction * np.linalg.norm(ship_velocity)

    ship_position.append(new_position)

    # 如果船只已经到达终点，提前结束模拟
    if np.allclose(new_position, target_point):
        break

# 转换船只轨迹为 NumPy 数组
ship_position = np.array(ship_position)

# 绘制船只的轨迹
plt.plot(ship_position[:, 0], ship_position[:, 1], '-o')
plt.scatter(start_point[0], start_point[1], color='green', label='Start')
plt.scatter(target_point[0], target_point[1], color='red', label='Target')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Ship Trajectory with Wind and Waves')
plt.legend()
plt.grid(True)
plt.show()
