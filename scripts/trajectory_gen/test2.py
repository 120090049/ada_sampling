import matplotlib.pyplot as plt

# 创建一个图形窗口和坐标轴
fig, ax = plt.subplots()

# 设置坐标轴的范围
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)

# 绘制箭头
arrow1 = plt.arrow(0.1, 0.1, 0.2, 0.2, width=0.02, color='red')
arrow2 = plt.arrow(0.1, 0.1, 0.2, -0.2, width=0.02, color='blue')

# 添加文字标签
ax.text(0.1, 0.1, "Arrow 1", ha='right', va='bottom', color='red')
ax.text(0.1, 0.1, "Arrow 2", ha='right', va='top', color='blue')

# 显示图形
plt.show()
