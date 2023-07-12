import numpy as np
import plotly.graph_objects as go
import time
# 创建数据
x = np.linspace(-5, 5, 100)
y = np.linspace(-5, 5, 100)
X, Y = np.meshgrid(x, y)

# 创建初始曲面
Z = np.sin(np.sqrt(X**2 + Y**2))
fig = go.Figure(data=[go.Surface(x=X, y=Y, z=Z)])

# 设置图形布局
fig.update_layout(
    scene=dict(
        xaxis=dict(range=[-5, 5]),
        yaxis=dict(range=[-5, 5]),
        zaxis=dict(range=[-1, 1]),
    ),
    title='动态曲面示例',
)

# 显示图形
fig.show()

# 循环更新曲面
i = 0
while i < 100:
    i += 1

    # 更新数据
    Z = np.sin(np.sqrt(X**2 + Y**2 + i))

    # 清除现有曲面
    fig.data = []

    # 添加新的曲面
    fig.add_trace(go.Surface(x=X, y=Y, z=Z))

    # 更新图形
    fig.update_traces()

    # 更新图形布局
    fig.update_layout(
        scene=dict(
            xaxis=dict(range=[-5, 5]),
            yaxis=dict(range=[-5, 5]),
            zaxis=dict(range=[-1, 1]),
        ),
        title='动态曲面示例 - 帧 {}'.format(i),
    )

    # 显示图形
    fig.show(renderer='iframe')
    time.sleep(0.5)
