import numpy as np
import plotly.graph_objects as go
import time


class Map:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.map = np.ones((height, width))
        self.fig = go.Figure(data=go.Surface(z=self.map))
        self.fig.update_layout(scene=dict(
            xaxis_title='X',
            yaxis_title='Y',
            zaxis_title='Z'
        ))

    def update_map(self, coordinates):
        self.map.fill(1)  # 重置地图为全1
        for coord in coordinates:
            x, y = coord
            self.map[y, x] = 0  # 根据坐标将对应位置设为0

    def print_map(self):
        self.fig.update_traces(z=self.map)
        self.fig.show()


my_map = Map(5, 5)  # 指定地图的宽为 5，高为 5
my_map.update_map([(1, 1), (2, 2), (3, 3)])
my_map.print_map()
time.sleep(0.5)

my_map.update_map([(4, 4), (2, 3), (0, 0)])
my_map.print_map()
time.sleep(0.5)

my_map.update_map([(1, 1), (2, 2), (3, 3)])
my_map.print_map()
time.sleep(0.5)

my_map.update_map([(4, 4), (2, 3), (0, 0)])
my_map.print_map()
