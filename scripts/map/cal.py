import numpy as np

point1 = np.array([1944.0632626964368, 943.8061210703007])
point2 = np.array([1957.1620129965922, 947.1640078525822])

distance = np.linalg.norm(point2 - point1)

print("两点之间的距离：", distance)
