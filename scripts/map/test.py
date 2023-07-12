import numpy as np

# 创建两个矩阵
matrix1 = np.array([[1, 2], [3, 4]])
matrix2 = np.array([[5, 6], [7, 8]])

# 进行矩阵对应位置的元素相乘
result = np.multiply(matrix1, matrix2)

print("矩阵对应位置的元素相乘的结果：")
print(result)
print(matrix1)
