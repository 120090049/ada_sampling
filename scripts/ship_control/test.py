import queue

# 创建一个队列对象
my_queue = queue.Queue()

# 定义一个数组

# 将数组作为一个整体入队列
my_queue.put([2,3])
my_queue.put([3,3])
print(list(my_queue.queue))
# 出队列
[x, y] = my_queue.get()
print(list(my_queue.queue))
print(x, y)  # 输出：[1, 2, 3, 4, 5]

item = my_queue.get()
print(list(my_queue.queue))
print(item)  # 输出：[1, 2, 3, 4, 5]