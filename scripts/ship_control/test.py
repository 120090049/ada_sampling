list1 = [[1, 2], [3, 4]]
list2 = [[3, 4], [1, 2]]

# 打开文件，以写入模式写入数据
with open('lists.txt', 'w') as file:
    # 写入 list1 的内容
    for sublist in list1:
        line = ' '.join(str(item) for item in sublist)
        file.write(line + '\n')
    
    file.write('\n')  # 在两个列表之间添加空行
    
    # 写入 list2 的内容
    for sublist in list2:
        line = ' '.join(str(item) for item in sublist)
        file.write(line + '\n')
