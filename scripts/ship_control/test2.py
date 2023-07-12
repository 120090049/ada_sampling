# 用于存储还原的列表
restored_list1 = []
restored_list2 = []

# 打开文件，以读取模式读取数据
with open('lists.txt', 'r') as file:
    lines = file.readlines()
    current_list = restored_list1  # 当前正在还原的列表
    
    for line in lines:
        line = line.strip()
        
        if line == '':  # 遇到空行切换到还原 list2
            current_list = restored_list2
        else:  # 解析列表的行数据
            sublist = [int(item) for item in line.split()]
            current_list.append(sublist)

print("还原的 list1:")
print(restored_list1)

print("还原的 list2:")
print(restored_list2)
