#!/usr/bin/python3

import sys

if __name__ == "__main__":
    # 从命令行参数获取数字
    if len(sys.argv) > 1:
        number = sys.argv[1]
        print("输入的数字是：" + number)
    else:
        print("请提供一个数字作为命令行参数")
