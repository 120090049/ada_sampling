import subprocess

def open_new_terminal():
    print("正在打开新的终端...")
    subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "python3 hello_world.py; sleep " + str(num)])

if __name__ == "__main__":
    while True:
        num = input("请输入一个数字：")
        if (int(num) == 1):
            open_new_terminal()
    