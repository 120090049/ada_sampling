import subprocess

def open_new_terminal(num):
    print("正在打开新的终端...")
    script_cmd = "python3 hello_world.py " + str(num) + ";"
    subprocess.Popen(["gnome-terminal", "--", "bash", "-c", script_cmd,  "; sleep infinity" ])

if __name__ == "__main__":
    while True:
        num = input("请输入一个数字：")
        if (int(num) == 1):
            open_new_terminal(num)
    