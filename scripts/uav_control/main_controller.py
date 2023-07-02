#!/usr/bin/python3

import subprocess
import os
import rospy
from gazebo_msgs.msg import ModelStates
import time


# 示例使用
def open_new_terminal(num):
    print("正在打开新的终端...")
    script_cmd = "python3 hello_world.py " + str(num) + ";"
    # subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "echo 'hello'; sleep infinity"])

    subprocess.Popen(["gnome-terminal", "--", "bash", "-c", f"rosrun ada_sampling hello_world.py {num}; sleep 5"])

def getnames(data): 
    #Defining the affiliations in Gazebo

    flag = 1
    for j in range(len(data.name)):
        print( data.name[j])
    subscriber.unregister()
    

if __name__ == "__main__":
    rospy.init_node('listener', anonymous = True)
    subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, getnames, queue_size = 10)
    print("We have following drones:")
    time.sleep(0.5)

    while True:
        num = input("Which UAV you want to control (enter number) or press q to quit: ")
        if (num == 'q'):
            break
        if (int(num) == 1 or int(num) == 2):
            open_new_terminal(num)
    