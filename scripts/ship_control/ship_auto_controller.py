#!/usr/bin/python3
import rospy
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Quaternion

import numpy as np
import matplotlib.pyplot as plt
import math

import queue
from trajectory_gen.trajectory_gen import ShipTrajectoryGenerator
import sys

OPEN_ROS = False
WRITE = False

def quaternion_from_yaw(yaw):
    quat = Quaternion()
    quat.x = 0.0
    quat.y = 0.0
    quat.z = math.sin(yaw/2)
    quat.w = math.cos(yaw/2)
    return quat


class Ship_controller:
    def __init__(self, offset):
        rospy.init_node('vessel_controller', anonymous=True)
 
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_ship_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.model_state_send = ModelState()
        self.offset = offset 
        

    def send(self, name, x, y, orient): # name = "d_0"
        self.model_state_send.model_name = 'vessel_' + name 
        self.model_state_send.pose.position.x = x + self.offset[0]
        self.model_state_send.pose.position.y = y + self.offset[1]
        if 'g' in name:
            self.model_state_send.pose.position.z = -2
        else:
            self.model_state_send.pose.position.z = 0
        self.model_state_send.pose.orientation = quaternion_from_yaw(orient)
        self.set_ship_state(self.model_state_send)

        
if __name__ == '__main__':
    print("with argument = 1: OPEN_ROS to send trajectory; arg = 2: write data; no argument: just show the trajectory")
    if len(sys.argv) > 1:
        number = int(sys.argv[1])
        if number == 1:
            OPEN_ROS = True
        elif number == 2:
            WRITE = True
   
        
    harbourT1 = np.array([400, 400])
    interestPT1 = np.array([800, 1500])
    interestPT2 = np.array([3700, 1000])
    
    harbourF1 = np.array([1500, 200])
    harbourF2 = np.array([3500, 200])
    interestPF1 = np.array([2500, 1800])
    
    tourboat_visit_pts = [harbourT1, interestPT1, interestPT2]
    fishboat_visit_pts = [harbourF1, harbourF2, interestPF1]
    
    wind=[0,2]
    wave=[2,0]
    frequency = 0.5
    randomness= 0.02
    
    shipT_num = 4
    shipF_num = 4
    shipTs_tra = []
    shipFs_tra = []
    
    
    shipTs_pos = queue.Queue()
    shipFs_pos = queue.Queue()
    
    if OPEN_ROS:
        command_sender = Ship_controller([-2400, -2000])
    
    # for tourist boats
    for i in range(shipT_num):
        instance = ShipTrajectoryGenerator(tourboat_visit_pts, wind, wave, randomness, frequency)
        shipTs_tra.append(instance)
        x, y, _ = instance.move()
        shipTs_pos.put([x, y])
        
    
    for i in range(shipF_num):
        instance = ShipTrajectoryGenerator(fishboat_visit_pts, wind, wave, randomness, frequency)
        shipFs_tra.append(instance)
        x, y, _ = instance.move()
        shipFs_pos.put([x, y])
    
# plot configuration
    fig, ax = plt.subplots()

    size_x = 4000
    size_y = 2000
    ax.set_xlim(0, size_x)
    ax.set_ylim(0, size_y)
    
    # wind and wave direction
    ax.arrow(200, 200, 150*wind[0], 150*wind[1], width=1, color='red')
    ax.text(200, 200, "Wind", ha='right', va='bottom', color='red')
    ax.arrow(200, 200, 150*wave[0], 150*wave[1], width=1, color='blue')
    ax.text(200, 200, "Wave", ha='right', va='top', color='blue')
    
    # harbour and interest points
    ax.plot(interestPT1[0], interestPT1[1], 'ro', label='interestPT1')
    ax.plot(interestPT2[0], interestPT2[1], 'ro', label='interestPT2')
    ax.plot(harbourT1[0], harbourT1[1], 'ro', label='harbourT1')
    
    ax.plot(harbourF1[0], harbourF1[1], 'ko', label='harbourF1')
    ax.plot(harbourF2[0], harbourF2[1], 'ko', label='harbourF2')
    ax.plot(interestPF1[0], interestPF1[1], 'ko', label='interestPF1')
    
# start to move the ships

    # 设置时间步长和总时间
    dt = 0.01
    # Map_output = Map(width=size_x, height=size_y, grid_cell_size=10, fov_radius=100)
    
    
    if WRITE:
        with open('ship_trajectory.txt', 'w') as file:
            
            
            while True:
                targets_cordinates = []
                for i in range(shipT_num): # d is pleausure boat
                    x, y, orient = shipTs_tra[i].move()
                    targets_cordinates.append([x, y])
                    if OPEN_ROS:
                        command_sender.send("d_"+str(i), x, y, orient)
                    
                    [pre_x, pre_y] = shipTs_pos.get()
                    shipTs_pos.put([x, y])
                    ax.plot([pre_x, x], [pre_y, y], 'b')
                    
                for i in range(shipF_num): # g is fishboat
                    x, y, orient = shipFs_tra[i].move()
                    targets_cordinates.append([x, y])
                    if OPEN_ROS:
                        command_sender.send("g_"+str(i), x, y, orient)
                    
                    [pre_x, pre_y] = shipFs_pos.get()
                    shipFs_pos.put([x, y])
                    ax.plot([pre_x, x], [pre_y, y], 'g')
                # map = Map_output.update_map(targets_cordinates)
                plt.pause(dt)
                for sublist in targets_cordinates:
                    line = ' '.join(str(item) for item in sublist)
                    file.write(line + '\n')
                file.write('\n')

            plt.show()
            
    else:
        while True:
            targets_cordinates = []
            for i in range(shipT_num): # d is pleausure boat
                x, y, orient = shipTs_tra[i].move()
                targets_cordinates.append([x, y])
                if OPEN_ROS:
                    command_sender.send("d_"+str(i), x, y, orient)
                
                [pre_x, pre_y] = shipTs_pos.get()
                shipTs_pos.put([x, y])
                ax.plot([pre_x, x], [pre_y, y], 'b')
                
            for i in range(shipF_num): # g is fishboat
                x, y, orient = shipFs_tra[i].move()
                targets_cordinates.append([x, y])
                if OPEN_ROS:
                    command_sender.send("g_"+str(i), x, y, orient)
                
                [pre_x, pre_y] = shipFs_pos.get()
                shipFs_pos.put([x, y])
                ax.plot([pre_x, x], [pre_y, y], 'g')
            # map = Map_output.update_map(targets_cordinates)
            plt.pause(dt)

        plt.show()