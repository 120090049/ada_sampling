#!/usr/bin/env python
import carb
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": False})
import math
import omni
from std_msgs.msg import Float64
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import viewports, stage, extensions, prims, rotations, nucleus
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core import World

enable_extension("omni.isaac.ros_bridge")
simulation_app.update()

import rosgraph

if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()
    
import numpy as np
import rospy
from std_msgs.msg import Empty
import time
from omni.isaac.core.robots import Robot
import omni.isaac.core
import os
from isaac_ros_messages.srv import IsaacPose
from isaac_ros_messages.srv import IsaacPoseRequest
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from gazebo_msgs.msg import ModelStates


# pathtoworld = "omniverse://128.237.74.24/Projects/Champ/Map/Coastline_Map/Map_Robot.usd"
pathtoworld = "file:///home/clp/map_model_res/CHAMP/Map_Robot.usd"

omni.usd.get_context().open_stage(pathtoworld, None)
simulation_app.update()
print("Loading stage...")
from omni.isaac.core.utils.stage import is_stage_loading

while is_stage_loading():
    simulation_app.update()

print("Loading Complete")

def getnames(data): 
    #Defining the affiliations in Gazebo
    global flag
    # global hexaindex
    global vtol1index
    global vtol2index
    global vtol3index
    flag = 1
    for j in range(len(data.name)):
        if data.name[j]=="uav0":
            vtol1index=j
        if data.name[j]=="uav1":
            vtol2index=j
        if data.name[j]=="uav2":
            vtol3index = j


def callback(data):
    #getting positions from Gazebo
    global flag
    if flag==0: 
        getnames(data)
    # vc_pose.position = data.pose[4].position
    # vc_pose.orientation = data.pose[4].orientation
    # vg_pose.position = data.pose[5].position
    # vg_pose.orientation = data.pose[5].orientation
    if vtol1index != 999:
        vtol1_pose.position = data.pose[vtol1index].position        #drone VTOL1  
        vtol1_pose.orientation = data.pose[vtol1index].orientation
    if vtol2index != 999:
        vtol2_pose.position = data.pose[vtol2index].position        #drone VTOL2  
        vtol2_pose.orientation = data.pose[vtol2index].orientation
    if vtol3index != 999:
        vtol3_pose.position = data.pose[vtol3index].position        #drone VTOL3  
        vtol3_pose.orientation = data.pose[vtol3index].orientation

def heading_cam1(data):
    #Implement an heading for the gimbal camera of fixed wing 1 between -180 and 180 degree
    global headingc1
    headingc1 = data.data
    if headingc1<0: 
        headingc1 = 360+headingc1

def heading_cam2(data):
    #Implement an heading for the gimbal camera of fixed wing 2 between -180 and 180 degree
    global headingc2
    headingc2 = data.data
    if headingc2<0:
        headingc2=360+headingc2
    
def get_quaternion_from_euler(roll, pitch, yaw):
     qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
     qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
     qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
     qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
     return [qw, qx, qy, qz]

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
     
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
     
    return [roll_x, pitch_y, yaw_z] 

def move():

    global camoffset1
    global camoffset2
    global headingc1
    global headingc2
    #Streaming positions from gazebo to Isaac Sim
    if vtol1index != 999:
        vtol1_prim.set_world_pose(position=np.array([vtol1_pose.position.x, vtol1_pose.position.y, vtol1_pose.position.z+0.75]),orientation=np.array([vtol1_pose.orientation.w, vtol1_pose.orientation.x, vtol1_pose.orientation.y, vtol1_pose.orientation.z]))
    if vtol2index != 999:
        vtol2_prim.set_world_pose(position=np.array([vtol2_pose.position.x, vtol2_pose.position.y, vtol2_pose.position.z+0.75]),orientation=np.array([vtol2_pose.orientation.w, vtol2_pose.orientation.x, vtol2_pose.orientation.y, vtol2_pose.orientation.z]))
    if vtol2index != 999:
        vtol3_prim.set_world_pose(position=np.array([vtol3_pose.position.x, vtol3_pose.position.y, vtol3_pose.position.z+0.75]),orientation=np.array([vtol3_pose.orientation.w, vtol3_pose.orientation.x, vtol3_pose.orientation.y, vtol3_pose.orientation.z]))
    vc_prim.set_world_pose(position=np.array([vc_pose.position.x, vc_pose.position.y, vc_pose.position.z]),orientation=get_quaternion_from_euler(vc_pose.orientation.x, vc_pose.orientation.y, vc_pose.orientation.z))
    vg_prim.set_world_pose(position=np.array([vg_pose.position.x, vg_pose.position.y, vg_pose.position.z]),orientation=get_quaternion_from_euler(vg_pose.orientation.x, vg_pose.orientation.y, vg_pose.orientation.z))
    
    #gimbal for down cameras
    #Fixed Wing 1
    cam1downarray = euler_from_quaternion(vtol1_pose.orientation.x, vtol1_pose.orientation.y, vtol1_pose.orientation.z, vtol1_pose.orientation.w)
    vcamera1down_prim.set_local_pose(translation=np.array([0, 0, 0]),orientation=get_quaternion_from_euler(1.0472, -cam1downarray[0], -1.57))
    #Fixed Wing 2
    cam2downarray = euler_from_quaternion(vtol2_pose.orientation.x, vtol2_pose.orientation.y, vtol2_pose.orientation.z, vtol2_pose.orientation.w)
    vcamera2down_prim.set_local_pose(translation=np.array([0, 0, 0]),orientation=get_quaternion_from_euler(-cam2downarray[0], -cam2downarray[1], 0))

    #gimbal for front cameras
    
  
    
def listener():
    #Initializing Ros nodes and topics
    rospy.init_node('listener', anonymous = True)
    
    rospy.Subscriber("/uav1/headingcam", Float64, heading_cam1, queue_size = 10)
    rospy.Subscriber("/uav2/headingcam", Float64, heading_cam2, queue_size = 10)
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback, queue_size = 10)

#defining global variables
flag =0
vtol1index=999
vtol2index=999
vtol3index=999


headingc1=0
headingc2=0
teleport_msg = IsaacPoseRequest()

vtol1_pose = Pose()
vtol2_pose = Pose()
vtol3_pose = Pose()


vc_pose = Pose()
vg_pose = Pose()

vcamera1down_pose = Pose()
vcamera2down_pose = Pose()
vtol1_prim = Robot("/World/UAVs/uav1","vtol1")
vtol2_prim = Robot("/World/UAVs/uav2","vtol2")
vtol3_prim = Robot("/World/UAVs/uav3","vtol3")

vc_prim = Robot("/World/Vessels/vessel_c","vc")
vg_prim = Robot("/World/Vessels/vessel_g","vg")
vcamera1down_prim = Robot("/World/UAVs/uav1/uav1_cam2","vcam1down")
vcamera2down_prim = Robot("/World/UAVs/uav2/uav2_cam2","vcam2down")

listener()

while simulation_app.is_running():
    simulation_app.update()
    move()
    
simulation_app.close()
