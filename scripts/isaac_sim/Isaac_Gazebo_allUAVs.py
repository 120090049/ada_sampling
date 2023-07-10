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
from omni.isaac.core.utils.viewports import set_camera_view
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

vessel_path = "/World/Vessels/"
vtol_path = "/World/UAVs/"

#paths for max number of vesselD
vessel_d_path = ["vessel_d", "vessel_d_01", "vessel_d_02", "vessel_d_03"]

#paths for max number of vesselG
vessel_g_path = ["vessel_g", "vessel_g_01", "vessel_g_02", "vessel_g_03"]

#paths for max number of vtols
standard_vtol_path = ["uav1","uav2", "uav3"]

#paths for forward facing cameras
forward_cam_path = ["uav1/uav1_cam2", "uav2/uav2_cam2"]

#paths for shore facing cameras
# shore_cam_path = ["/uav1_cam1", "/uav2_cam1"]

class IsaacGazeboInterface(object):
    def __init__(self):
        self.model_states_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)
        self.cam1_pose_sub = rospy.Subscriber("/uav1/movecam", Quaternion, self.callback_cam1, queue_size = 10)
        self.cam2_pose_sub = rospy.Subscriber("/uav2/movecam", Quaternion, self.callback_cam2, queue_size = 10)
        self.cam1_heading_sub = rospy.Subscriber("/uav1/headingcam", Float64, self.heading_cam1, queue_size = 10)
        self.cam2_heading_sub = rospy.Subscriber("/uav2headingcam", Float64,self.heading_cam2, queue_size = 10)
        #internal variables for model and camera states
        self.model_states = ModelStates()
        self.cam1_offset = np.zeros(3,dtype=float)
        self.cam2_offset = np.zeros(3,dtype=float)
        self.cam1_heading = 0.0
        self.cam2_heading = 0.0
        #initialize Robot objects for Isaac
        self.vessel_d_robots = [Robot(vessel_path + vessel_d_path[idx]) for idx in range(len(vessel_d_path))]
        self.vessel_g_robots = [Robot(vessel_path + vessel_g_path[idx]) for idx in range(len(vessel_g_path))]
        self.vtol_robots = [Robot(vtol_path + standard_vtol_path[idx]) for idx in range(len(standard_vtol_path))]
        self.front_camera_robots = [Robot(vtol_path + forward_cam_path[idx]) for idx in range(len(forward_cam_path))]
        # self.shore_camera_robots = [Robot(shore_cam_path[idx]) for idx in range(len(shore_cam_path))]
    
    def model_states_callback(self, data):
        self.model_states = data
    
    #Implement an offset for the camera of fixed wing 1 here to emulate a gimbal
    def callback_cam1(self,data):
        self.cam1_offset = self.euler_from_quaternion(data.x, data.y, data.z, data.w)

    #Implement an offset for the camera of fixed wing 2 here to emulate a gimbal
    def callback_cam2(self,data):
        self.cam2_offset = self.euler_from_quaternion(data.x, data.y, data.z, data.w)

    #Implement an heading for the gimbal camera of fixed wing 1 between -180 and 180 degree
    def heading_cam1(self,data):
        self.cam1_heading = data.data
        if self.cam1_heading < 0:
            self.cam1_heading += 360

    #Implement an heading for the gimbal camera of fixed wing 2 between -180 and 180 degree
    def heading_cam2(self,data):
        self.cam2_heading = data.data
        if self.cam2_heading < 0:
            self.cam2_heading += 360
    
    def euler_from_quaternion(self, x, y, z, w):
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
     
        return np.array([roll_x, pitch_y, yaw_z])

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return np.array([qw, qx, qy, qz])
    
    # move ships, cameras, and vtols based on gazebo
    def move_models(self):
        #update model positions
        for idx in range(len(self.model_states.name)):
            #extract model name, pose, and index
            model_name = self.model_states.name[idx]
            model_pose = self.model_states.pose[idx]
            model_idx = model_name[-1]
            #determine model pose and orientation
            position_vector = [model_pose.position.x, model_pose.position.y, model_pose.position.z]
            orientation_vector = [model_pose.orientation.w, model_pose.orientation.x, model_pose.orientation.y, model_pose.orientation.z]
            #determine which robot model it is
            if 'vessel_d' in model_name:
                self.vessel_d_robots[int(model_idx)].set_world_pose(position=position_vector, orientation=orientation_vector)
            elif 'vessel_g' in model_name:
                #assign to first vessel_g
                # position_vector[2] = -3.5
                # if 'pole' in model_name:
                #     self.vessel_g_robots[0].set_world_pose(position=position_vector, orientation=orientation_vector)
                # else:
                    # self.vessel_g_robots[int(model_idx)].set_world_pose(position=position_vector, orientation=orientation_vector)
                self.vessel_g_robots[int(model_idx)].set_world_pose(position=position_vector, orientation=orientation_vector)
            elif 'hex' in model_name:
                self.vtol_robots[2].set_world_pose(position=position_vector, orientation=orientation_vector)
            elif 'vtol' in model_name:
                self.vtol_robots[int(model_idx)-1].set_world_pose(position=position_vector, orientation=orientation_vector)
                #update downward facing camera
                forward_cam_array = self.euler_from_quaternion(model_pose.orientation.x, model_pose.orientation.y, model_pose.orientation.z, model_pose.orientation.w)
                self.front_camera_robots[int(model_idx)-1].set_local_pose(translation=np.array([0, 0, 0]),orientation=self.get_quaternion_from_euler(1.0472, -forward_cam_array[0], -1.57))
                #update shoreline facing camera
                shore_cam_array = [0.5*math.pi, 0, 0]
                cam_heading = self.cam1_heading if (model_idx == 0) else self.cam2_heading
                cam_offset = self.cam1_offset if (model_idx == 0) else self.cam2_offset
                if cam_heading >= 0.0 and cam_heading <= 90.0:
                    shore_cam_array[1] = -(cam_heading/180.0)*np.pi
                elif cam_heading > 90.0 and cam_heading < 270.0:
                    shore_cam_array[0] = -shore_cam_array[0]
                    shore_cam_array[1] = ((-180.0 + cam_heading)/180.0)*np.pi
                    shore_cam_array[2] = np.pi
                else:
                    shore_cam_array[1] = ((360.0 - cam_heading)/180.0)*np.pi
                shore_cam_array = np.add(shore_cam_array, cam_offset)
                quat_shore_cam_array = self.get_quaternion_from_euler(shore_cam_array[0], shore_cam_array[1], shore_cam_array[2])
                quat_shore_cam_array[3] = -quat_shore_cam_array[3]
                position_vector[2] = position_vector[2] - 0.5
                # self.shore_camera_robots[int(model_idx)-1].set_world_pose(position=position_vector, orientation=quat_shore_cam_array)

                #have camera follow VTOL1
                # if int(model_idx) == 0:
                #     vtol_overhead_pose = [model_pose.position.x-2, model_pose.position.y, model_pose.position.z+1]
                #     vtol_pose = [model_pose.position.x, model_pose.position.y, model_pose.position.z]

                #     set_camera_view(eye=vtol_overhead_pose, target=vtol_pose, camera_prim_path="/sl_detection")

if __name__ == '__main__':
    rospy.init_node('isaac_gazebo_interface', anonymous = True)
    interface = IsaacGazeboInterface()

    while simulation_app.is_running():
        simulation_app.update()
        interface.move_models()

    simulation_app.close()
