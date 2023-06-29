#!/usr/bin/env python
#open isaac sim and use the camera to follow a target

import carb
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": False})
import math
import omni
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
import random
import time
from isaac_ros_messages.srv import IsaacPose
from isaac_ros_messages.srv import IsaacPoseRequest
from geometry_msgs.msg import Pose
from std_msgs.msg import String

pathtoworld = "omniverse://128.237.74.24/Projects/Champ/Map/Coastline_Map/Map_Robot.usd?checkpoint=25"
omni.usd.get_context().open_stage(pathtoworld, None)
simulation_app.update()
print("Loading stage...")
from omni.isaac.core.utils.stage import is_stage_loading

while is_stage_loading():
    simulation_app.update()

print("Loading Complete")

class ShipDataset(object):
    def __init__(self,ships_dict,ocean_dict,shore_dict,num_positive,num_negative):
        self.last_request_time = time.time()
        self.init = False
        self.ships_dict = ships_dict
        self.ocean_dict = ocean_dict
        self.shore_dict = shore_dict
        self.ship_view_count = {}
        self.ocean_view_count = {}
        self.shore_view_count = {}
        self.views_per_ship = int(num_positive/float(len(ships_dict)))
        self.num_negative = num_negative
        self.ship_detection_bounds = [(-600,600,10), (0,-700,10), (20,120,10)]
        self.views_per_ocean = int(num_negative/2.0) / int(len(ocean_dict))
        self.views_per_shore = int(num_negative/2.0) / int(len(shore_dict))

        print("generating " + str(self.views_per_ship) + " images per ship and " + str(self.views_per_ocean) +
              " images per ocean " + str(self.views_per_shore) + " per shore")
    
    def start_callback(self, data):
        self.init = True

    def gen_ship_dataset(self):
        if self.init:
            if len(ships_dict) > 0:
                #generate images of ships
                for ship in ships_dict:
                    ship_pose = ships_dict[ship]
                    if ship not in self.ship_view_count:
                        #gen initial ship image
                        self.ship_view_count[ship] = 1
                        self.gen_ship_image(ship_pose, ship)
                        return
                    elif self.ship_view_count[ship] >= self.views_per_ship:
                        #remove from dictionary if we finish generating images for that ship
                        del self.ships_dict[ship]
                        return
                    else:
                        self.ship_view_count[ship] += 1
                        self.gen_ship_image(ship_pose, ship)
                        return
            elif len(self.ocean_dict) > 0:
                for ocean in self.ocean_dict:
                    ocean_pose = self.ocean_dict[ocean]
                    if ocean not in self.ocean_view_count:
                        self.ocean_view_count[ocean] = 1
                        self.gen_ocean_image(ocean_pose, ocean)
                        return
                    elif self.ocean_view_count[ocean] >= self.views_per_ocean:
                        del ocean_dict[ocean]
                        print("finished data generation for " + ocean)
                        return
                    else:
                        self.ocean_view_count[ocean] += 1
                        self.gen_ocean_image(ocean_pose, ocean)
                        return
            elif len(self.shore_dict) > 0:
                for shore in self.shore_dict:
                    shore_pose = self.shore_dict[shore]
                    if shore not in self.shore_view_count:
                        self.shore_view_count[shore] = 1
                        self.gen_shore_image(shore_pose, shore)
                        return
                    elif self.shore_view_count[shore] >= self.views_per_shore:
                        del self.shore_dict[shore]
                        print("finished data generation for " + shore)
                        return
                    else:
                        self.shore_view_count[shore] += 1
                        self.gen_shore_image(shore_pose, shore)
                        return
            else:
                print("finished data generation")
                return
                    
    def gen_ship_image(self, ship_pose, ship_name):
        #pick random coordinate
        x_bounds, y_bounds, z_bounds = self.ship_detection_bounds
        x_coord = random.uniform(x_bounds[0], x_bounds[1])
        y_coord = random.uniform(y_bounds[0], y_bounds[1])
        z_coord = random.uniform(z_bounds[0], z_bounds[1])
        #generate offset so ship isn't always in center of frame
        x_offset = random.uniform(-20,20) * (z_coord / z_bounds[0])
        y_offset = random.uniform(-20,20) * (z_coord / z_bounds[0])
        #move camera to face object
        set_camera_view(eye=np.array([x_coord,y_coord,z_coord]), target=np.array([ship_pose[0]+x_offset, ship_pose[1]+y_offset, ship_pose[2]]),camera_prim_path="/World/UAVs/uav2/uav2_cam2")
        print("generating image " + str(self.ship_view_count[ship_name]) + " out of " + str(self.views_per_ship) + " for " + ship_name)
        return True
    
    def gen_ocean_image(self, ocean_pose_info, ocean_name):
        #read in coords and random bounds
        ocean_pose, x_bounds, y_bounds, z_bounds = ocean_pose_info
        x_coord = random.uniform(x_bounds[0], x_bounds[1])
        y_coord = random.uniform(y_bounds[0], y_bounds[1])
        z_coord = random.uniform(z_bounds[0], z_bounds[1])
        #move camera to face object
        set_camera_view(eye=np.array([x_coord, y_coord, z_coord]), target=np.array([ocean_pose[0], ocean_pose[1], ocean_pose[2]]), camera_prim_path="/World/UAVs/uav2/uav2_cam2")
        print("generating image " + str(self.ocean_view_count[ocean_name]) + " out of " + str(self.views_per_ocean) + " for " + ocean_name)
    
    def gen_shore_image(self, shore_pose_info, shore_name):
        #read in coords and random bounds
        shore_pose, x_bounds, y_bounds, z_bounds = shore_pose_info
        x_coord = random.uniform(x_bounds[0], x_bounds[1])
        y_coord = random.uniform(y_bounds[0], y_bounds[1])
        z_coord = random.uniform(z_bounds[0], z_bounds[1])
        #move camera to face object
        set_camera_view(eye=np.array([x_coord, y_coord, z_coord]), target=np.array([shore_pose[0], shore_pose[1], shore_pose[2]]), camera_prim_path="/World/UAVs/uav2/uav2_cam2")
        print("generating image " + str(self.shore_view_count[shore_name]) + " out of " + str(self.views_per_shore) + " for " + shore_name)

ships_dict = {}
# ships_dict['vessel_a'] = (-197.0, -267.0, 0.0)
ships_dict['vessel_b'] = (321, -180.0, 0.0)
ships_dict['vessel_c'] = (-330.0, -800, 0.0)
ships_dict['vessel_d'] = (-200, -400.0, 0.0)
ships_dict['vessel_e'] = (300, -700.0, 0.0)
ships_dict['vessel_f'] = (900, -386.0, 0.0)
ships_dict['vessel_g'] = (0.0, -300, 0.0)
last_request_time = time.time()

ocean_dict = {}
ocean_dict['ocean_1'] = [(-1800,-200, 0.0),(-600,-200,10),(-600,200,10),(20,120,10)]
ocean_dict['ocean_2'] = [(1600, -400, 0.0),(800,1400,10),(-800,200,10),(20,120,10)]
ocean_dict['ocean_3'] = [(0,-1500,0.0), (-800,1600,10), (-1400,-1000,10), (20,120,10)]

shore_dict = {}
shore_dict['shore_1'] = [(0,1000,0),(-1000,1000,10),(0,900,10),(20,120,10)]
shore_dict['shore_2'] = [(-1000,1000,0),(-1000,1000,10),(0,900,10),(20,120,10)]
shore_dict['shore_3'] = [(1000,1000,0),(-1000,1000,10),(0,900,10),(20,120,10)]

THRESHOLD = 2.0 #time in seconds between images generated
NUM_POSITIVE = 700 #number of positive images to collect
NUM_NEGATIVE = 300 #number of negative images to collect

dataset_manager = ShipDataset(ships_dict, ocean_dict, shore_dict, NUM_POSITIVE, NUM_NEGATIVE)
rospy.init_node('ship_dataset', anonymous = True)
rospy.Subscriber("/start_image_gen", String, dataset_manager.start_callback, queue_size=10)

while simulation_app.is_running():
    simulation_app.update()
    #check if enough time has passed to generate new ship image
    time_elapsed = time.time() - last_request_time
    if time_elapsed > THRESHOLD:
        dataset_manager.gen_ship_dataset()
        last_request_time = time.time()
    
simulation_app.close()