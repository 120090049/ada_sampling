#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
import rospy
from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
# from img_warp_and_stitch.stitch_stream import MATCHING

import sys
import rospy


def RCInOverride(channel0, channel1, channel2, channel3):
    target_RC_yaw = OverrideRCIn()
    target_RC_yaw.channels[0] = channel0
    target_RC_yaw.channels[1] = channel1
    target_RC_yaw.channels[2] = channel2
    target_RC_yaw.channels[3] = channel3
    target_RC_yaw.channels[4] = 1100
    return target_RC_yaw


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class Ship_controller:
    def __init__(self, name):
        self.vessel_name = 'vessel_' + name
        self.vessel_link_name = self.vessel_name + "::base_link"
        rospy.init_node(self.vessel_name + '_controller', anonymous=True)
        
        # receive
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.pos_callback)
        self.ship_x = 0
        self.ship_y = 0
        self.orient = 0
        
        # send
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_ship_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.model_state_send = ModelState()
        self.model_state_send.model_name = self.vessel_name

        self.ship_x_send = 0
        self.ship_y_send = 0
        self.orient_send = 0
        
        self.rate = rospy.Rate(20)
        print("Initialized")


   
        # print(int(model_state.pose.position.x), model_state.pose.position.y, model_state.pose.position.z, end="\r")

    def pos_callback(self, msg):
        self.gazebo_link_states = msg
        index = self.gazebo_link_states.name.index(self.vessel_link_name)
        self.ship_x = self.gazebo_link_states.pose[index].position.x
        self.ship_y = self.gazebo_link_states.pose[index].position.y


    def action_control(self):
        self.ship_x_send = self.ship_x
        self.ship_y_send = self.ship_y
        self.orient_send = self.orient
    
        if self.key == 'e' or self.key == 'E':
            self.orient_send += 1
        if self.key == 'q' or self.key == 'Q':
            self.orient_send -= 1
            
        if self.key == 'w':
            self.ship_x_send += 10
       
        elif self.key == 's':
            self.ship_x_send -= 10
            
        elif self.key == 'a':
            self.ship_y_send -= 10
            
        elif self.key == 'd':
            self.ship_y_send += 10
        
        


    def main_loop(self):
        while (1):
            self.key = getKey()
            self.action_control()
            
            self.model_state_send.pose.position.x = self.ship_x_send
            self.model_state_send.pose.position.y = self.ship_y_send
            
            self.set_ship_state(self.model_state_send)
            if (controller.key == '\x03'):
                break


if __name__ == "__main__":

    vessel_id = "d_1" 
    controller = Ship_controller(vessel_id)
    controller.main_loop()