#!/usr/bin/python3

from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
import rospy
from mavros_msgs.msg import OverrideRCIn, State

from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
# from img_warp_and_stitch.stitch_stream import MATCHING
from geometry_msgs.msg import Pose, Quaternion
import math
import sys

def quaternion_from_yaw(yaw):
    quat = Quaternion()
    quat.x = 0.0
    quat.y = 0.0
    quat.z = math.sin(yaw/2)
    quat.w = math.cos(yaw/2)
    return quat

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


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
        self.vessel_link_name = self.vessel_name + "::link"
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
        self.rate = rospy.Rate(100)
        print("Initialized")


   
        # print(int(model_state.pose.position.x), model_state.pose.position.y, model_state.pose.position.z, end="\r")

    def pos_callback(self, msg):
        self.gazebo_link_states = msg
        index = self.gazebo_link_states.name.index(self.vessel_link_name)
        self.ship_x = self.gazebo_link_states.pose[index].position.x
        self.ship_y = self.gazebo_link_states.pose[index].position.y
        orientation = self.gazebo_link_states.pose[index].orientation
        _, _, self.orient = euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)

    def action_control(self):
        self.ship_x_send = self.ship_x
        self.ship_y_send = self.ship_y
        # self.orient_send = self.orient
    
        if self.key == 'e' or self.key == 'E':
            self.orient_send -= 1 * math.pi/180
        if self.key == 'q' or self.key == 'Q':
            self.orient_send += 1 * math.pi/180
            
        if self.key == 'w':
            self.ship_x_send += 5
       
        elif self.key == 's':
            self.ship_x_send -= 5
            
        elif self.key == 'a':
            self.ship_y_send += 5
            
        elif self.key == 'd':
            self.ship_y_send -= 5
        


    def main_loop(self):
        while (1):
            self.key = getKey()
            self.action_control()
            
            self.ship_x_send += 0
            self.model_state_send.pose.position.x = self.ship_x_send
            self.model_state_send.pose.position.y = self.ship_y_send
            self.model_state_send.pose.position.z = -2
            self.model_state_send.pose.orientation = quaternion_from_yaw(self.orient_send)
            self.set_ship_state(self.model_state_send)
            if (controller.key == '\x03'):
                break
            print("Current location: ", int(self.ship_x), int(self.ship_y), "Current yaw: ", self.orient_send, end="\r")

            self.rate.sleep()                                                                                                                                                                                                                                                                                                                         


if __name__ == "__main__":
    msg = """
---------------------------
command_cotrol
q and e : Orientation
w and s : forward and backward
a and d : left and right
---------------------------
CTRL-C to quit
"""
    print(msg)
    settings = termios.tcgetattr(sys.stdin)
      # - vessel_g_1::link
        # - vessel_g_0::link
        # - vessel_d_0::link
        # - vessel_d_1::link
        # - vessel_d_2::link
    vessel_id = "d_1" 
    controller = Ship_controller(vessel_id)
    controller.main_loop()
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)