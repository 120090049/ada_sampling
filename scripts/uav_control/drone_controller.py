#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
import rospy
from mavros_msgs.msg import OverrideRCIn, State, PositionTarget
from mavros_msgs.srv import CommandBool,SetMode
from geometry_msgs.msg import TwistStamped
from gazebo_msgs.msg import LinkStates
# from img_warp_and_stitch.stitch_stream import MATCHING

import sys
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

msg = """
---------------------------
%%%%%%%%%%%%%%%%%%%%%%%
command_cotrol
%%%%%%%%%%%%%%%%%%%%%%%
0 : ARM
1 : TAKEOFF
2 : OFFBOARDS
3 : LAND
---------------------------

CTRL-C to quit

"""


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class Controller:
    def __init__(self, name):

        self.uav_name = "/uav" + number
        rospy.init_node('controller'+number, anonymous=True)
        rospy.Subscriber(self.uav_name+"/mavros/state", State, self.mavros_state_callback)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.mavros_pos_callback)
      
        self.mavros_state = State()
        self.gazebo_link_states = LinkStates()
        self.height = 0
        self.armServer = rospy.ServiceProxy(self.uav_name+'/mavros/cmd/arming', CommandBool)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
        self.setModeServer = rospy.ServiceProxy(self.uav_name+'/mavros/set_mode', SetMode)
        self.setpoint_pub = rospy.Publisher(self.uav_name+'/mavros/setpoint_raw/local', PositionTarget, queue_size=1)

     
        self.swithch_front = 0
        self.swithch_down = 0

        self.pwd = sys.path[0]
      
        print("Initialized")


    def mavros_state_callback(self, msg):
        self.mavros_state = msg

    def mavros_pos_callback(self, msg):
        self.gazebo_link_states = msg
        # index = self.gazebo_link_states.name.index(
        #     'standard_vtol_1::base_link')
        # self.height = self.gazebo_link_states.pose[index].position.z
        # global INIT_HEIGHT
        # height = msg.altitude

    def command_control(self):
        if self.key == 'e' or self.key == 'E':
            self.swithch_down += 1
        if self.key == 'q' or self.key == 'Q':
            self.swithch_front += 1
        if self.key == '0':
            if self.armServer(True):
                print("Vehicle arming succeed!")
            else:
                print("Vehicle arming failed!")
        if self.key == '9':
            if self.armServer(False):
                print("Vehicle disarming succeed!")
            else:
                print("Vehicle arming failed!")
        elif self.key == '1':
            if self.setModeServer(custom_mode='AUTO.TAKEOFF'):
                print("Vehicle takeoff succeed!")
            else:
                print("Vehicle takeoff failed!")
        elif self.key == '2':
            if self.setModeServer(custom_mode='OFFBOARD'):
                print("Vehicle offboard succeed!")
            else:                                                                                                                                        
                print("Vehicle offboard failed!")
        elif self.key == '3':
            if self.setModeServer(custom_mode='AUTO.LAND'):
                print("Vehicle land succeed!")
            else:
                print("Vehicle land failed!")
        elif self.key == '4':        
            if self.setModeServer(custom_mode='POSCTL'):
                print("Vehicle posControl succeed!")
                print(self.mavros_state)
            else:
                print("Vehicle posControl failed!")
     
    def send_speed_command(self, velocity_x, velocity_y, velocity_z):
        msg = PositionTarget()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | \
                        PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                        PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        msg.velocity.x = velocity_x
        msg.velocity.y = velocity_y
        msg.velocity.z = velocity_z
        self.setpoint_pub.publish(msg)
    
    def action_control(self):
        # throttle
        if self.mavros_state.mode == 'POSCTL':
            if self.key == 'i' or self.key == 'I':
                self.send_speed_command(10.0, 10.0, 10.0)
            elif self.key == 'k' or self.key == 'K':
                self.send_speed_command(10.0, 10.0, -10.0)
            else:
                self.send_speed_command(10.0, 10.0, 10.0)
       
        # # pitch
        # if self.key == 'w' or self.key == 'W':
        #     channel2 = self.speed_control
        # elif self.key == 's' or self.key == 'S':
        #     channel2 = 3000-self.speed_control
        # else:
        #     channel2 = 1500
        # # roll
        # if self.key == 'a' or self.key == 'A':
        #     channel0 = 3000-self.speed_control
        # elif self.key == 'd' or self.key == 'D':
        #     channel0 = self.speed_control
        # else:
        #     channel0 = 1500
        # # yaw
        # if self.key == 'j' or self.key == 'J':
        #     channel3 = 1300
        # elif self.key == 'l' or self.key == 'L':
        #     channel3 = 1700
        # else:
        #     channel3 = 1500
        # self.cur_target_rc_yaw = RCInOverride(channel0, channel1, channel2, channel3)
        # if self.key == 'h' or self.key == 'H':
        #     self.speed_control = self.speed_control + 10
        #     print('Current control speed :', self.speed_control)
        # elif self.key == 'g' or self.key == 'G':
        #     self.speed_control = self.speed_control - 10
        #     print('Current control speed :', self.speed_control)

    def main_loop(self):
        while (1):
            self.key = getKey()
            self.command_control()
            self.action_control()
            # print(self.mavros_state.mode)
            # print(self.height)
            if (controller.key == '\x03'):
                break


if __name__ == "__main__":
    # 从命令行参数获取数字
    if len(sys.argv) > 1:
        number = sys.argv[1]
    else:
        sys.exit()
    
    
    settings = termios.tcgetattr(sys.stdin)
    
    print(msg)
    
    controller = Controller(number)
    controller.main_loop()
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)