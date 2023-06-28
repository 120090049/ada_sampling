#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
import rospy
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandVtolTransition, CommandVtolTransitionRequest
from geometry_msgs.msg import TwistStamped, PoseStamped
from gazebo_msgs.msg import LinkStates
# from img_warp_and_stitch.stitch_stream import MATCHING

import sys, rospy, math

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

msg = """
---------------------------
command_cotrol
0 : Takeoff
1 : Start mission
2 : Return
3 : Land
---------------------------
CTRL-C to quit
"""
FLYING_HEIGHT = 5

def calculate_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distance

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
    
    def __init__(self, number):
        self.number = number
        self.uav_name = "/uav" + number 
        self.link_name = "uav" + str(int(number)-1) + "::base_link"
        print("start to control vtol", number)
        # print("whose link name is ", self.link_name)
        node_name = 'controller_'+number 
        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(20)

        
        # subscribe uav state and position
        rospy.Subscriber(self.uav_name+"/mavros/state", State, self.mavros_state_callback)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.mavros_pos_callback)
      
        self.mavros_state = State()
        self.gazebo_link_states = LinkStates()
        self.ori_pos = [0, 0]
        self.pos = [0, 0]
        self.height = 0
        
        # service caller: arming, changing flight mode and vtol flying mode
        self.armServer = rospy.ServiceProxy(self.uav_name+'/mavros/cmd/arming', CommandBool)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
        self.setModeServer = rospy.ServiceProxy(self.uav_name+'/mavros/set_mode', SetMode)
        self.transitVtolServer = rospy.ServiceProxy(self.uav_name+"/mavros/cmd/vtol_transition", CommandVtolTransition)

        # publish setpoint
        # self.setpoint_pub = rospy.Publisher(self.uav_name+'/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.local_pos_pub = rospy.Publisher(self.uav_name+"/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.pose = PoseStamped()
        
        # self.pwd = sys.path[0]
        
        # initialize flight control
        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not self.mavros_state.connected):
            self.rate.sleep()

        pose = PoseStamped()
        pose.pose.position.z = FLYING_HEIGHT

        # Send a few setpoints before starting
        for i in range(10):
            if(rospy.is_shutdown()):
                break

            self.local_pos_pub.publish(pose)
            self.rate.sleep()
        self.ori_pos[0] = self.pos[0]
        self.ori_pos[1] = self.pos[1]
        rospy.loginfo("Vtol initialzed")
      
    def mavros_state_callback(self, msg):
        self.mavros_state = msg

    def mavros_pos_callback(self, msg):
        gazebo_link_states = msg
        index = gazebo_link_states.name.index(
            self.link_name)
        self.height = gazebo_link_states.pose[index].position.z
        self.pos = [int(gazebo_link_states.pose[index].position.x), int(gazebo_link_states.pose[index].position.y)]

    def take_off(self):
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'
        
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_req = rospy.Time.now()
        # self.pose.pose.position.x = self.ori_pos[0]
        # self.pose.pose.position.y = self.ori_pos[1]
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = FLYING_HEIGHT
        break_tag = False
        while(not break_tag):
            # arm
            while(not self.mavros_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(0.5)):
                last_req = rospy.Time.now()
                if(self.armServer.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
                    rospy.sleep(2)
                    # offboard   
                    while (True):
                        if(self.mavros_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(0.5)):
                            last_req = rospy.Time.now()
                            if((self.setModeServer.call(offb_set_mode).mode_sent == True)):
                                rospy.loginfo("OFFBOARD enabled")

                                # reach to inital height
                                target_height = FLYING_HEIGHT - 0.5
                                while (int(self.height) <= target_height):
                                    # print("originial pose = ", self.ori_pos, end='\r')
                                    # print("temp pose = ", self.pos, end='\r')
                                    print("height = " , self.height, end='\r')
                                    self.local_pos_pub.publish(self.pose)
                                    self.rate.sleep()
                                rospy.loginfo("reach to VTOL_TRANSITION height and wait for further command") 
                                break_tag = True
                                break
                        self.rate.sleep()
            if (break_tag):
                break
            self.rate.sleep()
            
    def mission(self):

        # vtol
        vtol_set_mode = CommandVtolTransitionRequest()
        vtol_set_mode.state = 4
        while (self.transitVtolServer.call(vtol_set_mode).success == False):
            self.rate.sleep()
        rospy.loginfo("VTOL_TRANSITION enabled (fix wing mode)")
        self.pose.pose.position.x = 10*(int(self.number)+1)
        self.pose.pose.position.y = 10
        self.pose.pose.position.z = 15
        
        self.rate.sleep()
    
    def command_control(self):
      
        if self.key == '0':
            rospy.loginfo("Start to takeoff")
            self.take_off()
            
        elif self.key == '1':
            rospy.loginfo("Start to execute mission")
            self.mission()
            
        elif self.key == '2':
            rospy.loginfo("Enter returning mode")
            switch = False
            while(True):
                if (calculate_distance(self.pos, self.ori_pos) < 10) and not switch:
                    switch = True
                    vtol_set_mode = CommandVtolTransitionRequest()
                    vtol_set_mode.state = 3
                    while (self.transitVtolServer.call(vtol_set_mode).success == False):
                        self.rate.sleep()
                    rospy.loginfo("Switch to quadrotor mode")

                self.pose.pose.position.x = self.ori_pos[0]
                self.pose.pose.position.y = self.ori_pos[1]
                self.pose.pose.position.z = FLYING_HEIGHT
                self.local_pos_pub.publish(self.pose)
                if (calculate_distance(self.pos, self.ori_pos) < 1):
                    break
                self.rate.sleep()
                
        elif self.key == '3':
            rospy.loginfo("Start to land")
            while(not self.setModeServer(custom_mode='AUTO.LAND')):
                print("Switching to LANDING mode", end='\r')
                self.rate.sleep()
            while(self.mavros_state.armed):
                print("LANDING", end='\r')
                self.rate.sleep()
            rospy.loginfo("Landed successfully and disarming the vehicle")
        elif self.key == '5':
            self.armServer(False)
            print("disarm")
     
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
       
    def main_loop(self):
        while (1):
            self.key = getKey()
            self.command_control()
            # self.action_control()
            self.local_pos_pub.publish(self.pose)
            if (controller.key == '\x03'):
                break
            self.rate.sleep()


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
