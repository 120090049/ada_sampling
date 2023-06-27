#! /usr/bin/env python

import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from std_msgs.msg import UInt8, UInt32, Float32
from mavros_msgs.msg import State, PositionTarget, ParamValue
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandVtolTransition, ParamSet

#ipp messages
from planner_map_interfaces.msg import Plan, PlanRequest

#threshold to send next waypoint
THRESHOLD = 40.0

# TODO: monitor extended state to keep track of VTOL state

class WaypointManager(object):
    def __init__(self):
        self.received_plan_request = False
        self.received_plan = False
        self.waypoint_num = 1
        self.remaining_budget = np.inf
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.current_state = State()
        self.start_pose = PoseStamped()
        self.current_pose = PoseStamped()
        self.previous_pose = PoseStamped()
        self.current_plan = Plan()
    
    def state_callback(self, msg):
        self.current_state = msg
    
    def pose_callback(self, msg):
        self.previous_pose = self.current_pose
        #calculate distance travelled
        dis_travelled = np.sqrt((msg.pose.position.x - self.previous_pose.pose.position.x)**2 +
                                (msg.pose.position.y - self.previous_pose.pose.position.y)**2 +
                                (msg.pose.position.z - self.previous_pose.pose.position.z)**2)
        self.remaining_budget -= dis_travelled
        self.current_pose = msg
    
    def plan_request_callback(self, msg):
        self.start_pose.pose = msg.start_pose
        self.received_plan_request = True
        self.remaining_budget = msg.maximum_range
        rospy.loginfo("received plan request")
    
    def plan_callback(self, msg):
        self.current_plan = msg
        self.received_plan = True
        self.waypoint_num = 0
        rospy.loginfo("received plan")
    
    def transform_to_local_pose(self, global_pose):
        local_pose = PoseStamped()
        local_pose.header.stamp = rospy.Time.now()
        local_pose.pose.position.x = global_pose.pose.position.x
        local_pose.pose.position.y = global_pose.pose.position.y
        local_pose.pose.position.z = global_pose.pose.position.z
        local_pose.pose.orientation = global_pose.pose.orientation
        return local_pose
    
    def reached_setpoint(self, setpoint):
        setpoint_pose = setpoint.pose.position
        current_pose = self.current_pose.pose.position
        
        setpoint_pose_mat = np.array([setpoint_pose.x, setpoint_pose.y, setpoint_pose.z])
        current_pose_mat = np.array([current_pose.x, current_pose.y, current_pose.z])

        return np.linalg.norm(setpoint_pose_mat - current_pose_mat) < THRESHOLD

    def main(self):
        #MAVROS TOPICS
        state_sub = rospy.Subscriber("mavros/state", State, self.state_callback)
        local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.pose_callback)
        local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

        #IPP TOPICS
        plan_request_sub = rospy.Subscriber("planner/plan_request", PlanRequest, self.plan_request_callback)
        plan_sub = rospy.Subscriber("global_path", Plan, self.plan_callback)
        waypoint_num_pub = rospy.Publisher('waypoint_num', UInt32, queue_size=10)
        remaining_budget_pub = rospy.Publisher('remaining_budget', Float32, queue_size=10)

        #MAVROS SERVICES
        rospy.loginfo("waiting for arming service")
        rospy.wait_for_service("mavros/cmd/arming")
        arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)   

        rospy.loginfo("waiting for set mode service")
        rospy.wait_for_service("mavros/set_mode")
        set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        rospy.loginfo("waiting for vtol transition client")
        rospy.wait_for_service("mavros/cmd/vtol_transition")
        transition_client = rospy.ServiceProxy("mavros/cmd/vtol_transition", CommandVtolTransition)

        # change loiter parameter
        rospy.loginfo("setting loiter radius smaller")
        rospy.wait_for_service("mavros/param/set")
        set_param_service = rospy.ServiceProxy("mavros/param/set", ParamSet)
        #change loiter parameter
        loiter_change_msg = offb_set_mode = ParamSet()
        set_param_service(param_id="NAV_LOITER_RAD", value=ParamValue(real=25.0))

        # Setpoint publishing MUST be faster than 2Hz
        rate = rospy.Rate(200)

        #wait for plan request before initiating takeoff
        rospy.loginfo("waiting for plan request")
        while not rospy.is_shutdown() and not self.received_plan_request:
            rate.sleep()

        #wait for connection to flight controller
        rospy.loginfo("waiting for connection to flight controller")
        while(not rospy.is_shutdown() and not self.current_state.connected):
            rate.sleep()

        current_setpoint = self.transform_to_local_pose(self.start_pose)
        # Send a few setpoints before starting
        for i in range(125):   
            if(rospy.is_shutdown()):
                break
            local_pos_pub.publish(current_setpoint)
            if i%10 == 0:
                rospy.loginfo("sent " + str(i) + " setpoints")
            rate.sleep()
        
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True
        
        last_req = rospy.Time.now()
        
        fixed_wing_mode = False
        while(not rospy.is_shutdown()):
            if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")
                
                last_req = rospy.Time.now()
            elif(not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                    if(arming_client.call(arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")
                    last_req = rospy.Time.now()
            else:
                if self.reached_setpoint(current_setpoint) and self.received_plan and len(self.current_plan.plan) > 0:
                    next_setpoint = PoseStamped()
                    next_setpoint.pose = self.current_plan.plan.pop(0).position
                    current_setpoint = self.transform_to_local_pose(next_setpoint)
                    # rospy.loginfo("sending next setpoint. " + str(len(self.current_plan.plan)) + " setpoints left and " + str(self.waypoint_num) + " waypoints complete")

                    self.waypoint_num += 1
                    waypoint_num_pub.publish(UInt32(data=self.waypoint_num))

                    #change to fixed wing mode while already flying 
                    if self.waypoint_num > 1 and not fixed_wing_mode:
                        if(transition_client(state=4).success == True):
                            print("FIXED WING MODE ENABLED")
                            fixed_wing_mode = True
                    
                    #only publish budget if we have plan left
                    remaining_budget_pub.publish(data=self.remaining_budget)

            local_pos_pub.publish(current_setpoint)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("mavros_waypoint_manager")
    manager = WaypointManager()
    manager.main()