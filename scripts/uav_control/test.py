#! /usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandVtolTransition, CommandVtolTransitionRequest
from gazebo_msgs.msg import LinkStates
import time



current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def mavros_pos_callback(msg):
        gazebo_link_states = msg
        index = gazebo_link_states.name.index(
            'standard_vtol_0::base_link')
        global height 
        height = gazebo_link_states.pose[index].position.z
        
        
if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("/uav1/mavros/state", State, callback = state_cb)
    gazebo_link_states = rospy.Subscriber("/gazebo/link_states", LinkStates, mavros_pos_callback)
    
    local_pos_pub = rospy.Publisher("/uav1/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/uav1/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("/uav1/mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/uav1/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("/uav1/mavros/set_mode", SetMode)
    
    rospy.wait_for_service("/uav1/mavros/cmd/vtol_transition")
    transitVtolServer = rospy.ServiceProxy("/uav1/mavros/cmd/vtol_transition", CommandVtolTransition)



    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    # pose.pose.position.x = 0
    # pose.pose.position.y = 0
    pose.pose.position.z = 5

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'
    
    vtol_set_mode = CommandVtolTransitionRequest()
    vtol_set_mode.state = 4
    
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        # arm
        if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(0.5)):
            last_req = rospy.Time.now()
            if(arming_client.call(arm_cmd).success == True):
                rospy.loginfo("Vehicle armed")
                time.sleep(5)
                # offboard   
                while (True):
                    print("wait")
                    if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(0.5)):
                        last_req = rospy.Time.now()
                        if(set_mode_client.call(offb_set_mode).mode_sent == True):
                            rospy.loginfo("OFFBOARD enabled")

                            # reach to inital height
                            while (int(height) <= 4):
                                print("height = " , height, end='\r')
                                local_pos_pub.publish(pose)
                                rate.sleep()
                            rospy.loginfo("reach to VTOL_TRANSITION height")
                            
                            
                            # vtol
                            print(transitVtolServer.call(vtol_set_mode))
                            
                            # if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(0.5)):
                            #     last_req = rospy.Time.now()
                            # transitVtolServer.call(vtol_set_mode)
                            rospy.loginfo("VTOL_TRANSITION enabled")
                            
                            while(True):
                                    
                                last_req = rospy.Time.now()

                                pose.pose.position.x = 0
                                pose.pose.position.y = 0
                                pose.pose.position.z = 10
                                local_pos_pub.publish(pose)
                                
                                rate.sleep()
                    rate.sleep()

        rate.sleep()
