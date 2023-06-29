#!/usr/bin/python3

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist

def move_vessel():
    rospy.init_node('vessel_controller')

    # 创建服务客户端
    rospy.wait_for_service('/gazebo/set_model_state')
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    model_state = ModelState()
    model_state.model_name = 'vessel_g_1'

    # 设置新的模型位置
    # model_state.pose.position.x = 1.0  # 设置x坐标为1.0
    # model_state.pose.position.y = 2.0  # 设置y坐标为2.0
    # model_state.pose.position.z = 0.0  # 设置z坐标为0.0

    # 设置模型速度（如果需要）
    model_state.twist.linear.x = 0.1  # 设置x方向的线速度为0.1 m/s
    model_state.twist.linear.y = 0.0
    model_state.twist.linear.z = 0.0

    # 创建Pose和Twist的空消息，用于姿态和速度的默认值
    pose = Pose()
    twist = Twist()

    # 设置模型的姿态和速度
    # model_state.pose = pose
    model_state.twist = twist

    # 发送服务请求，将模型状态更新为新位置和速度
    set_model_state(model_state)

if __name__ == '__main__':
    try:
        move_vessel()
    except rospy.ROSInterruptException:
        pass
