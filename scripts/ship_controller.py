#!/usr/bin/python3
import rospy
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose

def move_vessel():
    rospy.init_node('vessel_controller')
    rate = rospy.Rate(20)
    # 创建服务客户端
    rospy.wait_for_service('/gazebo/set_model_state')
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    model_state = ModelState()
    model_state.model_name = 'vessel_g_1'

    # 获取当前模型位置
    current_state = get_model_state(model_state.model_name, "")

    # 设置初始位置和步长
    initial_x = current_state.pose.position.x
    target_x = initial_x + 0.1  # 目标位置为当前位置+0.1m

    # 循环控制模型移动
    while True:
        # 设置新的模型位置
        model_state.pose.position.x = current_state.pose.position.x + 0.01  # 每次前进0.01m

        # 创建Pose的空消息，用于姿态的默认值
        pose = Pose()

        # 设置模型的姿态
        model_state.pose = pose

        # 发送服务请求，将模型状态更新为新位置
        set_model_state(model_state)

        # 获取更新后的模型位置
        current_state = get_model_state(model_state.model_name, "")

        rate.sleep()
        
if __name__ == '__main__':
    try:
        move_vessel()
    except rospy.ROSInterruptException:
        pass
