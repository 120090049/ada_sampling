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
    model_state.model_name = 'vessel_d_1'

 
    # 循环控制模型移动
    while True:
        # 设置新的模型位置

        current_state = get_model_state(model_state.model_name, "")
        model_state.pose.position.x = current_state.pose.position.x + 0.02  

        # 发送服务请求，将模型状态更新为新位置
        set_model_state(model_state)

        # 获取更新后的模型位置
        print(int(model_state.pose.position.x), model_state.pose.position.y, model_state.pose.position.z, end="\r")
        rate.sleep()
        
if __name__ == '__main__':
    try:
        move_vessel()
    except rospy.ROSInterruptException:
        pass


