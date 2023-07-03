#!/usr/bin/python3


import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from random import uniform

def random_movement():
    rospy.init_node('random_movement_node', anonymous=True)
    pub = rospy.Publisher('point_topic', Marker, queue_size=10)
    rate = rospy.Rate(1)  # 设置发布频率为1Hz

    while not rospy.is_shutdown():
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "map"  # 设置参考坐标系

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.r = 1.0
        marker.color.a = 1.0

        marker.pose.position.x = uniform(-10, 10)  # 随机生成点的位置
        marker.pose.position.y = uniform(-10, 10)
        marker.pose.position.z = 0.0

        pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        random_movement()
    except rospy.ROSInterruptException:
        pass
    
# 在rviz中，你需要设置以下内容才能正确显示点：

# 在左侧的Displays面板中，点击 "Add" 按钮，选择 "Marker"。
# 在Marker的属性面板中，将 "Marker Topic" 设置为 /point_topic。
# 在Fixed Frame字段中，设置为 "map" 或你想要的参考坐标系。
# 随后，你将能够在rviz中看到随机生成的点。

