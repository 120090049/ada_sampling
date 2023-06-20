import rospy
from geometry_msgs.msg import PoseStamped

rospy.init_node('position_controller')

position_pub = rospy.Publisher('/uav1/mavros/setpoint_position/local', PoseStamped, queue_size=1)
def send_position_command(x, y, z):
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    position_pub.publish(msg)
if __name__ == '__main__':
    while not rospy.is_shutdown():
        send_position_command(10.0, 10.0, 10.0)  # 发送目标位置指令 (1, 1, 1)
        rospy.sleep(0.1)  # 控制发送位置指令的频率
