import rospy
from geometry_msgs.msg import WrenchStamped


def callback(data):
    rospy.loginfo("I felt: %s", data.wrench)
    threshold_torque_y = 0.1  # Threshold for the torque around y axis in [Nm]
    object = False
    if data.wrench.torque.y >= threshold_torque_y:
        object = True
        print('Grasp: SUCCESSFUL')
    else:
        object = False
        print('Grasp: FAILED')
    return object


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/hero/wrist_wrench/compensated", WrenchStamped, callback)


if __name__ == "__main__":
    l = listener()
    rospy.spin()
