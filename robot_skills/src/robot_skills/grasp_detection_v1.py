import rospy
from geometry_msgs.msg import WrenchStamped


def callback(data):
    rospy.loginfo("I heard %s", data)
    object = False
    threshold_t_y = 0.1

    data_list = list(data)
    if data_list[4] >= threshold_t_y:
        object = True
    else:
        object = False


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("FT_sensor", WrenchStamped, callback)


if __name__ == "__main__":
    print("** running started **")
    l = listener()
    # print(dir(WrenchStamped))
    rospy.spin()
