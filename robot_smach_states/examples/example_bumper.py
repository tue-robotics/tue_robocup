# ROS
import PyKDL as kdl
import rospy
from std_msgs.msg import Bool

# TU/e Robotics
from robot_skills.get_robot import get_robot


def front_callback(data):
    rospy.loginfo(data)


if __name__ == "__main__":

    rospy.init_node("test_grasping")

    robot = get_robot("hero")

    front_bumper_sub = rospy.Subscriber("hero/base_f_bumper_sensor", Bool, front_callback)
