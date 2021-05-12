# ROS
import PyKDL as kdl
import rospy
from std_msgs.msg import Bool

# TU/e Robotics
from robot_skills.get_robot import get_robot


class BumperThing:
    def __init__(self, robot):
        self.robot = robot
        self.front_bumper_sub = rospy.Subscriber("hero/base_f_bumper_sensor", Bool, self.front_callback)

    def front_callback(self, msg):
        rospy.loginfo(msg)
        if msg.data:
            self.robot.base.force_drive(-0.1, 0, 0, 0.1)


if __name__ == "__main__":
    rospy.init_node("test_grasping")
    robot = get_robot("hero")
    bt = BumperThing(robot)
    rospy.spin()
