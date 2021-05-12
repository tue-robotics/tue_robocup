# ROS
import PyKDL as kdl
import rospy
from std_msgs.msg import Bool

# TU/e Robotics
from robot_skills.get_robot import get_robot


class BumperSetup:
    def __init__(self, robot):
        self.robot = robot
        self.front_bumper_sub = rospy.Subscriber("hero/base_f_bumper_sensor", Bool, self.front_callback)
        self.front_bumper_active = False
        self.back_bumper_sub = rospy.Subscriber("hero/base_b_bumper_sensor", Bool, self.back_callback)
        self.back_bumper_active = False

    def front_callback(self, msg):
        rospy.loginfo(msg)
        self.front_bumper_active = msg.data

    def back_callback(self, msg):
        rospy.loginfo(msg)
        self.back_bumper_active = msg.data

    def motion(self):
        if self.front_bumper_active and self.back_bumper_active:
            rospy.loginfo("robot_stuck")
            self.robot.base.force_drive(0, 0, 0, 1)
        elif self.front_bumper_active:
            rospy.loginfo("driving_back")
            self.robot.base.force_drive(-0.1, 0, 0, 1)
        elif self.back_bumper_active:
            rospy.loginfo("driving_forward")
            self.robot.base.force_drive(0.1, 0, 0, 1)


if __name__ == "__main__":
    rospy.init_node("test_grasping")
    robot = get_robot("hero")
    bt = BumperSetup(robot)
    while not rospy.is_shutdown():
        bt.motion()
        rospy.sleep(0.1)

