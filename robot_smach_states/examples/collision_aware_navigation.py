# ROS
import PyKDL as kdl
import rospy
from tf.transformations import euler_from_quaternion
import math
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

# TU/e Robotics
from robot_skills.get_robot import get_robot
from robot_smach_states.navigation import NavigateToPose


class SmartNavNode:
    def __init__(self, robot):
        self.robot=robot
        self.goal_sub = rospy.Subscriber("/smart_navigation/goal", PoseStamped, self.goal_callback)

    def goal_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        (roll, pitch, yaw) = euler_from_quaternion([msg.pose.orientation.x,
                                                     msg.pose.orientation.y,
                                                     msg.pose.orientation.z,
                                                     msg.pose.orientation.w])
        rospy.loginfo("Heading to x: {}, y: {}, yaw: {}".format(x, y, yaw))
        sm = NavigateToPose(self.robot, x, y, yaw)
        rospy.loginfo("Created navigation state-machine. Executing")
        sm.execute()


if __name__ == "__main__":
    rospy.init_node("test_navigation")
    robot = get_robot("hero")
    smartnav = SmartNavNode(robot)
    rospy.loginfo("ready to receive commands")
    rospy.spin()

