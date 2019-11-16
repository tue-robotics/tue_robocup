# ROS
import rospy
import sys

# TU/e Robotics
from robot_skills.get_robot import get_robot_from_argv

# Robot Smach States
from robot_smach_states import ForceDrive


if __name__ == "__main__":
    assert len(sys.argv) == 2, "Please provide the robot name" \
                               "e.g., 'python example_something.py hero'"

    rospy.init_node("example_force_drive")

    robot = get_robot_from_argv(index=1)

    rospy.loginfo("Creating forward force drive state")
    vx = 0.3
    t = 1.0
    sm = ForceDrive(robot, vx, 0, 0, t)

    rospy.loginfo("Executing forward force drive state")
    sm.execute()

    rospy.loginfo("Creating rotating force drive state")
    vtheta = 1.47
    t = 1.0
    sm = ForceDrive(robot, 0, 0, vtheta, t)

    rospy.loginfo("Executing forward force drive state")
    sm.execute()

    rospy.loginfo("Done")
