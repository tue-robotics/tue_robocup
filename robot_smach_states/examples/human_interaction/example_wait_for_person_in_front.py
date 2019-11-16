# ROS
import rospy
import sys

# TU/e Robotics
from robot_skills.get_robot import get_robot_from_argv

# Robot Smach States
from robot_smach_states.human_interaction import WaitForPersonInFront


if __name__ == "__main__":
    assert len(sys.argv) == 2, "Please provide the robot name" \
                               "e.g., 'python example_wait_for_person_in_front.py hero'"

    rospy.init_node("example_wait_for_person_in_front")

    robot = get_robot_from_argv(index=1)

    rospy.loginfo("Creating wait for person in front state")
    attempts = 10
    sleep_interval = 0.2
    sm = WaitForPersonInFront(robot, attempts, sleep_interval)

    rospy.loginfo("Executing wait for person in front state")
    sm.execute()

    rospy.loginfo("Done")
