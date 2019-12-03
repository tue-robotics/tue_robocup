# ROS
import rospy
import argparse

# TU/e Robotics
from robot_skills.get_robot import get_robot

# Robot Smach States
from robot_smach_states.human_interaction import WaitForPersonInFront


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test to see if the robot detects a person in front of him")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node("example_wait_for_person_in_front")

    robot = get_robot(args.robot)

    rospy.loginfo("Creating wait for person in front state")
    attempts = 10
    sleep_interval = 0.2
    sm = WaitForPersonInFront(robot, attempts, sleep_interval)
    outcome = sm.execute()

    rospy.loginfo("Executing wait for person in front state")
    if outcome == "success":
        rospy.loginfo("Detected person")
    else:
        rospy.loginfo("No person detected")
