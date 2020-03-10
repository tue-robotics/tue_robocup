# ROS
import rospy
import argparse

# TU/e Robotics
from robot_skills.get_robot import get_robot

# Robot Smach States
from robot_smach_states import ForceDrive


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test the force drive of the robot")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node("example_force_drive")
    robot = get_robot(args.robot)

    sentence = "I am going to execute a force drive, press enter if the area around me is clear"
    robot.speech.speak(sentence)
    confirm = raw_input(sentence)

    rospy.loginfo("Creating forward force drive state")
    vx = 0.3
    t = 1.0
    sm = ForceDrive(robot, vx, 0, 0, t)

    rospy.loginfo("Executing forward force drive state")
    sm.execute()

    rospy.loginfo("Creating rotating force drive state")
    vtheta = 1.57
    t = 2.0
    sm = ForceDrive(robot, 0, 0, vtheta, t)

    rospy.loginfo("Executing forward force drive state")
    sm.execute()

    rospy.loginfo("Done")
