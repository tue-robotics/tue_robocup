#ROS
import rospy
import argparse

# TU/e Robotics
from robot_skills.get_robot import get_robot

# Robot Smach States
from robot_smach_states.human_interaction import Say


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test the say state with a sentence")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node('test_say')
    robot = get_robot(args.robot)
    sentence = 'I have said something useful'

    say_state = Say(robot, sentence)
    say_state.execute()
