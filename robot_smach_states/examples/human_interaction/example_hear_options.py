# ROS
import rospy
import argparse

# TU/e Robotics
from robot_skills.get_robot import get_robot

# Robot Smach States
from robot_smach_states.human_interaction import HearOptions

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Test if the robot hears the things from the options available")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node('test_hear_options')
    robot = get_robot(args.robot)

    hear_option_state = HearOptions(robot, ['no', 'yes'])
    hear_option_state.execute()
