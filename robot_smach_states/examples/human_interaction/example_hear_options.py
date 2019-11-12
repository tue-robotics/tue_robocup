#ROS
import rospy
import argparse

# TU/e Robotics
from robot_skills.get_robot import get_robot_from_argv

# Robot Smach States
from robot_smach_states.human_interaction import HearOptions

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Test the HearOptions state with a sentence")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node('test_hear_options')

    robot = get_robot_from_argv(index=1)

    Hear_Option_State = HearOptions(robot, ['no', 'yes'])

    Hear_Option_State.execute()
