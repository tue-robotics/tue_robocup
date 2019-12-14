import argparse
import rospy

from challenge_final import LightSaber
from robot_skills import get_robot


if __name__ == "__main__":

    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--robot", default="hero", help="Name of the robot you want to use")
    args = parser.parse_args()

    rospy.init_node("test_get_drinks")

    robot = get_robot(args.robot)

    sm = LightSaber(robot)
    sm.execute()
