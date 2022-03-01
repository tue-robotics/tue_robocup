#! /usr/bin/env python3

import argparse
import rospy
from robot_smach_states.util.startup import startup
from move_obstacles import MoveObstacles


def sm(robot, x, y, gdx, gdy):
    return MoveObstacles(robot, x, y, gdx, gdy)


if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument("robot", help="obstacle x position", type=str)
    parser.add_argument("x", help="obstacle x position", type=float)
    parser.add_argument("y", help="obstacle y position", type=float)
    parser.add_argument("gdx", help="manipulation pose x distance from obstacle", type=float)
    parser.add_argument("gdy", help="manipulation pose x distance from obstacle", type=float)

    args = parser.parse_args()

    rospy.init_node('move_obstacles_exec')

    sm_args = (args.x, args.y, args.gdx, args.gdy)

    startup(sm, statemachine_args=sm_args, challenge_name='move_obstacles', argv=["challenge_move_obstacles.py", args.robot])
