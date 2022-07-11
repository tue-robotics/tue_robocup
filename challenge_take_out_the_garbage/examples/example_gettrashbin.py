#! /usr/bin/python

# System
import argparse

# ROS
import rospy

# TU/e Robotics

from robot_skills.arm import arms
from robot_skills.get_robot import get_robot

# Robot Smach States
import robot_smach_states.util.designators as ds

# Take out the garbage
from challenge_take_out_the_garbage.pick_up import GetTrashBin


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Pick up trash from the trashbin")
    parser.add_argument("trashbin_id", default="trashbin", help="Id of the trashbin to be grasped from.")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node("test_pick_up_garbage")

    robot = get_robot(args.robot)
    trashbin_id = args.trashbin_id

    trashbin_designator = ds.EntityByIdDesignator(robot, trashbin_id)

    state = GetTrashBin(robot, trashbin_designator)
    state.execute()
