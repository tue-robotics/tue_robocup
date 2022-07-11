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
from challenge_take_out_the_garbage.drop_down import DropTrash


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Drop trash being held")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node("test_drop_down_garbage")

    robot = get_robot(args.robot)

    arm = ds.ArmDesignator(
        robot,
        {
            "required_goals": ["handover", "reset", "handover_to_human"],
            "force_sensor_required": True,
            "required_gripper_types": [arms.GripperTypes.GRASPING],
        },
        name="arm_designator",
    ).lockable()

    grab_state = DropTrash(robot, arm)
    grab_state.execute()
