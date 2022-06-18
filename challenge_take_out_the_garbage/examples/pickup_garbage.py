# System
import argparse

# ROS
import PyKDL as kdl
import rospy

# TU/e Robotics

from robot_skills.arm import arms
from robot_skills.get_robot import get_robot

# Robot Smach States
import robot_smach_states.util.designators as ds
from robot_smach_states.manipulation import Grab

# Take out the garbage
from challenge_take_out_the_garbage.pick_up import PickUpTrash

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Pick up trash from the trashbin")
    parser.add_argument("trashbin_id", default="trashbin", help="Id of the trashbin to be grasped from.")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node("test_pick_up_garbage")

    robot = get_robot(args.robot)
    trashbin_id = args.trashbin_id

    trashbin_designator = ds.EntityByIdDesignator(robot, trashbin_id)

    arm = ds.UnoccupiedArmDesignator(
        robot,
        {"required_goals": ["handover", "reset", "handover_to_human"], "force_sensor_required": True,
         "required_gripper_types": [arms.GripperTypes.GRASPING]},
        name="empty_arm_designator").lockable()
    arm.lock()

    grab_state = PickUpTrash(robot, trashbin_designator, arm)
    grab_state.execute()
