#
# Copyright (c) 2022, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import math
import os

import rospy

from challenge_clean_the_table.knowledge import DISHWASHER_ID, DISHWASHER_AREA_ID
from robot_skills import get_robot
from robot_smach_states.navigation import NavigateToSymbolic, ForceDrive
from robot_smach_states.util.designators import EdEntityDesignator
from smach import StateMachine, cb_interface, CBState


class OpenDishwasher(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=["succeeded"])
        # noinspection PyProtectedMember
        arm = robot.get_arm()._arm

        def send_joint_goal(position_array, wait_for_motion_done=True):
            # noinspection PyProtectedMember
            arm._send_joint_trajectory([position_array], timeout=rospy.Duration(0))
            if wait_for_motion_done:
                arm.wait_for_motion_done()

        def send_gripper_goal(open_close_string, max_torque=0.1, wait_for_motion_done=True):
            arm.gripper.send_goal(open_close_string, max_torque=max_torque)

        @cb_interface(outcomes=["done"])
        def _align(_):
            robot.speech.speak("Aligning before grabbing the handle", block=True)
            return "done"

        @cb_interface(outcomes=["done"])
        def _grab_handle(_):
            robot.speech.speak("I am now grabbing the handle", block=True)
            return "done"

        @cb_interface(outcomes=["done"])
        def _open(_):
            robot.speech.speak("I am now opening the dishwasher so that you can put the rack in later", block=True)

            return "done"

        with self:
            self.add("ALIGN", CBState(_align), transitions={"done": "GRAB_HANDLE"})
            self.add("GRAB_HANDLE", CBState(_grab_handle), transitions={"done": "OPEN"})
            self.add("OPEN", CBState(_open), transitions={"done": "succeeded"})


class NavigateToAndOpenDishwasher(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        dishwasher = EdEntityDesignator(robot=robot, uuid=DISHWASHER_ID)

        with self:
            StateMachine.add(
                "NAVIGATE_TO_DISHWASHER",
                NavigateToSymbolic(robot, {dishwasher: DISHWASHER_AREA_ID}, dishwasher),
                transitions={
                    "arrived": "OPEN_DISHWASHER",
                    "unreachable": "NAVIGATE_TO_DISHWASHER_FAILED",
                    "goal_not_defined": "failed",
                },
            )

            StateMachine.add(
                "NAVIGATE_TO_DISHWASHER_FAILED",
                ForceDrive(robot, 0.0, 0, 0.5, math.pi / 0.5),
                transitions={"done": "NAVIGATE_TO_DISHWASHER"},
            )

            StateMachine.add("PLACE_ITEM_ON_TABLE", OpenDishwasher(robot), transitions={"succeeded": "succeeded"})


if __name__ == "__main__":
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot_instance = get_robot("hero")
    OpenDishwasher(robot_instance).execute()
