#
# Copyright (c) 2019, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn
import math
import os

import rospy

from challenge_set_the_table.knowledge import CUPBOARD_ID, CUPBOARD_NAVIGATION_AREA
from robot_skills import get_robot
from robot_smach_states.human_interaction import Say
from robot_smach_states.navigation import NavigateToSymbolic, ForceDrive
from robot_smach_states.util.designators import EdEntityDesignator
from smach import StateMachine


class NavigateToAndPickMugFromCupboardDrawer(StateMachine):
    def __init__(self, robot, cupboard_id, cupboard_navigation_area):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        cupboard = EdEntityDesignator(robot=robot, uuid=cupboard_id)

        with self:
            StateMachine.add("NAVIGATE_TO_CUPBOARD",
                             NavigateToSymbolic(robot, {cupboard: cupboard_navigation_area}, cupboard),
                             transitions={'arrived': 'PICK_ITEM_FROM_CUPBOARD',
                                          'unreachable': 'NAVIGATE_TO_CUPBOARD_FAILED',
                                          'goal_not_defined': 'failed'})

            StateMachine.add(
                "NAVIGATE_TO_CUPBOARD_FAILED",
                ForceDrive(robot, 0.0, 0, 0.5, math.pi / 0.5),
                transitions={"done": "NAVIGATE_TO_CUPBOARD"},
            )

            StateMachine.add("PICK_MUG_FROM_CUPBOARD", Say(robot, "I am picking the mug from the cupboard drawer"),
                             transitions={'spoken': 'succeeded'})


if __name__ == '__main__':
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot_instance = get_robot("hero")
    robot_instance.reset()
    NavigateToAndPickMugFromCupboardDrawer(robot_instance, CUPBOARD_ID, CUPBOARD_NAVIGATION_AREA).execute()
