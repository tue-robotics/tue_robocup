# Copyright (c) 2022, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import rospy

from robot_smach_states.human_interaction import Say
from robot_smach_states.navigation import NavigateToWaypoint
from robot_smach_states.startup import StartChallengeRobust
from robot_smach_states.util.designators import EdEntityDesignator
from robot_smach_states.utility import WaitTime
from smach import StateMachine, cb_interface, CBState
from .navigate_to_and_pick_item import NavigateToAndPickItem
from .navigate_to_and_place_item_on_table import NavigateToAndPlaceItemOnTable
from .pick_pour_place_cereal import PickPourPlaceCereal
from .tuning import REQUIRED_ITEMS

items_picked = []
iteration = 0
max_iterations = 10


@cb_interface(outcomes=["we_have_it_all", "keep_going"], input_keys=["item_picked"])
def check_if_we_have_it_all(user_data, robot):
    global items_picked
    global iteration

    iteration += 1
    if iteration > max_iterations:
        return "we_have_it_all"

    item_picked = user_data["item_picked"]

    if item_picked and item_picked not in items_picked:
        items_picked.append(item_picked)
    else:
        rospy.logwarn("Invalid item picked: %s", item_picked)

    missing_items = [item for item in REQUIRED_ITEMS if item not in items_picked]

    if missing_items:
        robot.speech.speak("So far we have: {}".format(" ".join(items_picked)), block=False)
        robot.speech.speak("Still missing the {}".format(missing_items), block=False)
    else:
        robot.speech.speak("We have everything now", block=False)

    return "keep_going" if missing_items else "we_have_it_all"


def setup_statemachine(robot):
    state_machine = StateMachine(outcomes=["done"])
    state_machine.userdata["item_picked"] = None
    pick_id = "dinner_table"
    pick_area_id = "in_front_of"
    place_id = "dinner_table"
    place_area_id = "in_front_of"
    exit_id = "starting_pose"

    with state_machine:
        # Intro

        StateMachine.add(
            "START_CHALLENGE_ROBUST",
            StartChallengeRobust(robot, "initial_pose"),
            transitions={"Done": "SAY_START", "Aborted": "done", "Failed": "SAY_START"},
        )

        StateMachine.add(
            "SAY_START",
            Say(
                robot,
                f"Lets serve some breakfast baby! If there are any chairs near the {place_id}, please remove them",
                block=False,
            ),
            transitions={"spoken": "NAVIGATE_AND_PICK_ITEM"},
        )

        # Main loop

        StateMachine.add(
            "NAVIGATE_AND_PICK_ITEM",
            NavigateToAndPickItem(robot, pick_id, pick_area_id),
            transitions={"succeeded": "PLACE_ITEM_ON_TABLE", "failed": "CHECK_IF_WE_HAVE_IT_ALL"},
        )

        StateMachine.add(
            "PLACE_ITEM_ON_TABLE",
            NavigateToAndPlaceItemOnTable(robot, place_id, place_area_id),
            transitions={"succeeded": "CHECK_IF_WE_HAVE_IT_ALL", "failed": "WAIT"},
        )

        StateMachine.add(
            "WAIT", WaitTime(robot, 2), transitions={"waited": "CHECK_IF_WE_HAVE_IT_ALL", "preempted": "done"}
        )

        StateMachine.add(
            "CHECK_IF_WE_HAVE_IT_ALL",
            CBState(check_if_we_have_it_all, cb_args=[robot]),
            transitions={"we_have_it_all": "PICK_POUR_PLACE_CEREAL", "keep_going": "NAVIGATE_AND_PICK_ITEM"},
        )

        # Outro

        StateMachine.add(
            "PICK_POUR_PLACE_CEREAL",
            PickPourPlaceCereal(robot, place_id),
            transitions={"succeeded": "SAY_END_CHALLENGE"},
        )

        StateMachine.add(
            "SAY_END_CHALLENGE",
            Say(robot, "That was all folks, enjoy your breakfast!"),
            transitions={"spoken": "NAVIGATE_TO_EXIT"},
        )

        StateMachine.add(
            "NAVIGATE_TO_EXIT",
            NavigateToWaypoint(robot, EdEntityDesignator(robot, uuid=exit_id)),
            transitions={
                "arrived": "GOODBYE",
                "unreachable": "GOODBYE",
                "goal_not_defined": "GOODBYE",
            },
        )

        StateMachine.add(
            "GOODBYE",
            Say(robot, "Goodbye!"),
            transitions={"spoken": "done"},
        )

    return state_machine
