# Copyright (c) 2022, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import rospy

from robot_smach_states.human_interaction import Say
from robot_smach_states.startup import StartChallengeRobust
from robot_smach_states.utility import WaitTime
from smach import State, StateMachine
from .knowledge import ITEMS
from .navigate_to_and_open_dishwasher import NavigateToAndOpenDishwasher
from .navigate_to_and_pick_item import NavigateToAndPickItem
from .navigate_to_and_place_item_in_dishwasher_rack import NavigateToAndPlaceItemInDishwasherRack


class CheckIfWeHaveItAll(State):
    def __init__(self, robot, max_iterations=20):
        State.__init__(self, outcomes=["we_have_it_all", "keep_going"], input_keys=["item_picked"])
        self.robot = robot
        self.iteration = 0
        self.items_picked = []
        self.max_iterations = max_iterations

    def execute(self, userdata):
        self.iteration += 1
        if self.iteration > self.max_iterations:
            return "we_have_it_all"

        item_picked = userdata["item_picked"]

        if item_picked and item_picked not in self.items_picked:
            self.items_picked.append(item_picked)
        else:
            rospy.logwarn("Invalid item picked: %s", item_picked)

        missing_items = [item for item in ITEMS if item not in self.items_picked]

        if missing_items:
            self.robot.speech.speak(f"So far we have: {' '.join(self.items_picked)}", block=False)
            self.robot.speech.speak(f"Still missing the {missing_items}", block=False)
        else:
            self.robot.speech.speak("We have everything now", block=False)

        return "keep_going" if missing_items else "we_have_it_all"


def setup_statemachine(robot):
    state_machine = StateMachine(outcomes=["done"])
    state_machine.userdata["item_picked"] = None

    with state_machine:
        # Intro

        StateMachine.add(
            "START_CHALLENGE_ROBUST",
            StartChallengeRobust(robot, "initial_pose"),
            transitions={"Done": "SAY_START", "Aborted": "done", "Failed": "SAY_START"},
        )

        StateMachine.add(
            "SAY_START",
            Say(robot, f"Lets cleanup the table baby!", block=False),
            transitions={"spoken": "NAVIGATE_TO_AND_OPEN_DISHWASHER"},
        )

        # First open the dishwasher
        StateMachine.add(
            "NAVIGATE_TO_AND_OPEN_DISHWASHER",
            NavigateToAndOpenDishwasher(robot),
            transitions={"succeeded": "NAVIGATE_AND_PICK_ITEM", "failed": "done"},
        )

        # Main loop

        StateMachine.add(
            "NAVIGATE_AND_PICK_ITEM",
            NavigateToAndPickItem(robot),
            transitions={"succeeded": "PLACE_ITEM_IN_DISHWASHER", "failed": "NAVIGATE_AND_PICK_ITEM_FAILED"},
        )

        StateMachine.add(
            "NAVIGATE_AND_PICK_ITEM_FAILED",
            WaitTime(robot, 2),
            transitions={"waited": "NAVIGATE_AND_PICK_ITEM", "preempted": "done"},
        )

        StateMachine.add(
            "PLACE_ITEM_IN_DISHWASHER",
            NavigateToAndPlaceItemInDishwasherRack(robot),
            transitions={"succeeded": "CHECK_IF_WE_HAVE_IT_ALL", "failed": "WAIT"},
        )

        StateMachine.add(
            "WAIT", WaitTime(robot, 2), transitions={"waited": "CHECK_IF_WE_HAVE_IT_ALL", "preempted": "done"}
        )

        StateMachine.add(
            "CHECK_IF_WE_HAVE_IT_ALL",
            CheckIfWeHaveItAll(robot),
            transitions={"we_have_it_all": "SAY_END_CHALLENGE", "keep_going": "NAVIGATE_AND_PICK_ITEM"},
        )

        # Outro
        StateMachine.add(
            "SAY_END_CHALLENGE",
            Say(robot, "That was all folks, good luck with that Bowl!"),
            transitions={"spoken": "GOODBYE"},
        )

        StateMachine.add("GOODBYE", Say(robot, "Goodbye!"), transitions={"spoken": "done"})

    return state_machine
