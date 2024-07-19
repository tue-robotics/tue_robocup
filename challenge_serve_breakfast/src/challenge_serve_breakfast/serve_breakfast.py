# Copyright (c) 2022, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import rospy

from robot_smach_states.human_interaction import Say
from robot_smach_states.navigation import NavigateToWaypoint
from robot_smach_states.startup import StartChallengeRobust
from robot_smach_states.util.designators import EdEntityDesignator
from robot_smach_states.world_model import UpdateEntityPose
from robot_smach_states.utility import WaitTime
from smach import cb_interface, CBState

from smach import State, StateMachine
from .navigate_to_and_pick_item import NavigateToAndPickItem, NavigateToSymbolic
from .navigate_to_and_place_item_on_table import NavigateToAndPlaceItemOnTable
from .pick_pour_place_cereal import PickPourPlaceCereal
from .tuning import REQUIRED_ITEMS

class CheckIfWeHaveItAll(State):
    def __init__(self, robot, max_iterations=10):
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

        missing_items = [item for item in REQUIRED_ITEMS if item not in self.items_picked]

        if missing_items:
            self.robot.speech.speak(f"So far we have: {' '.join(self.items_picked)}", block=False)
            self.robot.speech.speak(f"Still missing the {missing_items}", block=False)
        else:
            self.robot.speech.speak("We have everything now", block=False)

        return "keep_going" if missing_items else "we_have_it_all"


def setup_statemachine(robot):
    state_machine = StateMachine(outcomes=["done"])
    state_machine.userdata["item_picked"] = None
    pick_id_dishes = "dishwasher"
    pick_id_drink = "kitchen_cabinet"
    pick_id_food = "kitchen_counter"
    pick_area_id = "in_front_of"
    place_id = "dinner_table"
    place_area_id = "in_front_of"
    exit_id = "starting_pose"
    table_des = EdEntityDesignator(robot=robot, uuid=place_id)

    @cb_interface(outcomes=["pick_drink", "pick_food", "pick_other"], input_keys=["item_picked"])
    def determine_next_pick(userdata):
        rospy.loginfo("Item Picked: {}".format(userdata.item_picked))
        if userdata["item_picked"] == "bowl":
            return "pick_drink"
        elif userdata["item_picked"] == "milk_carton":
            return "pick_food"
        else:
            return "pick_other"

    with state_machine:
        # Intro

        StateMachine.add(
            "START_CHALLENGE_ROBUST",
            StartChallengeRobust(robot, "initial_pose_serve_breakfast"),
            transitions={"Done": "NAVIGATE_TO_TABLE", "Aborted": "GOODBYE", "Failed": "NAVIGATE_TO_TABLE"},
        )
        #Main loop
        StateMachine.add(
            "NAVIGATE_TO_TABLE",
            NavigateToSymbolic(robot, {table_des:place_area_id}, table_des, speak=True),
            transitions={"arrived": "UPDATE_TABLE_POSE", "unreachable": "SAY_START", "goal_not_defined": "SAY_START"},
            )

        StateMachine.add(
            "UPDATE_TABLE_POSE",
            UpdateEntityPose(robot=robot, entity_designator=table_des),
            transitions={"done": "SAY_START"},
        )

        StateMachine.add(
            "SAY_START",
            Say(
                robot,
                "Lets serve some breakfast baby! I will be asking for some fast handovers.",
                block=False,
            ),
            transitions={"spoken": "DETERMINE_NEXT_PICK"},
        )

        StateMachine.add(
            "DETERMINE_NEXT_PICK",
            CBState(determine_next_pick, input_keys=["item_picked"]),
            transitions={"pick_drink": "NAVIGATE_AND_PICK_DRINK", "pick_food": "NAVIGATE_AND_PICK_FOOD",
                         "pick_other": "NAVIGATE_AND_PICK_DISHES"},
        )

        StateMachine.add(
            "SAY_OPERATOR_WHERE_TO_STAND_FOR_DRINK",
            Say(
                robot,
                "Operator, please stand at the LEFT of the kitchen cabinet",
                block=False,
            ),
            transitions={"spoken": "NAVIGATE_AND_PICK_DRINK"},
        )

        StateMachine.add(
            "NAVIGATE_AND_PICK_DRINK",
            NavigateToAndPickItem(robot, pick_id_drink, pick_area_id),
            transitions={"succeeded": "PLACE_ITEM_ON_TABLE", "failed": "NAVIGATE_AND_PICK_ITEM_FAILED"},
        )

        StateMachine.add(
            "SAY_OPERATOR_WHERE_TO_STAND_FOR_FOOD",
            Say(
                robot,
                "Operator, please stand at the RIGHT of the kitchen counter",
                block=False,
            ),
            transitions={"spoken": "NAVIGATE_AND_PICK_FOOD"},
        )

        StateMachine.add(
            "NAVIGATE_AND_PICK_FOOD",
            NavigateToAndPickItem(robot, pick_id_food, pick_area_id),
            transitions={"succeeded": "PLACE_ITEM_ON_TABLE", "failed": "NAVIGATE_AND_PICK_ITEM_FAILED"},
        )

        StateMachine.add(
            "NAVIGATE_AND_PICK_DISHES",
            NavigateToAndPickItem(robot, pick_id_dishes, pick_area_id),
            transitions={"succeeded": "PLACE_ITEM_ON_TABLE", "failed": "NAVIGATE_AND_PICK_ITEM_FAILED"},
        )

        StateMachine.add(
            "NAVIGATE_AND_PICK_ITEM_FAILED", WaitTime(robot, 2),
            transitions={"waited": "DETERMINE_NEXT_PICK", "preempted": "GOODBYE"}
        )

        StateMachine.add(
            "PLACE_ITEM_ON_TABLE",
            NavigateToAndPlaceItemOnTable(robot, place_id, place_area_id),
            transitions={"succeeded": "CHECK_IF_WE_HAVE_IT_ALL", "failed": "WAIT"},
        )

        StateMachine.add(
            "WAIT", WaitTime(robot, 2), transitions={"waited": "CHECK_IF_WE_HAVE_IT_ALL", "preempted": "GOODBYE"}
        )

        StateMachine.add(
            "CHECK_IF_WE_HAVE_IT_ALL",
            CheckIfWeHaveItAll(robot),
            transitions={"we_have_it_all": "PICK_POUR_PLACE_CEREAL", "keep_going": "DETERMINE_NEXT_PICK"},
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
            NavigateToWaypoint(robot, EdEntityDesignator(robot, uuid=exit_id), speak=False),
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

