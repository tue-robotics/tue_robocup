# Copyright (c) 2019, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import rospy

from challenge_set_the_table.navigate_to_and_close_cupboard_drawer import NavigateToAndCloseCupboard
from challenge_set_the_table.navigate_to_and_open_cupboard_drawer import NavigateToAndOpenCupboard
from challenge_set_the_table.navigate_to_and_pick_item import NavigateToAndPickItem
from challenge_set_the_table.navigate_to_and_place_item_on_table import NavigateToAndPlaceItemOnTable
from robot_smach_states.human_interaction import Say
from robot_smach_states.navigation import NavigateToWaypoint
import robot_smach_states.util.designators as ds
from robot_smach_states.startup import StartChallengeRobust
from smach import StateMachine, cb_interface, CBState
from robocup_knowledge import load_knowledge
from robot_smach_states.utility import WaitTime

CHALLENGE_KNOWLEDGE = load_knowledge('challenge_set_the_table')

items_picked = []
required_items = ["cup", "napkin", "knife", "fork", "spoon"]
iteration = 0
max_iterations = 10


@cb_interface(outcomes=['we_have_it_all', 'keep_going'], input_keys=['item_picked'])
def check_if_we_have_it_all(user_data, robot):
    global items_picked
    global iteration

    iteration += 1
    if iteration > max_iterations:
        return 'we_have_it_all'

    item_picked = user_data['item_picked']

    if item_picked and item_picked not in items_picked:
        items_picked.append(item_picked)
    else:
        rospy.logwarn("Invalid item picked: %s", item_picked)

    missing_items = [item for item in required_items if item not in items_picked]

    robot.speech.speak("So far we have: {}".format(", ".join(items_picked)), block=False)
    if missing_items:
        robot.speech.speak("Still missing the {}".format(", ".join(missing_items)), block=False)

    return 'keep_going' if missing_items else 'we_have_it_all'


def setup_statemachine(robot):
    state_machine = StateMachine(outcomes=['done'])
    state_machine.userdata['item_picked'] = None

    with state_machine:
        # Intro

        StateMachine.add('START_CHALLENGE_ROBUST', StartChallengeRobust(robot, CHALLENGE_KNOWLEDGE.starting_point),
                         transitions={'Done': 'SAY_START',
                                      'Aborted': 'done',
                                      'Failed': 'SAY_START'})

        StateMachine.add('SAY_START',
                         Say(robot, "Let's set the table baby!", block=False),
                         transitions={'spoken': 'NAVIGATE_AND_PICK_ITEM'})

        # The loop
        StateMachine.add('NAVIGATE_AND_PICK_ITEM',
                         NavigateToAndPickItem(robot, CHALLENGE_KNOWLEDGE.dinner_table_id,
                                               required_items),
                         transitions={'succeeded': 'PLACE_ITEM_ON_TABLE',
                                      'failed': 'CHECK_IF_WE_HAVE_IT_ALL'})

        StateMachine.add('PLACE_ITEM_ON_TABLE',
                         NavigateToAndPlaceItemOnTable(robot, CHALLENGE_KNOWLEDGE.dinner_table_id, "in_front_of",
                                                       "in_front_of"),
                         transitions={'succeeded': 'CHECK_IF_WE_HAVE_IT_ALL',
                                      'failed': 'WAIT'})

        StateMachine.add('WAIT',
                         WaitTime(robot, 2),
                         transitions={'waited': 'CHECK_IF_WE_HAVE_IT_ALL', 'preempted': 'done'})

        StateMachine.add('CHECK_IF_WE_HAVE_IT_ALL',
                         CBState(check_if_we_have_it_all, cb_args=[robot]),
                         transitions={'we_have_it_all': 'SAY_END_CHALLENGE',
                                      'keep_going': 'NAVIGATE_AND_PICK_ITEM'})

        # Outro
        StateMachine.add('SAY_END_CHALLENGE',
                         Say(robot, "That was all folks, have a nice meal!"),
                         transitions={'spoken': 'done'})

        StateMachine.add('NAVIGATE_AND_CLOSE_CUPBOARD',
                         NavigateToAndCloseCupboard(robot, CHALLENGE_KNOWLEDGE.cupboard_id, "in_front_of"),
                         transitions={'succeeded': 'done',
                                      'failed': 'done'})

    return state_machine
