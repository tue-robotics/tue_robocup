#
# Copyright (c) 2019, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

from smach import StateMachine

import robot_smach_states.util.designators as ds
from robot_skills.util.entity import Entity
from robot_smach_states.navigation import NavigateToWaypoint
from robot_smach_states.startup import StartChallengeRobust
from robot_smach_states.human_interaction import Say
from robocup_knowledge import load_knowledge

from get_furniture_from_operator_pose import GetFurnitureFromOperatorPose
from identify_object import IdentifyObject
from inspect_furniture_entity import InspectFurniture

challenge_knowledge = load_knowledge('challenge_hand_me_that')

STARTING_POINT = challenge_knowledge.starting_point  # Location where the challenge starts
HOME_LOCATION = challenge_knowledge.home_location  # Location where the robot will go and look at the operator


def setup_statemachine(robot):
    state_machine = StateMachine(outcomes=['done'])

    furniture_designator = ds.VariableDesignator(resolve_type=Entity)
    entity_designator = ds.VariableDesignator(resolve_type=Entity)
    arm_designator = ds.UnoccupiedArmDesignator(robot).lockable()

    with state_machine:
        # Intro
        StateMachine.add('START_CHALLENGE_ROBUST', StartChallengeRobust(robot, STARTING_POINT),
                         transitions={'Done': 'SAY_START',
                                      'Aborted': 'done',
                                      'Failed': 'SAY_START'})

        # Say we're gonna start
        StateMachine.add('SAY_START', Say(robot, "Hand me that it is!", block=False),
                         transitions={'spoken': 'NAVIGATE_TO_START'})

        # Drive to the start location
        StateMachine.add('NAVIGATE_TO_START',
                         NavigateToWaypoint(robot, ds.EdEntityDesignator(robot, id=HOME_LOCATION)),
                         transitions={'arrived': 'GET_FURNITURE_FROM_OPERATOR_POSE',
                                      'unreachable': 'NAVIGATE_TO_START',  # ToDo: other fallback
                                      'goal_not_defined': 'done'})  # I'm not even going to fill this in

        # The pre-work
        StateMachine.add('GET_FURNITURE_FROM_OPERATOR_POSE',
                         GetFurnitureFromOperatorPose(robot, furniture_designator.writeable),
                         transitions={'done': 'INSPECT_FURNITURE'})

        # Go to the furniture object that was pointing to see what's there
        StateMachine.add('INSPECT_FURNITURE',
                         InspectFurniture(robot, furniture_designator, entity_designator.writeable),
                         transitions={"succeeded": "IDENTIFY_OBJECT",
                                      "failed": "NAVIGATE_TO_START"})  # If no entities, try again

        # Point at the object
        StateMachine.add('IDENTIFY_OBJECT',
                         IdentifyObject(robot, entity_designator, arm_designator),
                         transitions={'done': 'NAVIGATE_TO_START',  # Just keep on going
                                      'failed': 'NAVIGATE_TO_START'})  # Just keep on going

    return state_machine
