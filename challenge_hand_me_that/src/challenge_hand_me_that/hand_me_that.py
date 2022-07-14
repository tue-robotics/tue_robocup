#
# Copyright (c) 2019, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import smach

from ed.entity import Entity

import robot_smach_states.util.designators as ds
from robot_smach_states.navigation import NavigateToWaypoint
from robot_smach_states.startup import StartChallengeRobust
from robot_smach_states.human_interaction import Say, FindPerson
from robot_smach_states.perception import LookAtEntity
from robot_smach_states.utility import WaitTime
from robocup_knowledge import load_knowledge

from .get_furniture_from_operator_pose import GetFurnitureFromOperatorPose
from .identify_object import IdentifyObject
from .inspect_furniture_entity import InspectFurniture

challenge_knowledge = load_knowledge('challenge_hand_me_that')

STARTING_POINT = challenge_knowledge.starting_point  # Location where the challenge starts
HOME_LOCATION = challenge_knowledge.home_location  # Location where the robot will go and look at the operator
POSSIBLE_FURNITURE = challenge_knowledge.all_possible_furniture
ROOM = challenge_knowledge.room


class HandMeThat(smach.StateMachine):
    """ Main StateMachine for the challenge """

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['done'])

        furniture_designator = ds.VariableDesignator(resolve_type=Entity)
        entity_designator = ds.VariableDesignator(resolve_type=[Entity])
        operator_designator = ds.VariableDesignator(resolve_type=Entity).writeable
        arm_designator = ds.UnoccupiedArmDesignator(robot).lockable()
        room_designator = ds.EntityByIdDesignator(robot, uuid=ROOM)

        TESTING = False

        with self:
            if not TESTING:
                # Intro
                smach.StateMachine.add('START_CHALLENGE_ROBUST', StartChallengeRobust(robot, STARTING_POINT),
                                       transitions={'Done': 'SAY_START',
                                                    'Aborted': 'done',
                                                    'Failed': 'SAY_START'})

            # Say we're gonna start
            smach.StateMachine.add('SAY_START', Say(robot, "Hand me that it is!", block=False),
                                   transitions={'spoken': 'NAVIGATE_TO_START'})

            # Drive to the start location
            smach.StateMachine.add('NAVIGATE_TO_START',
                                   NavigateToWaypoint(robot, ds.EdEntityDesignator(robot, uuid=HOME_LOCATION)),
                                   transitions={'arrived': 'FIND_OPERATOR_IN_ROOM',
                                                'unreachable': 'NAVIGATE_TO_START',  # ToDo: other fallback
                                                'goal_not_defined': 'done'})  # I'm not even going to fill this in

            # The pre-work
            smach.StateMachine.add('FIND_OPERATOR_IN_ROOM',
                                   FindPerson(robot=robot,
                                              found_entity_designator=operator_designator,
                                              discard_other_labels=False,
                                              room=ROOM),
                                   transitions={'found': 'LOOK_AT_PERSON',
                                                'failed': 'LOOK_AT_CENTER_OF_ROOM'})

            smach.StateMachine.add('LOOK_AT_PERSON',
                                   LookAtEntity(robot=robot, entity=operator_designator, height=1.5),
                                   transitions={'succeeded': 'GET_FURNITURE_FROM_OPERATOR_POSE',
                                                'failed': 'LOOK_AT_CENTER_OF_ROOM'})

            smach.StateMachine.add('LOOK_AT_CENTER_OF_ROOM',
                                   LookAtEntity(robot=robot, entity=room_designator),
                                   transitions={'succeeded': 'SAY_CANT_FIND_PERSON',
                                                'failed': 'SAY_CANT_FIND_PERSON'})

            smach.StateMachine.add('SAY_CANT_FIND_PERSON',
                                   Say(robot, 'I did not find you, please stand in my view'),
                                   transitions={'spoken': 'WAIT_FOR_OPERATOR'})

            smach.StateMachine.add('WAIT_FOR_OPERATOR', WaitTime(robot, 5),
                                   transitions={'waited': 'GET_FURNITURE_FROM_OPERATOR_POSE',
                                                'preempted': 'GET_FURNITURE_FROM_OPERATOR_POSE'})

            smach.StateMachine.add('GET_FURNITURE_FROM_OPERATOR_POSE',
                                   GetFurnitureFromOperatorPose(robot, furniture_designator.writeable,
                                                                POSSIBLE_FURNITURE),
                                   transitions={'done': 'INSPECT_FURNITURE'})

            # Go to the furniture object that was pointing to see what's there
            smach.StateMachine.add('INSPECT_FURNITURE',
                                   InspectFurniture(robot, furniture_designator, entity_designator.writeable),
                                   transitions={"succeeded": "IDENTIFY_OBJECT",
                                                "failed": "SAY_NO_OBJECT"})  # If no entities, try again

            # Tell when you failed to
            smach.StateMachine.add('SAY_NO_OBJECT', Say(robot, ['I did not find any object object there']),
                                   transitions={'spoken': 'NAVIGATE_TO_START'})

            # Point at the object
            smach.StateMachine.add('IDENTIFY_OBJECT',
                                   IdentifyObject(robot, entity_designator, arm_designator),
                                   transitions={'done': 'NAVIGATE_TO_START',  # Just keep on going
                                                'failed': 'SAY_TRY_NEXT'})  # Just keep on going

            smach.StateMachine.add('SAY_TRY_NEXT', Say(robot, ['I am sorry, lets skip this one and'
                                                               ' let me try another one',
                                                               'I will go for the next item']),
                                   transitions={'spoken': 'NAVIGATE_TO_START'})
