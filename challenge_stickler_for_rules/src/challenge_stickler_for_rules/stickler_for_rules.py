# Copyright (c) 2022, TU/e Robotics, Netherlands
# All rights reserved.

import smach

from robot_smach_states.human_interaction import Say
from robot_smach_states.util.designators import EntityByIdDesignator
from robot_smach_states.utility import SetInitialPose
from robot_smach_states.reset import ResetArmsTorsoHead
from .patrol import Patrol
from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_stickler_for_the_rules')


class SticklerForRules(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted"])

        waypoint_ids = challenge_knowledge.waypoint_ids

        with self:
            # Intro
            smach.StateMachine.add('RESET',
                                   ResetArmsTorsoHead(robot),
                                   transitions={'done': 'SET_INITIAL_POSE'})

            smach.StateMachine.add('SET_INITIAL_POSE',
                                   SetInitialPose(robot, challenge_knowledge.starting_point),
                                   transitions={'done': 'SAY_START',
                                                "preempted": 'SAY_START',
                                                'error': 'SAY_START'})
            smach.StateMachine.add(
                "SAY_START", Say(robot, "Party police! Party police!. I'm coming.", block=False),
                transitions={"spoken": "GO_TO_BEDROOM"},
            )

            # Main loop
            # TODO make a proper loop over waypoints (AFTER ROBOCUP)
            smach.StateMachine.add('GO_TO_BEDROOM',
                                   Patrol(robot, EntityByIdDesignator(robot, uuid=waypoint_ids[2])),
                                   transitions={'done': 'GO_TO_KITCHEN'})

            smach.StateMachine.add('GO_TO_KITCHEN',
                                   Patrol(robot, EntityByIdDesignator(robot, uuid=waypoint_ids[1])),
                                   transitions={'done': 'GO_TO_LIVING_ROOM'})

            smach.StateMachine.add('GO_TO_LIVING_ROOM',
                                   Patrol(robot, EntityByIdDesignator(robot, uuid=waypoint_ids[0])),
                                   transitions={'done': 'GO_TO_STUDY'})

            smach.StateMachine.add('GO_TO_STUDY',
                                   Patrol(robot, EntityByIdDesignator(robot, uuid=waypoint_ids[3])),
                                   transitions={'done': 'GO_TO_LIVING_ROOM_AGAIN'})

            smach.StateMachine.add('GO_TO_LIVING_ROOM_AGAIN',
                                   Patrol(robot, EntityByIdDesignator(robot, uuid=waypoint_ids[0])),
                                   transitions={'done': 'GO_TO_KITCHEN_AGAIN'})

            smach.StateMachine.add('GO_TO_KITCHEN_AGAIN',
                                   Patrol(robot, EntityByIdDesignator(robot, uuid=waypoint_ids[1])),
                                   transitions={'done': 'GO_TO_BEDROOM'})


            # Outro
            smach.StateMachine.add(
                "GOODBYE",
                Say(robot, "I have had it with you naughty rulebreakers. Goodbye!"),
                transitions={"spoken": "Done"},
            )
