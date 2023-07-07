# Copyright (c) 2022, TU/e Robotics, Netherlands
# All rights reserved.

import smach

from robot_smach_states.human_interaction import Say
from robot_smach_states.startup import StartChallengeRobust
from robot_smach_states.util.designators import EntityByIdDesignator
from .patrol import Patrol
from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_stickler_for_the_rules')


class SticklerForRules(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted"])

        waypoint_ids = challenge_knowledge.waypoint_ids

        with self:
            # Intro

            smach.StateMachine.add(
                "START_CHALLENGE_ROBUST",
                StartChallengeRobust(robot, "initial_pose"),
                transitions={"Done": "SAY_START", "Aborted": "Aborted", "Failed": "SAY_START"},
            )

            smach.StateMachine.add(
                "SAY_START",
                Say(
                    robot,
                    "Party police! Party police!. I'm coming.",
                    block=False,
                ),
                transitions={"spoken": "GO_TO_LIVING_ROOM"},
            )

            # Main loop
            # TODO make a proper loop over waypoints (AFTER ROBOCUP)
            smach.StateMachine.add('GO_TO_LIVING_ROOM',
                                   PatrolToRoom(robot, EntityByIdDesignator(robot, uuid=waypoint_ids[0]),
                                                speak=False),
                                   transitions={'arrived': 'GO_TO_KITCHEN',
                                                'unreachable': 'GO_TO_KITCHEN',
                                                'goal_not_defined': 'GOODBYE',
                                                'preempted': 'Aborted'})

            smach.StateMachine.add('GO_TO_KITCHEN',
                                   PatrolToRoom(robot, EntityByIdDesignator(robot, uuid=waypoint_ids[1]),
                                                speak=False),
                                   transitions={'arrived': 'GO_TO_BEDROOM',
                                                'unreachable': 'GO_TO_BEDROOM',
                                                'goal_not_defined': 'GOODBYE',
                                                'preempted': 'Aborted'})

            smach.StateMachine.add('GO_TO_BEDROOM',
                                   PatrolToRoom(robot, EntityByIdDesignator(robot, uuid=waypoint_ids[2]),
                                                speak=False),
                                   transitions={'arrived': 'GO_TO_KITCHEN_AGAIN',
                                                'unreachable': 'GO_TO_KITCHEN_AGAIN',
                                                'goal_not_defined': 'GOODBYE',
                                                'preempted': 'Aborted'})

            smach.StateMachine.add('GO_TO_KITCHEN_AGAIN',
                                   PatrolToRoom(robot, EntityByIdDesignator(robot, uuid=waypoint_ids[1]),
                                                speak=False),
                                   transitions={'arrived': 'GO_TO_LIVING_ROOM_AGAIN',
                                                'unreachable': 'GO_TO_LIVING_ROOM_AGAIN',
                                                'goal_not_defined': 'GOODBYE',
                                                'preempted': 'Aborted'})

            smach.StateMachine.add('GO_TO_LIVING_ROOM_AGAIN',
                                   PatrolToRoom(robot, EntityByIdDesignator(robot, uuid=waypoint_ids[0]),
                                                speak=False),
                                   transitions={'arrived': 'GO_TO_STUDY',
                                                'unreachable': 'GO_TO_STUDY',
                                                'goal_not_defined': 'GOODBYE',
                                                'preempted': 'Aborted'})
            smach.StateMachine.add('GO_TO_STUDY',
                                   PatrolToRoom(robot, EntityByIdDesignator(robot, uuid=waypoint_ids[3]),
                                                speak=False),
                                   transitions={'arrived': 'GO_TO_LIVING_ROOM',
                                                'unreachable': 'GO_TO_LIVING_ROOM',
                                                'goal_not_defined': 'GOODBYE',
                                                'preempted': 'Aborted'})
            # Outro
            smach.StateMachine.add(
                "GOODBYE",
                Say(robot, "I have had it with you naughty rulebreakers. Goodbye!"),
                transitions={"spoken": "Done"},
            )
