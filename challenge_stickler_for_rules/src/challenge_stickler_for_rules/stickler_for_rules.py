# Copyright (c) 2022, TU/e Robotics, Netherlands
# All rights reserved.

import smach

from robot_smach_states.human_interaction import Say
from robot_smach_states.startup import StartChallengeRobust
from robot_smach_states.util.designators import EntityByIdDesignator
from .patrol import PatrolToRoom


class SticklerForRules(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted"])

        waypoint_ids = ["living_room", "kitchen", "bedroom", "study"]

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
                    "You all better stick to the rules. I am coming.",
                    block=False,
                ),
                transitions={"spoken": "GO_TO_WAYPOINT_1"},
            )

            # Main loop
            # TODO make a proper loop over waypoints (AFTER ROBOCUP)
            smach.StateMachine.add('GO_TO_WAYPOINT_1',
                                   PatrolToRoom(robot, EntityByIdDesignator(robot, uuid=waypoint_ids[0]),
                                                speak=False),
                                   transitions={'arrived': 'GO_TO_WAYPOINT_2',
                                                'unreachable': 'GO_TO_WAYPOINT_2',
                                                'goal_not_defined': 'GOODBYE',
                                                'preempted': 'Aborted'})

            smach.StateMachine.add('GO_TO_WAYPOINT_2',
                                   PatrolToRoom(robot, EntityByIdDesignator(robot, uuid=waypoint_ids[1]),
                                                speak=False),
                                   transitions={'arrived': 'GO_TO_WAYPOINT_3',
                                                'unreachable': 'GO_TO_WAYPOINT_3',
                                                'goal_not_defined': 'GOODBYE',
                                                'preempted': 'Aborted'})

            smach.StateMachine.add('GO_TO_WAYPOINT_3',
                                   PatrolToRoom(robot, EntityByIdDesignator(robot, uuid=waypoint_ids[2]),
                                                speak=False),
                                   transitions={'arrived': 'GO_TO_WAYPOINT_4',
                                                'unreachable': 'GO_TO_WAYPOINT_4',
                                                'goal_not_defined': 'GOODBYE',
                                                'preempted': 'Aborted'})

            smach.StateMachine.add('GO_TO_WAYPOINT_4',
                                   PatrolToRoom(robot, EntityByIdDesignator(robot, uuid=waypoint_ids[3]),
                                                speak=False),
                                   transitions={'arrived': 'GO_TO_WAYPOINT_1',
                                                'unreachable': 'GO_TO_WAYPOINT_1',
                                                'goal_not_defined': 'GOODBYE',
                                                'preempted': 'Aborted'})

            # Outro
            smach.StateMachine.add(
                "GOODBYE",
                Say(robot, "I have had it with you naughty rulebreakers. Goodbye!"),
                transitions={"spoken": "Done"},
            )
