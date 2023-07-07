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
                transitions={"spoken": "GO_TO_WAYPOINT_1"},
            )

            # Main loop
            # TODO make a proper loop over waypoints (AFTER ROBOCUP)
            smach.StateMachine.add('GO_TO_WAYPOINT_1',
                                   Patrol(robot, EntityByIdDesignator(robot, uuid=waypoint_ids[0])),
                                   transitions={'done':"GO_TO_WAYPOINT_2"})

            smach.StateMachine.add('GO_TO_WAYPOINT_2',
                                   Patrol(robot, EntityByIdDesignator(robot, uuid=waypoint_ids[1])),
                                   transitions={'done':"GO_TO_WAYPOINT_3"})

            smach.StateMachine.add('GO_TO_WAYPOINT_3',
                                   Patrol(robot, EntityByIdDesignator(robot, uuid=waypoint_ids[2])),
                                   transitions={'done':"GO_TO_WAYPOINT_4"})

            smach.StateMachine.add('GO_TO_WAYPOINT_4',
                                   Patrol(robot, EntityByIdDesignator(robot, uuid=waypoint_ids[3])),
                                   transitions={'done':"GO_TO_WAYPOINT_3_REVERSED"})

            smach.StateMachine.add('GO_TO_WAYPOINT_3_REVERSED',
                                   Patrol(robot, EntityByIdDesignator(robot, uuid=waypoint_ids[2])),
                                   transitions={'done': "GO_TO_WAYPOINT_2_REVERSED"})

            smach.StateMachine.add('GO_TO_WAYPOINT_2_REVERSED',
                                   Patrol(robot, EntityByIdDesignator(robot, uuid=waypoint_ids[1])),
                                   transitions={'done': "GO_TO_WAYPOINT_1"})

            # Outro
            smach.StateMachine.add(
                "GOODBYE",
                Say(robot, "I have had it with you naughty rulebreakers. Goodbye!"),
                transitions={"spoken": "Done"},
            )
