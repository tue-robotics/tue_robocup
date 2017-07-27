#!/usr/bin/python

import robot_smach_states
import rospy
import smach
from robocup_knowledge import load_knowledge
from robot_smach_states.util.designators import EdEntityDesignator, VariableDesignator, EntityByIdDesignator

from ssl_demo import SSLDemo
challenge_knowledge = load_knowledge('challenge_open')


def setup_statemachine(robot):
    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])
    robot.ed.reset()

    with sm:

        # Start challenge via StartChallengeRobust, skipped atm
        smach.StateMachine.add("START_CHALLENGE_ROBUST",
                               robot_smach_states.StartChallengeRobust(robot, challenge_knowledge.initial_pose),
                               transitions={"Done": "SAY_STARTING_OPEN_CHALLENGE",
                                            "Failed": "Aborted",
                                            "Aborted": "Aborted"})

        smach.StateMachine.add('SAY_STARTING_OPEN_CHALLENGE',
                               robot_smach_states.Say(robot, ["Hi there, welcome to the open challenge!"], block=False),
                               transitions={"spoken": "NAVIGATE_TO_SSL_WAYPOINT"})

        smach.StateMachine.add("NAVIGATE_TO_SSL_WAYPOINT",
                               robot_smach_states.NavigateToWaypoint(robot=robot,
                                                                     waypoint_designator=EntityByIdDesignator(
                                                                         robot=robot,
                                                                         id=challenge_knowledge.ssl_waypoint),
                                                                     radius=0.3),
                               transitions={'arrived': 'SSL_DEMO',
                                            'unreachable': 'SSL_DEMO',
                                            'goal_not_defined': 'SSL_DEMO'})

        smach.StateMachine.add("SSL_DEMO",
                               SSLDemo(robot),
                               transitions={"done": "Done", 'preempted': 'Aborted'})

    return sm


if __name__ == '__main__':
    rospy.init_node('challenge_open')
    robot_smach_states.util.startup(setup_statemachine, challenge_name="challenge_open")
