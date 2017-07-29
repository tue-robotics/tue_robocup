#!/usr/bin/python

import robot_smach_states
from robot_skills.util.robot_constructor import robot_constructor

import rospy
import smach
import sys
from robocup_knowledge import load_knowledge
from robot_smach_states.util.designators import EdEntityDesignator, VariableDesignator, EntityByIdDesignator

from ssl_demo import SSLDemo
from raytrace_demo import RayTraceDemo
from order_counter import OrderCounter
from hsr_interaction import HsrInteraction

challenge_knowledge = load_knowledge('challenge_open')


def setup_statemachine(robot):
    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])

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
                               transitions={"done": "NAVIGATE_TO_LASER_DEMO", 'preempted': 'Aborted'})

        smach.StateMachine.add("NAVIGATE_TO_LASER_DEMO",
                               robot_smach_states.NavigateToWaypoint(robot=robot,
                                                                     waypoint_designator=EntityByIdDesignator(
                                                                         robot=robot,
                                                                         id=challenge_knowledge.raytrace_waypoint),
                                                                     radius=0.3),
                               transitions={'arrived': 'RESET_HEAD_BEFORE_RAYTRACE_DEMO',
                                            'unreachable': 'RESET_HEAD_BEFORE_RAYTRACE_DEMO',
                                            'goal_not_defined': 'RESET_HEAD_BEFORE_RAYTRACE_DEMO'})

        smach.StateMachine.add("RESET_HEAD_BEFORE_RAYTRACE_DEMO",
                               robot_smach_states.ResetHead(robot),
                               transitions={'done': 'WAIT_FOR_TRIGGER_BEFORE_RAYTRACE_DEMO'})

        smach.StateMachine.add("WAIT_FOR_TRIGGER_BEFORE_RAYTRACE_DEMO",
                               robot_smach_states.WaitForTrigger(robot, ["continue"], "/amigo/trigger"),
                               transitions={'continue': 'RAYTRACE_DEMO',
                                            'preempted': 'RAYTRACE_DEMO'})

        smach.StateMachine.add("RAYTRACE_DEMO",
                               RayTraceDemo(robot, breakout_id=challenge_knowledge.raytrace_waypoint),
                               transitions={"done": "NAVIGATE_TO_ORDER_COUNTER"})

        smach.StateMachine.add("NAVIGATE_TO_ORDER_COUNTER",
                               robot_smach_states.NavigateToWaypoint(robot=robot,
                                                                     waypoint_designator=EntityByIdDesignator(
                                                                         robot=robot,
                                                                         id=challenge_knowledge.order_counter_waypoint),
                                                                     radius=0.3),
                               transitions={'arrived': 'ORDER_COUNTER',
                                            'unreachable': 'ORDER_COUNTER',
                                            'goal_not_defined': 'ORDER_COUNTER'})

        smach.StateMachine.add("ORDER_COUNTER",
                               OrderCounter(robot, room_id=challenge_knowledge.audience_room),
                               transitions={"done": "NAVIGATE_TO_HSR_DEMO"})

        smach.StateMachine.add("NAVIGATE_TO_HSR_DEMO",
                               robot_smach_states.NavigateToWaypoint(robot=robot,
                                                                     waypoint_designator=EntityByIdDesignator(
                                                                         robot=robot,
                                                                         id=challenge_knowledge.hsr_demo_waypoint),
                                                                     radius=0.025),
                               transitions={'arrived': 'WAIT_FOR_BEER',
                                            'unreachable': 'CANNOT_REACH_BEER_LOCATION',
                                            'goal_not_defined': 'CANNOT_REACH_BEER_LOCATION'})

        smach.StateMachine.add("CANNOT_REACH_BEER_LOCATION",
                               robot_smach_states.Say(robot, "I cannot reach my beer buddy, let's give it another try"),
                               transitions={"spoken": "Done"})

        smach.StateMachine.add("WAIT_FOR_BEER",
                               HsrInteraction(robot=robot),
                               transitions={"done": "RETURN_TO_AUDIENCE"})

        smach.StateMachine.add("RETURN_TO_AUDIENCE",
                               robot_smach_states.NavigateToWaypoint(robot=robot,
                                                                     waypoint_designator=EntityByIdDesignator(
                                                                         robot=robot,
                                                                         id=challenge_knowledge.order_counter_waypoint),
                                                                     radius=0.3),
                               transitions={'arrived': 'SAY_BEER',
                                            'unreachable': 'SAY_BEER',
                                            'goal_not_defined': 'SAY_BEER'})

        smach.StateMachine.add("SAY_BEER",
                               robot_smach_states.Say(robot, "Hey guys, here's your beer. That will be all"),
                               transitions={"spoken": "Done"})

    return sm


if __name__ == '__main__':
    rospy.init_node('challenge_open')

    amigo = robot_constructor('amigo')

    with amigo:
        # build the state machine
        executioner = setup_statemachine(amigo)
        children = executioner.get_children().keys()

        # Verify that the start_states are in the children list of the executive
        start_states = ["START_CHALLENGE_ROBUST",
                        "NAVIGATE_TO_SSL_WAYPOINT",
                        "NAVIGATE_TO_LASER_DEMO",
                        "NAVIGATE_TO_HSR_DEMO",
                        "RETURN_TO_AUDIENCE"]

        for start_state in start_states:
            if start_state not in children:
                rospy.logerr("Provided start state {} not in children: {}".format(start_state, children))
                sys.exit(1)

        print ""
        print "Select start state:"
        for i, state in enumerate(start_states):
            print "[{}] {}".format(i, state)
        print ""

        invalid_option_received = "I received an invalid input ... Aborting ..."
        try:
            choice = raw_input().lower()
        except:
            print invalid_option_received
            sys.exit(1)

        # raw_input returns the empty string for "enter"
        options = [str(e) for e in range(0, len(start_states))] + ['']

        if choice not in options:
            print invalid_option_received
            sys.exit(1)

        initial_state = start_states[int(choice)] if choice != '' else None

        print ""

        if initial_state:
            rospy.loginfo("Starting challenge with initial state '{}'".format(initial_state))
            executioner.set_initial_state([initial_state])
        else:
            rospy.loginfo("Starting challenge with default initial state")

        print ""

        # Run the statemachine
        outcome = executioner.execute()
        print "Final outcome: {0}".format(outcome)
