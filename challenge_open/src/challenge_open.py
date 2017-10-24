#!/usr/bin/python

import sys
from argparse import ArgumentParser

import rospy
import smach
from robocup_knowledge import load_knowledge
from robot_skills.util.robot_constructor import robot_constructor
from robot_smach_states import StartChallengeRobust, NavigateToWaypoint, ResetHead, Say, WaitForTrigger, \
    NavigateToSymbolic, HandoverToHuman
from robot_smach_states.util.designators import EntityByIdDesignator, UnoccupiedArmDesignator
from smach_ros import IntrospectionServer

from inspect_and_grab import InspectAndGrab
from raytrace_demo import RayTraceDemo
from raytrace_selector import RayTraceSelector
from simple_raytrace_selector import SimpleRayTraceSelector
from ssl_demo import SSLDemo

challenge_knowledge = load_knowledge('challenge_open')


class BeerCounter(object):
    MAX_COUNT = 3

    def __init__(self):
        self.count = 0


def setup_statemachine(robot):
    furniture = EntityByIdDesignator(robot, 'selected_furniture')
    # arm_designator = ArmDesignator(robot.arms, robot.arms['left'])
    arm_designator = UnoccupiedArmDesignator(robot.arms, robot.arms['left'])

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])
    with sm:
        # Start challenge via StartChallengeRobust, skipped atm
        smach.StateMachine.add("START_CHALLENGE_ROBUST",
                               StartChallengeRobust(robot, challenge_knowledge.initial_pose),
                               transitions={"Done": "NAVIGATE_TO_SSL_WAYPOINT",
                                            "Failed": "Aborted",
                                            "Aborted": "Aborted"})

        # smach.StateMachine.add('SAY_STARTING_OPEN_CHALLENGE',
        #                        robot_smach_states.Say(robot, ["Hi there, welcome to the open challenge!"], block=False),
        #                        transitions={"spoken": "NAVIGATE_TO_SSL_WAYPOINT"})

        smach.StateMachine.add("NAVIGATE_TO_SSL_WAYPOINT",
                               NavigateToWaypoint(robot=robot,
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
                               NavigateToWaypoint(robot=robot,
                                                  waypoint_designator=EntityByIdDesignator(
                                                      robot=robot,
                                                      id=challenge_knowledge.raytrace_waypoint),
                                                  radius=0.3),
                               transitions={'arrived': 'RESET_HEAD_BEFORE_RAYTRACE_DEMO',
                                            'unreachable': 'RESET_HEAD_BEFORE_RAYTRACE_DEMO',
                                            'goal_not_defined': 'RESET_HEAD_BEFORE_RAYTRACE_DEMO'})

        smach.StateMachine.add("RESET_HEAD_BEFORE_RAYTRACE_DEMO",
                               ResetHead(robot),
                               transitions={'done': 'SAY_PEOPLE_DETECTOR'})

        smach.StateMachine.add("SAY_PEOPLE_DETECTOR",
                               Say(robot, "Now I will show you my awesome people detector"),
                               transitions={"spoken": "WAIT_FOR_TRIGGER_BEFORE_RAYTRACE_DEMO"})

        smach.StateMachine.add("WAIT_FOR_TRIGGER_BEFORE_RAYTRACE_DEMO",
                               WaitForTrigger(robot, ["continue"], "/amigo/trigger"),
                               transitions={'continue': 'RAYTRACE_DEMO',
                                            'preempted': 'RAYTRACE_DEMO'})

        smach.StateMachine.add("RAYTRACE_DEMO",
                               RayTraceDemo(robot, breakout_id=challenge_knowledge.raytrace_waypoint),
                               transitions={"done": "SAY_RAYTRACE_SELECTOR"})

        smach.StateMachine.add("SAY_RAYTRACE_SELECTOR",
                               Say(robot, "You can interact with me by pointing at objects!"),
                               transitions={"spoken": "RAYTRACE_SELECTOR"})

        smach.StateMachine.add("RAYTRACE_SELECTOR",
                               SimpleRayTraceSelector(robot, waypoint=None, furniture_designator=furniture),
                               transitions={
                                   "waypoint": 'NAVIGATE_TO_WAYPOINT',
                                   "furniture": 'NAVIGATE_TO_FURNITURE',
                                   "grasp": 'INSPECT_AND_GRAB',
                                   "done": 'RAYTRACE_SELECTOR'
                               })

        smach.StateMachine.add("NAVIGATE_TO_FURNITURE",
                               NavigateToSymbolic(robot,
                                                  entity_designator_area_name_map={furniture: 'in_front_of'},
                                                  entity_lookat_designator=furniture),
                               transitions={
                                   'arrived': 'NAVIGATE_BACK_TO_LASER_DEMO',
                                   'unreachable': 'NAVIGATE_BACK_TO_LASER_DEMO',
                                   'goal_not_defined': 'NAVIGATE_BACK_TO_LASER_DEMO',
                               })

        smach.StateMachine.add("NAVIGATE_TO_WAYPOINT",
                               NavigateToWaypoint(robot=robot,
                                                  waypoint_designator=EntityByIdDesignator(
                                                      robot=robot,
                                                      id='final_waypoint'),
                                                  radius=0.3),
                               transitions={'arrived': 'NAVIGATE_BACK_TO_LASER_DEMO',
                                            'unreachable': 'NAVIGATE_BACK_TO_LASER_DEMO',
                                            'goal_not_defined': 'NAVIGATE_BACK_TO_LASER_DEMO'})

        smach.StateMachine.add("INSPECT_AND_GRAB",
                               InspectAndGrab(robot, supporting_entity_designator=furniture,
                                              arm_designator=arm_designator),
                               transitions={
                                   'succeeded': 'NAVIGATE_BACK_TO_LASER_DEMO',
                                   'inspect_failed': 'NAVIGATE_BACK_TO_LASER_DEMO',
                                   'grasp_failed': 'NAVIGATE_BACK_TO_LASER_DEMO'
                               })

        smach.StateMachine.add("HANDOVER_TO_HUMAN", HandoverToHuman(robot, arm_designator, timeout=10),
                               transitions={
                                   'succeeded': 'NAVIGATE_BACK_TO_LASER_DEMO',
                                   'failed': 'NAVIGATE_BACK_TO_LASER_DEMO',
                               })

        smach.StateMachine.add("NAVIGATE_BACK_TO_LASER_DEMO",
                               NavigateToWaypoint(robot=robot,
                                                  waypoint_designator=EntityByIdDesignator(
                                                      robot=robot,
                                                      id=challenge_knowledge.raytrace_waypoint),
                                                  radius=0.3),
                               transitions={'arrived': 'RAYTRACE_SELECTOR',
                                            'unreachable': 'RAYTRACE_SELECTOR',
                                            'goal_not_defined': 'RAYTRACE_SELECTOR'})

        # smach.StateMachine.add("NAVIGATE_TO_ORDER_COUNTER",
        #                        robot_smach_states.NavigateToWaypoint(robot=robot,
        #                                                              waypoint_designator=EntityByIdDesignator(
        #                                                                  robot=robot,
        #                                                                  id=challenge_knowledge.order_counter_waypoint),
        #                                                              radius=0.3),
        #                        transitions={'arrived': 'ORDER_COUNTER',
        #                                     'unreachable': 'ORDER_COUNTER',
        #                                     'goal_not_defined': 'ORDER_COUNTER'})
        #
        # smach.StateMachine.add("ORDER_COUNTER",
        #                        OrderCounter(robot, room_id=challenge_knowledge.audience_room,
        #                                     beercounter=beercounter),
        #                        transitions={"done": "NAVIGATE_TO_HSR_DEMO"})
        #
        # smach.StateMachine.add("NAVIGATE_TO_HSR_DEMO",
        #                        robot_smach_states.NavigateToWaypoint(robot=robot,
        #                                                              waypoint_designator=EntityByIdDesignator(
        #                                                                  robot=robot,
        #                                                                  id=challenge_knowledge.hsr_demo_waypoint),
        #                                                              radius=0.025),
        #                        transitions={'arrived': 'WAIT_FOR_BEER',
        #                                     'unreachable': 'CANNOT_REACH_BEER_LOCATION',
        #                                     'goal_not_defined': 'CANNOT_REACH_BEER_LOCATION'})
        #
        # smach.StateMachine.add("CANNOT_REACH_BEER_LOCATION",
        #                        robot_smach_states.Say(robot, "I cannot reach my beer buddy, let's give it another try"),
        #                        transitions={"spoken": "Done"})
        #
        # smach.StateMachine.add("WAIT_FOR_BEER",
        #                        HsrInteraction(robot=robot, beercounter=beercounter),
        #                        transitions={"done": "RETURN_TO_AUDIENCE"})
        #
        # smach.StateMachine.add("RETURN_TO_AUDIENCE",
        #                        robot_smach_states.NavigateToWaypoint(robot=robot,
        #                                                              waypoint_designator=EntityByIdDesignator(
        #                                                                  robot=robot,
        #                                                                  id=challenge_knowledge.order_counter_waypoint),
        #                                                              radius=0.3),
        #                        transitions={'arrived': 'SAY_BEER',
        #                                     'unreachable': 'SAY_BEER',
        #                                     'goal_not_defined': 'SAY_BEER'})
        #
        # smach.StateMachine.add("SAY_BEER",
        #                        robot_smach_states.Say(robot, "Hey guys, here's your beer. That will be all. "
        #                                                      "By the way, if you leave the balcony door open,"
        #                                                      "birds will fly in"),
        #                        transitions={"spoken": "Done"})

    return sm


if __name__ == '__main__':
    # parse args
    parser = ArgumentParser()
    parser.add_argument('--debug', action='store_true')
    args = parser.parse_args(rospy.myargv()[1:])

    enable_debug = args.debug

    rospy.init_node('challenge_open')
    rospy.loginfo('args: %s', args)

    amigo = robot_constructor('amigo')
    with amigo:
        # build the state machine
        executioner = setup_statemachine(amigo)
        children = executioner.get_children().keys()

        # Verify that the start_states are in the children list of the executive
        start_states = ["START_CHALLENGE_ROBUST",
                        "NAVIGATE_TO_SSL_WAYPOINT",
                        "NAVIGATE_TO_LASER_DEMO",
                        "RAYTRACE_DEMO",
                        "RAYTRACE_SELECTOR"]

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

        introserver = None
        if enable_debug:
            introserver = IntrospectionServer('amigo', executioner, '/SM_ROOT_PRIMARY')
            introserver.start()

        # Run the statemachine
        outcome = executioner.execute()
        print "Final outcome: {0}".format(outcome)

        if introserver:
            introserver.stop()
