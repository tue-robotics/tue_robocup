#!/usr/bin/python

import robot_smach_states
import rospy
import smach
from robocup_knowledge import load_knowledge
from robot_smach_states.util.designators import EdEntityDesignator, VariableDesignator, EntityByIdDesignator

from ssl_demo import SSLDemo
from raytrace_demo import RayTraceDemo
from order_counter import OrderCounter
from hsr_interaction import HsrInteraction

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
                               transitions={"done": "NAVIGATE_TO_LASER_DEMO", 'preempted': 'Aborted'})

        smach.StateMachine.add("NAVIGATE_TO_LASER_DEMO",
                               robot_smach_states.NavigateToWaypoint(robot=robot,
                                                                     waypoint_designator=EntityByIdDesignator(
                                                                         robot=robot,
                                                                         id=challenge_knowledge.raytrace_waypoint),
                                                                     radius=0.3),
                               transitions={'arrived': 'RAYTRACE_DEMO',
                                            'unreachable': 'RAYTRACE_DEMO',
                                            'goal_not_defined': 'RAYTRACE_DEMO'})

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
                               transitions={'arrived': 'Done',
                                            'unreachable': 'Done',
                                            'goal_not_defined': 'Done'})

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
    robot_smach_states.util.startup(setup_statemachine, challenge_name="challenge_open")
