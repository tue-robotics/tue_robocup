#! /usr/bin/env python
import rospy

import smach

from robot_smach_states.navigation import NavigateToWaypoint
from robot_smach_states.human_interaction import Say

from robot_smach_states.designators.designator import Designator, VariableDesignator


class AvoidThat(smach.StateMachine):

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        with self:

            smach.StateMachine.add('NAVIGATE_TO_WAYPOINT',
                                    NavigateToWaypoint(robot, Designator("avoid_that_wp1")),
                                    transitions={   "arrived" : "NAVIGATE_TO_WAYPOINT2",
                                                    "unreachable" : 'NAVIGATE_TO_WAYPOINT2',
                                                    "goal_not_defined" : 'NAVIGATE_TO_WAYPOINT2'})

            smach.StateMachine.add('NAVIGATE_TO_WAYPOINT2',
                                    NavigateToWaypoint(robot, Designator("avoid_that_wp2")),
                                    transitions={   "arrived" : "NAVIGATE_TO_WAYPOINT3",
                                                    "unreachable" : 'NAVIGATE_TO_WAYPOINT3',
                                                    "goal_not_defined" : 'NAVIGATE_TO_WAYPOINT3'})

            smach.StateMachine.add('NAVIGATE_TO_WAYPOINT3',
                                    NavigateToWaypoint(robot, Designator("avoid_that_wp3")),
                                    transitions={   "arrived" : "SAY_GOAL_REACHED",
                                                    "unreachable" : 'Aborted',
                                                    "goal_not_defined" : 'Aborted'})

            smach.StateMachine.add("SAY_GOAL_REACHED",
                                    Say(robot, [ "Yeah, I reached my goal"]),
                                    transitions={   'spoken':'Done'})

if __name__ == "__main__":
    rospy.init_node('avoid_that_exec')
    robot_smach_states.util.startup(AvoidThat)
