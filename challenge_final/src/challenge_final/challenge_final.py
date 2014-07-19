#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_final')
import rospy, sys

import smach

from robot_skills.reasoner  import Conjunction, Compound, Sequence
from wire_fitter.srv import *

import robot_smach_states as states

class FinalChallenge2014(smach.StateMachine):

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Failed', "Aborted"])
        self.robot = robot

        with self:
            smach.StateMachine.add('INITIALIZE',
                                states.Initialize(robot),
                                transitions={   'initialized':'DRIVE_TO_WAYPOINT_1',    ###### IN CASE NEXT STATE IS NOT "GO_TO_DOOR" SOMETHING IS SKIPPED
                                                'abort':'Aborted'})

            smach.StateMachine.add('DRIVE_TO_WAYPOINT_1', 
                                    states.NavigateGeneric(robot, goal_name="final_waypoint_1"),
                                    transitions={   'arrived':'WAIT_1', 
                                                    'preempted':'Failed', 
                                                    'unreachable':'Failed', 
                                                    'goal_not_defined':'Failed'})

            smach.StateMachine.add('WAIT_1', 
                                    states.Wait_time(robot, 3),
                                    transitions={   'waited':'DRIVE_TO_WAYPOINT_2', 
                                                    'preempted':'Aborted'})

            smach.StateMachine.add('DRIVE_TO_WAYPOINT_2',
                                    states.NavigateGeneric(robot, goal_name="final_waypoint_2"),
                                    transitions={   'arrived':'WAIT_2', 
                                                    'preempted':'Failed', 
                                                    'unreachable':'Failed', 
                                                    'goal_not_defined':'Failed'})

            smach.StateMachine.add('WAIT_2', 
                                    states.Wait_time(robot, 3),
                                    transitions={   'waited':'DRIVE_TO_WAYPOINT_3', 
                                                    'preempted':'Aborted'})

            smach.StateMachine.add('DRIVE_TO_WAYPOINT_3', 
                                    states.NavigateGeneric(robot, goal_name="final_waypoint_3"),
                                    transitions={   'arrived':'WAIT_3', 
                                                    'preempted':'Failed', 
                                                    'unreachable':'Failed', 
                                                    'goal_not_defined':'Failed'})

            smach.StateMachine.add('WAIT_3', 
                                    states.Wait_time(robot, 3),
                                    transitions={   'waited':'Done', 
                                                    'preempted':'Aborted'})


if __name__ == "__main__":
    rospy.init_node('exec_challenge_final_2014')

    states.util.startup(FinalChallenge2014)
