#! /usr/bin/env python

import rospy
import smach
import time
import os
import datetime
import math

from robot_smach_states import Initialize, Say, WaitTime
from robot_smach_states.util.startup import startup
import robot_smach_states.util.designators as ds

#from challenge_set_a_table_states import detect


class ChallengeSetATable(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        with self:
            smach.StateMachine.add('INITIALIZE',
                                   Initialize(robot),
                                   transitions={'initialized': 'ANNOUNCEMENT',
                                                'abort': 'Aborted'})
       
            # End
            smach.StateMachine.add('END_CHALLENGE',
                                   Say(robot, "My work here is done, goodbye!"),
                                   transitions={'spoken': 'Done'})

            ds.analyse_designators(self, "set_a_table")

if __name__ == "__main__":
    rospy.init_node('set_a_table_exec')

    startup(ChallengeSetATable, challenge_name="challenge_set_a_table")
