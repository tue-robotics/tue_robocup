#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_atomic_actions')
import rospy

import smach

from robot_skills.amigo import Amigo
import robot_smach_states as states

from robot_skills.reasoner  import Conjunction, Compound
from robot_smach_states.util.startup import startup

class AvoidThat(smach.StateMachine):

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])
        print "lol"


if __name__ == "__main__":
    rospy.init_node('avoid_that_exec')
    startup(AvoidThat)
