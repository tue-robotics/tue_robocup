#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_demo')
import rospy

import smach

from robot_skills.amigo import Amigo
import robot_smach_states as states

from util.startup import startup #gives you speech-exception reading-out-loud and smach_viewer server

from psi import Conjunction, Compound

class DemoChallenge(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/locations.pl'))
        robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/objects.pl'))
        #Assert the current challenge.
        robot.reasoner.assertz(Compound("challenge", "demo"))

        with self:
            smach.StateMachine.add( 'INITIALIZE', 
                                    states.Initialize(robot), 
                                    transitions={   'initialized':'ASK_OPEN_DOOR',
                                                    'abort':'Aborted'})



if __name__ == "__main__":
rospy.init_node('demo_chalenge_exec')

startup(DemoChallenge)