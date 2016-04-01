#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_open_rgo_2014')
import rospy, sys

import smach

from robot_skills.reasoner  import Conjunction, Compound, Sequence

from math import cos, sin
from geometry_msgs.msg import *

import robot_smach_states as states

from speech_interpreter.srv import AskUser

class AskOpenChallenge(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["location_selected", "all_visited"])
        self.robot = robot
        self.ask_user_service = rospy.ServiceProxy('interpreter/ask_user', AskUser)

        self.locations = ["bar","bar","table"] #Go to the bar twice! The bar will be moved, tracked and then we must go there again, to its new location

    def execute(self, userdata=None):

	target="coke"

        #self.robot.base2.pc.constraint = 'x^2 + y^2 < 0.5385164807134505^2'
        #self.robot.base2.pc.constraint = 'x^2 + y^2 < 0.59^2'
        #self.robot.base2.pc.constraint = 'x^2 + y^2 < 0.59^2 and x^2 + y^2 > 0.49'
        self.robot.base2.pc.constraint = 'x^2 + y^2 < 0.59^2 and x^2 + y^2 > 0.30'
        self.robot.base2.pc.frame      = target

        self.robot.base2.oc.look_at    = Point()
        self.robot.base2.oc.angle_offset = -0.3805063771123649
        self.robot.base2.oc.frame      = target

        return "location_selected"

class AskAndNavigate(smach.StateMachine):
    def __init__(self, robot, turn_before_ask=True):
        smach.StateMachine.__init__(self, outcomes=["Done","Failed"])

        self.robot = robot

        with self:

            smach.StateMachine.add("ASK_OPENCHALLENGE",
                                    AskOpenChallenge(robot),
                                    transitions={'location_selected':   'INITIALIZE',
                                                 'all_visited':         'Done'})

            smach.StateMachine.add("INITIALIZE",
                                    states.ResetArmsSpindleHead(robot),
                                    transitions={'done'             :   'NAVIGATE_TO_TARGET'})

            smach.StateMachine.add("NAVIGATE_TO_TARGET",
                                    states.NavigateWithConstraints(robot),
                                    transitions={'arrived'          :   'SAY_ARRIVED',
                                                 'unreachable'      :   'SAY_UNREACHABLE',
                                                 'goal_not_defined' :   'SAY_UNDEFINED'})              

            smach.StateMachine.add( "SAY_ARRIVED",
                                    states.Say(robot, ["Hey, I reached my goal."]),
                                    transitions={"spoken":"Done"})          

            smach.StateMachine.add( "SAY_UNREACHABLE",
                                    states.Say(robot, ["I can't reach the goal you asked me to go to"]),
                                    transitions={"spoken":"Failed"})          

            smach.StateMachine.add( "SAY_UNDEFINED",
                                    states.Say(robot, ["I can't reach the location you asked me to go to."]),
                                    transitions={"spoken":"Failed"})

#######################################################################################################################################################################################################

class FinalRgo2014(smach.StateMachine):

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Failed'])
        self.robot = robot

        side = robot.leftArm

        query = Conjunction( Compound("property_expected", "ObjectID", "class_label", "coke"),
                             Compound("property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

        with self:
            
            smach.StateMachine.add( "ASK_AND_NAV", #User asks to go to the table
                                    AskAndNavigate(robot),
                                    transitions={   "Done"      :"GRAB_OBJECT", 
                                                    "Failed"    :"Failed"}) 
            @smach.cb_interface(outcomes=["done"])
            def look_down(*args, **kwargs):
                robot.head.reset()
                return "done"

            smach.StateMachine.add( "LOOK_FOR_DRINK",
                                    smach.CBState(look_down), 
                                    transitions={   'done'      :'GRAB_OBJECT'})
            
            smach.StateMachine.add( "GRAB_OBJECT",
                                    states.GrabMachineWithoutBase(side, robot, query),
                                    transitions={   'succeeded' :'Done',
                                                    'failed'    :'Failed' })


if __name__ == "__main__":
    rospy.init_node('open_challenge_2014')
    states.util.startup(FinalRgo2014)
