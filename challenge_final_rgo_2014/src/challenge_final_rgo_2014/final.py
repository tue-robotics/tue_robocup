#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_final_rgo_2014')
import rospy, sys

import smach

from robot_skills.arms import State as ArmState
from robot_skills.reasoner  import Conjunction, Compound, Sequence

from math import cos, sin
from geometry_msgs.msg import *

import robot_smach_states as states

from speech_interpreter.srv import AskUser

class TurnAround(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        self.robot = robot

    def execute(self, userdata=None):
        b = self.robot.base
        #b.force_drive(0.25, 0, 0, 3)
        b.force_drive(0, 0, 0.5, 6.28) #turn yourself around, 0.5*2PI rads = 1 pi rads = 180 degrees
        #b.force_drive(-0.25, 0, 0, 3)
        return "Done"

class AskOpenChallenge(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["location_selected", "all_visited"])
        self.robot = robot
        self.ask_user_service = rospy.ServiceProxy('interpreter/ask_user', AskUser)

        self.locations = ["bar","bar","table"] #Go to the bar twice! The bar will be moved, tracked and then we must go there again, to its new location

    def execute(self, userdata=None):

        self.robot.head.look_up()
        
        try:
            self.response = self.ask_user_service("challenge_open_2014", 4 , rospy.Duration(18))  # This means that within 4 tries and within 60 seconds an answer is received. 

            #self.response is an object with 2 members. It can be represented as a dict, but isn't. Here we make it a dict ourselves
            response_dict = dict(zip(self.response.keys, self.response.values)) 
            response_answer = response_dict.get("answer", "no_answer")

            if response_answer in ["no_answer", "wrong_answer", ""]: #If response answer is one to these things:...
                if self.locations:
                    target = self.locations.pop(0) #Get the first item from the list
                    self.robot.speech.speak("I was not able to understand you but I'll drive to the %s."%target)
                else:
                    return "all_visited"
            else:
                target = response_answer

        except Exception, e:
            rospy.logerr(e)
            target = "table"
            self.robot.speech.speak("There is something wrong with my ears, I will go to %s"%target)

        self.robot.base2.pc.constraint = 'x^2 + y^2 < 1.2^2'
        self.robot.base2.pc.frame      = target

        self.robot.base2.oc.look_at    = Point()
        self.robot.base2.oc.frame      = target

        return "location_selected"

class AskAndNavigate(smach.StateMachine):
    def __init__(self, robot, turn_before_ask=False):
        smach.StateMachine.__init__(self, outcomes=["Done","Failed"])

        self.robot = robot

        with self:
            if turn_before_ask:
                smach.StateMachine.add( "TURN_AROUND",
                                    TurnAround(robot),
                                    transitions={   "Done"      :"ASK_OPENCHALLENGE", 
                                                    "Aborted"   :"ASK_OPENCHALLENGE", 
                                                    "Failed"    :"ASK_OPENCHALLENGE"})

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

        object_query = Conjunction(
                                    Compound( "property_expected", "ObjectID", "class_label", "Anything"), #TODO: Specify class?
                                    Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                    Compound( "property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")))

        with self:
            smach.StateMachine.add( "ASK_AND_NAV_1", #User must say bar
                                    AskAndNavigate(robot),
                                    transitions={   "Done"      :"ASK_AND_NAV_2", 
                                                    "Failed"    :"ASK_AND_NAV_2"})
            #Amigo arrived at the bar. 
            smach.StateMachine.add( "ASK_AND_NAV_2", #The bar is moved and amigo is asked to go to the bar once more, after it moved and was tracked in the WM
                                    AskAndNavigate(robot),
                                    transitions={   "Done"      :"ASK_AND_NAV_3", 
                                                    "Failed"    :"ASK_AND_NAV_3"}) 
            
            smach.StateMachine.add( "ASK_AND_NAV_3", #User asks to go to the table
                                    AskAndNavigate(robot),
                                    transitions={   "Done"      :"LOOK_FOR_DRINK", 
                                                    "Failed"    :"SAY_FAILED"})        

            smach.StateMachine.add( "SAY_FAILED",
                                    states.Say(robot, ["I don't know where that thing went, it was just right here!"]),
                                    transitions={"spoken":"Failed"}) 
            
            @smach.cb_interface(outcomes=["done"])
            def look_down(*args, **kwargs):
                robot.head.look_down()
                return "done"
            smach.StateMachine.add( "LOOK_FOR_DRINK",
                                    smach.CBState(look_down), 
                                    transitions={   'done'      :'GRAB_OBJECT'})
            
            smach.StateMachine.add( "GRAB_OBJECT",
                                    states.GrabMachine(side, robot, object_query),
                                    transitions={   'succeeded' :'SET_PARAMS',
                                                    'failed'    :'Failed' })

            @smach.cb_interface(outcomes=["done"])
            def set_nav_constraints(*args, **kwargs):
                self.robot.base2.pc.constraint = 'x^2 + y^2 < 0.45^2' #In the pose defined in ARM_TO_DROPPOS, the object is (about) 0.45m from the center of the base. 
                self.robot.base2.pc.frame      = target

                self.robot.base2.oc.look_at    = Point()
                self.robot.base2.oc.frame      = target
                return "done"

            smach.StateMachine.add( "SET_PARAMS",
                                    smach.CBState(set_nav_constraints), 
                                    transitions={   'done'      :'NAVIGATE_TO_TRASHBIN'})

            smach.StateMachine.add("NAVIGATE_TO_TRASHBIN",
                                    states.NavigateWithConstraints(robot),
                                    transitions={'arrived'          :   'ARM_TO_DROPPOS',
                                                 'unreachable'      :   'SAY_UNREACHABLE',
                                                 'goal_not_defined' :   'SAY_UNDEFINED'})        

            smach.StateMachine.add( "SAY_UNREACHABLE",
                                    states.Say(robot, ["I can't reach the location I was supposed to go to"]),
                                    transitions={"spoken":"Failed"})        

            smach.StateMachine.add( "SAY_UNDEFINED",
                                    states.Say(robot, ["I don't know where to go, sorry"]),
                                    transitions={"spoken":"Failed"}) 

            smach.StateMachine.add("ARM_TO_DROPPOS",
                                    states.ArmToJointPos(robot, side, [-0.100, 0.400, 0.600, 1.130, -0.300, 0.100, 0.000], timeout=4),
                                    transitions={'done'             :'OPEN_GRIPPER',
                                                 'failed'           :'SAY_CANNOT_DROP' })       

            smach.StateMachine.add( "SAY_CANNOT_DROP",
                                    states.Say(robot, ["I can't drop the object, sorry"]),
                                    transitions={"spoken":"TURN_AROUND"}) 

            smach.StateMachine.add( "OPEN_GRIPPER",
                                    states.SetGripper(robot, side, gripperstate=ArmState.OPEN),
                                    transitions={   'succeeded'         :'TURN_AROUND',
                                                    'failed'            :'TURN_AROUND' })

            smach.StateMachine.add( "TURN_AROUND",
                    TurnAround(robot),
                    transitions={   "Done"      :"SAY_DONE", 
                                    "Aborted"   :"SAY_DONE", 
                                    "Failed"    :"SAY_DONE"})            

            smach.StateMachine.add( "SAY_DONE",
                                    states.Say(robot, ["Thats all folks"]),
                                    transitions={"spoken":"Done"}) 


if __name__ == "__main__":
    rospy.init_node('open_challenge_2014')
    states.util.startup(FinalRgo2014)
