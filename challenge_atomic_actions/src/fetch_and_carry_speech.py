#!/usr/bin/python
import roslib; roslib.load_manifest('challenge_atomic_actions')
import rospy
#import robot_parts.speech
from std_msgs.msg import String
import geometry_msgs
import smach

from robot_skills.amigo import Amigo
import robot_smach_states as states
import robot_skills.util.msg_constructors as msgs

from robot_smach_states.util.startup import startup

from speech_interpreter.srv import AskUser # for speech_to_text only

from psi import *

###########################
# Created by: Erik Geerts #
###########################


class Ask_fetch_carry(smach.State):
    def __init__(self, robot, tracking=True, rate=2):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

        self.robot = robot
        self.preempted = False
        self.ask_user_service_fetch_carry = rospy.ServiceProxy('interpreter/ask_user', AskUser)

    def execute(self, userdata=None):

        # Here you can define how many times you want to try to listen and want the maximum duration is to listen to operator.
        self.response = self.ask_user_service_fetch_carry("fetch_carry", 10 , rospy.Duration(60))

        if self.response.keys[0] == "answer":
            response_answer = self.response.values[0]

        if response_answer:
            self.robot.reasoner.query(Compound("assertz", Compound("fetch_carry_object", response_answer)))
            rospy.loginfo("Object to fetch is: {0} ".format(response_answer))
            return "succeeded"
        else:
            return "failed"

########################
##### STATEMACHINE #####
########################


def setup_statemachine(robot):

    #retract old facts
    robot.reasoner.query(Compound("retractall", Compound("challenge", "X")))
    robot.reasoner.query(Compound("retractall", Compound("fetch_carry_object", "X")))

    #Load database
    robot.reasoner.query(Compound("load_database","tue_knowledge",'prolog/locations.pl'))
    robot.reasoner.query(Compound("load_database","tue_knowledge",'prolog/egpsr.pl'))

    #Assert the current challenge.
    robot.reasoner.query(Compound("assertz",Compound("challenge", "atomic_actions")))

    # Define arm used.    
    robot = Amigo()

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    with sm:
        # DURING A CHALLENGE, AMIGO STARTS AT A DESIGNATED POSITION, NOT IN FRONT OF A DOOR

        ######################################################
        ##################### INITIALIZE #####################             
        ######################################################

        smach.StateMachine.add('INITIALIZE',
                                states.Initialize(robot),
                                transitions={   'initialized':'ASK_FETCH_CARRY',    ###### IN CASE NEXT STATE IS NOT "GO_TO_DOOR" SOMETHING IS SKIPPED
                                                'abort':'Done'})


        ######################################################
        ################### ASK FETCH CARRY ##################            
        ######################################################

        smach.StateMachine.add("ASK_FETCH_CARRY",
                                Ask_fetch_carry(robot),
                                transitions={'succeeded':'Done',
                                             'failed':'Done'})

    return sm

if __name__ == "__main__":
    rospy.init_node('fetch_and_carry')
    rospy.loginfo("-------------------- FETCH AND CARRY ----------------------")
    rospy.loginfo("-----------------------------------------------------------")
    rospy.loginfo("------- MAKE SURE YOU ADDED THE DESIRED OBJECTS TO --------")
    rospy.loginfo("-------------- speech_interpreter.pl in map ---------------")
    rospy.loginfo("----------- tue_reasoner/tue_knowledge/prolog -------------")
    rospy.loginfo("-------- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -----------")
    rospy.loginfo("-----------------------------------------------------------")
         
    startup(setup_statemachine)
