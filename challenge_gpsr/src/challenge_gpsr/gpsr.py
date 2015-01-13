#!/usr/bin/python
import roslib; roslib.load_manifest('challenge_egpsr')
import rospy
#import robot_parts.speech
from std_msgs.msg import String
import geometry_msgs
import smach
import sys

from robot_skills.amigo import Amigo
import robot_smach_states as states
import robot_skills.util.msg_constructors as msgs

from robot_smach_states.util.startup import startup

###########################
# Created by: Erik Geerts #
###########################

#######################
##### TODO LIST!! #####
## updated 21-4-2013 ##
#######################


#######################
##### TODO LIST!! #####
#### NA EINDHOVEN  ####
#######################

# - dropoff points in eindhoven definieren.
# - remove timeout of 5 minutes -> DID YOU SAY SOMETHING, IN ANY CASE, I DID NOT HEAR YOU!

##########################################
############## What to run: ##############
############ updated 15-4-2013 ###########
##########################################
# - see README file

#############################################################
## Locations that must be defined in database on forehand: ##
##################### updated 15-4-2013 #####################
#############################################################
# - initial
# - meeting_point
# - exit_1
# - exit_2

############################
### Action possibilities ###
#### updated 15-4-2013 #####
############################

# See /challenge_egpsr/input_speech_not_used/sentences.corpus for available sentences to say during questioning.
# Available locations and objects can be found in /challenge_egpsr/input_speech_not_used/tue_test_lab/

# If speech files for tue_test_lab are used ONLY DRINKS AND BATHROOMSTUFF can be questioned at this point!



class Ask_action(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done", "no_action"])

        self.robot = robot
        self.ask_user_service_get_action = rospy.ServiceProxy('interpreter/ask_user', AskUser)

    def execute(self, userdata):

        self.robot.head.look_up()




        # spec = '<actionCategory> (an|a|some) <objectCategory> (from|to) (an|a|some) <locationCategory1> (from|to) (an|a|some) <locationCategory2>'

        # {"locationCategory1": ["door","shelf","seating", "table", "trashbin","appliance"], 
        #  "locationCategory2": ["door","shelf","seating", "table", "trashbin","appliance"], 
        #  "actionCategory": ["bring","carry","get", "give", "move"], 
        #  "objectCategory" : ["drink", "food", "tool", "bathroomstuff", "snack"]}


        # spec = '<actionCategory> (an|a|some) <objectCategory> (from|to) (an|a|some) <locationCategory1> (from|to) (an|a|some) <locationCategory2>'



        # Go to the bedroom, find a person and tell the time (missing object-interaction part).
        # Go to the kitchen, find a person and follow her (missing object-interaction part).
        # Go to the dinner-table, grasp the crackers, and take them to the TV.
        # Go to the shelf, count the drinks and report to me.
        # Take this object and bring it to Susan at the hall.
        # Bring a coke to the person in the living room and answer him a question.
        # Offer a drink to the person at the door.


        spec = '(<action_go> to (a|an|the) <location>)|
                (<action_) |'


        locations = ["rooms", 
        rooms = ["kitchen", "hallway", "livingroom", "bedroom", "corridor"]
        {"action_go" : ["go", "navigate"],
         "location"  : [[x for x in locations], [ x for x in rooms] ]



########################
##### STATEMACHINE #####
########################


def setup_statemachine(robot):

    # Define arm used.    
    robot = Amigo()
    arm = rospy.get_param('~arm', 'left')
    if arm == 'left':
        selectedArm = robot.leftArm
    else:
        selectedArm = robot.rightArm


    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    with sm:
        # DURING A CHALLENGE, AMIGO STARTS AT A DESIGNATED POSITION, NOT IN FRONT OF A DOOR

        ######################################################
        ##################### INITIALIZE #####################             
        ######################################################

        smach.StateMachine.add('INITIALIZE',
                                states.Initialize(robot),
                                transitions={   'initialized':'INTRODUCE_SHORT',    ###### IN CASE NEXT STATE IS NOT "GO_TO_DOOR" SOMETHING IS SKIPPED
                                                'abort':'Aborted'})


        ######################################################
        #################### INSTRUCTIONS ####################             
        ######################################################


        smach.StateMachine.add("INTRODUCE_SHORT",
                               states.Say(robot,"Hi! I will just wait here and wonder if I can do something for you", block=False),
                               transitions={'spoken':'INIT_POSE'})

        smach.StateMachine.add("ASK_ACTION",
                                Ask_action(robot),
                                transitions={'done':'RESET_HEAD_SPINDLE',
                                             'no_action':'ASK_ACTION'})

        smach.StateMachine.add("GIVE_ACTION_WITHOUT_MIC",
                                Ask_action_without_mic(robot),
                                transitions={'done':'RESET_HEAD_SPINDLE',
                                             'no_action':'INTRODUCE_SHORT'})

        smach.StateMachine.add("RESET_HEAD_SPINDLE",
                                states.ResetHeadSpindle(robot),
                                transitions={'done':'INTRODUCE_SHORT'})


    return sm

if __name__ == "__main__":
    rospy.init_node('gpsr_exec')
    rospy.loginfo("-------------------------- GPSR --------------------------")
    rospy.loginfo("- See README_SPEECH_POSSIBILITIES for input possibilities -")
    rospy.loginfo("----------------------------------------------------------")
    startup(setup_statemachine)

