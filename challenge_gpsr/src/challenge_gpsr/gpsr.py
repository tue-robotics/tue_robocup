#!/usr/bin/python
import roslib; roslib.load_manifest('challenge_gpsr')
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
        smach.State.__init__(self, outcomes=["done", "failed"])
        self.robot = robot

    def execute(self, userdata):

        #self.robot.head.look_up()

        # spec = '<actionCategory> (an|a|some) <objectCategory> (from|to) (an|a|some) <locationCategory1> (from|to) (an|a|some) <locationCategory2>'

        # {"locationCategory1": ["door","shelf","seating", "table", "trashbin","appliance"], 
        #  "locationCategory2": ["door","shelf","seating", "table", "trashbin","appliance"], 
        #  "actionCategory": ["bring","carry","get", "give", "move"], 
        #  "objectCategory" : ["drink", "food", "tool", "bathroomstuff", "snack"]}


        # spec = '<actionCategory> (an|a|some) <objectCategory> (from|to) (an|a|some) <locationCategory1> (from|to) (an|a|some) <locationCategory2>'


        ### SENTENCES stated in new conceptual rulebook.
        # Go to the bedroom, find a person and tell the time (missing object-interaction part).
        # Go to the kitchen, find a person and follow her (missing object-interaction part).
        # Go to the dinner-table, grasp the crackers, and take them to the TV.
        # Go to the shelf, count the drinks and report to me.
        # Take this object and bring it to Susan at the hall.
        # Bring a coke to the person in the living room and answer him a question.
        # Offer a drink to the person at the door.


        self.robot.speech.speak("Say it!")

        spec = "(<action_go> to (a|an|the) <location>) | (<action_transport> (a|an|the) <object> to (a|an|the) <location>)"

        locations = ["bed", "shelf", "table", "trashbin", "bin", "appliance"]
        objects = ["beer", "orange_juice", "milk", "sevenup"]
        rooms = ["kitchen", "hallway", "livingroom", "bedroom", "corridor"]

        choices = {"action_go" : ["go", "navigate"],
         "action_transport" : ["transport", "bring"],
         "location"  : [x for x in locations],
         "object": [x for x in objects]}


        # spec = "<questions>"
        # choices = { "questions": [ x for x in QA_MAP ] }

        res = self.robot.ears.recognize(spec=spec, choices=choices)

        if not res:
            self.robot.speech.speak("I could not hear your question.")
            return "failed"

        try:
            print "jaja ", res
            #q_answer = QA_MAP[res.choices["questions"]]
            #q_answer = QA_MAP[res.result]
        except KeyError:
            print "[what_did_you_say] Received question is not in map. THIS SHOULD NEVER HAPPEN!"
            return "failed"

        print res

        #self.robot.speech.speak("Your question was: " + res.choices["questions"] + ". The answer is: " + q_answer)

        return "done"



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
                                transitions={   'initialized':'ASK_ACTION',    ###### IN CASE NEXT STATE IS NOT "GO_TO_DOOR" SOMETHING IS SKIPPED
                                                'abort':'Aborted'})


        ######################################################
        #################### INSTRUCTIONS ####################             
        ######################################################


        smach.StateMachine.add("INTRODUCE_SHORT",
                               states.Say(robot,"Hi! I will just wait here and wonder if I can do something for you", block=False),
                               transitions={'spoken':'ASK_ACTION'})

        smach.StateMachine.add("ASK_ACTION",
                                Ask_action(robot),
                                transitions={'done':'ASK_ACTION',
                                             'failed':'ASK_ACTION'})


    return sm

if __name__ == "__main__":
    rospy.init_node('gpsr_exec')
    rospy.loginfo("-------------------------- GPSR --------------------------")
    rospy.loginfo("- See README_SPEECH_POSSIBILITIES for input possibilities -")
    rospy.loginfo("----------------------------------------------------------")
    startup(setup_statemachine)

