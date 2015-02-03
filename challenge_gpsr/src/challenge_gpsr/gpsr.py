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


        ### EXAMPLE SENTENCES stated in new conceptual rulebook.
        # Go to the bedroom, find a person and tell the time (missing object-interaction part).
        # Go to the kitchen, find a person and follow her (missing object-interaction part).
        # Go to the dinner-table, grasp the crackers, and take them to the TV.
        # Go to the shelf, count the drinks and report to me.

        # Take this object and bring it to Susan at the hall.
        # Bring a coke to the person in the living room and answer him a question.
        # Offer a drink to the person at the door.


        self.robot.speech.speak("Say it!")

        #spec = "(<action_go> to (a|an|the) <location>) | (<action_transport> (a|an|the) <object> to (a|an|the) <location>)"

        ## spec2: is able to retrieve the first 4 example sentences 
        spec2 = """
        (<1_action> to (((a|the) <1_locations_rest>)|((an|the) <1_locations_aeuoi>)|(the <1_locations_rooms>)), 
                ((<2_action_person> <2_person>) | (<2_action_count> the <2_objects_types_plural>) | (<2_action> ((the <2_objects_plural>) | (an <2_objects_aeuoi>) | (a <2_objects_rest_singular>))))
                    and (<3_action_special> | (<3_action> (them|it) to <3_location_person>) | (<3_action> to <3_location_person>) | (<3_action_person> to <3_person>))
        )"""

        objects_aeuoi = []
        objects_rest = ["beer", "orange juice", "milk", "sevenup"]
        objects = objects_aeuoi + objects_rest
        object_types_plural = ["drinks","snacks","cleaningstuff", "food"]
        object_types_singular = ["drink","snack"]
        object_types = object_types_plural + object_types_singular

        locations_aeuoi = ["appliance"]
        locations_rest = ["bed", "shelf", "table", "trashbin", "bin"]
        locations = locations_aeuoi + locations_rest
        rooms = ["kitchen", "hallway", "livingroom", "bedroom", "corridor"]
        persons = ["me","a person", "anna", "beth", "carmen", "jennifer", "jessica", 
                  "kimberly", "kristina", "laura", "mary", "sarah", "alfred", 
                  "charles", "daniel", "james", "john", "luis", "paul", 
                  "richard", "robert", "steve"]

        # choices = {"action_go" : ["go", "navigate", "move","advance"],
        #  "action_transport" : ["transport", "bring"],
        #  "location"  : [x for x in locations],
        #  "object": [x for x in objects]}

        choices2 = {"1_action":["go","navigate","move","advance"],
         "1_locations_aeuoi":locations_aeuoi,
         "1_locations_rest":locations_rest,
         "1_locations_rooms":rooms,

         "2_action_person": ["follow","find"],
         "2_person":persons,
         "2_action":["grasp","get","take","find"],
         "2_action_count":["count"],
         "2_objects_plural":objects + object_types_plural,
         "2_objects_types_plural":object_types_plural,
         "2_objects_aeuoi":objects_aeuoi,
         "2_objects_rest_singular":objects_rest + object_types_singular,

         "3_action_special":["tell the time","follow her","follow him"],
         "3_action":["take","move","bring",""],
         "3_location_person":locations + rooms + persons,
         "3_action_person":["report"],
         "3_person":persons}



        # spec = "<questions>"
        # choices = { "questions": [ x for x in QA_MAP ] }

        res = self.robot.ears.recognize(spec=spec2, choices=choices2)

        if not res:
            self.robot.speech.speak("I could not hear your question.")
            return "failed"

        try:
            print "jaja ", res
            #q_answer = QA_MAP[res.choices["questions"]]
            #q_answer = QA_MAP[res.result]
            self.robot.speech.speak(res.result)
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

