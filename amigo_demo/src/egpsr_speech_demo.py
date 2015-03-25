#!/usr/bin/python
import roslib; roslib.load_manifest('challenge_egpsr')
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

#######################
##### TODO LIST!! #####
## updated 15-4-2013 ##
#######################

# - Add all kinds of objects in the reasoner
# - While executing a task, failure handling is very weak. This should be optimized.

# - Placing objects on tables/etc at transport.

#######################
##### TODO LIST!! #####
#### NA EINDHOVEN  ####
#######################

# - andere microfoon bekijken
# - speech -> geen actie in stappen. Direct verstaan.
# - dropoff points in eindhoven definieren.
# - Navigate Generic
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

#################
### Questions ###
#################
# - See questions on top of every action in the statemachine
# - Is AMIGO easily able to put something down? I haven't seen him doing this. 
# - In case amigo is unable to say something, then class Say_and_Navigate will get into a deadlock. 
#   has this ever happened before (that amigo is not able to say something)?


class Ask_action(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done", "no_action"])

        self.robot = robot
        self.ask_user_service_get_action = rospy.ServiceProxy('interpreter/ask_user', AskUser)

    def execute(self, userdata):

        self.robot.head.look_at_standing_person()

        try:
            self.response = self.ask_user_service_get_action("action", 1 , rospy.Duration(3600))  # = 1 hour because amigo has to be on standby to receive an action in e-gpsr
            
            for x in range(0,len(self.response.keys)):
                if self.response.keys[x] == "action":
                    response_action = self.response.values[x]
                elif self.response.keys[x] == "start_location":
                    response_start_location = self.response.values[x]
                elif self.response.keys[x] == "end_location":
                    response_end_location = self.response.values[x]
                elif self.response.keys[x] == "object":
                    response_object = self.response.values[x]
                elif self.response.keys[x] == "object_room":
                    response_object_room = self.response.values[x]
                elif self.response.keys[x] == "object_location":
                    response_object_location = self.response.values[x]

            self.robot.reasoner.query(Compound("assertz", Compound("goal", response_action, response_start_location, response_end_location, response_object, response_object_room, response_object_location)))
            
            if response_object == "no_answer" or response_object == "wrong_answer":
                return "no_action"
            # Show values for action/start_location/end_location/object      
            rospy.loginfo("action = {0}".format(response_action))
            rospy.loginfo("start_location = {0}".format(response_start_location))
            rospy.loginfo("end_location = {0}".format(response_end_location))
            rospy.loginfo("object = {0}".format(response_object))
            rospy.loginfo("object_room = {0}".format(response_object_room))
            rospy.loginfo("object_location = {0}".format(response_object_location))

            return "done"

        except rospy.ServiceException, e:

            rospy.loginfo("No action is heared")
            #rospy.loginfo("Service call failed ({0})".format(e))

            return "no_action"

class Query_specific_action(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["action_get", "action_transport","action_point","action_find","action_navigate","action_leave","error"])

        self.robot = robot
        self.preempted = False

    def execute(self, userdata):

        raw_goals = self.robot.reasoner.query(Compound("goal", "Action","Loc_from","Loc_to","Object_action","Object_room","Object_location"))
        
        goals = [(answer["Action"], answer["Loc_from"], answer["Loc_to"], answer["Object_action"],answer["Object_room"],answer["Object_location"]) for answer in raw_goals]       
        action,loc_from,loc_to,object_action,object_room,object_location = min(goals)

        action = action.get_string()
        loc_from = loc_from.get_string()
        loc_to = loc_to.get_string()
        object_action = object_action.get_string()
        object_room = object_room.get_string()
        object_location = object_location.get_string()

        if action == 'get':
            return "action_get"
        elif action == 'transport':
            return "action_transport"
        elif action == 'point':
            return "action_point"
        elif action == 'find':
            return "action_find"
        elif action == 'navigate':
            return "action_navigate"
        elif action == 'leave':
            return "action_leave"
        else:
            return "error"


########################
##### STATEMACHINE #####
########################


def setup_statemachine(robot):

    #retract old facts
    robot.reasoner.query(Compound("retractall", Compound("challenge", "X")))
    robot.reasoner.query(Compound("retractall", Compound("goal", "U","V","W", "X", "Y", "Z")))
    robot.reasoner.query(Compound("retractall", Compound("explored", "X")))
    robot.reasoner.query(Compound("retractall", Compound("state", "X", "Y")))
    robot.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X")))
    robot.reasoner.query(Compound("retractall", Compound("current_object", "X")))
    robot.reasoner.query(Compound("retractall", Compound("visited", "X")))
    robot.reasoner.query(Compound("retractall", Compound("unreachable", "X")))
    robot.reasoner.query(Compound("retractall", Compound("disposed", "X")))
    robot.reasoner.query(Compound("retractall", Compound("point_roi_tried", "X")))   

    robot.reasoner.query(Compound("retractall", Compound("tasks_done", "X")))
    robot.reasoner.query(Compound("retractall", Compound("tasks_max", "X")))
    robot.reasoner.query(Compound("retractall", Compound("tasks_failed", "X")))
    
    #Load database
    robot.reasoner.query(Compound("load_database","tue_knowledge",'prolog/locations.pl'))
    robot.reasoner.query(Compound("load_database","tue_knowledge",'prolog/egpsr.pl'))

    #Assert the current challenge.
    robot.reasoner.query(Compound("assertz",Compound("challenge", "egpsr")))
    robot.reasoner.query(Compound("assertz",Compound("tasks_done", "0.0")))
    robot.reasoner.query(Compound("assertz",Compound("tasks_max", "20.0")))  # Define how many tasks you want to perform

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    with sm:
        

        ######################################################
        ##################### INITIALIZE #####################             
        ######################################################

        smach.StateMachine.add('INITIALIZE',
                                states.Initialize(robot),
                                transitions={   'initialized':'INTRODUCE_SHORT',    ###### IN CASE NEXT STATE IS NOT "GO_TO_DOOR" SOMETHING IS SKIPPED
                                                'abort':'Aborted'})


        ######################################################
        #################### DEMO Version ####################             
        ######################################################


        smach.StateMachine.add("INTRODUCE_SHORT",
                               states.Say(robot,"Hi! I will just wait here and wonder if I can do something for you", block=True),
                               transitions={'spoken':'ASK_ACTION'})

        smach.StateMachine.add("DEMO_SENTENCE",
                               states.Say(robot,"Although I know what to do, for now I am just showing that I am able to understand you!", block=True),
                               transitions={'spoken':'ASK_ACTION'})

        smach.StateMachine.add("ASK_ACTION",
                                Ask_action(robot),
                                transitions={'done':'DEMO_SENTENCE',
                                             'no_action':'ASK_ACTION'})

    return sm

if __name__ == "__main__":
    rospy.init_node('gpsr_exec')
    rospy.loginfo("----------------------------- EGPSR -------------------------------")
    rospy.loginfo("----- See README_SPEECH_POSSIBILITIES for input possibilities -----")
    rospy.loginfo("- If you are in another location, set map in rsettings to rwc2013 -")
    rospy.loginfo("-------------------------------------------------------------------")
    startup(setup_statemachine)
