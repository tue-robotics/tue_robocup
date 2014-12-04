#!/usr/bin/python
import roslib;
import rospy
import smach

from robot_skills.amigo import Amigo
from robot_skills.sergio import Sergio
import robot_smach_states as states
from robot_smach_states.util.startup import startup
from geometry_msgs.msg import Point

from robot_smach_states.navigation import NavigateToWaypoint
from robot_smach_states.navigation import NavigateToObserve

from speech_interpreter.srv import AskUser

from psi import *
import robot_skills.util.msg_constructors as msgs

import sys

from robot_smach_states.designators.designator import Designator, VariableDesignator


###########################
# Created by: Erik Geerts #
###########################

##########################################
############## What to run: ##############
##########################################
# - see README file

#############################################################
## Locations that must be defined in database on forehand: ##
##################### updated 16-4-2013 #####################
#############################################################
# - initial
# - registration_table
# - exit

##########################################
############### TODO list: ###############
##########################################
# -  Instead of having a goal pose, a goal area should be enough, so that it will drive as close as possible to the goal position.
#    In this challenge this would be handy, since amigo will then introduce itself near the table instead of outside the door.
# -  In case an obstacle is detected, altough it is not there and amigo is not looking in that direction, a path cannot be found
#    if this 'obstacle' is blocking the way. A solution might be that Amigo is forced to look in the direction of the goal pose
#    if no solution is found at the first try.


#class AmigoIntroductionRIPS(smach.State):
#    def __init__(self, robot=None):
#        smach.State.__init__(self, outcomes=['finished'])
#
#        self.robot = robot
#
#    def execute(self, userdata):
#        rospy.loginfo("Introducing AMIGO")
#
#        self.robot.head.reset_position()
#
#        self.robot.speech.speak("Hello, my name is amigo")
#        rospy.sleep(1.0)
#        self.robot.speech.speak("I am participating in robocup 2014 on behalf of Tech United Eindhoven")
#        self.robot.speech.speak("If you want me to stop, you can press my emergency button on my back")
#        self.robot.speech.speak("Thank you for your attention, I will now leave the arena")
#
#        return 'finished'

#TODO after RWC2014: Move this to the robot_smach_states library
class Ask_continue(smach.State):
    def __init__(self, robot, timeout=60):
        smach.State.__init__(self, outcomes=["done", "no_continue"])

        self.robot = robot
        self.timeout = timeout
        self.ask_user_service_continue = rospy.ServiceProxy('interpreter/ask_user', AskUser)

    def execute(self, userdata):
        response = self.ask_user_service_continue("continue", 1 , rospy.Duration(self.timeout))
        response_dict = dict(zip(response.keys, response.values)) #Turn the list of values and keys into a list of (key, value) tuples and convert that to a dictionary
        if response_dict.has_key("answer") and response_dict["answer"] == "answer":
            return "done"
        else:
            return "no_continue"

        rospy.loginfo("answer was not found in response of interpreter. Should not happen!!")
        return "no_continue"


def setup_statemachine(robot):

    #retract old facts
    #TODO: maybe retract more facts like other challenges?
    robot.reasoner.query(Compound("retractall", Compound("challenge", "X")))

    #Load database
    robot.reasoner.query(Compound("load_database","tue_knowledge",'prolog/locations.pl'))

    #Assert the current challenge.
    robot.reasoner.query(Compound("assertz",Compound("challenge", "registration")))

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    with sm:

        #smach.StateMachine.add('INITIALIZE_FIRST',
        #                        states.Initialize(robot),
        #                        transitions={   'initialized':'START_CHALLENGE_ROBUST',
        #                                        'abort':'Aborted'})


        # Start challenge via StartChallengeRobust
        smach.StateMachine.add( "START_CHALLENGE_ROBUST",
                                    states.StartChallengeRobust(robot, "initial_pose", use_entry_points = True),
                                    transitions={   "Done":"GO_TO_INTERMEDIATE_WAYPOINT",
                                                    "Aborted":"GO_TO_INTERMEDIATE_WAYPOINT",
                                                    "Failed":"GO_TO_INTERMEDIATE_WAYPOINT"})   # There is no transition to Failed in StartChallengeRobust (28 May)

        #smach.StateMachine.add("SAY_START_CHALLENGE",
        #                            states.Say(robot, "Hello, I am Amigo, human cyborg relations", block=False),
        #                            transitions={   "spoken":"GO_TO_INTERMEDIATE_WAYPOINT"})

        smach.StateMachine.add('GO_TO_INTERMEDIATE_WAYPOINT',
                                    states.NavigateToObserve(robot, Designator("dinner_table"), radius=0.7),
                                    transitions={   'arrived':'SAY_AWAIT_CONTINUE',
                                                    'unreachable':'SAY_AWAIT_CONTINUE',
                                                    'goal_not_defined':'SAY_AWAIT_CONTINUE'})

        smach.StateMachine.add( 'SAY_AWAIT_CONTINUE',
                                states.Say(robot, ["I'll pause here until you say continue", "I'm waiting for you to say continue"]),
                                transitions={'spoken':'ASK_CONTINUE_0'})

        smach.StateMachine.add("ASK_CONTINUE_0",
                        Ask_continue(robot),
                        transitions={   'done':'SAY_CONTINUEING',
                                        'no_continue':'SAY_CONTINUEING'})

        smach.StateMachine.add( 'SAY_CONTINUEING',
                                states.Say(robot, ["I heard continue, so I will move to the exit now. See you guys later!"], block=False),
                                transitions={'spoken':'GO_TO_EXIT'})

        # Amigo goes to the exit (waypoint stated in knowledge base)
        smach.StateMachine.add('GO_TO_EXIT',
                                    states.NavigateToWaypoint(robot, Designator("exit_1_rips"), radius = 0.5),
                                    transitions={   'arrived':'AT_END',
                                                    'unreachable':'AT_END',
                                                    'goal_not_defined':'AT_END'})

        # Finally amigo will stop and says 'goodbye' to show that he's done.
        smach.StateMachine.add('AT_END',
                                states.Say(robot, "Goodbye"),
                                transitions={'spoken':'Done'})
    return sm


############################## initializing program ##############################
if __name__ == '__main__':
    rospy.init_node('rips_exec')

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "[CHALLENGE RIPS] Please provide robot name as argument."
        exit(1)

    startup(setup_statemachine, robot_name=robot_name)
