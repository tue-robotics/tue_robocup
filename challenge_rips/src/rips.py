#!/usr/bin/python
import roslib; roslib.load_manifest('challenge_rips')
import rospy
import smach

from robot_skills.amigo import Amigo
import robot_smach_states as states
from robot_smach_states.util.startup import startup
from geometry_msgs.msg import Point

from speech_interpreter.srv import AskUser

from psi import *
import robot_skills.util.msg_constructors as msgs


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
                                    states.StartChallengeRobust(robot, "initial_exit"), 
                                    transitions={   "Done":"GO_TO_INTERMEDIATE_WAYPOINT", 
                                                    "Aborted":"GO_TO_INTERMEDIATE_WAYPOINT", 
                                                    "Failed":"GO_TO_INTERMEDIATE_WAYPOINT"})   # There is no transition to Failed in StartChallengeRobust (28 May)

        #smach.StateMachine.add("SAY_START_CHALLENGE",
        #                            states.Say(robot, "Hello, I am Amigo, human cyborg relations", block=False),
        #                            transitions={   "spoken":"GO_TO_INTERMEDIATE_WAYPOINT"})

        smach.StateMachine.add('GO_TO_INTERMEDIATE_WAYPOINT', 
                                    states.NavigateGeneric(robot, goal_name="registration_table1"),
                                    transitions={   'arrived':'GO_TO_EXIT', 
                                                    'preempted':'CLEAR_PATH_TO_INTERMEDIATE_WAYPOINT', 
                                                    'unreachable':'CLEAR_PATH_TO_INTERMEDIATE_WAYPOINT', 
                                                    'goal_not_defined':'CLEAR_PATH_TO_INTERMEDIATE_WAYPOINT'})

        # Amigo will say that it arrives at the intermediate waypoint table
        smach.StateMachine.add('CLEAR_PATH_TO_INTERMEDIATE_WAYPOINT',
                                    states.Say(robot, "Please clear the path, so that I can find my way."),
                                    transitions={'spoken':'GO_TO_INTERMEDIATE_WAYPOINT_SECOND_TRY'}) 

        # Then amigo will drive to the intermediate waypoint. Defined in knowledge base. Now it is the table in the test map.
        smach.StateMachine.add('GO_TO_INTERMEDIATE_WAYPOINT_SECOND_TRY', 
                                    states.NavigateGeneric(robot, goal_name="registration_table1", goal_area_radius=0.5), # within 1m of the target
                                    transitions={   'arrived':'GO_TO_EXIT', 
                                                    'preempted':'GO_TO_INTERMEDIATE_WAYPOINT_THIRD_TRY', 
                                                    'unreachable':'GO_TO_INTERMEDIATE_WAYPOINT_THIRD_TRY', 
                                                    'goal_not_defined':'GO_TO_INTERMEDIATE_WAYPOINT_THIRD_TRY'})

        # Then amigo will drive to the intermediate waypoint. Defined in knowledge base. Now it is the table in the test map.
        smach.StateMachine.add('GO_TO_INTERMEDIATE_WAYPOINT_THIRD_TRY', 
                                    states.NavigateGeneric(robot, goal_name="registration_table2", goal_area_radius=0.5), # within 1m of the target
                                    transitions={   'arrived':'GO_TO_EXIT', 
                                                    'preempted':'GO_TO_INTERMEDIATE_WAYPOINT_FORTH_TRY', 
                                                    'unreachable':'GO_TO_INTERMEDIATE_WAYPOINT_FORTH_TRY', 
                                                    'goal_not_defined':'GO_TO_INTERMEDIATE_WAYPOINT_FORTH_TRY'})

        # Then amigo will drive to the intermediate waypoint. Defined in knowledge base. Now it is the table in the test map.
        smach.StateMachine.add('GO_TO_INTERMEDIATE_WAYPOINT_FORTH_TRY', 
                                    states.NavigateGeneric(robot, goal_name="registration_table3", goal_area_radius=0.5), # within 1m of the target
                                    transitions={   'arrived':'GO_TO_EXIT', 
                                                    'preempted':'GO_TO_EXIT', 
                                                    'unreachable':'GO_TO_EXIT', 
                                                    'goal_not_defined':'GO_TO_EXIT'})

        # Amigo will say that it arrives at the intermediate waypoint
        #smach.StateMachine.add('ARRIVED_AT_INTERMEDIATE_WAYPOINT',
        #                            states.Say(robot, "I'm at the intermediate waypoint, I will now introduce myself"),
        #                            transitions={'spoken':'INTRODUCE_AMIGO'}) 

        # In case the path is blocked, amigo will say that it still introduce itself.
        #smach.StateMachine.add('FAIL_BUT_INTRODUCE',
        #                            states.Say(robot, "I stil couldn't get to the intermediate waypoint, I will now introduce myself"),
        #                            transitions={'spoken':'INTRODUCE_AMIGO'}) 

        # It will start the introduction (MAKE SURE THAT THE FULL INTRODUCTION IS PLAYED DURING COMPETITION!!!))
        #smach.StateMachine.add('INTRODUCE_AMIGO', 
        #                            AmigoIntroductionRIPS(robot),
        #                            transitions={'finished':'GO_TO_EXIT'})

        # Amigo goes to the exit (waypoint stated in knowledge base)
        smach.StateMachine.add('GO_TO_EXIT', 
                                    states.NavigateGeneric(robot, goal_name="exit_1_rips"),
                                    transitions={   'arrived':'AT_END', 
                                                    'preempted':'CLEAR_PATH_TO_EXIT', 
                                                    'unreachable':'CLEAR_PATH_TO_EXIT', 
                                                    'goal_not_defined':'CLEAR_PATH_TO_EXIT'})

        # Amigo will say that it arrives at the intermediate waypoint
        smach.StateMachine.add('CLEAR_PATH_TO_EXIT',
                                    states.Say(robot, "I couldn't go to the exit. Please clear the path, I will give it another try."),
                                    transitions={'spoken':'GO_TO_EXIT_SECOND_TRY'}) 

        # Amigo goes to the exit (waypoint stated in knowledge base)
        smach.StateMachine.add('GO_TO_EXIT_SECOND_TRY', 
                                    states.NavigateGeneric(robot, goal_name="exit_1_rips", goal_area_radius=0.5), # within 1m of the target
                                    transitions={   'arrived':'AT_END', 
                                                    'preempted':'GO_TO_EXIT_THIRD_TRY', 
                                                    'unreachable':'GO_TO_EXIT_THIRD_TRY', 
                                                    'goal_not_defined':'GO_TO_EXIT_THIRD_TRY'})
    
        # Then amigo will drive to the intermediate waypoint. Defined in knowledge base. Now it is the table in the test map.
        smach.StateMachine.add('GO_TO_EXIT_THIRD_TRY', 
                                    states.NavigateGeneric(robot, goal_name="exit_2_rips", goal_area_radius=0.5), # within 1m of the target
                                    transitions={   'arrived':'AT_END', 
                                                    'preempted':'GO_TO_EXIT_FOURTH_TRY', 
                                                    'unreachable':'GO_TO_EXIT_FOURTH_TRY', 
                                                    'goal_not_defined':'GO_TO_EXIT_FOURTH_TRY'})

        # Then amigo will drive to the intermediate waypoint. Defined in knowledge base. Now it is the table in the test map.
        smach.StateMachine.add('GO_TO_EXIT_FOURTH_TRY', 
                                    states.NavigateGeneric(robot, goal_name="exit_3_rips", goal_area_radius=0.5), # within 1m of the target
                                    transitions={   'arrived':'AT_END', 
                                                    'preempted':'GO_TO_EXIT_FIFTH_TRY', 
                                                    'unreachable':'GO_TO_EXIT_FIFTH_TRY', 
                                                    'goal_not_defined':'GO_TO_EXIT_FIFTH_TRY'})
                                                    
        # Then amigo will drive to the intermediate waypoint. Defined in knowledge base. Now it is the table in the test map.
        smach.StateMachine.add('GO_TO_EXIT_FIFTH_TRY', 
                                    states.NavigateGeneric(robot, goal_name="exit_4_rips", goal_area_radius=0.5), # within 1m of the target
                                    transitions={   'arrived':'AT_END', 
                                                    'preempted':'AT_END', 
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

    startup(setup_statemachine)
