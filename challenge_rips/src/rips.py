#!/usr/bin/python
import roslib; roslib.load_manifest('challenge_rips')
import rospy
import smach

from robot_skills.amigo import Amigo
import robot_smach_states as states
from robot_smach_states.util.startup import startup
from geometry_msgs.msg import Point

from speech_interpreter.srv import GetContinue # for speech_to_text only

from psi import *


###########################
# Created by: Erik Geerts #
###########################

##########################################
############## What to run: ##############
############ updated 16-4-2013 ###########
##########################################
# - astart
# - amiddle
# - roslaunch create_speech_files speech.launch   (in tue_test_lab the launch file is: speech_tue_test_lab.launch)
# - !! Wait for speech.launch to finish before !!
#   !!   launching speech interpreter          !!
#   roslaunch speech_interpreter start.launch     (in tue_test_lab the launch file is: speech_tue_test_lab.launch)
# - rosrun challenge_egpsr egpsr.py

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
# -  When the arm's suddenly stop working during execution, Amigo should detect this and say in his introduction that because his 
#    arms are not working at the moment someone should take the registration form out of his hand.


class Ask_continue(smach.State):
    def __init__(self, robot, tracking=True, rate=2):
        smach.State.__init__(self, outcomes=["done", "no_continue"])

        self.robot = robot
        self.preempted = False
        self.rate = rate
        self.get_continue_service = rospy.ServiceProxy('interpreter/get_continue', GetContinue)

    def execute(self, userdata):

        self.response = self.get_continue_service(2 , 15)

        if self.response.answer == "true":
            return "done"
        else:
            return "no_continue"


class AmigoIntroductionRIPS(smach.State):
    def __init__(self, robot=None, gripper="left"):
        smach.State.__init__(self, outcomes=['finished'])
        
        self.robot = robot
        self.gripper = gripper
        
    def execute(self, userdata):      
        rospy.loginfo("Introducing AMIGO")
        
        self.robot.head.reset_position()
        self.robot.leftArm.send_joint_goal(-0.1,-0.2,0.2,0.8,0.0,0.0,0.0)
        self.robot.rightArm.send_joint_goal(-0.1,-0.2,0.2,0.8,0.0,0.0,0.0)
        
        self.robot.speech.speak("Hello, my name is amigo")
        rospy.sleep(1.0)
        self.robot.speech.speak("I am participating in robocup 2013 on behalf of Tech United Eindhoven")
        
        rospy.loginfo("Hand over registration form...")
        
        self.robot.speech.speak("Here is my registration form")
        
        if self.gripper == "left":
            ''' Left arm '''
            head_goal = Point()
            head_goal.x = 0.0
            head_goal.y = 0.0
            head_goal.z = 0.0
            self.robot.head.send_goal_topic(head_goal,"/grippoint_left")
            #self.robot.leftArm.send_joint_goal(-1, 0.5819, 0.208278, 1.34569383, 0.56438928, -0.2, -0.0188)
            self.robot.leftArm.send_goal(0.6,0.3,1.1,1.5,0.0,0.0,10.0)
            #rospy.sleep(0.5)
            self.robot.leftArm.send_gripper_goal_open(10)
            
            #self.robot.leftArm.send_goal(0.3,0.3,0.8,1.5,0.0,0.0,10.0)
            self.robot.leftArm.send_joint_goal(-0.1,-0.2,0.2,0.8,0.0,0.0,0.0)
            self.robot.leftArm.send_gripper_goal_close(5)
        else:
            ''' Right arm '''
            head_goal = Point()
            head_goal.x = 0.0
            head_goal.y = 0.0
            head_goal.z = 0.0
            self.robot.head.send_goal_topic(head_goal,"/grippoint_right")
            self.robot.rightArm.send_joint_goal(-1, 0.5819, 0.208278, 1.34569383, 0.56438928, -0.2, -0.0188)
            #rospy.sleep(0.5)
            self.robot.rightArm.send_gripper_goal_open(10)
            
            #self.robot.leftArm.send_goal(0.3,0.3,0.8,1.5,0.0,0.0,10.0)
            self.robot.rightArm.send_joint_goal(-0.1,-0.2,0.2,0.8,0.0,0.0,0.0)
            self.robot.rightArm.send_gripper_goal_close(5)
        
        self.robot.head.reset_position()
        
        self.robot.speech.speak("If you want me to stop, you can press my emergency button on my back")
        self.robot.speech.speak("I will leave the arena if you say continue after my lights become green")  # green light is done in speech interpreter
        
        return 'finished'


def setup_statemachine(robot):

    #retract old facts
    robot.reasoner.query(Compound("retractall", Compound("challenge", "X")))

    #Load database
    robot.reasoner.query(Compound("load_database","tue_knowledge",'prolog/locations.pl'))

    #Assert the current challenge.
    robot.reasoner.query(Compound("assertz",Compound("challenge", "registration")))

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    with sm:

        smach.StateMachine.add('INITIALIZE_FIRST',
                                states.Initialize(robot),
                                transitions={   'initialized':'OPENING_GRIPPER',
                                                'abort':'Aborted'})

        smach.StateMachine.add('OPENING_GRIPPER',
                                    states.Say(robot, 'I will open my left gripper now, so that you can put my registration form in it.'),
                                    transitions={'spoken':'OPEN_GRIPPER'}) 

        smach.StateMachine.add('OPEN_GRIPPER',
                                    states.SetGripper(robot, robot.leftArm, gripperstate=0),
                                    transitions={'succeeded':'CLOSING_GRIPPER',
                                                 'failed'   :'CLOSING_GRIPPER'})

        smach.StateMachine.add('CLOSING_GRIPPER',
                                    states.Say(robot, 'I will close my gripper now.'),
                                    transitions={'spoken':'CLOSE_GRIPPER'}) 

        smach.StateMachine.add('CLOSE_GRIPPER',
                                    states.SetGripper(robot, robot.leftArm, gripperstate=1),
                                    transitions={'succeeded':'START_ACTUAL_CHALLENGE',
                                                 'failed':'START_ACTUAL_CHALLENGE'})

        # Start challenge via StartChallengeRobust
        smach.StateMachine.add( "START_ACTUAL_CHALLENGE",
                                    states.StartChallengeRobust(robot, "initial"), 
                                    transitions={   "Done":"SAY_GO_TO_MEETING_POINT", 
                                                    "Aborted":"SAY_GO_TO_MEETING_POINT", 
                                                    "Failed":"SAY_GO_TO_MEETING_POINT"})   # There is no transition to Failed in StartChallengeRobust (28 May)

        smach.StateMachine.add("SAY_GO_TO_MEETING_POINT",
                                    states.Say(robot, "I will go to the meeting point"),
                                    transitions={   "spoken":"GO_TO_REGISTRATION_TABLE"})

        smach.StateMachine.add('GO_TO_REGISTRATION_TABLE', 
                                    states.Navigate_named(robot, "registration_table1"),
                                    transitions={   'arrived':'ARRIVED_AT_REGISTRATION_TABLE', 
                                                    'preempted':'CLEAR_PATH_TO_REGISTRATION_TABLE', 
                                                    'unreachable':'CLEAR_PATH_TO_REGISTRATION_TABLE', 
                                                    'goal_not_defined':'CLEAR_PATH_TO_REGISTRATION_TABLE'})

        # Amigo will say that it arrives at the registration table
        smach.StateMachine.add('CLEAR_PATH_TO_REGISTRATION_TABLE',
                                    states.Say(robot, "At my first attempt I could not go to the meeting point. Please clear the path, I will give it another try."),
                                    transitions={'spoken':'GO_TO_REGISTRATION_TABLE_SECOND_TRY'}) 

        # Then amigo will drive to the registration table. Defined in knowledge base. Now it is the table in the test map.
        smach.StateMachine.add('GO_TO_REGISTRATION_TABLE_SECOND_TRY', 
                                    states.Navigate_named(robot, "registration_table1"),
                                    transitions={   'arrived':'ARRIVED_AT_REGISTRATION_TABLE', 
                                                    'preempted':'GO_TO_REGISTRATION_TABLE_THIRD_TRY', 
                                                    'unreachable':'GO_TO_REGISTRATION_TABLE_THIRD_TRY', 
                                                    'goal_not_defined':'GO_TO_REGISTRATION_TABLE_THIRD_TRY'})

        # Then amigo will drive to the registration table. Defined in knowledge base. Now it is the table in the test map.
        smach.StateMachine.add('GO_TO_REGISTRATION_TABLE_THIRD_TRY', 
                                    states.Navigate_named(robot, "registration_table2"),
                                    transitions={   'arrived':'ARRIVED_AT_REGISTRATION_TABLE', 
                                                    'preempted':'GO_TO_REGISTRATION_TABLE_FORTH_TRY', 
                                                    'unreachable':'GO_TO_REGISTRATION_TABLE_FORTH_TRY', 
                                                    'goal_not_defined':'GO_TO_REGISTRATION_TABLE_FORTH_TRY'})

        # Then amigo will drive to the registration table. Defined in knowledge base. Now it is the table in the test map.
        smach.StateMachine.add('GO_TO_REGISTRATION_TABLE_FORTH_TRY', 
                                    states.Navigate_named(robot, "registration_table3"),
                                    transitions={   'arrived':'ARRIVED_AT_REGISTRATION_TABLE', 
                                                    'preempted':'FAIL_BUT_INTRODUCE', 
                                                    'unreachable':'FAIL_BUT_INTRODUCE', 
                                                    'goal_not_defined':'FAIL_BUT_INTRODUCE'})

        # Amigo will say that it arrives at the registration table
        smach.StateMachine.add('ARRIVED_AT_REGISTRATION_TABLE',
                                    states.Say(robot, "I'm at the registration table, I will now introduce myself"),
                                    transitions={'spoken':'INTRODUCE_AMIGO'}) 

        # In case the path is blocked, amigo will say that it still introduce itself.
        smach.StateMachine.add('FAIL_BUT_INTRODUCE',
                                    states.Say(robot, "I stil couldn't get to the registration table, I will now introduce myself and leave my registration form here"),
                                    transitions={'spoken':'INTRODUCE_AMIGO'}) 

        # It will start the introduction (MAKE SURE THAT THE FULL INTRODUCTION IS PLAYED DURING COMPETITION!!!))
        smach.StateMachine.add('INTRODUCE_AMIGO', 
                                    AmigoIntroductionRIPS(robot,gripper="left"),
                                    transitions={'finished':'ASK_CONTINUE'})

        smach.StateMachine.add("ASK_CONTINUE",
                                Ask_continue(robot),
                                transitions={'done':'TEXT_HEARD',
                                             'no_continue':'TEXT_NOT_HEARD'})

        # If heard, then amigo says that it will leave the room
        smach.StateMachine.add('TEXT_HEARD',
                                    states.Say(robot, 'I heard continue, i will leave the room'),
                                    transitions={'spoken':'GO_TO_EXIT'})

        # If not heard, then amigo says that it has not heard 'cotinue' but still leaves the room
        smach.StateMachine.add('TEXT_NOT_HEARD',
                                    states.Say(robot, 'Although I did not hear continue, i will leave the room'),
                                    transitions={'spoken':'GO_TO_EXIT'})

        # Amigo goes to the exit (waypoint stated in knowledge base)
        smach.StateMachine.add('GO_TO_EXIT', 
                                    states.Navigate_named(robot, "exit_1_rips"),
                                    transitions={   'arrived':'AT_END', 
                                                    'preempted':'CLEAR_PATH_TO_EXIT', 
                                                    'unreachable':'CLEAR_PATH_TO_EXIT', 
                                                    'goal_not_defined':'CLEAR_PATH_TO_EXIT'})

        # Amigo will say that it arrives at the registration table
        smach.StateMachine.add('CLEAR_PATH_TO_EXIT',
                                    states.Say(robot, "I couldn't go to the exit. Please clear the path, I will give it another try."),
                                    transitions={'spoken':'GO_TO_EXIT_FIRST_TRY_AGAIN'}) 

        # Amigo goes to the exit (waypoint stated in knowledge base)
        smach.StateMachine.add('GO_TO_EXIT_FIRST_TRY_AGAIN', 
                                    states.Navigate_named(robot, "exit_1_rips"),
                                    transitions={   'arrived':'AT_END', 
                                                    'preempted':'GO_TO_EXIT_SECOND_TRY', 
                                                    'unreachable':'GO_TO_EXIT_SECOND_TRY', 
                                                    'goal_not_defined':'GO_TO_EXIT_SECOND_TRY'})

        # Then amigo will drive to the registration table. Defined in knowledge base. Now it is the table in the test map.
        smach.StateMachine.add('GO_TO_EXIT_SECOND_TRY', 
                                    states.Navigate_named(robot, "exit_2_rips"),
                                    transitions={   'arrived':'AT_END', 
                                                    'preempted':'GO_TO_EXIT_THIRD_TRY', 
                                                    'unreachable':'GO_TO_EXIT_THIRD_TRY', 
                                                    'goal_not_defined':'GO_TO_EXIT_THIRD_TRY'})

        # Then amigo will drive to the registration table. Defined in knowledge base. Now it is the table in the test map.
        smach.StateMachine.add('GO_TO_EXIT_THIRD_TRY', 
                                    states.Navigate_named(robot, "exit_3_rips"),
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
    rospy.init_node('executioner')
    startup(setup_statemachine)
