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
        smach.State.__init__(self, outcomes=["done", "preempted", "no_continue"])

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
    def __init__(self, robot=None):
        smach.State.__init__(self, outcomes=['finished'])
        
        self.robot = robot
        
    def execute(self, userdata):      
        rospy.loginfo("Introducing AMIGO")
        
        self.robot.leftArm.send_joint_goal(-0.1,-0.2,0.2,0.8,0.0,0.0,0.0)
        self.robot.rightArm.send_joint_goal(-0.1,-0.2,0.2,0.8,0.0,0.0,0.0)
        
        self.robot.speech.speak("Hello, my name is amigo")
        rospy.sleep(1.0)
        self.robot.speech.speak("I am participating in robocup 2013 on behalf of Tech United Eindhoven")
        
        rospy.loginfo("Hand over registration form...")
        
        self.robot.speech.speak("Here is my registration form")
                
        ''' Left arm '''
        head_goal = Point()
        head_goal.x = 0.0
        head_goal.y = 0.0
        head_goal.z = 0.0
        self.robot.head.send_goal_topic(head_goal,"/grippoint_left")
        self.robot.leftArm.send_goal(0.6,0.3,1.1,1.5,0.0,0.0,10.0)
        self.robot.leftArm.send_gripper_goal_open(10)
        
        #self.robot.leftArm.send_goal(0.3,0.3,0.8,1.5,0.0,0.0,10.0)
        self.robot.leftArm.send_joint_goal(-0.1,-0.2,0.2,0.8,0.0,0.0,0.0)
        self.robot.leftArm.send_gripper_goal_close(5)
        
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
                
        smach.StateMachine.add('INITIALIZE',
                                states.Initialize(robot),
                                transitions={   'initialized':'CLOSING_GRIPPER',
                                                'abort':'Aborted'})

        smach.StateMachine.add('CLOSING_GRIPPER',
                                    states.Say(robot, 'I will open my gripper now, so that you can put my registration form in my left hand.'),
                                    transitions={'spoken':'CLOSE_GRIPPER'}) 

        smach.StateMachine.add('CLOSE_GRIPPER',
                                    states.SetGripper(robot, robot.leftArm, gripperstate=0),
                                    transitions={'state_set':'AT_FRONT_OF_DOOR'})

        smach.StateMachine.add('CLOSING_GRIPPER',
                                    states.Say(robot, 'I will close my gripper now.'),
                                    transitions={'spoken':'CLOSE_GRIPPER'}) 

        smach.StateMachine.add('CLOSE_GRIPPER',
                                    states.SetGripper(robot, robot.leftArm, gripperstate=1),
                                    transitions={'state_set':'AT_FRONT_OF_DOOR'})

        # If the door is open, amigo will say that it goes to the registration table
        smach.StateMachine.add('AT_FRONT_OF_DOOR',
                                    states.Say(robot, 'I will now check if it is open or not'),
                                    transitions={'spoken':'STATE_DOOR'})

        # Start laser sensor that may change the state of the door if the door is open:
        smach.StateMachine.add('STATE_DOOR', 
                                    states.Read_laser(robot, "entrance_door"),
                                    transitions={'laser_read':'WAIT_FOR_DOOR'})       
        
        # define query for the question wether the door is open in the state WAIT_FOR_DOOR
        dooropen_query = Compound("state", "entrance_door", "open")
        
        # Query if the door is open:
        smach.StateMachine.add('WAIT_FOR_DOOR', 
                                    states.Ask_query_true(robot, dooropen_query),
                                    transitions={   'query_false':'STATE_DOOR',
                                                    'query_true':'THROUGH_DOOR',
                                                    'waiting':'DOOR_CLOSED',
                                                    'preempted':'FAIL_BUT_INTRODUCE'})

        # If the door is still closed after certain number of iterations, defined in Ask_query_true 
        # in perception.py, amigo will speak and check again if the door is open
        smach.StateMachine.add('DOOR_CLOSED',
                                    states.Say(robot, 'Door is closed, please open the door'),
                                    transitions={'spoken':'STATE_DOOR'}) 

        # If the door is open, amigo will say that it goes to the registration table
        smach.StateMachine.add('THROUGH_DOOR',
                                    states.Say(robot, 'Door is open, so I will go to the registration table'),
                                    transitions={'spoken':'INIT_POSE'}) 

        # Initial pose is set after opening door, otherwise snapmap will fail if door is still closed and initial pose is set.
        smach.StateMachine.add('INIT_POSE',
                                states.Set_initial_pose(robot, 'initial'),
                                transitions={   'done':'GO_TO_REGISTRATION_TABLE',
                                                'preempted':'WAIT_FOR_DOOR',
                                                'error':'WAIT_FOR_DOOR'})

        # Then amigo will drive to the registration table. Defined in knowledge base. Now it is the table in the test map.
        smach.StateMachine.add('GO_TO_REGISTRATION_TABLE', 
                                    states.Navigate_named(robot, "registration_table"),
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
                                    states.Say(robot, "I couldn't get to the registration table, I will now introduce myself and leave my registration form here"),
                                    transitions={'spoken':'INTRODUCE_AMIGO'}) 

        # It will start the introduction (MAKE SURE THAT THE FULL INTRODUCTION IS PLAYED DURING COMPETITION!!!))
        smach.StateMachine.add('INTRODUCE_AMIGO', 
                                    AmigoIntroductionRIPS(robot),
                                    transitions={'finished':'ASK_CONTINUE'})

        smach.StateMachine.add("ASK_CONTINUE",
                                Ask_continue(robot),
                                transitions={'done':'TEXT_HEARD',
                                             'preempted':'AT_END',                         ##### Should there still be a preempted in it?
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
                                    states.Navigate_named(robot, "exit"),
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
