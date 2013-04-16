#!/usr/bin/python
import roslib; roslib.load_manifest('challenge_emergency')
import rospy
import smach
import math
import cv
import roslib
#import sys
import roslib.packages as p

from robot_skills.amigo import Amigo
import robot_smach_states as states
from robot_smach_states.util.startup import startup

from std_msgs.msg import String
from speech_interpreter.srv import GetContinue
from speech_interpreter.srv import GetYesNo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


#import states_new as states 
from psi import *



#########################################
# Created by: Erik Geerts, Teun Derksen #
#########################################

#################
## TODO LIST!! ##
#################

# Implement
#   - Detect smoke
#   - Detect people
#   - Questions to person
#   - Write data/pictures to pdf
#   - Guiding person to exit
#   - ..

##########################################
############## What to run: ##############
############ updated 15-4-2013 ###########
##########################################
# - astart
# - amiddle
# - roslaunch create_speech_files speech.launch   (in tue_test_lab the launch file is: speech_tue_test_lab.launch)
# - !! Wait for speech.launch to finish before !!
#   !!   launching speech interpreter          !!
#   roslaunch speech_interpreter start.launch     (in tue_test_lab the launch file is: speech_tue_test_lab.launch)
# - rosrun challenge_emergency emergency.py



#############################################################
## Locations that must be defined in database on forehand: ##
#############################################################
# - initial
# - meeting_point
# - other_side_room
# - exit

#################
### Questions ###
#################
# -


########################
##### STATEMACHINE #####
########################

class turn_Around_z_axis(smach.State):
    def __init__(self, robot, rotation):
        """
        @param robot
        @param rotation, the rotation in radian with which amigo needs to turn
        """
        smach.State.__init__(self, outcomes=["done", "abort"])
        self.robot    = robot
        self.rotation = rotation

    def execute(self, userdata):
        """ 
        @return
        """
        ##rospy.logger('Emergency.py:turnAmigo.Execute: {0} degrees.'.format((self.rotation)))

        rospy.loginfo("Executing: {0} radians".format(self.rotation))
        # get current position and rotation
        pos, rot = self.robot.base.get_location()

        # set rotation
        rot.z = rot.z - self.rotation

        # create new path
        path = self.robot.base.get_plan(pos,rot)

        # execute new path
        if self.robot.base.execute_plan(path):
            return "done"
        else:
            return "abort"

class Ask_yes_no(smach.State):
    def __init__(self, robot, tracking=True, rate=2):
        smach.State.__init__(self, outcomes=["yes", "preempted", "no"])

        self.robot = robot
        self.preempted = False
        self.rate = rate
        self.get_yes_no_service = rospy.ServiceProxy('interpreter/get_yes_no', GetYesNo)

    def execute(self, userdata):

        self.response = self.get_yes_no_service(3 , 15)

        if self.response.answer == "true":
            return "yes"
        else:
            return "no"

def setup_statemachine(robot):

    # Retract old facts
    robot.reasoner.query(Compound("retractall", Compound("challenge", "X")))
    robot.reasoner.query(Compound("retractall", Compound("explored", "X")))
    robot.reasoner.query(Compound("retractall", Compound("state", "X", "Y")))
    robot.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X")))
    robot.reasoner.query(Compound("retractall", Compound("current_object", "X")))
    robot.reasoner.query(Compound("retractall", Compound("not_visited", "X")))
    robot.reasoner.query(Compound("retractall", Compound("not_unreachable", "X")))

    # Load database
    robot.reasoner.query(Compound("load_database","tue_knowledge",'prolog/locations.pl'))

    # Assert the current challenge.
    robot.reasoner.query(Compound("assertz",Compound("challenge", "emergency")))

    #Assert not_visited locations
    robot.reasoner.query(Compound("init_not_roi","Location"))

    # create state machine for searching smoke

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    with sm:

        ######################################################
        ##################### INITIALIZE #####################             
        ######################################################

        smach.StateMachine.add('INITIALIZE',
                                    states.Initialize(robot),
                                    transitions={   'initialized' : 'AT_FRONT_OF_DOOR',  ##'AT_FRONT_OF_DOOR','DETECT_PEOPLE'
                                                    'abort'       : 'Aborted'})
    
        smach.StateMachine.add('AT_FRONT_OF_DOOR',
                                    states.Say(robot, 'I will now check if the door is open or not'),
                                    transitions={   'spoken':'STATE_DOOR'}) 

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
                                                    'preempted':'INIT_POSE'})

        # If the door is still closed after certain number of iterations, defined in Ask_query_true 
        # in perception.py, amigo will speak and check again if the door is open
        smach.StateMachine.add('DOOR_CLOSED',
                                    states.Say(robot, 'Door is closed, please open the door'),
                                    transitions={'spoken':'STATE_DOOR'}) 

        ######################################################
        ############ ENTER ROOM AND GO TO KITCHEN ############
        ######################################################
        # If the door is open, amigo will say that it goes to the registration table
        smach.StateMachine.add('THROUGH_DOOR',
                                    states.Say(robot, 'Door is open, so I will go to in the kitchen'),
                                    transitions={'spoken':'INIT_POSE'}) 

        # Initial pose is set after opening door, otherwise snapmap will fail if door is still closed and initial pose is set.
        smach.StateMachine.add('INIT_POSE',
                                    states.Set_initial_pose(robot, 'initial'),
                                    transitions={   'done':'SAY_IS_THERE_SOMETHING_BURNING',
                                                    'preempted':'Aborted',
                                                    'error':'Aborted'})
        # Say that something is burning
        smach.StateMachine.add('SAY_IS_THERE_SOMETHING_BURNING',
                                    states.Say(robot, 'Is something burning?'),
                                    transitions={'spoken':'DETECT_PEOPLE'})  ###{'spoken':'TURN_ROUND_Z_AXIS'})

        ######################################################
        ################# DETECT FIRE/SMOKE ##################
        ######################################################
        # Turn 360 degrees to search for the smoke
        smach.StateMachine.add('TURN_ROUND_Z_AXIS',
                                    turn_Around_z_axis(robot, 10),
                                    transitions={   'done':'SAY_NEXT_LOCATION',    ###### 
                                                    'abort':'Aborted'})
        # Inform people
        smach.StateMachine.add('SAY_NEXT_LOCATION',
                                    states.Say(robot, 'Did not find fire. Maybe at the other side of the room'),
                                    transitions={'spoken':'NEXT_LOCATION'})

        # Query reasoner to find other side of goal
        query_kitchen = Compound("waypoint", "kitchen", Compound("pose_2d", "X", "Y", "Phi"))
        smach.StateMachine.add('NEXT_LOCATION',
                                states.Navigate_to_queryoutcome(robot, query_kitchen, X="X", Y="Y", Phi="Phi"),
                                transitions={   "arrived":"TURN_AROUND_Z_AXIS2",
                                                "unreachable":'CANNOT_GOTO_MEETINGPOINT',
                                                "preempted":'CANNOT_GOTO_MEETINGPOINT',
                                                "goal_not_defined":'CANNOT_GOTO_MEETINGPOINT'})

        # Cannot reach the Goal
        smach.StateMachine.add("CANNOT_GOTO_MEETINGPOINT", 
                                states.Say(robot, ["I can't find a way to the meeting point. Please teach me the correct position and clear the path to it"]),
                                transitions={   'spoken':'Aborted'})

        # Turn around your z-axis to detect person/smoke
        smach.StateMachine.add('TURN_AROUND_Z_AXIS2',
                                    turn_Around_z_axis(robot, 10),
                                    transitions={   'done':'WAIT_SOME_SECONDS', 
                                                    'abort':'Aborted'})
        # Wait for some seconds 
        smach.StateMachine.add('WAIT_SOME_SECONDS',
                                    states.Wait_time(robot,1),
                                    transitions={   'waited':'SAY_DETECT_SMOKE',
                                                    'preempted':'Aborted'})
        # Time ran out say, you couldn't find smoke
        smach.StateMachine.add('SAY_DETECT_SMOKE',
                                    states.Say(robot, 'Time ran out, could not find fire. But I smell something burning. Going to look for people!'),
                                    transitions={'spoken':'FIND_PEOPLE'})

        ######################################################
        ################### SAVING PERSONS ###################
        ######################################################
        smach.StateMachine.add('SAY_FIND_PEOPLE',
                                    states.Say(robot, 'Going to look for more people.'),
                                    transitions={'spoken':'FIND_PEOPLE'})

        query_point = Compound("region_of_interest_emergency","ROI_Location", Compound("point_3d", "X", "Y", "Z"))
        object_identifier_query = "ROI_Location"

        smach.StateMachine.add("FIND_PEOPLE",
                                states.Visit_query_outcome_3d(robot, 
                                                                  query_point, 
                                                                  x_offset=0.7, y_offset=0.0001,
                                                                  identifier=object_identifier_query),
                                transitions={   'arrived':'START_PEOPLE_DETECTION',
                                                'unreachable':'FAILED_DRIVING_TO_LOCATION',
                                                'preempted':'FAILED_DRIVING_TO_LOCATION',
                                                'goal_not_defined':'FAILED_DRIVING_TO_LOCATION',
                                                'all_matches_tried':'AT_END'})
        # Could not reach ROI
        smach.StateMachine.add("FAILED_DRIVING_TO_LOCATION",
                                states.Say(robot,"Now I should start my perception, but is not build in yet..."),
                                transitions={'spoken':'START_PEOPLE_DETECTION'})

        # Start people detection
        # TODO ACTUALLY USE PEOPLE DETECTION
        smach.StateMachine.add('START_PEOPLE_DETECTION',
                                    states.Say(robot, 'Detected person. Are you okay?'),
                                    transitions={'spoken':'ANSWER_ARE_YOU_OKAY'})

        # People detection
        smach.StateMachine.add('DETECT_PEOPLE',
                                    states.Say(robot, 'Detected person. Are you okay?'),
                                    transitions={'spoken':'ANSWER_ARE_YOU_OKAY'})
        # Await anser of question 1
        smach.StateMachine.add('ANSWER_ARE_YOU_OKAY',
                                    Ask_yes_no(robot),
                                    transitions={   'yes':'ASK_IF_KNOWS_DIRECTION',
                                                    'preempted':'REGISTER_PERSON',
                                                    'no':'REGISTER_PERSON'})

        # Person is ok and is asked if he/she knows the way
        smach.StateMachine.add('ASK_IF_KNOWS_DIRECTION',
                                    states.Say(robot, 'Do you know the way to the exit'),
                                    transitions={'spoken':'ANSWER_IF_KNOWS_DIRECTION'})

        # Await question of he/she knows the way
        smach.StateMachine.add('ANSWER_IF_KNOWS_DIRECTION',
                                    Ask_yes_no(robot),
                                    transitions={   'yes':'SAY_MOVE_TO_EXIT',
                                                    'preempted':'SAY_FOLLOW_ME',
                                                    'no':'MOVE_ARM_BACK'})

         # Class for registering person
        class Register(smach.State):
            def __init__(self, robot=None):
                smach.State.__init__(self, outcomes=['finished'])
                self.person_no = 1
                self.robot = robot
        
            def execute(self, gl):      
                rospy.loginfo("Register person in file ....")

                
                f = open(p.get_pkg_dir('challenge_emergency')+'/output/status.txt','a')
                #f = open('status.txt','a')
                f.write('person_%d;1;' % self.person_no) 
                pos, rot = self.robot.base.get_location()
                f.write('%.2f;%.2f \n' % (pos.x, pos.y))
                f.close()

                self.robot.speech.speak("Registering your location and status, so the fire department can find you.")
                self.robot.speech.speak("Please stay at this location and stay calm.")
                self.person_no = self.person_no + 1
                return 'finished'

        # Class for taking a picture

        class TakePicture(smach.State):
            def __init__(self, robot=None):
                smach.State.__init__(self, outcomes=['finished'])
                self.robot = robot
                cv.NamedWindow("Image window", 1)
                self.bridge = CvBridge()
                self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color",Image,self.callback)

            def callback(self,data):
                try:
                    cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
                except CvBridgeError, e:
                    print e

                (cols,rows) = cv.GetSize(cv_image)
                if cols > 60 and rows > 60 :
                    cv.Circle(cv_image, (50,50), 10, 255)

                cv.ShowImage("Image window", cv_image)
                cv.WaitKey(3)


            def execute(self, gl):      
                rospy.loginfo("Taking a picture of person/fire ....")
                return 'finished'

        # Person is not ok and is left alone and registered
        smach.StateMachine.add('REGISTER_PERSON',
                                    Register(robot),
                                    transitions={'finished':'SAY_FIND_PEOPLE'})
        # Person is ok and needs no assistance to exit
        smach.StateMachine.add('SAY_MOVE_TO_EXIT',
                                    states.Say(robot, 'Okay, please move to the exit'),
                                    transitions={'spoken':'SAY_FIND_PEOPLE'})

        # Class for moving arm back for guidance
        class MoveArmBack(smach.State):
            def __init__(self, robot=None):
                smach.State.__init__(self, outcomes=['finished'])
        
                self.robot = robot
        
            def execute(self, gl):      
                rospy.loginfo("Moving arm back")
            
                self.robot.leftArm.send_joint_goal(0.0,-1.57,0.0,1.57,0.0,0.0,0.0)
                self.robot.rightArm.send_joint_goal(0.0,-1.57,0.0,1.57,0.0,0.0,0.0)
                rospy.sleep(1.5)
                self.robot.leftArm.send_gripper_goal_open(10)
                self.robot.rightArm.send_gripper_goal_open(10)
                
                self.robot.speech.speak("Please grab my hand")
                return 'finished'

        # Move arm back when person needs guidance
        smach.StateMachine.add('MOVE_ARM_BACK',
                                    MoveArmBack(robot),
                                    transitions={'finished':'SAY_FOLLOW_ME'})

        
        # Person is ok and needs assistance to exit
        smach.StateMachine.add('SAY_FOLLOW_ME',
                                    states.Say(robot, 'Stay calm and follow me, please!'),
                                    transitions={'spoken':'GO_TO_EXIT'})

        # Amigo goes to the exit (waypoint stated in knowledge base)
        smach.StateMachine.add('GO_TO_EXIT', 
                                    states.Navigate_named(robot, "exit"),
                                    transitions={   'arrived':'MOVE_ARM_INITIAL', 
                                                    'preempted':'AT_END', 
                                                    'unreachable':'AT_END', 
                                                    'goal_not_defined':'AT_END'})
        # Class to move arm to initial pose
        class MoveArmInitial(smach.State):
            def __init__(self, robot=None):
                smach.State.__init__(self, outcomes=['finished'])
        
                self.robot = robot
        
            def execute(self, gl):   
                self.robot.speech.speak("Can you please let go of my arm")   
                rospy.loginfo("Moving arm back")
            
                self.robot.leftArm.send_joint_goal(-0.1,-0.2,0.2,0.8,0.0,0.0,0.0)
                self.robot.rightArm.send_joint_goal(-0.1,-0.2,0.2,0.8,0.0,0.0,0.0)
                rospy.sleep(1.5)
                self.robot.leftArm.send_gripper_goal_close(10)
                self.robot.rightArm.send_gripper_goal_close(10)
                
                self.robot.speech.speak("We arrived at the exit, please stay here and wait for help")
                return 'finished'

        # When arrived at exit move arm to natural look
        smach.StateMachine.add('MOVE_ARM_INITIAL',
                                    MoveArmInitial(robot),
                                    transitions={'finished':'SAY_FIND_PEOPLE'})


        # Finally amigo will stop and says 'goodbye' to show that he's done.
        smach.StateMachine.add('AT_END',
                                    states.Say(robot, "Goodbye"),
                                    transitions={'spoken':'Done'})

        
        
        '''
        # If the door is open, amigo will say that it goes to the registration table
        smach.StateMachine.add('AT_FRONT_OF_DOOR',
                                    states.Say(robot, 'I will now check if it is open or not'),
                                    transitions={'spoken':'STATE_DOOR'}) 
        
        smach.StateMachine.add('STATE_DOOR', states.Read_laser(robot,"entrance_door"),
                                  transitions={'laser_read':'CHECK_DOOR'})
              
        dooropen_query = Compound("state","entrance_door", "open")
        smach.StateMachine.add('CHECK_DOOR', 
                                    states.Ask_query_true(robot, dooropen_query),
                                    transitions={   'query_false':'STATE_DOOR',
                                                    'query_true':'SAY_NAVIGATE_TO_KITCHEN',
                                                    'waiting':'DOOR_CLOSED',
                                                    'preempted':'Aborted'})

        # If the door is still closed after certain number of iterations, defined in Ask_query_true 
        # in perception.py, amigo will speak and check again if the door is open
        smach.StateMachine.add('DOOR_CLOSED',
                                    states.Say(robot, 'Door is closed, please open the door'),
                                    transitions={'spoken':'STATE_DOOR'}) 

        # Go 
        query_kitchen_1 = Compound("waypoint", Compound("kitchen", "one"), Compound("pose_2d", "X", "Y", "Phi"))

        smach.StateMachine.add('SAY_NAVIGATE_TO_KITCHEN', 
                                    states.Say_and_Navigate(
                                        robot=robot,
                                        sentence = "Door is open, so I will go to the kitchen",
                                        input_keys=['navigate_to_queryoutcome',query_kitchen_1]),
                                    transitions={'succeeded':'SAY_DETECT_SMOKE',
                                                 'not_at_loc':'Aborted'})

        

        # My idea is to make a 360 degree turn, so that it seems that amigo is 'checking the kitchen for smoke'

        # In the rulebook it is said that smoke should be detected within 1 minute and that the robot should look and point 
        # from a close distance (less than 1 m)

        # Idea: FLASH RED light on and off after detecting smoke until the end of the challenge (not during interpretation). Then say: "If anyone can see or hear me, 
        #try to get to the exit!!"

        smach.StateMachine.add('SAY_IS_THERE_SOMETHING_BURNING',
                                    states.Say(robot, 'Hmm, is something burning?'),
                                    transitions={'spoken':'TURN_ROUND_Z_AXIS'})

        smach.StateMachine.add('TURN_ROUND_Z_AXIS',
                                Turn_around_z_axis(robot,"slow"),
                                transitions={   'done':'SAY_DETECT_SMOKE',    ###### IN CASE NEXT STATE IS NOT "GO_TO_DOOR" SOMETHING IS SKIPPED
                                                'abort':'Aborted'})

        smach.StateMachine.add('SAY_DETECT_SMOKE',
                                    states.Say(robot, 'With my sensors I noticed that there is smoke! I will try to find people!'),
                                    transitions={'spoken':'FINISH'})
        '''
        

        ################### DETECT PERSON ####################

        # Question: will a person lay on the ground?

        # WHILE PERSONS_SAVED IS-NOT 1 AND NO MORE PEOPLE CAN BE FOUND -> try saving people
        # Try several region of interests in kitchen
        # Save to knowledge base: 
        #   - how many times a region has been viewed (in case a double check for a certain region of interest is wanted)
        #   - how many people have been detected in a certain 
        #   - how many people are saved
        #   - ...

        ############### GET INFO STATUS PERSON ###############

        # - Say: "Are you able to speak?"
        # IF YES
        #   - Say: "Are you able to walk?"
        #   IF YES
        #       - Say: "Do you now the way to the exit?"
        #       IF YES
        #           - Say: "Allright, you should go to the exit then!"
        #       ELSE 
        #           - Say: "Follow me then to the exit"
        #   ELSE (Answer is no or no answer, person can't walk)
        #       - Say: "I will register your position."
        #       - Register position (parse to teun)
        # ELSE (Answer is no or no answer, person can't walk)
        #   - Say: "I did not hear you, I will register your position"
        #   - Register position (parse to teun)

        # Store information in the way teun would like it to have

        ################### GUIDE TO EXIT ####################

        # If person does not know how to get to the exit, get him there.

        # Since it is hard to look back constantly, we will put one or two arms backwards and say: 
        # "Please hold on to one of my arms and I will drive slowely to the exit."

        ######################################################
        ###################### FINISHED ######################
        ######################################################

        ## TODO: Amigo should leave the room i guess and say that he's done.


        smach.StateMachine.add('FINISH', states.Finish(robot),
                                        transitions={'stop':'Done'})
    return sm

if __name__ == "__main__":
    rospy.init_node('emergency_exec', log_level=rospy.DEBUG)
    startup(setup_statemachine)
