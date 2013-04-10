#! /usr/bin/env python
import roslib; roslib.load_manifest('tue_execution_pack')
import rospy
import smach
import smach_ros
#from object_msgs.msg import ExecutionTarget
#from exc_to_ros import *
#from ros_to_exc import *
#from exc_functions import *
#from ros_functions import *
#import time
#import copy

#import components

#from human_interaction import Say

############################## State Introduce ######################################
class AmigoIntroductionEnglish(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self, outcomes=['finished_introduction'])
        #,
        #                    input_keys=['target','locations'],
        #                    output_keys=['target'])
        self.robot = robot
        
    def execute(self, userdata):

        self.robot.leftArm.reset_arm()
        self.robot.rightArm.reset_arm()
        self.robot.spindle.send_goal(0.35)
        
        rospy.loginfo("Introducing AMIGO")
        
        self.robot.speech.speak("Hello, my name is amigo")
        self.robot.speech.speak("It is a pleasure to meet you")
        self.robot.speech.speak("I am the care robot of the Eindhoven University of Technology")
        self.robot.speech.speak("The design is actually based on that of my brothers")
        self.robot.speech.speak("They are playing soccer while I have to do the work at home")
        
        self.robot.speech.speak("With my special wheels, I can drive in any direction")
        self.robot.base.force_drive(0.25,0.0,0.0,1.5)
        self.robot.base.force_drive(0.0,-0.25,0.0,1.5)
        self.robot.base.force_drive(-0.25,0.0,0.0,1.5)
        self.robot.base.force_drive(0.125,0.25,0.0,1.5)
        self.robot.base.force_drive(0.25,0.0,0.0,0.75)        
        self.robot.base.force_drive(0.0,0.0,1.0,6.0)
        self.robot.base.force_drive(-0.25,0.0,0.0,0.75)
        
        self.robot.speech.speak("Furthermore, I have got two human like robotic arms")
        self.robot.head.set_position(0,0,0,frame_id="/grippoint_left",keep_tracking=True)
        self.robot.leftArm.send_joint_goal(-1.4,1.4,-0.8,1.2,-0.6,-0.4,-0.4)
        
        self.robot.speech.speak("With these, I can reach both high and low")
        #TODO Loy: check if arm is working, also in Dutch
        #self.robot.head.set_position(0,0,0,frame_id="/grippoint_right",keep_tracking=True)
        #self.robot.rightArm.send_goal(0.3,-0.3,0.4,0,0,0)
        
        self.robot.spindle.send_goal(0.35)
        self.robot.leftArm.reset_arm()
        self.robot.rightArm.reset_arm()
        self.robot.head.reset_position()
        
        self.robot.rightArm.send_joint_goal(-1.2,1.2,-0.3,1.9,0,0,0)
        self.robot.speech.speak("With the cameras in my head, I can see people and objects")
        
        self.robot.spindle.send_goal(0.05)
        self.robot.leftArm.send_joint_goal(-0.4,-0.2,0.6,1.2,0,0,0)
        self.robot.speech.speak("With my laser I can see where I am")
        self.robot.speech.speak("And with my second camera I make sure I don't bump into tables")
        
        self.robot.spindle.send_goal(0.35, waittime = 10.0)
        self.robot.leftArm.reset_arm()
        self.robot.rightArm.reset_arm()
        self.robot.head.reset_position()
        
        self.robot.speech.speak("Could I be of any other service")
        
        return "finished_introduction"
    
############################## State Introduce ######################################
class AmigoIntroductionDutch(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self, outcomes=['finished_introduction'])
        #,
        #                    input_keys=['target','locations'],
        #                    output_keys=['target'])
        self.robot = robot
        
    def execute(self, gl):

        self.robot.leftArm.reset_arm()
        self.robot.rightArm.reset_arm()
        self.robot.spindle.send_goal(0.35)
        
        rospy.loginfo("Introducing AMIGO")
        
        self.robot.speech.speak("Hallo, mijn naam is amigo",language='nl',personality='marjolijn')
        self.robot.speech.speak("Leuk jullie hier te ontmoeten",language='nl',personality='marjolijn')
        self.robot.speech.speak("Ik ben de zorgrobot van de Technische Universiteit Eindhoven",language='nl',personality='marjolijn')
        self.robot.speech.speak("Mijn ontwerp is gebaseerd op dat van mijn broertjes",language='nl',personality='marjolijn')
        self.robot.speech.speak("Zij spelen voetbal terwijl ik al het werk doe",language='nl',personality='marjolijn')
        
        self.robot.speech.speak("Met mijn speciale wielen kan ik alle kanten op rijden, kijk maar",language='nl',personality='marjolijn')
        self.robot.base.force_drive(0.25,0.0,0.0,1.5)
        self.robot.base.force_drive(0.0,-0.25,0.0,1.5)
        self.robot.base.force_drive(-0.25,0.0,0.0,1.5)
        self.robot.base.force_drive(0.125,0.25,0.0,1.5)
        self.robot.base.force_drive(0.25,0.0,0.0,0.75)        
        self.robot.base.force_drive(0.0,0.0,1.0,6.0)
        self.robot.base.force_drive(-0.25,0.0,0.0,0.75)
        
        self.robot.speech.speak("Verder heb ik twee mensachtige armen",language='nl',personality='marjolijn')
        self.robot.head.set_position(0,0,0,frame_id="/grippoint_left",keep_tracking=True)
        self.robot.leftArm.send_joint_goal(-1.4,1.4,-0.8,1.2,-0.6,-0.4,-0.4)
        
        self.robot.speech.speak("Hiermee kan ik zowel hoog als laag iets oppakken",language='nl',personality='marjolijn')
        #self.robot.head.set_position(0,0,0,frame_id="/grippoint_right",keep_tracking=True)
        #self.robot.rightArm.send_goal(0.3,-0.3,0.4,0,0,0)
        
        self.robot.spindle.send_goal(0.35)
        self.robot.leftArm.reset_arm()
        self.robot.rightArm.reset_arm()
        self.robot.head.reset_position()
        
        self.robot.rightArm.send_joint_goal(-1.2,1.2,-0.3,1.9,0,0,0)
        self.robot.speech.speak("Met de cameras in mijn hoofd kan ik mensen en dingen zien",language='nl',personality='marjolijn')
        
        self.robot.spindle.send_goal(0.05)
        self.robot.leftArm.send_joint_goal(-0.4,-0.2,0.6,1.2,0,0,0)
        self.robot.speech.speak("Met mijn laser kan ik zien waar ik ben",language='nl',personality='marjolijn')
        self.robot.speech.speak("En met mijn tweede camera zorg ik ervoor dat ik niet tegen tafels aan rijdt",language='nl',personality='marjolijn')
        
        self.robot.spindle.send_goal(0.35,waittime = 10.0)
        self.robot.leftArm.reset_arm()
        self.robot.rightArm.reset_arm()
        self.robot.head.reset_position()
        
        self.robot.speech.speak("Waar zou ik jullie nog meer mee kunnen helpen",language='nl',personality='marjolijn')
        
        return "finished_introduction"
    