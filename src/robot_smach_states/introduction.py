#! /usr/bin/env python
import roslib; 
import rospy
import smach
import smach_ros

from state import State

import robot_skills.util.msg_constructors as msgs

############################## State Introduce ######################################
class AmigoIntroductionEnglish(State):
    def __init__(self, robot=None):
        State.__init__(self, locals(), outcomes=['finished_introduction'])
        
    def run(self, robot):

        robot.leftArm.reset_arm()
        robot.rightArm.reset_arm()
        robot.spindle.send_goal(0.35)
        
        rospy.loginfo("Introducing AMIGO")
        
        robot.speech.speak("Hello, my name is amigo")
        robot.speech.speak("It is a pleasure to meet you")
        robot.speech.speak("I am the care robot of the Eindhoven University of Technology")
        robot.speech.speak("The design is actually based on that of my brothers")
        robot.speech.speak("They are playing soccer while I have to do the work at home")
        
        robot.speech.speak("With my special wheels, I can drive in any direction")
        robot.base.force_drive(0.25,0.0,0.0,1.5)
        robot.base.force_drive(0.0,-0.25,0.0,1.5)
        robot.base.force_drive(-0.25,0.0,0.0,1.5)
        robot.base.force_drive(0.125,0.25,0.0,1.5)
        robot.base.force_drive(0.25,0.0,0.0,0.75)        
        robot.base.force_drive(0.0,0.0,1.0,6.0)
        robot.base.force_drive(-0.25,0.0,0.0,0.75)
        
        robot.speech.speak("Furthermore, I have got two human like robotic arms")
        robot.head.set_position(msgs.PointStamped(0,0,0,frame_id="/amigo/grippoint_left"),keep_tracking=True)
        robot.leftArm.send_joint_goal(-1.4,1.4,-0.8,1.2,-0.6,-0.4,-0.4)
        
        robot.speech.speak("With these, I can reach both high and low")
        
        robot.spindle.send_goal(0.35)
        robot.leftArm.reset_arm()
        robot.rightArm.reset_arm()
        robot.head.reset_position()
        
        robot.rightArm.send_joint_goal(-1.2,1.2,-0.3,1.9,0,0,0)
        robot.speech.speak("With the cameras in my head, I can see people and objects")
        
        robot.spindle.send_goal(0.05)
        robot.leftArm.send_joint_goal(-0.4,-0.2,0.6,1.2,0,0,0)
        robot.speech.speak("With my laser I can see where I am")
        robot.speech.speak("And with my second camera I make sure I don't bump into tables")
        
        robot.spindle.send_goal(0.35, timeout = 10.0)
        robot.leftArm.reset_arm()
        robot.rightArm.reset_arm()
        robot.head.reset_position()
        
        robot.speech.speak("Could I be of any other service")
        
        return "finished_introduction"
    
############################## State Introduce ######################################
class AmigoIntroductionDutch(State):
    def __init__(self, robot=None):
        State.__init__(self, locals(), outcomes=['finished_introduction'])
        
    def run(self, robot):

        robot.leftArm.reset_arm()
        robot.rightArm.reset_arm()
        robot.spindle.send_goal(0.35)
        
        rospy.loginfo("Introducing AMIGO")
        
        robot.speech.speak("Hallo, mijn naam is amigo",language='nl',personality='marjolijn')
        robot.speech.speak("Leuk jullie hier te ontmoeten",language='nl',personality='marjolijn')
        robot.speech.speak("Ik ben de zorgrobot van de Technische Universiteit Eindhoven",language='nl',personality='marjolijn')
        robot.speech.speak("Mijn ontwerp is gebaseerd op dat van mijn broertjes",language='nl',personality='marjolijn')
        robot.speech.speak("Zij spelen voetbal terwijl ik al het werk doe",language='nl',personality='marjolijn')
        
        robot.speech.speak("Met mijn speciale wielen kan ik alle kanten op rijden, kijk maar",language='nl',personality='marjolijn')
        robot.base.force_drive(0.25,0.0,0.0,1.5)
        robot.base.force_drive(0.0,-0.25,0.0,1.5)
        robot.base.force_drive(-0.25,0.0,0.0,1.5)
        robot.base.force_drive(0.125,0.25,0.0,1.5)
        robot.base.force_drive(0.25,0.0,0.0,0.75)        
        robot.base.force_drive(0.0,0.0,1.0,6.0)
        robot.base.force_drive(-0.25,0.0,0.0,0.75)
        
        robot.speech.speak("Verder heb ik twee mensachtige armen",language='nl',personality='marjolijn')
        robot.head.set_position(msgs.PointStamped(0,0,0,frame_id="/amigo/grippoint_left"), keep_tracking=True)
        robot.leftArm.send_joint_goal(-1.4,1.4,-0.8,1.2,-0.6,-0.4,-0.4)
        
        robot.speech.speak("Hiermee kan ik zowel hoog als laag iets oppakken",language='nl',personality='marjolijn')
        
        robot.spindle.send_goal(0.35)
        robot.leftArm.reset_arm()
        robot.rightArm.reset_arm()
        robot.head.reset_position()
        
        robot.rightArm.send_joint_goal(-1.2,1.2,-0.3,1.9,0,0,0)
        robot.speech.speak("Met de cameras in mijn hoofd kan ik mensen en dingen zien",language='nl',personality='marjolijn')
        
        robot.spindle.send_goal(0.05)
        robot.leftArm.send_joint_goal(-0.4,-0.2,0.6,1.2,0,0,0)
        robot.speech.speak("Met mijn laser kan ik zien waar ik ben",language='nl',personality='marjolijn')
        robot.speech.speak("En met mijn tweede camera zorg ik ervoor dat ik niet tegen tafels aan rijdt",language='nl',personality='marjolijn')
        
        robot.spindle.send_goal(0.35,timeout = 10.0)
        robot.leftArm.reset_arm()
        robot.rightArm.reset_arm()
        robot.head.reset_position()
        
        robot.speech.speak("Waar zou ik jullie nog meer mee kunnen helpen",language='nl',personality='marjolijn')
        
        return "finished_introduction"