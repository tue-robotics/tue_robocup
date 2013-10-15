#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_smach_states')
import rospy
import smach
import smach_ros

import robot_skills.util.msg_constructors as msgs

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
        self.robot.head.set_position(msgs.PointStamped(0,0,0,frame_id="/amigo/grippoint_left"),keep_tracking=True)
        self.robot.leftArm.send_joint_goal(-1.4,1.4,-0.8,1.2,-0.6,-0.4,-0.4)
        
        self.robot.speech.speak("With these, I can reach both high and low")
        
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
        self.robot.head.set_position(msgs.PointStamped(0,0,0,frame_id="/amigo/grippoint_left"), keep_tracking=True)
        self.robot.leftArm.send_joint_goal(-1.4,1.4,-0.8,1.2,-0.6,-0.4,-0.4)
        
        self.robot.speech.speak("Hiermee kan ik zowel hoog als laag iets oppakken",language='nl',personality='marjolijn')
        
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

''' Additions for the reception of the queen during RoboCup 2013 '''
class PrepareWelcomeMaxima(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot
        
    def execute(self, gl):
        self.robot.spindle.reset()
        self.robot.head.reset_position()
        self.robot.leftArm.reset_arm()
        self.robot.rightArm.send_joint_goal(-0.15, -0.22, 0.70, 1.77, -0.08, 0.09, -0.11)
        self.robot.lights.set_color(1,0.25,0)
        return "done"

class WelcomeMaxima1(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot
        
    def execute(self, gl):
        ''' Intro talk '''
        self.robot.head.send_goal(msgs.PointStamped(1.0, 0, 1.6, "/base_link"), 0)
        self.robot.speech.speak("Welkom majesteit","nl","marjolijn","default","neutral",block=True)
        self.robot.speech.speak("Mijn naam is amigo, de zorgrobot van de Technische Universiteit Eindhoven","nl","marjolijn","default","neutral",block=True)
        self.robot.speech.speak("Namens de organisatie heet ik u van harte welkom op RoboCup twee duizend dertien","nl","marjolijn","default","neutral",block=True)
        #self.robot.head.look_up()
        return "done"
        
class WelcomeMaxima2(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot
        
    def execute(self, gl):
        ''' Overhandigen bloemen '''
        self.robot.speech.speak("Graag bied ik u dit boeket aan!","nl","marjolijn","default","neutral",block=False)
        self.robot.spindle.high()
        self.robot.head.send_goal(msgs.PointStamped(0.0,0,0.0, "/amigo/grippoint_right"),0)
        self.robot.rightArm.send_joint_goal(-0.1, 0.58, 0.49, 1.62, -0.21, 0.09, -0.11, timeout=2.0)
        self.robot.head.send_goal(msgs.PointStamped(1.0,0,1.6,"/base_link"),0)
        self.robot.rightArm.send_joint_goal(-0.1, 0.58, 0.49, 1.62, -0.41, -0.51, 0.1, timeout=1.5)
        self.robot.rightArm.send_joint_goal(-0.1, 0.88, 0.49, 1.32, -0.41, -0.51, 0.1)
        #self.robot.rightArm.send_joint_goal(-0.02744625, 0.11145711, 0.15037341, 2.214187105, -0.108680065, 0.08952796, 0.05084748)
        #self.robot.rightArm.send_joint_goal(-0.1, 0.73425897, 0.17566431, 0.53222897, -0.14837615, 0.25673428, 0.00852748)
        #rospy.sleep(rospy.Duration(2.0))

        return "done"

class WelcomeMaxima3(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot
        
    def execute(self, gl):
        ''' Reset robot '''
        #self.robot.head.reset_position()
        self.robot.speech.speak("Tot slot wens ik u veel plezier tijdens uw bezoek","nl","marjolijn","default","neutral",block=False)
        self.robot.spindle.reset()
        self.robot.leftArm.reset_arm()
        self.robot.rightArm.reset_arm()
        return "done"

