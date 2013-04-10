#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_smach_states')
import rospy
import smach
import smach_ros
#from object_msgs.msg import ExecutionTarget 
from exc_to_ros import *
from ros_to_exc import *
from exc_functions import *
from ros_functions import *
import time
import copy

''' For siren demo challenge '''
import thread
import os

import threading #For monitoring the ROS topic while waiting on the answer
from std_msgs.msg import String

import components

class Help_up(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot
        
    def execute(self, userdata):
        res2 = self.robot.leftArm.send_joint_goal(0, 0.79, 0, 0, 0, 0, 0)
        res3 = self.robot.rightArm.send_joint_goal(0, 0.79, 0, 0, 0, 0, 0)
        res1 = self.robot.spindle.send_goal(0.0, 5)
        rospy.sleep(10)
        if not (res1 and res2 and res3):
            return "failed"
        
        self.robot.speech.speak("Get my hands, I will help you get back on your feet!")
        
        res4 = self.robot.spindle.send_goal(0.4)
        res5 = self.robot.base.force_drive(-0.2, 0, 0, 5)
        
        #rospy.sleep(1.5)
        
        self.robot.leftArm.reset_arm()
        self.robot.rightArm.reset_arm()
        self.robot.head.reset_position()
        
        if not (res4 and res5):
            return "failed"
        
        return "succeeded"
