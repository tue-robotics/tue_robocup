#! /usr/bin/env python
import roslib; roslib.load_manifest('tue_execution_pack')
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

class Target_lost_follow(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self,
                                   outcomes=['target_found','target_really_lost'],
                                    input_keys=['world_info','target','rate'],
                                    output_keys=['target'])
        self.robot = robot
                                    
    def execute(self, gl):
        self.robot.speech.speak("Lost target. Will wait some time until target returns")
        i = 0
        k = 100 #time till target is really lost
        self.robot.base.cancel_goal()
        while not rospy.is_shutdown() and i < (gl.rate*k):
            
            rospy.sleep(1/gl.rate)
            
            if self.robot.worldmodel.target_is_available(gl.target):
                self.robot.speech.speak("Target found")
                rospy.sleep(1)
                return 'target_found'
            
            i = i + 1
        
        self.robot.base.cancel_goal()
        self.robot.speech.speak("I have lost you")
        
        return 'target_really_lost'
     
class Identify_Follow_Me(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self,
                                   outcomes=['identifying','identified','who_is_who_finished','abort'],
                                   input_keys=['rate','command','world_info','target'])
        self.target = ExecutionTarget()
        self.sentence = ""
        self.no_of_known_identifications = 0
        self.total_no_of_identifications = 0
        self.identifiedID = []
        self.robot = robot
        self.face_detection_started = False
    
    def execute(self, gl):
        self.robot.speech.speak('Found person, Identifying..')
        if not self.face_detection_started:
            rospy.loginfo("Starting face recog.")
            self.robot.perception.toggle_recognition(faces=True)
        else:
            rospy.loginfo("Face recog. already started")
        # start recognizing
        loop_rate(gl.rate)
        
        if gl.command == "abort":
            if self.face_detection_started:
                self.robot.perception.toggle_recognition(faces=False)
            return 'abort'
        else:
            rospy.loginfo("Waiting until on face recog. until ID '{0}' is available in WM".format(gl.target.ID))
            self.robot.perception.toggle_recognition(faces=True)
            #tries = 0
            if self.robot.worldmodel.target_is_available_by_ID(gl.target.ID):
                rospy.loginfo("ID '{0}' available in WM!".format(gl.target.ID))
                #tries += 1
                self.robot.perception.toggle_recognition(faces=False)
                self.robot.speech.speak('Identified person')
                return "identified"
            else:
                rospy.loginfo("ID '{0}' was not found in time".format(gl.target.ID))
                return 'identifying'    
   
class Stop_follow(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self,
                                   outcomes=['stop_finished'],
                                input_keys=['rate','world_info','target'],
                                output_keys=['target'])
        self.robot = robot
                                
    def execute(self, gl):
                
                gl.target.name = "operator"
                
                self.robot.speech.speak("I will now wait for you, so that you can hide and then return with one other person and then I will try continue following you again.")
                self.robot.speech.speak("Is this what you meant when you told me to stop?")
                
                
                
                rospy.sleep(7.5)
                
                self.robot.head.reset_position()
                return 'stop_finished'