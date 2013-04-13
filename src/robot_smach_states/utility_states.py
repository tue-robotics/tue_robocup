#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_smach_states')
import rospy
import smach
import smach_ros
#from object_msgs.msg import ExecutionTarget 
import time
import copy

''' For siren demo challenge '''
import thread
import os

import threading #For monitoring the ROS topic while waiting on the answer
from std_msgs.msg import String

from psi import *

class Initialize(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self, outcomes=['initialized',
                                             'abort'])
        self.robot = robot

    def execute(self, userdata):
        self.robot.head.reset_position()
        self.robot.leftArm.reset_arm()
        #self.robot.leftArm.send_gripper_goal_close()
        self.robot.rightArm.reset_arm()
        #self.robot.rightArm.send_gripper_goal_close()
        self.robot.reasoner.reset()
        self.robot.spindle.reset()


        return 'initialized'

class InitializeOld(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self,
                                   outcomes=['initializing','initialized','abort'],
                                   input_keys=['rate','command','challenge','world_info','target','locations','key_cmd','initial_pose','emergency_switch','start_time'],
                                   output_keys=['target','start_time','no_people_to_be_learned','global_end_time'])
        self.no_of_initializations = 0
        self.robot = robot
        
    def set_home_location(self):
        home = geometry_msgs.msg.Point()
        try:
            self.robot.tf_listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(2.0))
            (ro_trans, ro_rot) = self.robot.tf_listener.lookupTransform("/map", "/base_link", rospy.Time())
            
            home.x = ro_trans[0]
            home.y = ro_trans[1]
            
            return home
        except (tf.LookupException, tf.ConnectivityException):
            rospy.logwarn("home location not retrieved!!")
            return home

    def execute(self, gl):
        
        loop_rate(gl.rate)
        
        self.robot.head.reset_position()
        self.robot.leftArm.reset_arm()
        self.robot.rightArm.reset_arm()
        self.robot.worldmodel.reset()
        # no_persons: number of persons that have to be learned
        print "Starting to execute", gl.challenge
        # Clean up
        if gl.challenge == "clean_up":
            
            ''' test if  the target.name is used, if not, delete these lines'''
            # gl.target.name = "bottle_1"
            # gl.target.class_label = "Bottle"
            
            
            ''' Comment this line for integral testing ''' 
            #rospy.logwarn("No inital pose set, should be enabled before real challenge!!!")
            set_initial_pose(gl.initial_pose)
            return 'initialized'
        
        # Go get it       
        if gl.challenge == "go_get_it":
            gl.target.name = "bottle_1"
            gl.target.class_label = "Bottle"
            
            # Commented to test manipulation
            #rospy.logwarn("No inital pose set, should be enabled before real challenge!!!")
            set_initial_pose(gl.initial_pose)
            return 'initialized'
            
        # Registration
        elif gl.challenge == "registration":
            gl.target.name = "desk"
            gl.target.class_label = "location"
            
            set_initial_pose(gl.initial_pose)
            
            #send_arm_left_goal(0.4,0.2,0.17,0.0,0.67,0.0,10)
            
            #self.robot.spindle.send_spindle_goal(0.8)
            
            self.robot.leftArm.send_joint_goal(-0.1,-0.2,0.2,0.8,0.0,0.0,0.0)
            self.robot.rightArm.send_joint_goal(-0.1,-0.2,0.2,0.8,0.0,0.0,0.0)
            
            return 'initialized'
        
        # Follow me
        elif gl.challenge == "follow_me":
            gl.target.class_label = "person"
            gl.target.ID = -1
            rospy.loginfo("target.ID = {0}".format(gl.target.ID))
            #Do it once more, disable if this causes problems
            self.robot.leftArm.reset_arm()
            self.robot.rightArm.reset_arm()
            return 'initialized'
            
        # Who is who
        elif gl.challenge == "who_is_who":
            gl.no_people_to_be_learned = 2
            gl.target.name = ""
            gl.target.class_label = "person"
            return 'initialized'
        
        # Who is who
        elif gl.challenge == "who_is_who_2012":
            gl.no_people_to_be_learned = 3
            gl.target.name = "learn_person_pos"
            gl.target.class_label = "exploration"
            return 'initialized'

        # Who is who enhanced
        elif gl.challenge == "who_is_who_enhanced":
            gl.no_people_to_be_learned = 3
            gl.target.name = ""
            gl.target.class_label = "person"
            gl.home = self.set_home_location()
            return 'initialized'
        
        # Cups
        elif gl.challenge == "cups":
            
            gl.target.class_label = "cup"
            gl.target.name = "cup_with_ball"
            
            self.robot.head.reset_position()
            
            if gl.command == "cups" or gl.key_cmd == "z":
                self.robot.speech.speak("Yeah sure!")
                self.robot.speech.speak("I like to play cups")
                
                
                
                gl.command = " "
                gl.key_cmd = " "
                
                return 'initialized'
            
            return 'initializing'
        
        elif gl.challenge == "restaurant":
            self.robot.head.reset_position()
            self.robot.leftArm.reset_arm()
            self.robot.rightArm.reset_arm()
            
            while gl.emergency_switch == True:
                rospy.loginfo("Release emergency button to start")
                rospy.sleep(0.1)
            
            return 'initialized'
        
        elif (gl.challenge == "open_challenge_2012" or gl.challenge == "demo_challenge_2012"):
            
            #rospy.logwarn("No initial pose set")
            set_initial_pose(gl.initial_pose)
            self.robot.spindle.send_goal(0.35)
            self.robot.leftArm.reset_arm()
            self.robot.rightArm.reset_arm()
            ''' Wait until the ebutton is released '''
            while self.robot.ebutton.read_ebutton() == True:
                rospy.loginfo("Release emergency button to start")
                rospy.sleep(1)
                
            ''' When the ebutton is released '''
            gl.start_time = set_start_time()
            
            thread.start_new_thread(os.system,('sleep 5; aplay /home/amigo/Music/siren1.wav',))
            
            return 'initialized'
                
                # Final Challenge
        elif gl.challenge == "rdo_finale":
            self.robot.head.reset_position()
            self.robot.leftArm.reset_arm()
            self.robot.rightArm.reset_arm()
            
            return 'initialized'
            
        elif gl.command == "abort":
            return 'abort'
        
        else:
            return 'initializing'

class Introduction_follow_me(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self,
                                   outcomes=['introducing','introduced','abort'],
                                input_keys=['rate','command','world_info','target'],
                                output_keys=['rate', 'target'])
        self.no_of_introductions = 0
        self.robot = robot
        
    def execute(self, gl):
        
        loop_rate(gl.rate)
        
        # only introduce when iterating for the first time
        if self.no_of_introductions == 0:
            self.robot.speech.speak("Before I start following you, let me take a look at you so I can follow you better")
            self.robot.speech.speak("Please stand in front of me and look into my eyes")
                
        i = 0
        j = 0
        # check if a person is found
        rospy.loginfo("Check for person")
        while not self.robot.target_closer_than(2, class_label="person") and i < 100 and not rospy.is_shutdown():
            rospy.logdebug("Waiting for target '{0}' to be available".format(gl.target))
            rospy.sleep(0.05)
            i = i + 1
                        
            if i == 100:
                self.robot.speech.speak("I cannot see you properly. Please stand in front of me")
                       
                i = 0
                j = j + 1
                
                if j > 6:
                    rospy.loginfo("No person available in LabeledDetectionsArray")
                    self.robot.speech.speak("I am terribly sorry")
                    self.robot.speech.speak("I cannot see any people")
                    self.robot.speech.speak("I give up on the follow me challenge")
                    return 'abort'
                
        # Determine which ID from the SeenObjectsArray is to be tracked
        # Hereto, the robots own position should be known
        (robot_position, robot_orientation) = self.robot.base.get_location()
        tempID = self.robot.worldmodel.assign_ID(gl.target, robot_position, robot_orientation)
        if tempID == -1:
            return 'introducing'
        else:
            gl.target.ID = tempID
                    
        # determine position
        target_position = self.robot.worldmodel.determine_target_position(gl.target)
        gl.target.name = 'operator'
        return 'introduced'


class Set_initial_pose(smach.State):
    ## To call upon this state:
    # example 1: Set_initial_pose(robot, "front_of_door"),
    # OR
    # Set_initial_pose(robot, [1,0,0])
    def __init__(self, robot, init_position):       
        smach.State.__init__(self, outcomes=["done", "preempted", "error"])

        self.robot = robot
        self.preempted = False

        self.initial_position = init_position
    
    def location_2d(self, location):
        query_location = self.robot.reasoner.base_pose(location, self.robot.reasoner.pose_2d("X", "Y", "Phi"))
        
        answers = self.robot.reasoner.query(query_location)
        
        if not answers:
            rospy.logerr("No answers found for query {query}".format(query=query_location))
            return []
        else:
            possible_locations = [(float(answer["X"]), float(answer["Y"]), float(answer["Phi"])) for answer in answers]
        
        x, y, phi = min(possible_locations)

        return x, y, phi

    def execute(self, userdata):
        rospy.loginfo('Set initial pose')

        if isinstance(self.initial_position, str):
            x,y,phi = self.location_2d(self.initial_position)
        elif len(self.initial_position) == 3: #Tuple or list        
            x = self.initial_position[0]
            y = self.initial_position[1]
            phi = self.initial_position[2]
        else:
            rospy.logerr("Initial pose {0} could not be set".format(self.initial_position))
            return "error"

        self.robot.base.set_initial_pose(x,y,phi)

        return "done"

class Pause(smach.State):
    def __init__(self, robot=None, timeout = 300.0):
        # Default timeout of 5 minutes
        smach.State.__init__(self, outcomes=['pausing','pause_done','abort'],
                                    input_keys=['rate','command'])
        self.robot = robot
        self.waiting = False # Bool indicating when waiting is turned on, used for timeout
        self.timeout = timeout
    
    def execute(self, gl):
        
        loop_rate(gl.rate)
        
        if self.waiting == False:
            self.start_time = rospy.Time.now()
            self.waiting = True
        
        # wait for command from operator
        if gl.command == "abort":
            self.waiting = False
            return 'abort'
        elif ((gl.command == "continue") or ((rospy.Time.now()-self.start_time)>rospy.Duration(self.timeout))):
            self.waiting = False
            return 'pause_done'
        else:
            return 'pausing'

############################## State Wait ##############################
class Wait_time(smach.State):
    def __init__(self, robot=None, waittime=10):
        smach.State.__init__(self, outcomes=['waited','preempted'])
        self.robot = robot
        self.waittime = waittime
    
    def execute(self, *args, **kwargs):
        total_sleep = 0
        sleep_interval = 0.1
        #import ipdb; ipdb.set_trace()
        while total_sleep <= self.waittime:
            rospy.sleep(sleep_interval)
            total_sleep += sleep_interval
            if self.preempt_requested():
                rospy.loginfo('Wait_time preempted at {0} of {1}'.format(total_sleep, self.waittime))
                self.service_preempt()
                return 'preempted'
        return 'waited'

class Wait(smach.State):
    def __init__(self, robot=None, waittime=10):
        smach.State.__init__(self,
                                   outcomes=['waiting_finished'],
                                input_keys=['rate','world_info','target'])
        self.robot = robot
        self.waittime = waittime
                                
    def execute(self, gl):
        
        i = 0
        self.robot.speech.speak("I will wait here for {0} seconds".format(self.waittime))

        wait_time = self.waittime
        start_time = rospy.Time.now()
        while (not rospy.is_shutdown()) and (rospy.Time.now() - start_time) < rospy.Duration(wait_time):

            rospy.sleep(0.2)
            
            if self.robot.worldmodel.target_is_available(gl.target):
                
                target_position = self.robot.worldmodel.determine_target_position(gl.target)
                
                self.robot.head.send_goal(target_position, "/map")
            i = i + 1
            
        self.robot.speech.speak("I am done waiting")
        return 'waiting_finished'

class Wait_Condition(smach.State):
    '''Wait until a condition is satisfied, possible on a robot. 
    When the condtion is satisfied, the value that matched the condition is stored in the userdata.
    The callback must return that value or something that evaluates to False otherwise.
    The arguments to the callback are userdata, robot'''
    def __init__(self, robot, condition_callback, timeout):
        ''' '''
        smach.State.__init__(self,
                             outcomes=['satisfied', 'timed_out', 'preempted'],
                             output_keys=['trigger_value'])
        self.condition_callback = condition_callback
        self.robot = robot
        self.timeout = timeout

    def execute(self, userdata):
        starttime = rospy.Time.now()

        while (rospy.Time.now() - starttime) < rospy.Duration(self.timeout)\
         and not rospy.is_shutdown():
            cb_output = self.condition_callback(userdata, self.robot)
            if cb_output:
                userdata.trigger_value = cb_output
                return "satisfied"
            if self.preempt_requested():
                self.service_preempt()
                return "preempted"
            rospy.sleep(0.1)
        return 'timed_out'

class Finish(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self,
                                   outcomes=['stop'])
        self.robot = robot
    
    def execute(self, gl):
        
        print "Finished executing task."
        return 'stop'
    
class FinishOld(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self,
                                   outcomes=['stop'],input_keys=['challenge','start_time'])
        self.robot = robot
    
    def execute(self, gl):
        
        duration = calculate_duration(gl.start_time)
        print "Finished executing task", gl.challenge, "in", duration, "seconds."
        return 'stop'

class PlaySound(smach.State):
    def __init__(self, filename, blocking=False):
        smach.State.__init__(self, outcomes=['played','error'])
        self.file = filename
        self.blocking = blocking
        
    def execute(self, ud):
        try:
            rospy.loginfo("Playing {0}".format(self.file))
            if self.blocking:
                os.system('aplay {0}'.format(self.file))
            else:
                thread.start_new_thread(os.system,('aplay {0}'.format(self.file),))
            return "played"
        except:
            return "error"
