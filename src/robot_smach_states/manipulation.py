#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_smach_states')
import rospy
import smach
import smach_ros
import time
import copy
from util import transformations

''' For siren demo challenge '''
import thread
import os

import threading #For monitoring the ROS topic while waiting on the answer
from std_msgs.msg import String
import geometry_msgs

from robot_skills.arms import State as ArmState

from human_interaction import Say

#TODO: Replace Point_location_hardcoded with a ArmToJointPos-sequence.
#TODO: Make Place_Object also use a query 

#Enum class.
class StandardPoses(object): 
    POINT_AT_OBJECT_BACKWARD = [-0.4 ,-0.750 , 0.50 , 1.50 , 0.000 , 0.7500 , 0.000]
    POINT_AT_OBJECT_FORWARD = [-0.2 ,-0.250 , 0.40 , 1.25 , 0.000 ,0.500 , 0.000]

class Prepare_orientation(smach.State):
    def __init__(self, side, robot, grabpoint_query, x_offset=None, y_offset=None):
        smach.State.__init__(self, outcomes=['orientation_succeeded','orientation_failed','abort','target_lost'])
        self.side = side
        self.robot = robot       
        self.grabpoint_query = grabpoint_query

        # New 23-03-2012
        self.nr_turns = -1
        self.grasp_angle = -0.0 #1.0472
        if x_offset == None:
            #import ipdb; ipdb.set_trace()
            rospy.logwarn("x_offset not specified, defaulting to 0.5 for arms.")
            self.grasp_distance_x = 0.5
        else:
            self.grasp_distance_x = x_offset

        if y_offset == None:
            #import ipdb; ipdb.set_trace()
            rospy.logwarn("y_offset not specified, defaulting to (-)0.2 for ... arm.")
            if self.side == self.robot.leftArm:
                self.grasp_distance_y = 0.2
            elif self.side == self.robot.rightArm:
                self.grasp_distance_y = -0.2
        else:
            self.grasp_distance_y = y_offset

    def execute(self, userdata):
        
        self.nr_turns = self.nr_turns + 1
        
        if self.nr_turns > 5:
            rospy.logerr('Unable to orientate to the object correctly')
            self.robot.speech.speak("I am terribly sorry, but I am not able to grasp the object")
            self.nr_turns = 0
            return 'orientation_failed'

        #import ipdb; ipdb.set_trace()
        answers = self.robot.reasoner.query(self.grabpoint_query)
        
        if answers:
            answer = answers[0] #TODO Loy/Sjoerd: sort answers by distance to gripper/base? 
            grasp_point = geometry_msgs.msg.PointStamped()
            grasp_point.header.frame_id = "/map"
            grasp_point.header.stamp = rospy.Time()
            grasp_point.point.x = float(answer["X"])
            grasp_point.point.y = float(answer["Y"])
            grasp_point.point.z = float(answer["Z"])
            
            
            grasp_point_BASE_LINK = transformations.tf_transform(grasp_point, "/map", "/base_link", self.robot.tf_listener)

            # desired_base_pose_BASE_LINK = geometry_msgs.msg.PoseStamped()

            # desired_base_pose_BASE_LINK.header.frame_id = "/base_link"
            # desired_base_pose_BASE_LINK.header.stamp = rospy.Time()

            # desired_base_pose_BASE_LINK.pose.position.x = grasp_point_BASE_LINK.x - self.grasp_distance_x
            # desired_base_pose_BASE_LINK.pose.position.y = grasp_point_BASE_LINK.y - self.grasp_distance_y
            # desired_base_pose_BASE_LINK.pose.position.z = 0

            # quat = tf.transformations.quaternion_from_euler(0, 0, self.grasp_angle)
            # desired_base_pose_BASE_LINK.pose.orientation.x = quat[0]
            # desired_base_pose_BASE_LINK.pose.orientation.y = quat[1]
            # desired_base_pose_BASE_LINK.pose.orientation.z = quat[2]
            # desired_base_pose_BASE_LINK.pose.orientation.w = quat[3]

            # desired_base_pose_MAP = self.robot.tf_transform_pose(desired_base_pose_BASE_LINK, "/map")
            
            desired_base_pose_MAP = self.robot.base.get_base_pose(grasp_point, self.grasp_distance_x, self.grasp_distance_y)
            
            ''' Sanity check: if the orientation is all zero, no feasible base pose has been found '''
            if (desired_base_pose_MAP.pose.orientation.x == 0 and desired_base_pose_MAP.pose.orientation.y == 0 and desired_base_pose_MAP.pose.orientation.z == 0 and desired_base_pose_MAP.pose.orientation.w == 0):
                self.robot.speech.speak("I am very sorry but this object is out of my reach",mood="sad")
                return 'orientation_failed'
            
            rospy.loginfo("[robot_smach_states] Desired target position: x = %f, y = %f",desired_base_pose_MAP.pose.position.x, desired_base_pose_MAP.pose.position.y)

            if self.robot.base.send_goal(desired_base_pose_MAP.pose.position, desired_base_pose_MAP.pose.orientation, time=60):
                '''wait for base goal to succeed for orientation'''
                #self.robot.base.wait(wait_time=60)
                rospy.loginfo("Moved the base to desired pose")  
                return 'orientation_succeeded'
            else:
                rospy.logerr("Not able to move base to desired pose")
                return 'orientation_failed'

        else:
            rospy.logerr("No answers for grabpoint_query {query}.".format(query=repr(self.grabpoint_query)))
            return 'target_lost'        

########################################### State Prepare grab###############################################
class Carrying_pose(smach.State):
    #Cannot be replaced by ArmToJointPos because sending an xyz goal also uses the spindle
    def __init__(self, arm, robot=None):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        
        self.robot = robot
        self.arm = arm

    def execute(self, gl):
        if self.arm == self.robot.leftArm:
            y_home = 0.199
        elif self.arm == self.robot.rightArm:
            y_home = -0.199
        
        rospy.loginfo("start moving to carrying pose")        
        if self.arm.send_goal(0.265, y_home, 0.816, 0, 0, 0, 60):
            rospy.loginfo("arm at carrying pose")   
            return 'succeeded'                 
        else:
            rospy.logerr("failed to go to the approach pose") 
            return 'failed' 
        
class Handover_pose(smach.State):
    def __init__(self, arm, robot=None):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        
        self.robot = robot
        self.arm = arm

    def execute(self, gl):
        if self.arm == self.robot.leftArm:
            y_home = 0.2
        elif self.arm == self.robot.rightArm:
            y_home = -0.2
        
        rospy.loginfo("start moving to handover pose")        
        if self.arm.send_goal(0.6, y_home, 0.816, 0, 0, 0, 30):
            rospy.loginfo("arm at handover pose")   
            return 'succeeded'                 
        else:
            rospy.logerr("failed to go to the handover pose") 
            return 'failed'
        
@smach.cb_interface(outcomes=['done'])
def reset_arm(userdata, side):
    side.reset_arm()
    return 'done'
        
########################################### State Grab ###############################################
class GrabMachine(smach.StateMachine):
    def __init__(self, side, robot, grabpoint_query):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])
        self.side = side
        self.robot = robot
        self.grabpoint_query = grabpoint_query
        '''check check input and output keys'''
        with self:
            smach.StateMachine.add('PREPARE_GRAB', Carrying_pose(self.side, self.robot),
                        transitions={'succeeded':'PREPARE_ORIENTATION','failed':'failed'})
        
            smach.StateMachine.add('PREPARE_ORIENTATION', Prepare_orientation(self.side, self.robot, grabpoint_query),
                        transitions={'orientation_succeeded':'GRAB','orientation_failed':'failed','abort':'failed','target_lost':'failed'})
        
            smach.StateMachine.add('GRAB', Grab(self.side, self.robot, grabpoint_query),
                        transitions={'grab_succeeded':'CARR_POS','grab_failed':'failed','target_lost':'failed'})
        
            smach.StateMachine.add('CARR_POS', Carrying_pose(self.side, self.robot),
                        transitions={'succeeded':'succeeded','failed':'failed'})
            
class Grab(smach.State):
    def __init__(self, side, robot, grabpoint_query):
        smach.State.__init__(self, outcomes=['grab_succeeded','grab_failed','target_lost'],
                                    input_keys=['world_info','target','object_position'])
        self.side = side
        self.robot = robot
        self.grabpoint_query = grabpoint_query
        
    def execute(self, gl):
        
        '''grasp_offset at between grippoint and object'''
        if self.side == self.robot.leftArm:
            go_x = 0.08
            go_y = -0.05 #0.025 #An offset of 7cm was obvserved...?
            go_z = 0.06
            end_effector_frame_id = "/grippoint_left"
            ar_frame_id = "/hand_marker_left"
        elif self.side == self.robot.rightArm:
            go_x = 0.06
            go_y = 0.025
            go_z = 0.05 
            end_effector_frame_id = "/grippoint_right"
            ar_frame_id = "/hand_marker_right"
        
        answers = self.robot.reasoner.query(self.grabpoint_query)
        
        if answers:
            answer = answers[0] #TODO Loy/Sjoerd: sort answers by distance to gripper/base? 
            target_position = geometry_msgs.msg.PointStamped()
            target_position.header.frame_id = "/map"
            target_position.header.stamp = rospy.Time()
            target_position.point.x = float(answer["X"])
            target_position.point.y = float(answer["Y"])
            target_position.point.z = float(answer["Z"])
            
            ''' Keep looking at end-effector for ar marker detection '''
            self.robot.head.set_position(0,0,0,frame_id=end_effector_frame_id,keep_tracking=True)
            
            rospy.loginfo("[robot_smach_states:grasp] Target position: {0}".format(target_position))
            
            '''problem: Point not defined'''
            #if self.side == self.robot.leftArm:
            #    self.robot.head.send_goal(Point(0.0,0.0,0.0) ,"/grippoint_left",1)
            #elif self.side == self.robot.rightArm:
            #    self.robot.head.send_goal(Point(0.0,0.0,0.0) ,"/grippoint_right",1)
            #publish_marker(target_position,12345,1.0,0.0,0.0)
            
                        #retrieve robot position and orientation
                        
            #self.robot.head.send_goal(target_position, frame_id="/map", timeout=4.0, keep_tracking=False)
            
            (robot_position, robot_orientation) = self.robot.base.location
            
            '''wait for the snapmap, average publish rate ~ 1.2 hz'''
            
            target_position_bl = transformations.tf_transform(target_position, "/map","/base_link", tf_listener=self.robot.tf_listener)
            
            rospy.loginfo("[robot_smach_states] Target position in base link: {0}".format(target_position_bl))
                        #rospy.loginfo("[robot_smach_states] Target position: %s",target_position)
        
            #retrieve robot position and orientation
            #(robot_position, robot_orientation) = self.robot.base.location
            
            
            
            rospy.loginfo("Open gripper")
            if self.side.send_gripper_goal_open(10):
                rospy.loginfo("Gripper opened")
            else:
                rospy.loginfo("opening gripper failed, good luck")
                return 'grab_failed'
            
            #rospy.sleep(2.0)
            rospy.loginfo("Moving arm")
            #send_gripper_goal("open", self.side, 10)
            
            ''' Move to pre-grasp position '''
            if self.side.send_goal(target_position_bl.x+go_x, target_position_bl.y+go_y, target_position_bl.z+go_z, 0, 0, 0, 120, pre_grasp = True, first_joint_pos_only=True):
                rospy.loginfo("Arm in pre-grasp pose: this is the time to start thinking about visual servoing")                    
            else:
                rospy.logerr("Failed to move arm to pre-grasp position")
                self.robot.speech.speak("I am sorry, but I cannot move my arm to the pre-grasp position")
                return 'grab_failed'
            
            ''' Toggle perception to measure the distance between the marker and the object '''
            self.robot.head.send_goal(target_position, frame_id="/map", timeout=4.0, keep_tracking=False)
            #rospy.logwarn("Perception at object location turned off")
            self.robot.perception.toggle_recognition(objects=True)
            rospy.sleep(rospy.Duration(2.0))
            self.robot.perception.toggle_recognition(False,False)
            
            self.robot.head.set_position(0,0,0, frame_id = end_effector_frame_id, keep_tracking=True)
            
            answers = self.robot.reasoner.query(self.grabpoint_query)
            
            if not answers:
                rospy.loginfo("No answers for query {0}".format(self.grabpoint_query))
                return "target_lost"
            answer = answers[0] #TODO Loy/Sjoerd: sort answers by distance to gripper/base? 
            target_position = geometry_msgs.msg.PointStamped()
            target_position.header.frame_id = "/map"
            target_position.header.stamp = rospy.Time()
            target_position.point.x = float(answer["X"])
            target_position.point.y = float(answer["Y"])
            target_position.point.z = float(answer["Z"])


            target_position_delta = geometry_msgs.msg.Point()
            
            ''' First check to see if visual servoing is possible '''
            try: 
                #import ipdb; ipdb.set_trace()
                target_position_delta = transformations.tf_transform(target_position, "/map", ar_frame_id, tf_listener=self.robot.tf_listener)
                ar_marker_available = True
            except Exception, ex:
                rospy.logerr(ex)
                ar_marker_available = False
                
            ''' If transform is not available, try again, but use head movement as well '''
            if not ar_marker_available:
                self.robot.perception.toggle_recognition(objects=True)
                self.robot.head.set_position(0,0,0,frame_id=end_effector_frame_id,keep_tracking=True)
                self.side.send_delta_goal(0.05,0.0,0.0,0.0,0.0,0.0, time_out=5.0, frame_id=end_effector_frame_id, pre_grasp = False)
                self.robot.speech.speak("What's that on my hand?")
                rospy.sleep(2.0)
                self.robot.perception.toggle_recognition(False,False)
                
            try: 
                target_position_delta = transformations.tf_transform(target_position, "/map", ar_frame_id, tf_listener=self.robot.tf_listener)
                ar_marker_available = True
            except Exception, ex:
                rospy.logerr(ex)
                ar_marker_available = False
                
            ''' Sanity check '''
            if ar_marker_available:
                rospy.loginfo("Delta target = {0}".format(target_position_delta))
                if (target_position_delta.x < 0 or target_position_delta.x > 0.6 or target_position_delta.y < -0.3 or target_position_delta.y > 0.3 or target_position_delta.z < -0.3 or target_position_delta.z > 0.3):
                    rospy.logwarn("Ar marker detection probably incorrect")
                    self.robot.speech.speak("I guess I cannot see my hand properly")
                    ar_marker_available = False
            
            ''' Original, pregrasp is performed by the compute_pre_grasp node '''
            if not ar_marker_available:
                self.robot.speech.speak("No visual feedback, let's see if I can grasp with my eyes closed")                
                if self.side.send_goal(target_position_bl.x+go_x, target_position_bl.y+go_y, target_position_bl.z+go_z, 0, 0, 0, 120, pre_grasp = True):
                    rospy.loginfo("arm at object")                    
                else:
                    if self.side.send_gripper_goal_close(10):
                        try:
                            #import ipdb;ipdb.set_trace()
                            self.robot.reasoner.attach_object_to_gripper(answer["ObjectID"], end_effector_frame_id, True)
                        except KeyError, ke:
                            rospy.logerr("Could not attach object to gripper, do not know which ID: {0}".format(ke))
                        rospy.loginfo("Gripper closed")
                    else:
                        rospy.loginfo("opening gripper failed, good luck")
                        return 'grab_failed'
                    rospy.logerr("failed to go to the arm position")
                    self.robot.speech.speak("I am sorry but I cannot move my arm to the object position")
                    
                    ''' Future: This should be a delta based on some visual feedback '''
            else:
                self.robot.speech.speak("I can see both my hand and the object, now I shouldn't miss")
                #import ipdb;ipdb.set_trace()
                if self.side.send_delta_goal(target_position_delta.x, target_position_delta.y, target_position_delta.z, 0, 0, 0, 120, frame_id=end_effector_frame_id, pre_grasp = True):                    
                    rospy.loginfo("arm at object")                    
                else:
                    if self.side.send_gripper_goal_close(10):
                        try:
                            #import ipdb;ipdb.set_trace()
                            self.robot.reasoner.attach_object_to_gripper(answer["ObjectID"], end_effector_frame_id, True)
                        except KeyError, ke:
                            rospy.logerr("Could not attach object to gripper, do not know which ID: {0}".format(ke))
                        rospy.loginfo("Gripper closed")
                    else:
                        rospy.loginfo("opening gripper failed, good luck")
                        return 'grab_failed'
                    rospy.logerr("failed to go to the arm position")
                    self.robot.speech.speak("I am sorry but I cannot move my arm to the object position")
                
            rospy.loginfo("Closing gripper")
            if self.side.send_gripper_goal_close(10):
                try:
                	#import ipdb;ipdb.set_trace()
                        self.robot.reasoner.attach_object_to_gripper(answer["ObjectID"], end_effector_frame_id, True)
                except KeyError, ke:
                        rospy.logerr("Could not attach object to gripper, do not know which ID: {0}".format(ke))
                rospy.loginfo("Gripper closed")
            else:
                rospy.loginfo("Closing gripper failed, good luck")
                # ToDo: shouldn't this be something different? Now we don't know why it didn't work
                return 'grab_failed'
                    
            #Lift arm
            if self.side.send_goal(target_position_bl.x+go_x, target_position_bl.y+go_y, target_position_bl.z+go_z+0.1, 0, 0, 0, 60, pre_grasp = False):
                rospy.loginfo("arm lifted 10 cm")   
            else:
                rospy.logwarn("failed to lift arm, continue with retract")
        
            #Move back to prepare distance
            #x_hand = tx - self.prepare_distance * cos(angle_hand)
            #y_hand = ty - self.prepare_distance * sin(angle_hand)
            if self.side.send_goal(target_position_bl.x-0.1, target_position_bl.y+go_y, target_position_bl.z+go_z+0.10, 0, 0, 0, 60, pre_grasp = False):
                rospy.loginfo("arm retracted")
            else:
                rospy.logwarn("problem with retracting arm, continuing ")
                
            self.robot.head.reset_position()
            return 'grab_succeeded'
        
        return 'target_lost'    

class Human_handover(smach.StateMachine):
    def __init__(self, side, robot=None):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])
        self.side = side
        self.robot = robot
        
        with self:
            smach.StateMachine.add("POSE", Carrying_pose(self.side,self.robot),
                            transitions={'succeeded':'OPEN_BEFORE_INSERT','failed':'OPEN_BEFORE_INSERT'})
            
            smach.StateMachine.add( 'OPEN_BEFORE_INSERT', SetGripper(robot, robot.leftArm, gripperstate=0), #open
                                transitions={   'state_set':'SAY1'})
            
            smach.StateMachine.add("SAY1", Say(self.robot,'Please hand over the object by sliding it in my gripper, I am not able to grasp'),
                            transitions={'spoken':'SAY2'})
            
            smach.StateMachine.add("SAY2", Say(self.robot,'Pretty please, hand over the object by sliding it in my gripper, I am not able to grasp'),
                            transitions={'spoken':'SAY3'})
            
            smach.StateMachine.add("SAY3", Say(self.robot,'Pretty please, with sugar on top, hand over the object!'),
                            transitions={'spoken':'CLOSE_AFTER_INSERT'})
            
            smach.StateMachine.add( 'CLOSE_AFTER_INSERT', SetGripper(robot, robot.leftArm, gripperstate=1), #close
                                transitions={   'state_set':'succeeded'})
                  
class Place_Object(smach.State):
    def __init__(self, side, robot=None):
        smach.State.__init__(self, outcomes=['object_placed'],
                                input_keys=['dropoff_location','locations'])
        self.side = side
        self.robot = robot
        
    def execute(self, gl):
        
        dropoff_height_offset = 0.2  #offset in meters
        # Simple implementation: move arm relative to base link and drop object
        # Future: specify exact target, move robot accordingly
        
        ''' Move upper body down '''
        # TODO: Is this necessary?
        #target_position = determine_target_position(gl.locations, gl.dropoff_location)
        #z_drop = target_position.z + dropoff_height_offset
        self.robot.spindle.send_goal(0.35)
        
        #y_drop = 0.7
        z_drop = 0.9
        
        ''' Move arm '''
        rospy.loginfo("Move arm to dropoff pose")
        # TODO: Make real difference between left and right arm
        head_goal = geometry_msgs.msg.Point()
        head_goal.x = 0.0
        head_goal.y = 0.0
        head_goal.z = 0.0
        if self.side == self.robot.leftArm:
            #y_drop = 0.3
            y_drop = 0.4
            self.robot.head.send_goal(head_goal,"/grippoint_left")
        elif self.side == self.robot.rightArm:
            #y_drop = -0.3
            y_drop = -0.4
            self.robot.head.send_goal(head_goal,"/grippoint_left")
        
        # Pose (left arm) Position: 0.24 0.43 0.15 Orientation: 0.12 0.58 0.09 0.80 
        if self.side.send_goal(0.5, y_drop, z_drop, 0.0 ,0.0 ,0.0 , time_out = 60, pre_grasp = False, frame_id = '/base_link'):
            rospy.loginfo("Arm at dropoff pose")
        else:
            rospy.logerr("Failed to reach dropoff pose")
        
        ''' Open gripper '''
        rospy.loginfo("Opening gripper")
        if self.side.send_gripper_goal_open(10):
            rospy.loginfo("Gripper opened")
        else:
            rospy.loginfo("Opening of gripper failed")
            # TODO: What are we going to do in this case?
        
        rospy.sleep(3.0)    
        
        #if self.side.send_goal(0.5, y_drop-0.2, z_drop, 0.0 ,0.0 ,0.0 , time_out = 60, pre_grasp = False, frame_id = '/base_link'):
        #    rospy.loginfo("Arm retracted")
        #else:
        #    rospy.logerr("Failed to retract arm")
        
        ''' Close gripper and move arm to suitable driving position, no need to wait '''    
        rospy.loginfo("Closing gripper")
        self.side.send_gripper_goal_close(0.01)
        
        #rospy.logwarn("Arms not yet reset, will probably hit the table")
        rospy.logwarn("Resetting arm: this does work for putting something on a table (needs a more refined approach)")
        self.side.reset_arm()
        self.robot.spindle.send_goal(0.35, 0.0 , 0.0, 0, 0.01)
        self.robot.head.reset_position()   
        
        return 'object_placed'

class SetGripper(smach.State):
    def __init__(self, robot, side, gripperstate=ArmState.OPEN, drop_from_frame=None):
        smach.State.__init__(self, outcomes=['state_set'])
        self.side = side
        self.robot = robot
        self.gripperstate = gripperstate
        self.drop_from_frame = drop_from_frame

    def execute(self, userdata):
        self.side.send_gripper_goal(self.gripperstate)
	
	if self.drop_from_frame:
		#import ipdb; ipdb.set_trace()
		self.robot.reasoner.detach_all_from_gripper(self.drop_from_frame)
	
        return "state_set"

class Gripper_to_query_position(smach.StateMachine):
    def __init__(self, robot, side, point_query):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed', 'target_lost'])
        self.side = side
        self.robot = robot
        self.point_query = point_query
        
        with self:
            smach.StateMachine.add("PREPARE",
                                    Prepare_orientation(side, robot, point_query),
                                    transitions={   'orientation_succeeded':'MOVE_ARM',
                                                    'orientation_failed':'failed',
                                                    'abort':'failed',
                                                    'target_lost':'target_lost'})
            #TODO: Replace with ArmToUserPose
            @smach.cb_interface(outcomes=['succeeded', 'failed', 'target_lost'])
            def move_to_point(userdata):
                #import ipdb; ipdb.set_trace()
                answers = self.robot.reasoner.query(self.point_query)
                
                if not answers:
                    rospy.loginfo("No answers for query {0}".format(self.point_query))
                    return "target_lost"
                else:
                    answer = answers[0]
                    target_position = geometry_msgs.msg.PointStamped()
                    target_position.header.frame_id = "/map"
                    target_position.header.stamp = rospy.Time()
                    target_position.point.x = float(answer["X"])
                    target_position.point.y = float(answer["Y"])
                    target_position.point.z = float(answer["Z"])
                    # Question: why is pre_grasp per definition true?
                    if self.side.send_goal(target_position.point.x, target_position.point.y, target_position.point.z, 0, 0, 0, 120, 
                                            pre_grasp=True, 
                                            frame_id="/map"):
                        return 'succeeded'                
                    else:
                        return 'failed'
            smach.StateMachine.add('MOVE_ARM', smach.CBState(move_to_point),
                                transitions={   'succeeded':'succeeded',
                                                'failed':'failed', 
                                                'target_lost':'target_lost'})

# ToDo Loy: ArmToJointPos is a much better name, this is confusing
class ArmToJointPos(smach.State):
    def __init__(self, robot, side, jointgoal):
        smach.State.__init__(self, outcomes=['done', "failed"])
        self.side = side
        self.robot = robot
        self.jointgoal = jointgoal

    def execute(self, userdata):
        result = self.side.send_joint_goal(*self.jointgoal)
        if result:
            return "done"
        else:
            return "failed"

ArmToPose = ArmToJointPos

class ArmToUserPose(smach.State):
    def __init__(self, side, x, y, z, roll=0, pitch=0, yaw=0, time_out=20, pre_grasp=False, frame_id="/base_link", delta=False):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.side = side
        self.x = x
        self.y = y
        self.z = z
        self.roll  = roll
        self.pitch = pitch
        self.yaw   = yaw
        self.time_out   = time_out
        self.pre_grasp = pre_grasp
        self.frame_id  = frame_id
        self.delta = delta

    def execute(self, userdata):
        if not self.delta:
            if self.side.send_goal(self.x, self.y, self.z, self.roll, self.pitch, self.yaw,
                time_out=self.time_out,
                pre_grasp=self.pre_grasp,
                frame_id=self.frame_id):
                return 'succeeded'
            else:
                return 'failed'
        else:
            if self.side.send_delta_goal(self.x, self.y, self.z, self.roll, self.pitch, self.yaw,
                time_out=self.time_out,
                pre_grasp=self.pre_grasp,
                frame_id=self.frame_id):
                return 'succeeded'
            else:
                return 'failed'
########################################### State Point ###############################################

class PointMachine(smach.StateMachine):
    def __init__(self, side, robot, point_query):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])
        self.side = side
        self.robot = robot
        self.point_query = point_query
        '''check check input and output keys'''
        with self:
            smach.StateMachine.add('PREPARE_POINT', Carrying_pose(self.side, self.robot),
                        transitions={'succeeded':'PREPARE_ORIENTATION','failed':'failed'})
        
            smach.StateMachine.add('PREPARE_ORIENTATION', Prepare_orientation(self.side, self.robot, point_query),
                        transitions={'orientation_succeeded':'POINT','orientation_failed':'failed','abort':'failed','target_lost':'failed'})
        
            smach.StateMachine.add('POINT', Point_at_object(self.side, self.robot, point_query),
                        transitions={'point_succeeded':'succeeded','point_failed':'failed','target_lost':'failed'})

class Point_at_object(smach.State):
    def __init__(self, side, robot, point_query):
        smach.State.__init__(self, outcomes=['point_succeeded','point_failed','target_lost'],
                                    input_keys=['world_info','target','object_position'])
        self.side = side
        self.robot = robot
        self.point_query = point_query
        
    def execute(self, gl):
        
        '''grasp_offset at between grippoint and object'''
        if self.side == self.robot.leftArm:
            go_x = 0.08
            go_y = -0.05 #0.025 #An offset of 7cm was obvserved...?
            go_z = 0.06
            end_effector_frame_id = "/grippoint_left"
            ar_frame_id = "/hand_marker_left"
        elif self.side == self.robot.rightArm:
            go_x = 0.06
            go_y = 0.025
            go_z = 0.05 
            end_effector_frame_id = "/grippoint_right"
            ar_frame_id = "/hand_marker_right"
        
        answers = self.robot.reasoner.query(self.point_query)
        
        if answers:
            answer = answers[0] #TODO Loy/Sjoerd: sort answers by distance to gripper/base? 
            target_position = geometry_msgs.msg.PointStamped()
            target_position.header.frame_id = "/map"
            target_position.header.stamp = rospy.Time()
            target_position.point.x = float(answer["X"])
            target_position.point.y = float(answer["Y"])
            target_position.point.z = float(answer["Z"])
            
            ''' Keep looking at end-effector for ar marker detection '''
            self.robot.head.set_position(0,0,0,frame_id=end_effector_frame_id,keep_tracking=True)
            
            rospy.loginfo("[robot_smach_states:grasp] Target position: {0}".format(target_position))
            
            '''problem: Point not defined'''
            #if self.side == self.robot.leftArm:
            #    self.robot.head.send_goal(Point(0.0,0.0,0.0) ,"/grippoint_left",1)
            #elif self.side == self.robot.rightArm:
            #    self.robot.head.send_goal(Point(0.0,0.0,0.0) ,"/grippoint_right",1)
            #publish_marker(target_position,12345,1.0,0.0,0.0)
            
                        #retrieve robot position and orientation
                        
            #self.robot.head.send_goal(target_position, frame_id="/map", timeout=4.0, keep_tracking=False)
            
            (robot_position, robot_orientation) = self.robot.base.location
            
            '''wait for the snapmap, average publish rate ~ 1.2 hz'''
            
            target_position_bl = transformations.tf_transform(target_position, "/map","/base_link", tf_listener=self.robot.tf_listener)
            
            rospy.loginfo("[robot_smach_states] Target position in base link: {0}".format(target_position_bl))
                        #rospy.loginfo("[robot_smach_states] Target position: %s",target_position)
        
            #retrieve robot position and orientation
            #(robot_position, robot_orientation) = self.robot.base.location
            
            
            
            rospy.loginfo("Close gripper")
            if self.side.send_gripper_goal_close(10):
                rospy.loginfo("Gripper closed")
            else:
                rospy.loginfo("opening gripper failed, good luck")
                return '_failed'
            
            #rospy.sleep(2.0)
            rospy.loginfo("Moving arm")
            #send_gripper_goal("open", self.side, 10)
            
            ''' Move to pre-point position '''
            if self.side.send_goal(target_position_bl.x+go_x, target_position_bl.y+go_y, target_position_bl.z+go_z, 0, 0, 0, 120, pre_grasp = True, first_joint_pos_only=True):
                rospy.loginfo("Arm in pre-point pose: this is the time to start thinking about visual servoing")                    
            else:
                rospy.logerr("Failed to move arm to pre-point position")
                self.robot.speech.speak("I am sorry, but I cannot move my arm to the pre-point position")
                return 'point_failed'
            
            ''' Toggle perception to measure the distance between the marker and the object '''
            self.robot.head.send_goal(target_position, frame_id="/map", timeout=4.0, keep_tracking=False)
            #rospy.logwarn("Perception at object location turned off")
            self.robot.perception.toggle_recognition(objects=True)
            rospy.sleep(rospy.Duration(2.0))
            self.robot.perception.toggle_recognition(False,False)
            
            self.robot.head.set_position(0,0,0, frame_id = end_effector_frame_id, keep_tracking=True)
            
            answers = self.robot.reasoner.query(self.point_query)
            
            if not answers:
                rospy.loginfo("No answers for query {0}".format(self.point_query))
                return "target_lost"
            answer = answers[0] #TODO Loy/Sjoerd: sort answers by distance to gripper/base? 
            target_position = geometry_msgs.msg.PointStamped()
            target_position.header.frame_id = "/map"
            target_position.header.stamp = rospy.Time()
            target_position.point.x = float(answer["X"])
            target_position.point.y = float(answer["Y"])
            target_position.point.z = float(answer["Z"])


            target_position_delta = geometry_msgs.msg.Point()
            
            ''' First check to see if visual servoing is possible '''
            try: 
                #import ipdb; ipdb.set_trace()
                target_position_delta = transformations.tf_transform(target_position, "/map", ar_frame_id, tf_listener=self.robot.tf_listener)
                ar_marker_available = True
            except Exception, ex:
                rospy.logerr(ex)
                ar_marker_available = False
                
            ''' If transform is not available, try again, but use head movement as well '''
            if not ar_marker_available:
                self.robot.perception.toggle_recognition(objects=True)
                self.robot.head.set_position(0,0,0,frame_id=end_effector_frame_id,keep_tracking=True)
                self.side.send_delta_goal(0.05,0.0,0.0,0.0,0.0,0.0, time_out=5.0, frame_id=end_effector_frame_id, pre_grasp = False)
                self.robot.speech.speak("What's that on my hand?")
                rospy.sleep(2.0)
                self.robot.perception.toggle_recognition(False,False)
                
            try: 
                target_position_delta = transformations.tf_transform(target_position, "/map", ar_frame_id, tf_listener=self.robot.tf_listener)
                ar_marker_available = True
            except Exception, ex:
                rospy.logerr(ex)
                ar_marker_available = False
                
            ''' Sanity check '''
            if ar_marker_available:
                rospy.loginfo("Delta target = {0}".format(target_position_delta))
                if (target_position_delta.x < 0 or target_position_delta.x > 0.6 or target_position_delta.y < -0.3 or target_position_delta.y > 0.3 or target_position_delta.z < -0.3 or target_position_delta.z > 0.3):
                    rospy.logwarn("Ar marker detection probably incorrect")
                    self.robot.speech.speak("I guess I cannot see my hand properly")
                    ar_marker_available = False
            
            ''' Original, prepoint is performed by the compute_pre_grasp node '''
            if not ar_marker_available:
                self.robot.speech.speak("No visual feedback, let's see if I can point with my eyes closed")                
                if self.side.send_goal(target_position_bl.x+go_x-0.1, target_position_bl.y+go_y, target_position_bl.z+go_z, 0, 0, 0, 120, pre_grasp = True):
                    rospy.loginfo("arm at object")                    
                else:
                    rospy.logerr("failed to go to the arm position")
                    self.robot.speech.speak("I am sorry but I cannot move my arm to the object position")
                    return 'point_failed'
                    
                    ''' Future: This should be a delta base on some visual feedback '''
            else:
                self.robot.speech.speak("I can see both my hand and the object, I will show you where the object is!")
                if self.side.send_delta_goal((target_position_delta.x-0.1), target_position_delta.y, target_position_delta.z, 0, 0, 0, 120, frame_id=end_effector_frame_id, pre_grasp = True):
                    self.robot.speech.speak("This is what you are looking for, am I right?")
                    rospy.sleep(1)
                    rospy.loginfo("arm at object")                    
                else:
                    rospy.logerr("failed to go to the arm position")
                    self.robot.speech.speak("I am sorry but I cannot move my arm to the object position")
                    return 'point_failed'
                        
            #Move back to prepare distance
            #x_hand = tx - self.prepare_distance * cos(angle_hand)
            #y_hand = ty - self.prepare_distance * sin(angle_hand)
            if self.side.send_goal(target_position_bl.x-0.1, target_position_bl.y+go_y, target_position_bl.z+go_z+0.10, 0, 0, 0, 60, pre_grasp = False):
                rospy.loginfo("arm retracted")
            else:
                rospy.logwarn("problem with retracting arm, continuing ")

            self.side.reset_arm()
                
            self.robot.head.reset_position()
            return 'point_succeeded'
        
        return 'target_lost'                

class Point_location_hardcoded(smach.State):
    def __init__(self, robot, side):
        smach.State.__init__(self, outcomes=['pointed'])
        self.side = side
        self.robot = robot
        self.i = 0

    def execute(self, userdata):

        self.side.send_gripper_goal_close()
        i = self.i
        while i < 2: 
            #Backwards
            self.side.send_joint_goal(-0.4 ,-0.750 , 0.50 , 1.50 , 0.000 , 0.7500 , 0.000)
            rospy.sleep(3.2)
            #Forwards
            self.side.send_joint_goal(-0.2 ,-0.250 , 0.40 , 1.25 , 0.000 ,0.500 , 0.000)
            rospy.sleep(3.2)
            i += 1

        self.side.reset_arm()

        return 'pointed'
        
