#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_smach_states')
import rospy
import smach
import smach_ros
import time
import copy
import navigation
from util import transformations
from psi import Term, Compound, Conjunction

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
# ToDo: move to robot_skills because this is robot specific
class StandardPoses: 
    POINT_AT_OBJECT_BACKWARD = [-0.4 ,-0.750 , 0.50 , 1.50 , 0.000 , 0.7500 , 0.000]
    POINT_AT_OBJECT_FORWARD = [-0.2 ,-0.250 , 0.40 , 1.25 , 0.000 ,0.500 , 0.000]
    HOLD_TRAY_POSE = [-0.1, 0.13, 0.4, 1.5, 0, 0.5, 0]
    SUPPORT_PERSON_POSE = [-0.1, -1.57, 0, 1.57, 0,0,0]
    RESET_POSE = [-0.1, 0.13, 0, 0.3, 0, 0.3, 0]

#class GrabMachine(smach.StateMachine):
#    def __init__(self, side, robot, grabpoint_query):
#        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])
#        self.side = side
#        self.robot = robot
#       self.grabpoint_query = grabpoint_query
#        '''check check input and output keys'''
#        with self:
#            smach.StateMachine.add('PREPARE_GRAB', PrepareGrasp(self.side, self.robot, self.grabpoint_query),
#                        transitions={'succeeded'    :   'PREPARE_ORIENTATION',
#                                     'failed'       :   'failed'})

class DetermineBaseGraspPose(smach.State):
    def __init__(self, side, robot, grabpoint_query, x_offset=None, y_offset=None):
        smach.State.__init__(self, outcomes=['succeeded','failed','target_lost'])
        self.side = side
        self.robot = robot
        self.grabpoint_query = grabpoint_query
        if x_offset == None:
            rospy.logwarn("x_offset not specified, defaulting to 0.5 for arms.")
            self.x_offset = 0.5
        else:
            self.x_offset = x_offset

        if y_offset == None:
            rospy.logwarn("y_offset not specified, defaulting to (-)0.2 for ... arm.")
            if self.side == self.robot.leftArm:
                self.y_offset = 0.2
            elif self.side == self.robot.rightArm:
                self.y_offset = -0.2
        else:
            self.y_offset = y_offset

        self.nr_inverse_reachability_calls = 0
        self.max_nr_inverse_reachability_calls = 2
        self.desired_base_poses_MAP = []
        self.grasp_point = geometry_msgs.msg.PointStamped()

    def execute(self, userdata):

        ''' Only query the reasoner once (grasp position will not be updated anyway so there is no use in doing this more often)'''
        if self.nr_inverse_reachability_calls == 0:
            answers = self.robot.reasoner.query(self.grabpoint_query)
            #rospy.loginfo('Answers = {0}'.format(answers))
            #import ipdb
            #ipdb.set_trace()

            if answers:
                answer = answers[0]
                rospy.loginfo("Answer(0) = {0}".format(answer))
                #grasp_point = geometry_msgs.msg.PointStamped()
                self.grasp_point.header.frame_id = "/map"
                self.grasp_point.header.stamp = rospy.Time()
                self.grasp_point.point.x = float(answer["X"])
                self.grasp_point.point.y = float(answer["Y"])
                self.grasp_point.point.z = float(answer["Z"])
                #rospy.loginfo("Grasp_point = {0}".format(self.grasp_point))
            else:
                #rospy.logerr("Cannot get target from reasoner, query = {0}".format(self.grabpoint_query))
                return 'target_lost'

        ''' Do a new inverse reachability request if nr < max_nr.
            This way, the grasp pose will be optimal (and the first few tries the change of new information is the highest)
            If nr == max_nr: check all entries of the latest call to increase robustness'''
        #rospy.loginfo('Nr of calls = {0} of {1}'.format(self.nr_inverse_reachability_calls,self.max_nr_inverse_reachability_calls))
        if self.nr_inverse_reachability_calls < self.max_nr_inverse_reachability_calls:
            self.nr_inverse_reachability_calls += 1
            #grasp_point_BASE_LINK = transformations.tf_transform(grasp_point, "/map", "/base_link", self.robot.tf_listener)               
            #rospy.loginfo('Grasp point base link = {0}'.format(grasp_point_BASE_LINK))
            self.desired_base_poses_MAP = self.robot.base.get_base_goal_poses(self.grasp_point, self.x_offset, self.y_offset)                
            #rospy.loginfo('Grasp poses base in map = {0}'.format(self.desired_base_poses_MAP))
            
            ''' If there are no base poses, reset costmap and retry '''
            if not self.desired_base_poses_MAP:
                self.robot.base.reset_costmap()
                self.desired_base_poses_MAP = self.robot.base.get_base_goal_poses(self.grasp_point, self.x_offset, self.y_offset)                
        
        ''' Sanity check: if the orientation is all zero, no feasible base pose has been found '''
        if not self.desired_base_poses_MAP:
            self.robot.speech.speak("I am very sorry but the goal point is out of my reach",mood="sad")
            return 'failed'
        
        x   = self.desired_base_poses_MAP[0].pose.position.x
        y   = self.desired_base_poses_MAP[0].pose.position.y
        phi = transformations.euler_z_from_quaternion(self.desired_base_poses_MAP[0].pose.orientation)
        self.desired_base_poses_MAP.pop(0)

        ''' Assert the goal to the reasoner such that navigate generic can use it '''
        self.robot.reasoner.query(Compound("retractall", Compound("base_grasp_pose", Compound("pose_2d", "X", "Y", "Phi"))))
        self.robot.reasoner.assertz(Compound("base_grasp_pose", Compound("pose_2d", x, y, phi)))

        return 'succeeded'


class PrepareOrientation(smach.StateMachine):
    def __init__(self, side, robot, grabpoint_query, x_offset=None, y_offset=None):
        smach.StateMachine.__init__(self, outcomes=['orientation_succeeded','orientation_failed','abort','target_lost'])
        self.side = side
        self.robot = robot
        self.grabpoint_query = grabpoint_query
        if x_offset == None:
            rospy.logdebug("x_offset not specified, defaulting to 0.5 for arms.")
            self.x_offset = 0.5
        else:
            self.x_offset = x_offset

        if y_offset == None:
            rospy.logdebug("y_offset not specified, defaulting to (-)0.2 for ... arm.")
            if self.side == self.robot.leftArm:
                self.y_offset = 0.2
            elif self.side == self.robot.rightArm:
                self.y_offset = -0.2
        else:
            self.y_offset = y_offset

        self.determine_grasp_pose = DetermineBaseGraspPose(self.side, self.robot, self.grabpoint_query, self.x_offset, self.y_offset)

        @smach.cb_interface(outcomes=['done'])
        def init_prepare_orientation(userdata):
            self.determine_grasp_pose.nr_inverse_reachability_calls = 0
            return "done"

        with self:

            smach.StateMachine.add('INITIALIZE_PREPARE_ORIENTATION', smach.CBState(init_prepare_orientation),
                                    transitions={   'done':'DETERMINE_GRASP_POSE'})

            smach.StateMachine.add('DETERMINE_GRASP_POSE', self.determine_grasp_pose,
                transitions={'succeeded'    :'DRIVE_TO_GRASP_POSE',
                             'failed'       :'orientation_failed',
                             'target_lost'  :'target_lost'})            

            smach.StateMachine.add('DRIVE_TO_GRASP_POSE',
                navigation.NavigateGeneric(robot=self.robot, goal_query=Compound("base_grasp_pose", Compound("pose_2d", "X", "Y", "Phi"))),
                transitions={'arrived'          :'orientation_succeeded',
                             'unreachable'      :'DETERMINE_GRASP_POSE',
                             'preempted'        :'abort',
                             'goal_not_defined' :'DETERMINE_GRASP_POSE'})


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
            rospy.logdebug("x_offset not specified, defaulting to 0.5 for arms.")
            self.grasp_distance_x = 0.5
        else:
            self.grasp_distance_x = x_offset

        if y_offset == None:
            #import ipdb; ipdb.set_trace()
            rospy.logdebug("y_offset not specified, defaulting to (-)0.2 for ... arm.")
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
            
            desired_base_poses_MAP = self.robot.base.get_base_goal_poses(grasp_point, self.grasp_distance_x, self.grasp_distance_y)
            
            ''' Sanity check: if the orientation is all zero, no feasible base pose has been found '''
            if not desired_base_poses_MAP:
                self.robot.speech.speak("I am very sorry but the goal point is out of my reach",mood="sad")
                return 'orientation_failed'
            
            rospy.loginfo("[robot_smach_states] Desired target position: x = %f, y = %f",desired_base_poses_MAP[0].pose.position.x, desired_base_poses_MAP[0].pose.position.y)

            if self.robot.base.send_goal(desired_base_poses_MAP[0].pose.position, desired_base_poses_MAP[0].pose.orientation, time=60):
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
    def __init__(self, arm, robot=None):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        
        self.robot = robot
        self.arm = arm

    def execute(self, gl):
        if self.arm == self.robot.leftArm:
            y_home = 0.2
        elif self.arm == self.robot.rightArm:
            y_home = -0.2
        
        rospy.loginfo("start moving to carrying pose")        
        if self.arm.send_goal(0.18, y_home, 0.75, 0, 0, 0, 60):
            rospy.loginfo("arm at carrying pose")   
            return 'succeeded'                 
        else:
            rospy.logerr("failed to go to the approach pose") 
            return 'failed' 

class PrepareGrasp(smach.State):
    def __init__(self, arm, robot, grabpoint_query):
        smach.State.__init__(self, outcomes=['succeeded','failed'])

        self.arm = arm
        self.robot = robot
        self.grabpoint_query = grabpoint_query


    def execute(self, gl):

        # Move arm to desired joint coordinates (no need to wait)
        # ToDo: determine joint coordinates
        #self.arm.send_joint_goal(-0.1, -0.3, 0.0, 1.87, 0.0, 0.0, 0.0)
        #self.arm.send_joint_goal(-0.035, -0.58, 0.2, 2.2, 0.13, 0.12, 0.23)
        self.arm.send_joint_goal(-0.2, -0.044, 0.69, 1.4, -0.13, 0.38, 0.42)

        # If the z-position of the object is above a suitable threshold, move the spindle so that the object position can later be updated using the laser
        # Probably do this afterwards
        answers = self.robot.reasoner.query(self.grabpoint_query)
        
        if answers:
            answer = answers[0] #ToDo: 0.55 is a dirty hack, might be better
            spindle_target = float(answer["Z"]) - 0.45

            rospy.logwarn("Temp spindle target")
            spindle_target = 0.4

            # ToDo: parameterize
            if (spindle_target > self.robot.spindle.lower_limit):
                self.robot.spindle.send_goal(spindle_target,waittime=5.0)
            # Else: there's no point in doing it any other way

        return 'succeeded'

class UpdateObjectPose(smach.State):
    def __init__(self, arm, robot, grabpoint_query):
        smach.State.__init__(self, outcomes=['succeeded','failed','target_lost'])

        self.arm = arm
        self.robot = robot
        self.grabpoint_query = grabpoint_query

    def execute(self, gl):

        rospy.loginfo("Trying to update object pose")

        ''' Remember current spindle position '''
        spindle_pos = self.robot.spindle.get_position()

        answers = self.robot.reasoner.query(self.grabpoint_query)
        
        if answers:
            answer = answers[0] #TODO Loy/Sjoerd: sort answers by distance to gripper/base? 
            target_point = geometry_msgs.msg.PointStamped()
            target_point.header.frame_id = "/map"
            target_point.header.stamp = rospy.Time()
            target_point.point.x = float(answer["X"])
            target_point.point.y = float(answer["Y"])
            target_point.point.z = float(answer["Z"])
            self.robot.head.send_goal(target_point, keep_tracking=True)
        else:
            return 'target_lost'

        ''' If height is feasible for LRF, use this. Else: use head and tabletop/clustering '''
        rospy.logwarn("Spindle timeout temporarily increased to 30 seconds")
        #if self.robot.spindle.send_laser_goal(float(answer["Z"]), timeout=30.0):
        ''' Hack to disable laser !!! '''
        nolaser = False
        if nolaser:
            self.robot.perception.toggle(["object_detector_2d"])
            self.robot.perception.set_perception_roi(target_point, length_x=0.3, length_y=0.3, length_z=0.2)
            rospy.logwarn("Here we should keep track of the uncertainty, how can we do that? Now we simply use a sleep")
            rospy.logwarn("Waiting for 2.0 seconds for laser update")
            rospy.sleep(rospy.Duration(2.0))
        else:
            self.robot.head.send_goal(target_point, keep_tracking=False, timeout=10.0)
            self.robot.perception.toggle(["tabletop_segmentation"])
            self.robot.perception.set_perception_roi(target_point, length_x=0.3, length_y=0.3, length_z=0.4)
            rospy.logwarn("Here we should keep track of the uncertainty, how can we do that? Now we simply use a sleep")
            waittime = 5.0
            rospy.logwarn("Waiting for {0} seconds for tabletop segmentation update".format(waittime))
            rospy.sleep(rospy.Duration(waittime))
            #self.robot.speech.speak("Now I need Simons stuff because the height of the object is {0:.2f}".format(target_point.point.z),block=False)

        ''' Reset head and stop all perception stuff '''
        self.robot.perception.toggle([])
        
        # ToDo: don't do this here this way:
        #self.robot.head.reset_position(timeout=0)
        if self.arm == self.robot.leftArm:
            self.robot.head.look_at_hand("left")
        elif self.arm == self.robot.rightArm:
            self.robot.head.look_at_hand("right")
        
        rospy.logwarn("Sending spindle to top for safety")
        spindle_pos = 0.4
        self.robot.spindle.send_goal(spindle_pos,waittime=40.0)

        return 'succeeded'
        
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
        if self.arm.send_goal(0.6, y_home, 0.966, 0, 0, 0, 30):
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
            smach.StateMachine.add('PREPARE_GRAB', PrepareGrasp(self.side, self.robot, self.grabpoint_query),
                        transitions={'succeeded'    :   'PREPARE_ORIENTATION',
                                     'failed'       :   'failed'})
        
            # Uses old Prepare_orientation
            #smach.StateMachine.add('PREPARE_ORIENTATION', Prepare_orientation(self.side, self.robot, self.grabpoint_query),
            #            transitions={'orientation_succeeded':'OPEN_GRIPPER','orientation_failed':'OPEN_GRIPPER','abort':'failed','target_lost':'failed'})

            # Uses new Prepare Orientation
            smach.StateMachine.add('PREPARE_ORIENTATION', PrepareOrientation(self.side, self.robot, self.grabpoint_query),
                        transitions={'orientation_succeeded':'OPEN_GRIPPER','orientation_failed':'OPEN_GRIPPER','abort':'failed','target_lost':'failed'})

            smach.StateMachine.add('OPEN_GRIPPER', SetGripper(self.robot, self.side, gripperstate=ArmState.OPEN),
                        transitions={'succeeded'    :   'UPDATE_OBJECT_POSE',
                                     'failed'       :   'UPDATE_OBJECT_POSE'})

            # Even if the update state fails, try to grasp anyway
            smach.StateMachine.add('UPDATE_OBJECT_POSE', UpdateObjectPose(self.side, self.robot, self.grabpoint_query),
                        transitions={'succeeded'    :   'PRE_GRASP',
                                     'failed'       :   'PRE_GRASP',
                                     'target_lost'  :   'CLOSE_GRIPPER_UPON_FAIL'})
            
            smach.StateMachine.add('PRE_GRASP', ArmToQueryPoint(self.robot, self.side, self.grabpoint_query, time_out=20, pre_grasp=True, first_joint_pos_only=True),
                        transitions={'succeeded'    :   'GRAB',
                                     'failed'       :   'CLOSE_GRIPPER_UPON_FAIL'})
        
            smach.StateMachine.add('GRAB', Grab(self.side, self.robot, self.grabpoint_query),
                        transitions={'grab_succeeded':  'CLOSE_GRIPPER',
                                     'grab_failed'   :  'CLOSE_GRIPPER',
                                     'target_lost'   :  'CLOSE_GRIPPER_UPON_FAIL'})

            smach.StateMachine.add('CLOSE_GRIPPER', SetGripper(self.robot, self.side, gripperstate=ArmState.CLOSE, grabpoint_query=self.grabpoint_query),
                        transitions={'succeeded'    :   'LIFT',
                                     'failed'       :   'LIFT'})

            smach.StateMachine.add('LIFT', ArmToUserPose(self.side, 0.0, 0.0, 0.1, 0.0, 0.0 , 0.0, time_out=20, pre_grasp=False, frame_id="/base_link", delta=True),
                        transitions={'succeeded':'RETRACT','failed':'RETRACT'})

            smach.StateMachine.add('RETRACT', ArmToUserPose(self.side, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, time_out=20, pre_grasp=False, frame_id="/base_link", delta=True),
                        transitions={'succeeded':'CARR_POS','failed':'CARR_POS'})
        
            smach.StateMachine.add('CARR_POS', Carrying_pose(self.side, self.robot),
                        transitions={'succeeded':'succeeded','failed':'CLOSE_GRIPPER_UPON_FAIL'})

            smach.StateMachine.add('CLOSE_GRIPPER_UPON_FAIL', SetGripper(self.robot, self.side, gripperstate=ArmState.CLOSE),
                        transitions={'succeeded'    :   'failed',
                                     'failed'       :   'failed'})

            
class Grab(smach.State):
    def __init__(self, side, robot, grabpoint_query):
        smach.State.__init__(self, outcomes=['grab_succeeded','grab_failed','target_lost'])

        self.side = side
        self.robot = robot
        self.grabpoint_query = grabpoint_query

        if self.side == self.robot.leftArm:
            self.end_effector_frame_id = "/grippoint_left"
            self.ar_frame_id = "/hand_marker_left"
        elif self.side == self.robot.rightArm:
            self.end_effector_frame_id = "/grippoint_right"
            self.ar_frame_id = "/hand_marker_right"
        
    def execute(self, gl):
        
        answers = self.robot.reasoner.query(self.grabpoint_query)
        
        if answers:
            answer = answers[0] #TODO Loy/Sjoerd: sort answers by distance to gripper/base? 
            target_position = geometry_msgs.msg.PointStamped()
            target_position.header.frame_id = "/map"
            target_position.header.stamp = rospy.Time()
            target_position.point.x = float(answer["X"])
            target_position.point.y = float(answer["Y"])
            target_position.point.z = float(answer["Z"])
        else:
            rospy.loginfo("No answers for query {0}".format(self.grabpoint_query))
            return "target_lost"
            
        ''' Keep looking at end-effector for ar marker detection '''
        self.robot.head.set_position(0,0,0,frame_id=self.end_effector_frame_id,keep_tracking=True)
        
        rospy.loginfo("[robot_smach_states:grasp] Target position: {0}".format(target_position))
        
        (robot_position, robot_orientation) = self.robot.base.location
        
        target_position_bl = transformations.tf_transform(target_position, "/map","/base_link", tf_listener=self.robot.tf_listener)
        
        rospy.loginfo("[robot_smach_states] Target position in base link: {0}".format(target_position_bl))

        target_position_delta = geometry_msgs.msg.Point()
        
        ''' First check to see if visual servoing is possible '''
        self.robot.perception.toggle(['ar_pose'])
        #rospy.logwarn("ar marker check disabled")
        try:
            self.robot.tf_listener.waitForTransform(self.end_effector_frame_id, self.ar_frame_id, rospy.Time(), rospy.Duration(2.5))
        except:
            rospy.logerr("Transformation between {0} and {1} failed".format(self.end_effector_frame_id,self.ar_frame_id))
        ar_marker_available = False
        rospy.sleep(2.5)
        target_position_delta = transformations.tf_transform(target_position, self.end_effector_frame_id, self.ar_frame_id, tf_listener=self.robot.tf_listener)
        if target_position_delta == None:
            ar_marker_available = False
        else:
            ar_marker_available = True

        rospy.logwarn("ar_marker_available (1) = {0}".format(ar_marker_available))
            
        ''' If transform is not available, try again, but use head movement as well '''
        if not ar_marker_available:
            self.robot.head.set_position(0,0,0,frame_id=self.end_effector_frame_id,keep_tracking=True)
            self.side.send_delta_goal(0.05,0.0,0.0,0.0,0.0,0.0, time_out=5.0, frame_id=self.end_effector_frame_id, pre_grasp = False)
            self.robot.speech.speak("Let me have a closer look", block=False)
        
        ''' New ar marker detection stuff ''' 
        ar_point = geometry_msgs.msg.PointStamped()
        ar_point.header.frame_id = self.ar_frame_id
        ar_point.header.stamp = rospy.Time()
        ar_point.point.x = 0
        ar_point.point.y = 0
        ar_point.point.z = 0
        #import ipdb; ipdb.set_trace()
        ''' Transform point(0,0,0) in ar marker frame to grippoint frame '''
        ar_point_grippoint = transformations.tf_transform(ar_point, self.ar_frame_id, self.end_effector_frame_id, tf_listener=self.robot.tf_listener)
        rospy.loginfo("AR marker in end-effector frame = {0}".format(ar_point_grippoint))
        ''' Transform target position to grippoint frame '''
        target_position_grippoint = transformations.tf_transform(target_position, "/map", self.end_effector_frame_id, tf_listener=self.robot.tf_listener)
        rospy.loginfo("Target position in end-effector frame = {0}".format(target_position_grippoint))
        ''' Compute difference = delta (only when both transformations have succeeded) and correct for offset ar_marker and grippoint '''
        if not (ar_point_grippoint == None or target_position_grippoint == None):
            target_position_delta = geometry_msgs.msg.Point()
            target_position_delta.x = target_position_grippoint.x - ar_point_grippoint.x + self.side.markerToGrippointOffset.x
            target_position_delta.y = target_position_grippoint.y - ar_point_grippoint.y + self.side.markerToGrippointOffset.y
            target_position_delta.z = target_position_grippoint.z - ar_point_grippoint.z + self.side.markerToGrippointOffset.z
            rospy.loginfo("Delta target = {0}".format(target_position_delta))
            ar_marker_available = True
        else:
            ar_marker_available = False
        ''' End '''
        
        rospy.logwarn("ar_marker_available (2) = {0}".format(ar_marker_available))
            
        ''' Sanity check '''
        if ar_marker_available:
            #rospy.loginfo("Delta target = {0}".format(target_position_delta))
            if (target_position_delta.x < 0 or target_position_delta.x > 0.6 or target_position_delta.y < -0.3 or target_position_delta.y > 0.3 or target_position_delta.z < -0.3 or target_position_delta.z > 0.3):
                rospy.logwarn("Ar marker detection probably incorrect")
                self.robot.speech.speak("I guess I cannot see my hand properly")
                ar_marker_available = False

        rospy.logwarn("ar_marker_available (3) = {0}".format(ar_marker_available))

        ''' Switch off ar marker detection '''
        self.robot.perception.toggle([])                
        
        ''' Original, pregrasp is performed by the compute_pre_grasp node '''
        if not ar_marker_available:
            self.robot.speech.speak("No visual feedback, let's see if I can grasp with my eyes closed", block=False)                
            if self.side.send_goal(target_position_bl.x, target_position_bl.y, target_position_bl.z, 0, 0, 0, 120, pre_grasp = True):
                rospy.loginfo("arm at object")                    
            else:
                if self.side.send_gripper_goal_close(10):
                    try:
                        #import ipdb;ipdb.set_trace()
                        self.robot.reasoner.attach_object_to_gripper(answer["ObjectID"], self.end_effector_frame_id, True)
                    except KeyError, ke:
                        rospy.logerr("Could not attach object to gripper, do not know which ID: {0}".format(ke))
                    rospy.loginfo("Gripper closed")
                else:
                    rospy.loginfo("opening gripper failed, good luck")
                    return 'grab_failed'
                rospy.logerr("Goal unreachable: {0}".format(target_position_bl).replace("\n", " "))
                self.robot.speech.speak("I am sorry but I cannot move my arm to the object position")
                
        else:
            self.robot.speech.speak("I can see both my hand and the object, now I shouldn't miss", block=False)
            #import ipdb;ipdb.set_trace()
            if self.side.send_delta_goal(target_position_delta.x, target_position_delta.y, target_position_delta.z,
                                        0, 0, 0, 120, frame_id=self.end_effector_frame_id, pre_grasp = True):                    
                rospy.loginfo("arm at object")                    
            else:
                rospy.logerr("Goal unreachable: {0}".format(target_position_bl).replace("\n", " "))
                self.robot.speech.speak("I am sorry but I cannot move my arm to the object position")
                return 'grab_failed'
            
        self.robot.head.reset_position()
        return 'grab_succeeded'

class Human_handover(smach.StateMachine):
    def __init__(self, side, robot=None):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])
        self.side = side
        self.robot = robot
        
        with self:
            smach.StateMachine.add("POSE", Carrying_pose(self.side,self.robot),
                            transitions={'succeeded':'OPEN_BEFORE_INSERT','failed':'OPEN_BEFORE_INSERT'})
            
            smach.StateMachine.add( 'OPEN_BEFORE_INSERT', SetGripper(robot, robot.leftArm, gripperstate=0), #open
                                transitions={'succeeded'    :   'SAY1',
                                             'failed'       :   'SAY1'})
            
            smach.StateMachine.add("SAY1", Say(self.robot,'Please hand over the object by sliding it in my gripper, I am not able to grasp the object.'),
                            transitions={'spoken':'CLOSE_AFTER_INSERT'})
            
            smach.StateMachine.add( 'CLOSE_AFTER_INSERT', SetGripper(robot, robot.leftArm, gripperstate=1), #close
                                transitions={'succeeded'    :   'succeeded',
                                             'failed'       :   'failed'})

class PlaceObject(smach.StateMachine):
    def __init__(self, side, robot, placement_query, dropoff_height_offset=0.1):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed','target_lost'])
        self.side = side
        self.robot = robot
        self.placement_query = placement_query
        self.dropoff_height_offset = dropoff_height_offset

        with self:
            smach.StateMachine.add('PREPARE_PLACEMENT', PrepareGrasp(self.side, self.robot, self.placement_query),
                        transitions={'succeeded'    :   'PREPARE_ORIENTATION',
                                     'failed'       :   'failed'})
        
            # Old implementation
            #smach.StateMachine.add('PREPARE_ORIENTATION', Prepare_orientation(self.side, self.robot, self.placement_query),
            #            transitions={'orientation_succeeded':'PRE_POSITION','orientation_failed':'PRE_POSITION','abort':'failed','target_lost':'target_lost'})

            # New implementation with NavigateGeneric
            smach.StateMachine.add('PREPARE_ORIENTATION', PrepareOrientation(self.side, self.robot, self.placement_query),
                        transitions={'orientation_succeeded':'PRE_POSITION','orientation_failed':'PRE_POSITION','abort':'failed','target_lost':'target_lost'})
            
            smach.StateMachine.add('PRE_POSITION', ArmToQueryPoint(self.robot, self.side, self.placement_query, time_out=20, pre_grasp=True, first_joint_pos_only=False),
                        transitions={'succeeded'    :   'POSITION',
                                     'failed'       :   'failed'})
            # When pre-position fails, there is no use in dropping the object
            smach.StateMachine.add('POSITION', ArmToUserPose(   self.side, 0.0, 0.0, -self.dropoff_height_offset, 0.0, 0.0 , 0.0, 
                                                                time_out=20, pre_grasp=False, frame_id="/base_link", delta=True),
                        transitions={'succeeded':'OPEN_GRIPPER','failed':'OPEN_GRIPPER'})
            # When position fails, it might be tried 
            smach.StateMachine.add('OPEN_GRIPPER', SetGripper(self.robot, self.side, gripperstate=ArmState.OPEN),
                        transitions={'succeeded'    :   'LIFT',
                                     'failed'       :   'LIFT'})

            smach.StateMachine.add('LIFT', ArmToUserPose(self.side, 0.0, 0.0, self.dropoff_height_offset, 0.0, 0.0 , 0.0, time_out=20, pre_grasp=False, frame_id="/base_link", delta=True),
                        transitions={'succeeded':'RETRACT','failed':'RETRACT'})

            smach.StateMachine.add('RETRACT', ArmToUserPose(self.side, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, time_out=20, pre_grasp=False, frame_id="/base_link", delta=True),
                        transitions={'succeeded':'CARR_POS','failed':'CARR_POS'})
        
            smach.StateMachine.add('CARR_POS', Carrying_pose(self.side, self.robot),
                        transitions={'succeeded':'CLOSE_GRIPPER','failed':'CLOSE_GRIPPER'})

            smach.StateMachine.add('CLOSE_GRIPPER', SetGripper(self.robot, self.side, gripperstate=ArmState.CLOSE),
                        transitions={'succeeded'    :   'RESET_ARM',
                                     'failed'       :   'RESET_ARM'})

            smach.StateMachine.add('RESET_ARM', 
                        ArmToJointPos(self.robot, self.side, (-0.0830 , -0.2178 , 0.0000 , 0.5900 , 0.3250 , 0.0838 , 0.0800)), #Copied from demo_executioner NORMAL
                        transitions={   'done':'RESET_TORSO',
                                      'failed':'RESET_TORSO'    })

            smach.StateMachine.add('RESET_TORSO',
                        ResetTorso(self.robot),
                        transitions={'succeeded':'succeeded',
                                     'failed'   :'succeeded'})

class HandoverToHuman(smach.StateMachine):
    def __init__(self, side, robot=None):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])
        self.side = side
        self.robot = robot

        with self:
            smach.StateMachine.add('MOVE_HUMAN_HANDOVER', ArmToUserPose(self.side, 0.2, 0.3, 1.0, 0.0, 0.0 , 0.0, time_out=20, pre_grasp=False, frame_id="/base_link", delta=False),
                        transitions={'succeeded':'SAY_OPEN_GRIPPER','failed':'SAY_OPEN_GRIPPER'})

            smach.StateMachine.add("SAY_OPEN_GRIPPER", 
                        Say(robot, [ "Be careful, I will open my gripper now"]),
                        transitions={   'spoken':'OPEN_GRIPPER_HANDOVER'})

            smach.StateMachine.add('OPEN_GRIPPER_HANDOVER', SetGripper(self.robot, self.side, gripperstate=ArmState.OPEN),
                        transitions={'succeeded'    :   'CLOSE_GRIPPER_HANDOVER',
                                     'failed'       :   'CLOSE_GRIPPER_HANDOVER'})

            smach.StateMachine.add('CLOSE_GRIPPER_HANDOVER', SetGripper(self.robot, self.side, gripperstate=ArmState.CLOSE),
                        transitions={'succeeded'    :   'RESET_ARM',
                                     'failed'       :   'RESET_ARM'})

            smach.StateMachine.add('RESET_ARM', 
                        ArmToJointPos(self.robot, self.side, (-0.0830 , -0.2178 , 0.0000 , 0.5900 , 0.3250 , 0.0838 , 0.0800)), #Copied from demo_executioner NORMAL
                        transitions={   'done':'RESET_TORSO',
                                      'failed':'RESET_TORSO'    })

            smach.StateMachine.add('RESET_TORSO',
                        ResetTorso(self.robot),
                        transitions={'succeeded':'succeeded',
                                     'failed'   :'succeeded'})

class DropObject(smach.StateMachine):
    def __init__(self, side, robot, placement_query, dropoff_height_offset=0.1):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed', 'target_lost'])
        self.side = side
        self.robot = robot
        self.placement_query = placement_query
        self.dropoff_height_offset = dropoff_height_offset

        otherarm = self.robot.rightArm
        if otherarm == side:
            otherarm = self.robot.leftArm

        with self:
            smach.StateMachine.add( "RESET_OTHER_ARM",
                                    ArmToJointPos(robot, otherarm, StandardPoses.RESET_POSE, timeout=1.0),
                                    transitions={   'done'                  : 'PLACE_OBJECT',
                                                    'failed'                : 'PLACE_OBJECT'}) #This is risky, but try anyways

            smach.StateMachine.add( 'PLACE_OBJECT', 
                                    PlaceObject(self.side, self.robot, self.placement_query, self.dropoff_height_offset),
                                    transitions={'succeeded'    : 'succeeded',
                                                 'failed'       : 'SAY_HUMAN_HANDOVER',
                                                 'target_lost'  : 'target_lost' })

            smach.StateMachine.add( 'SAY_HUMAN_HANDOVER', 
                                    Say(robot, [ "I am terribly sorry, but I cannot place the object. Can you please take it from me", 
                                                        "My apologies, but i cannot place the object. Would you be so kind to take it from me"]),
                                     transitions={   'spoken':'HANDOVER_TO_HUMAN'})

            smach.StateMachine.add( 'HANDOVER_TO_HUMAN', 
                                    HandoverToHuman(self.side, self.robot),
                                    transitions={'succeeded'    : 'succeeded',
                                                 'failed'       : 'failed'})

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
    def __init__(self, robot, side, gripperstate=ArmState.OPEN, drop_from_frame=None, grabpoint_query=None):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.side = side
        self.robot = robot
        self.gripperstate = gripperstate
        self.grabpoint_query = grabpoint_query
        if self.side == self.robot.leftArm:
            self.end_effector_frame_id = "/grippoint_left"
        elif self.side == self.robot.rightArm:
            self.end_effector_frame_id = "/grippoint_right"

    def execute(self, userdata):
        ''' If needs attaching to gripper, the grabpoint_query is used '''
        if self.grabpoint_query:
            answers = self.robot.reasoner.query(self.grabpoint_query)
            if answers:
                answer = answers[0] #TODO Loy/Sjoerd: sort answers by distance to gripper/base? 
                try:
                #import ipdb;ipdb.set_trace()
                    self.robot.reasoner.attach_object_to_gripper(answer["ObjectID"], self.end_effector_frame_id, True)
                except KeyError, ke:
                    rospy.logerr("Could not attach object to gripper, do not know which ID: {0}".format(ke))

        if self.side.send_gripper_goal(self.gripperstate):
            result = True
        else:
            result = False

        # ToDo: make sure things can get attached to the gripper in this state. Use userdata?
        if self.gripperstate == ArmState.OPEN:
            self.robot.reasoner.detach_all_from_gripper(self.end_effector_frame_id)

        # ToDo: check for failed in other states
        if result:
            return 'succeeded'
        else:
            return 'failed'

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
    def __init__(self, robot, side, jointgoal, timeout=0, delta=False):
        smach.State.__init__(self, outcomes=['done', "failed"])
        self.side = side
        self.robot = robot
        self.jointgoal = jointgoal
        self.timeout = timeout
        self.delta = delta

    def execute(self, userdata):
        if not self.delta:
            result = self.side.send_joint_goal(*self.jointgoal,timeout=self.timeout)
        if self.delta:
            result = self.side.send_delta_joint_goal(*self.jointgoal,timeout=self.timeout)
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
            rospy.logwarn("Transforming to baselink, should become obsolete but this is not yet the case")
            #target_position_bl = transformations.tf_transform(target_position, "/map","/base_link", tf_listener=self.robot.tf_listener)
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

class ArmToQueryPoint(smach.State):
    def __init__(self, robot, side, query, time_out=20, pre_grasp=False, first_joint_pos_only=False):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot                = robot
        self.side                 = side
        self.query                = query
        self.time_out             = time_out
        self.pre_grasp            = pre_grasp
        self.first_joint_pos_only = first_joint_pos_only

    def execute(self, userdata):
        # ToDo: check query?
        answers = self.robot.reasoner.query(self.query)
            
        if not answers:
            rospy.loginfo("No answers for query {0}".format(self.grabpoint_query))
            return 'failed'
        answer = answers[0] #TODO Loy/Sjoerd: sort answers by distance to gripper/base? 
        rospy.loginfo("ArmToQueryPoint: goal = {0}".format(answer))

        # Note: answers are typically in "map"frame, check whether this works out
        rospy.logwarn("Transforming to base_link frame for amigo_arm_navigation")

        goal_map = geometry_msgs.msg.Point()
        goal_map.x = float(answer["X"])
        goal_map.y = float(answer["Y"])
        goal_map.z = float(answer["Z"])
        
        goal_bl = transformations.tf_transform(goal_map, "/map", "/base_link", tf_listener=self.robot.tf_listener)
        if goal_bl == None:
            return 'failed'

        if self.side.send_goal(goal_bl.x, goal_bl.y, goal_bl.z, 0, 0, 0, 
            frame_id="/base_link",
            time_out=self.time_out,
            pre_grasp=self.pre_grasp,
            first_joint_pos_only=self.first_joint_pos_only):
            return 'succeeded'
        else:
            rospy.logwarn("Goal unreachable: {0}".format(goal_bl).replace("\n", " "))
            return 'failed'

class TorsoToUserPos(smach.State):
    def __init__(self, robot, torso_pos, time_out=0.0):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot
        self.torso_pos = torso_pos
        self.time_out = time_out

    def execute(self, userdata):
        if self.robot.spindle.send_goal(self.torso_pos,waittime=self.time_out):
            return 'succeeded'
        else:
            return 'failed'

class ResetTorso(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot

    def execute(self, userdata):
        if self.robot.spindle.reset():
            return 'succeeded'
        else:
            return 'failed'

########################################### State Point ###############################################

class PointMachine(smach.StateMachine):
    def __init__(self, side, robot, grabpoint_query):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])
        self.side = side
        self.robot = robot
        self.grabpoint_query = grabpoint_query
        '''check check input and output keys'''
        with self:
            smach.StateMachine.add('PREPARE_POINT', PrepareGrasp(self.side, self.robot, self.grabpoint_query),
                        transitions={'succeeded'    :   'PREPARE_ORIENTATION',
                                     'failed'       :   'failed'})
        
            smach.StateMachine.add('PREPARE_ORIENTATION', Prepare_orientation(self.side, self.robot, self.grabpoint_query),
                        transitions={'orientation_succeeded':'CLOSE_GRIPPER','orientation_failed':'CLOSE_GRIPPER','abort':'failed','target_lost':'failed'})

            smach.StateMachine.add('CLOSE_GRIPPER', SetGripper(self.robot, self.side, gripperstate=ArmState.CLOSE),
                        transitions={'succeeded'    :   'PRE_POINT',
                                     'failed'       :   'PRE_POINT'})

            # # Even if the update state fails, try to grasp anyway
            # smach.StateMachine.add('UPDATE_OBJECT_POSE', UpdateObjectPose(self.side, self.robot, self.grabpoint_query),
            #             transitions={'succeeded'    :   'PRE_POINT',
            #                          'failed'       :   'PRE_POINT',
            #                          'target_lost'  :   'RESET_ARM_FAILED'})
            
            smach.StateMachine.add('PRE_POINT', ArmToQueryPoint(self.robot, self.side, self.grabpoint_query, time_out=20, pre_grasp=True, first_joint_pos_only=True),
                        transitions={'succeeded'    :   'POINT',
                                     'failed'       :   'RESET_ARM_FAILED'})
        
            smach.StateMachine.add('POINT', Point_at_object(self.side, self.robot, self.grabpoint_query),
                        transitions={'point_succeeded':  'SAY_POINTED',
                                     'point_failed'   :  'RETRACT',
                                     'target_lost'   :  'RESET_ARM_FAILED'})

            smach.StateMachine.add('SAY_POINTED',
                                    Say(robot, "I hope this is the object you were looking for."),
                        transitions={ 'spoken':'RETRACT' })

            smach.StateMachine.add('RETRACT', ArmToUserPose(self.side, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, time_out=20, pre_grasp=False, frame_id="/base_link", delta=True),
                        transitions={'succeeded':'RESET_ARM_SUCCEEDED','failed':'RESET_ARM_SUCCEEDED'})

            smach.StateMachine.add('RESET_ARM_SUCCEEDED', 
                                    ArmToJointPos(robot, robot.leftArm, (-0.0830 , -0.2178 , 0.0000 , 0.5900 , 0.3250 , 0.0838 , 0.0800)),
                        transitions={   'done':'succeeded',
                                        'failed':'succeeded'})

            smach.StateMachine.add('RESET_ARM_FAILED', 
                                    ArmToJointPos(robot, robot.leftArm, (-0.0830 , -0.2178 , 0.0000 , 0.5900 , 0.3250 , 0.0838 , 0.0800)),
                        transitions={   'done':'failed',
                                        'failed':'failed'})

class Point_at_object(smach.State):
    def __init__(self, side, robot, grabpoint_query):
        smach.State.__init__(self, outcomes=['point_succeeded','point_failed','target_lost'])

        self.side = side
        self.robot = robot
        self.grabpoint_query = grabpoint_query

        if self.side == self.robot.leftArm:
            self.end_effector_frame_id = "/grippoint_left"
            self.ar_frame_id = "/hand_marker_left"
        elif self.side == self.robot.rightArm:
            self.end_effector_frame_id = "/grippoint_right"
            self.ar_frame_id = "/hand_marker_right"
        
    def execute(self, gl):
        
        answers = self.robot.reasoner.query(self.grabpoint_query)
        
        if answers:
            answer = answers[0] #TODO Loy/Sjoerd: sort answers by distance to gripper/base? 
            target_position = geometry_msgs.msg.PointStamped()
            target_position.header.frame_id = "/map"
            target_position.header.stamp = rospy.Time()
            target_position.point.x = float(answer["X"])
            target_position.point.y = float(answer["Y"])
            target_position.point.z = float(answer["Z"])
        else:
            rospy.loginfo("No answers for query {0}".format(self.grabpoint_query))
            return "target_lost"
            
        ''' Keep looking at end-effector for ar marker detection '''
        self.robot.head.set_position(0,0,0,frame_id=self.end_effector_frame_id,keep_tracking=True)
        
        rospy.loginfo("[robot_smach_states:grasp] Target position: {0}".format(target_position))
        
        (robot_position, robot_orientation) = self.robot.base.location
        
        target_position_bl = transformations.tf_transform(target_position, "/map","/base_link", tf_listener=self.robot.tf_listener)
        
        rospy.loginfo("[robot_smach_states] Target position in base link: {0}".format(target_position_bl))

        target_position_delta = geometry_msgs.msg.Point()
        
        ''' First check to see if visual servoing is possible '''
        #self.robot.perception.toggle(['ar_pose'])
        rospy.logwarn("ar marker check disabled")
        ar_marker_available = False
        
        target_position_delta = transformations.tf_transform(target_position, "/map", self.ar_frame_id, tf_listener=self.robot.tf_listener)
        if target_position_delta == None:
            ar_marker_available = False
        else:
            ar_marker_available = True

        rospy.logwarn("ar_marker_available (1) = {0}".format(ar_marker_available))
            
        ''' If transform is not available, try again, but use head movement as well '''
        if not ar_marker_available:
            self.robot.head.set_position(0,0,0,frame_id=self.end_effector_frame_id,keep_tracking=True)
            self.side.send_delta_goal(0.05,0.0,0.0,0.0,0.0,0.0, time_out=5.0, frame_id=self.end_effector_frame_id, pre_grasp = False)
            self.robot.speech.speak("What's that on my hand?")
            rospy.sleep(2.0)
            
        target_position_delta = transformations.tf_transform(target_position, "/map", self.ar_frame_id, tf_listener=self.robot.tf_listener)
        if target_position_delta == None:
            ar_marker_available = False
        else:
            ar_marker_available = True

        rospy.logwarn("ar_marker_available (2) = {0}".format(ar_marker_available))
            
        ''' Sanity check '''
        if ar_marker_available:
            rospy.loginfo("Delta target = {0}".format(target_position_delta))
            if (target_position_delta.x < 0 or target_position_delta.x > 0.6 or target_position_delta.y < -0.3 or target_position_delta.y > 0.3 or target_position_delta.z < -0.3 or target_position_delta.z > 0.3):
                rospy.logwarn("Ar marker detection probably incorrect")
                self.robot.speech.speak("I guess I cannot see my hand properly")
                ar_marker_available = False

        rospy.logwarn("ar_marker_available (3) = {0}".format(ar_marker_available))

        ''' Switch off ar marker detection '''
        self.robot.perception.toggle([])                
        
        ''' Original, pregrasp is performed by the compute_pre_grasp node '''
        if not ar_marker_available:
            self.robot.speech.speak("No visual feedback, let's see if I can point with my eyes closed")                
            if self.side.send_goal(target_position_bl.x-0.1, target_position_bl.y, target_position_bl.z, 0, 0, 0, 120, pre_grasp = True):
                rospy.loginfo("arm at object")                    
                
        else:
            self.robot.speech.speak("I can see both my hand and the object, pointing would be no problem.")
            #import ipdb;ipdb.set_trace()
            if self.side.send_delta_goal(target_position_delta.x + self.side.markerToGrippointOffset.x - 0.1, 
                                        target_position_delta.y + self.side.markerToGrippointOffset.y, 
                                        target_position_delta.z + self.side.markerToGrippointOffset.z, 
                                        0, 0, 0, 120, frame_id=self.end_effector_frame_id, pre_grasp = True):                    
                rospy.loginfo("arm at object")                    
            else:
                #if self.side.send_gripper_goal_close(10):
                #    try:
                #        #import ipdb;ipdb.set_trace()
                #        self.robot.reasoner.attach_object_to_gripper(answer["ObjectID"], self.end_effector_frame_id, True)
                #    except KeyError, ke:
                #        rospy.logerr("Could not attach object to gripper, do not know which ID: {0}".format(ke))
                #    rospy.loginfo("Gripper closed")
                #else:
                #    rospy.loginfo("opening gripper failed, good luck")
                rospy.logerr("failed to go to the arm position")
                self.robot.speech.speak("I am sorry but I cannot move my arm to the object position")
                return 'point_failed'
            
        self.robot.head.reset_position()
        return 'point_succeeded'

class Point_location_hardcoded(smach.State):
    def __init__(self, robot, side, point_amount):
        smach.State.__init__(self, outcomes=['pointed'])
        self.side = side
        self.robot = robot
        self.i = 0
        self.point_amount = point_amount

    def execute(self, userdata):

        self.side.send_gripper_goal_close()
        i = self.i
        while i < self.point_amount: 
            #Backwards
            self.side.send_joint_goal(-0.4 ,-0.750 , 0.50 , 1.50 , 0.000 , 0.7500 , 0.000)
            rospy.sleep(3.2)
            #Forwards
            self.side.send_joint_goal(-0.2 ,-0.250 , 0.40 , 1.25 , 0.000 ,0.500 , 0.000)
            rospy.sleep(3.2)
            i += 1

        self.side.reset_arm()

        return 'pointed'
        
