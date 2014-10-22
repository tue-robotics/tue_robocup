#! /usr/bin/env python

#
#  Rein Appeldoorn
#  March '14
#

import roslib, rospy
roslib.load_manifest('robot_skills')

from geometry_msgs.msg import PoseStamped
from cb_planner_msgs_srvs.srv import *
from cb_planner_msgs_srvs.msg import *
import actionlib

import tf
import tf_server

from util import transformations

class Base(object):
    """Interface to the planners """
    
    def __init__(self, robot, tf_listener, wait_service=True, use_2d=None):

        # Some member vars to store some data
        self.plan = []
        self.pc = PositionConstraint()
        self.oc = OrientationConstraint()
        self.local_planner_status = "idle"
        self.tf_listener = tf_listener
        self.robot = robot

        # Wait for services to be active
        rospy.loginfo("Waiting for the global planner services ...")
#        rospy.wait_for_service("/gp/get_plan_srv")
#        rospy.wait_for_service("/gp/check_plan_srv")

        # ROS Services for global planner
        self.get_plan_client = rospy.ServiceProxy("/cb_base_navigation/global_planner_interface/get_plan_srv", GetPlan)
        self.check_plan_client = rospy.ServiceProxy("/cb_base_navigation/global_planner_interface/check_plan_srv", CheckPlan)

        # ROS ActionLib for local planner
        self.action_client = actionlib.SimpleActionClient('/cb_base_navigation/local_planner_interface/action_server', LocalPlannerAction)

        self.cmd_vel = rospy.Publisher('/' + self.robot.robot_name + '/base/references', geometry_msgs.msg.Twist)

        # Initial pose publisher
        self.initial_pose_publisher = rospy.Publisher('/' + self.robot.robot_name + '/initialpose', geometry_msgs.msg.PoseWithCovarianceStamped)

        rospy.loginfo("Navigation Interface Initialized [(base2)]")

    ################### LOCAL PLANNER ############################

    def localPlannerSetPlan(self, plan, orientation_constraint):
        goal = LocalPlannerGoal()
        goal.plan = plan
        goal.orientation_constraint = orientation_constraint
        self.action_client.send_goal(goal, done_cb = self.__localPlannerDoneCallback, feedback_cb = self.__localPlannerFeedbackCallback) 
        self.local_planner_status = "controlling"

    def localPlannerCancelCurrentPlan(self):
        self.action_client.cancel_goal()

    def __localPlannerFeedbackCallback(self, feedback):
        self.local_planner_status = "controlling" # or stuck (blocked)

    def __localPlannerDoneCallback(self, terminal_state, result):
        self.local_planner_status = "arrived"

    ################### GLOBAL PLANNER ###########################

    def globalPlannerGetPlan(self, pos_constraint):

        self.position_constraint = pos_constraint

        pcs = []
        pcs.append(pos_constraint)

        try:
            resp = self.get_plan_client(pcs)
        except:
            rospy.logerr("Could not get plan from global planner via service call, is the global planner running?")
            return -2

        if not resp.succes:
            rospy.logerr("Global planner couldn't figure out your request. Are your constraints set correctly?")
            return -1

        return resp.plan

    def globalPlannerCheckPlan(self, plan):

        try:
            resp = self.check_plan_client(plan)
        except:
            rospy.logerr("Could not check plan, is the global planner running?")
            return False

        return resp.valid

    ####################### NECESSARY BUT NEED REFACTORING #######################
    
    def get_location(self):
        rospy.logwarn("[base2.py] Function 'get_location' is obsolete.")

        try:
            #tf_listener = tf_server.TFClient()
            time = rospy.Time.now()
            self.tf_listener.waitForTransform("/map", "/" + self.robot.robot_name + "/base_link", time, rospy.Duration(20.0))
            (ro_trans, ro_rot) = self.tf_listener.lookupTransform("/map", "/" + self.robot.robot_name + "/base_link", time)
            
            position = geometry_msgs.msg.Point()
            orientation = geometry_msgs.msg.Quaternion()
            
            position.x = ro_trans[0]
            position.y = ro_trans[1]
            orientation.x = ro_rot[0]
            orientation.y = ro_rot[1]
            orientation.z = ro_rot[2]
            orientation.w = ro_rot[3]

            target_pose =  geometry_msgs.msg.PoseStamped(pose=geometry_msgs.msg.Pose(position=position, orientation=orientation))
            target_pose.header.frame_id = "/map"
            target_pose.header.stamp = time
            return target_pose
        
        except (tf.LookupException, tf.ConnectivityException):
            rospy.logerr("tf request failed!!!")        
            target_pose =  geometry_msgs.msg.PoseStamped(pose=geometry_msgs.msg.Pose(position=position, orientation=orientation))
            target_pose.header.frame_id = "/map"
            return target_pose

    def set_initial_pose(self, x, y, phi):

        initial_pose = geometry_msgs.msg.PoseWithCovarianceStamped()
        
        initial_pose.header.frame_id = "/map"
        
        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation = transformations.euler_z_to_quaternion(phi) 
        initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        #rospy.loginfo("initalpose = {0}".format(initial_pose))

        self.initial_pose_publisher.publish(initial_pose)

        return True    

    def force_drive(self, vx,vy,vthe,timeout):
        
        # Cancel base goal to prevent interference between 
        # move_base and force_drive
        self.cancel_goal()
        
        velocity = geometry_msgs.msg.Twist()        # Initialize velocity
        start_time = rospy.Time.now()
        
        # Drive
        velocity.linear.x = vx
        velocity.linear.y = vy
        velocity.angular.z= vthe
        while (rospy.Time.now() - start_time) < rospy.Duration(timeout):
            self.cmd_vel.publish(velocity)
            rospy.sleep(0.1)
        
        # Stop driving
        velocity.linear.x = 0.0
        velocity.linear.y = 0.0
        velocity.angular.z= 0.0
        self.cmd_vel.publish(velocity)
        
        return True                   

    ####################### DUMMY FUNCTIONS #######################

    def reset_costmap(self):
        rospy.logwarn("[base2.py] Function 'reset_costmap' is obsolete.")
        return True

    def cancel_goal(self):
        rospy.logwarn("[base2.py] Function 'cancel_goal' is obsolete.")
        return True
 
