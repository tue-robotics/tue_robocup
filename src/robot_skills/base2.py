#! /usr/bin/env python

#
#  Rein Appeldoorn
#  October '14
#

import roslib, rospy
roslib.load_manifest('robot_skills')
from geometry_msgs.msg import PoseStamped, Point, Twist, PoseWithCovarianceStamped
from cb_planner_msgs_srvs.srv import *
from cb_planner_msgs_srvs.msg import *
import actionlib
from util import transformations

###########################################################################################################################

class LocalPlanner():
    def __init__(self):
        self._action_client = actionlib.SimpleActionClient('/cb_base_navigation/local_planner_interface/action_server', LocalPlannerAction)

        # Public members!
        self.status = "idle" # idle, controlling, blocked, arrived
        self.obstacle_point = None
        self.dtg = None

    def setPlan(self, plan, orientation_constraint):
        goal = LocalPlannerGoal()
        goal.plan = plan
        goal.orientation_constraint = orientation_constraint
        self._action_client.send_goal(goal, done_cb = self.__doneCallback, feedback_cb = self.__feedbackCallback) 

    def cancelCurrentPlan(self):
        self._action_client.cancel_goal()

    def __feedbackCallback(self, feedback):
        if feedback.blocked:
            self.status = "blocked"
            self.obstacle_point = feedback.point_blocked
        else:
            self.status = "controlling" 
            self.obstacle_point = None
        self.dtg = feedback.dtg

    def __doneCallback(self, terminal_state, result):
        self.dtg = None
        self.obstacle_point = None
        self.status = "arrived"

###########################################################################################################################

class GlobalPlanner():
    def __init__(self):
        self._get_plan_client = rospy.ServiceProxy("/cb_base_navigation/global_planner_interface/get_plan_srv", GetPlan)
        self._check_plan_client = rospy.ServiceProxy("/cb_base_navigation/global_planner_interface/check_plan_srv", CheckPlan)
        rospy.loginfo("Waiting for the global planner services ...")

    def getPlan(self, position_constraint):

        self.position_constraint = position_constraint

        pcs = []
        pcs.append(position_constraint)

        try:
            resp = self._get_plan_client(pcs)
        except:
            rospy.logerr("Could not get plan from global planner via service call, is the global planner running?")
            return None

        if not resp.succes:
            rospy.logerr("Global planner couldn't plan a path to the specified constraints. Are the constraints you specified valid?")
            return None

        return resp.plan

    def checkPlan(self, plan):
        try:
            resp = self._check_plan_client(plan)
        except:
            rospy.logerr("Could not check plan, is the global planner running?")
            return False

        return resp.valid

###########################################################################################################################

class Base(object):
    def __init__(self, robot, tf_listener, wait_service=True, use_2d=None):
        self._tf_listener = tf_listener
        self._robot = robot
        self._cmd_vel = rospy.Publisher('/' + self._robot.robot_name + '/base/references', Twist)
        self._initial_pose_publisher = rospy.Publisher('/' + self._robot.robot_name + '/initialpose', PoseWithCovarianceStamped)

        # The plannners
        self.global_planner = GlobalPlanner()
        self.local_planner = LocalPlanner()

    def move(self, position_constraint_string, frame):
        p = PositionConstraint()
        p.constraint = position_constraint_string
        p.frame = frame
        plan = self.global_planner.getPlan(p)
        if plan:
            o = OrientationConstraint()
            o.frame = frame
            self.local_planner.setPlan(plan,o)

        return plan

    def force_drive(self, vx, vy, vth, timeout):
        
        # Cancel the local planner goal
        self.local_planner.cancelCurrentPlan()
        
        v = Twist()        # Initialize velocity
        t_start = rospy.Time.now()
        
        # Drive
        v.linear.x = vx
        v.linear.y = vy
        v.angular.z= vth
        while (rospy.Time.now() - start_time) < rospy.Duration(timeout):
            self._cmd_vel.publish(v)
            rospy.sleep(0.1)
        
        # Stop driving
        v.linear.x = 0.0
        v.linear.y = 0.0
        v.angular.z= 0.0
        self._cmd_vel.publish(v)
        
        return True       

################################################################################################################################################################
################################################################################################################################################################
################################################################################################################################################################
################################################################################################################################################################
####################################################### DEPRECATED BELOW THESE LINES ###########################################################################
################################################################################################################################################################
################################################################################################################################################################
################################################################################################################################################################
################################################################################################################################################################

    def go(self, x, y, phi, frame="/map", timeout=0):
        rospy.logwarn("[base2.py] Function 'go' is obsolete.")
        return True

    def get_location(self):
        rospy.logwarn("[base2.py] Function 'get_location' is obsolete.")

        try:
            #tf_listener = tf_server.TFClient()
            time = rospy.Time.now()
            self._tf_listener.waitForTransform("/map", "/" + self._robot.robot_name + "/base_link", time, rospy.Duration(20.0))
            (ro_trans, ro_rot) = self._tf_listener.lookupTransform("/map", "/" + self._robot.robot_name + "/base_link", time)
            
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
        rospy.logwarn("[base2.py] Function 'set_initial_pose' is obsolete.")

        initial_pose = geometry_msgs.msg.PoseWithCovarianceStamped()
        
        initial_pose.header.frame_id = "/map"
        
        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation = transformations.euler_z_to_quaternion(phi) 
        initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        #rospy.loginfo("initalpose = {0}".format(initial_pose))

        self._initial_pose_publisher.publish(initial_pose)

        return True     

    def reset_costmap(self):
        rospy.logwarn("[base2.py] Function 'reset_costmap' is obsolete.")
        return True

    def cancel_goal(self):
        rospy.logwarn("[base2.py] Function 'cancel_goal' is obsolete.")
        return True

