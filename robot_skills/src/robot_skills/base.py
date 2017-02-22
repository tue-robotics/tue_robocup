#! /usr/bin/env python

#
#  Rein Appeldoorn
#  October '14
#

import math

import geometry_msgs.msg
import rospy
import tf
from actionlib import SimpleActionClient
from cb_planner_msgs_srvs.msg import LocalPlannerAction, OrientationConstraint, PositionConstraint, LocalPlannerGoal
from cb_planner_msgs_srvs.srv import GetPlan, CheckPlan

from .util import nav_analyzer
from .util import transformations


###########################################################################################################################

class LocalPlanner():
    def __init__(self, robot_name, tf_listener, analyzer):
        self.analyzer = analyzer
        self._robot_name = robot_name
        self._tf_listener = tf_listener
        self._action_client = SimpleActionClient('/'+ robot_name +'/local_planner/action_server', LocalPlannerAction)

        # Public members!
        self._status = "idle" # idle, controlling, blocked, arrived
        self._obstacle_point = None
        self._dtg = None
        self._plan = None
        self._goal_handle = None

    def setPlan(self, plan, position_constraint, orientation_constraint):
        goal = LocalPlannerGoal()
        goal.plan = plan
        goal.orientation_constraint = orientation_constraint
        self._orientation_constraint = orientation_constraint
        #self.analyzer.count_plan(plan[0], plan[-1], 0.0, computePathLength(plan))
        self._action_client.send_goal(goal, done_cb = self.__doneCallback, feedback_cb = self.__feedbackCallback)
        self._goal_handle = self._action_client.gh
        rospy.loginfo("Goal handle = {0}".format(self._goal_handle))
        self.__setState("controlling", None, None, plan)

    def cancelCurrentPlan(self):
        state = self._action_client.get_state()
        # Only cancel goal when pending or active
        if state ==0 or state == 1:
            self._action_client.cancel_goal()
            self.__setState("idle")

    def getGoalHandle(self):
        return self._goal_handle

    def getStatus(self):
        return self._status

    def getDistanceToGoal(self):
        return self._dtg

    def getObstaclePoint(self):
        return self._obstacle_point

    def getPlan(self):
        return plan

    def getCurrentOrientationConstraint(self):
        return self._orientation_constraint

    def __feedbackCallback(self, feedback):
        if feedback.blocked:
            self.__setState("blocked", feedback.point_blocked, feedback.dtg, self._plan)
        else:
            self.__setState("controlling", None, feedback.dtg, self._plan)

    def __doneCallback(self, terminal_state, result):
        self.__setState("arrived")

    def __setState(self, status, obstacle_point=None, dtg=None, plan=None):
        self._status = status
        self._obstacle_point = obstacle_point
        self._dtg = dtg
        self._plan = plan

###########################################################################################################################

class GlobalPlanner():
    def __init__(self, robot_name, tf_listener, analyzer):
        self.analyzer = analyzer
        self._robot_name = robot_name
        self._tf_listener = tf_listener
        self._get_plan_client = rospy.ServiceProxy("/" + robot_name + "/global_planner/get_plan_srv", GetPlan)
        self._check_plan_client = rospy.ServiceProxy("/" + robot_name +"/global_planner/check_plan_srv", CheckPlan)
        rospy.loginfo("Waiting for the global planner services ...")

    def getPlan(self, position_constraint):

        self.position_constraint = position_constraint

        pcs = [position_constraint]

        start_time = rospy.Time.now()

        try:
            resp = self._get_plan_client(pcs)
        except Exception as e:
            rospy.logerr("Could not get plan from global planner via service call, is the global planner running?")
            rospy.logerr(e)
            return None

        if not resp.succes:
            rospy.logerr("Global planner couldn't plan a path to the specified constraints. Are the constraints you specified valid?")
            return None

        end_time = rospy.Time.now()
        plan_time = (end_time-start_time).to_sec()

        path_length = self.computePathLength(resp.plan)

        #if path_length > 0:
        #    self.analyzer.count_plan(resp.plan[0], resp.plan[-1], plan_time, path_length)

        return resp.plan

    def checkPlan(self, plan):
        try:
            resp = self._check_plan_client(plan)
        except:
            rospy.logerr("Could not check plan, is the global planner running?")
            return False

        return resp.valid

    def getCurrentPositionConstraint(self):
        return self.position_constraint

    def computePathLength(self, path):
        #rospy.logwarn("Please use the other computepathlength")
        distance = 0.0
        for index, pose in enumerate(path):
            if not index == 0:
                dx = path[index].pose.position.x - path[index-1].pose.position.x
                dy = path[index].pose.position.y - path[index-1].pose.position.y
                distance += math.sqrt( dx*dx + dy*dy)
        return distance

###########################################################################################################################

class Base(object):
    def __init__(self, robot_name, tf_listener, wait_service=True, use_2d=None):
        self._tf_listener = tf_listener
        self._robot_name = robot_name
        self._cmd_vel = rospy.Publisher('/' + self._robot_name + '/base/references', geometry_msgs.msg.Twist, queue_size=10)
        self._initial_pose_publisher = rospy.Publisher('/' + self._robot_name + '/initialpose', geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)

        self.analyzer = nav_analyzer.NavAnalyzer(self._robot_name)

        # The plannners
        self.global_planner = GlobalPlanner(self._robot_name, self._tf_listener, self.analyzer)
        self.local_planner = LocalPlanner(self._robot_name, self._tf_listener, self.analyzer)

    def move(self, position_constraint_string, frame):
        p = PositionConstraint()
        p.constraint = position_constraint_string
        p.frame = frame
        plan = self.global_planner.getPlan(p)
        if plan:
            o = OrientationConstraint()
            o.frame = frame
            self.local_planner.setPlan(plan, p, o)

        return plan

    def force_drive(self, vx, vy, vth, timeout):

        # Cancel the local planner goal
        self.local_planner.cancelCurrentPlan()

        v = geometry_msgs.msg.Twist()        # Initialize velocity
        t_start = rospy.Time.now()

        # Drive
        v.linear.x = vx
        v.linear.y = vy
        v.angular.z= vth
        while (rospy.Time.now() - t_start) < rospy.Duration(timeout):
            self._cmd_vel.publish(v)
            rospy.sleep(0.1)

        # Stop driving
        v.linear.x = 0.0
        v.linear.y = 0.0
        v.angular.z= 0.0
        self._cmd_vel.publish(v)

        return True

    def get_location(self):
        """ Returns a PoseStamped with the robot pose
        :return: PoseStamped with robot pose
        """
        return get_location(self._robot_name, self._tf_listener)

    def set_initial_pose(self, x, y, phi):

        initial_pose = geometry_msgs.msg.PoseWithCovarianceStamped()

        initial_pose.header.frame_id = "/map"

        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation = transformations.euler_z_to_quaternion(phi)
        initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        #rospy.loginfo("initalpose = {0}".format(initial_pose))

        # We have to do this twice for some reason, somewhere in ed_localization
        self._initial_pose_publisher.publish(initial_pose)
        rospy.sleep(0.5)
        self._initial_pose_publisher.publish(initial_pose)

        return True

    ########################################################
    ###### Are the following functions deprecated ??? ######
    ########################################################
    def go(self, x, y, phi, frame="/map", timeout=0):
        rospy.logwarn("[constraint_based_base.py] Function 'go' is obsolete.")
        return True

    def reset_costmap(self):
        rospy.logwarn("[constraint_based_base.py] Function 'reset_costmap' is obsolete.")
        return True

    def cancel_goal(self):
        rospy.logwarn("[constraint_based_base.py] Function 'cancel_goal' is obsolete.")
        return True

    ########################################################
    ########################################################
    ########################################################

###########################################################################################################################

def get_location(robot_name, tf_listener):

    try:
        time = rospy.Time.now()
        tf_listener.waitForTransform("/map", "/" + robot_name + "/base_link", time, rospy.Duration(20.0))
        (ro_trans, ro_rot) = tf_listener.lookupTransform("/map", "/" + robot_name + "/base_link", time)

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

def computePathLength(path):
    distance = 0.0
    for index, pose in enumerate(path):
        if not index == 0:
            dx = path[index].pose.position.x - path[index-1].pose.position.x
            dy = path[index].pose.position.y - path[index-1].pose.position.y
            distance += math.sqrt( dx*dx + dy*dy)
    return distance
