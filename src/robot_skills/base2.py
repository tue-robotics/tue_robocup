#! /usr/bin/env python

#
#  Rein Appeldoorn
#  October '14
#

import roslib, rospy
roslib.load_manifest('robot_skills')
from geometry_msgs.msg import PoseStamped, Point, Twist
from cb_planner_msgs_srvs.srv import *
from cb_planner_msgs_srvs.msg import *
import actionlib

###########################################################################################################################

class Base(object):
    def __init__(self):
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
