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

class Base(object):
    """Interface to the planners """
    
    def __init__(self, tf_listener, wait_service=True, use_2d=None):

        # Some member vars to store some data
        self.plan = []
        self.pc = PositionConstraint()
        self.oc = OrientationConstraint()
        self.local_planner_status = "idle"

        # Wait for services to be active
        rospy.loginfo("Waiting for the global planner services ...")
#        rospy.wait_for_service("/gp/get_plan_srv")
#        rospy.wait_for_service("/gp/check_plan_srv")

        # ROS Services for global planner
        self.get_plan_client = rospy.ServiceProxy("/gp/get_plan_srv", GetPlan)
        self.check_plan_client = rospy.ServiceProxy("/gp/check_plan_srv", CheckPlan)

        # ROS ActionLib for local planner
        self.action_client = actionlib.SimpleActionClient('/lp/action_server', LocalPlannerAction)

        rospy.loginfo("Navigation Interface Initialized [(base2)]")

    ################### LOCAL PLANNER ############################

    def localPlannerSetPlan(self, plan, orientation_constraint):
        goal = LocalPlannerGoal()
        goal.plan = plan
        goal.orientation_constraint = orientation_constraint
        self.action_client.send_goal(goal, done_cb = self.__localPlannerDoneCallback, feedback_cb = self.__localPlannerFeedbackCallback) 
        self.local_planner_status = "controlling"

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
