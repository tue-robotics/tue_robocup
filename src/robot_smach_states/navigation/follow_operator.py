#! /usr/bin/env python

import smach, rospy, sys
from robot_smach_states.util.startup import startup
import robot_smach_states as states

import threading
import time

import math

from cb_planner_msgs_srvs.msg import *

from robot_skills.util import transformations, msg_constructors

class FollowOperator(smach.State):
    def __init__(self, robot, operator_position_constraint = "x^2 + y^2 < 0.7^2", timeout = 3.0):
        smach.State.__init__(self, outcomes=["stopped",'lost_operator'])
        self._robot = robot
        self._operator_id = None

        self._at_location = False
        self._first_time_at_location = None
        self._operator_position_constraint = operator_position_constraint
        self._timeout = timeout

    def _register_operator(self):
        operator = None
        while not operator:
            self._robot.speech.speak("Please stand in front of me so that I can follow you!")
            rospy.sleep(2)
            operator = self._robot.ed.get_closest_entity(self, center_point=msg_constructors.PointStamped(x=1.7, y=0, z=0, frame_id="/%s/base_link"%self._robot.robot_name))
        print "We have a new operator: %s"%operator.id
        self._robot.speech.speak("Okay, I will follow you!")
        self._operator_id = operator.id

    def _get_operator(self, operator_id):        
        if self._operator_id:
            operator = self._robot.ed.get_entity(id=operator_id)
        else:
            operator = None
                
        return operator
        
    def _update_navigation(self, operator):
        p = PositionConstraint()
        p.constraint = self._operator_position_constraint
        p.frame = operator.id
        plan = self._robot.base.global_planner.getPlan(p)
        if plan:
            # Check whether we are already there
            if len(plan) <= 2:
                if not self._at_location:
                    self._first_time_at_location = rospy.Time.now()
                self._at_location = True

                if (rospy.Time.now() - self._first_time_at_location) > rospy.Duration(self._timeout):
                    return True # We are there
            else:
                self._first_time_at_location = None
                self._at_location = False

            o = OrientationConstraint()
            o.frame = frame
            self._robot.base.local_planner.setPlan(plan, p, o)

        return False # We are not there
        
    def execute(self, userdata):
        self._register_operator()

        while not rospy.is_shutdown():
            
            # Check if operator present still present
            operator = self._get_operator(self._operator_id)

            if not operator:
                return "lost_operator"
            
            # Update the navigation and check if we are already there
            if self._update_navigation(operator):
                return "stopped"

            rospy.sleep(1) # Loop at 1Hz