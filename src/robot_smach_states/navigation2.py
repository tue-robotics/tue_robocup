#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_smach_states')
import rospy
import smach
import geometry_msgs.msg

from math import cos, sin
from geometry_msgs.msg import *
from cb_planner_msgs_srvs.srv import *
from cb_planner_msgs_srvs.msg import *

import math
from psi import Term, Compound, Conjunction
import actionlib

class checkGoalPositionConstraint(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,outcomes=['unreachable','goal_not_defined','goal_ok'])
        self.robot = robot 

    def execute(self, userdata):

        # Perform some typechecks
        if not isinstance(self.robot.base2.pc, PositionConstraint) or not isinstance(self.robot.base2.oc, OrientationConstraint):
            rospy.loginfo("Invalid constraints given to NavigateGeneric")
            return "goal_not_defined"

        plan = self.robot.base2.globalPlannerGetPlan(self.robot.base2.pc)

        if (plan < 0):
            return "goal_not_defined"

        if (len(plan) == 0):
            return "unreachable"

        # Constraints and plan seem to be valid, so store plan
        self.robot.base2.plan = plan

        return "goal_ok"

class prepareNavigation(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,outcomes=['succeeded'])
        self.robot = robot 

    def execute(self, userdata):
        # Set all poses for navigation

        return 'succeeded'

class executePlan(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,outcomes=['arrived','blocked'])
        self.robot = robot 

    def execute(self, userdate):

        time_path_free = rospy.Time.now()
        count = 5
        r = rospy.Rate(1.0) # 10hz
        while not rospy.is_shutdown():

            if self.robot.base2.local_planner_status == "arrived":
                self.robot.base2.local_planner_status = "idle"
                return "arrived"

            # Check if we can find an other plan
            plan = self.robot.base2.globalPlannerGetPlan(self.robot.base2.pc)

            if plan < 0 or len(plan) == 0:
                count+=1
            else:
                count = 0

            if count > 5:
                self.robot.base2.localPlannerCancelCurrentPlan()
                return "blocked"
            
            # Send the plan to the local_planner
            self.robot.base2.localPlannerSetPlan(self.robot.base2.plan, self.robot.base2.oc)

            r.sleep()

#class planBlocked(smach.State):
#    def __init__(self, robot):
#        smach.State.__init__(self,outcomes=['execute','unreachable'])
#        self.robot = robot 
#
#    def execute(self, userdate):
#
#        # Check if we can find an other plan
#        plan = self.robot.base2.globalPlannerGetPlan(self.robot.base2.pc)
#
#        if plan < 0:
#            self.robot.base2.localPlannerCancelCurrentPlan()
#            return "unreachable"
#
#        if len(plan) == 0:
#            self.robot.base2.localPlannerCancelCurrentPlan()
#            return "unreachable"
#
#        # Set the new plan
#        self.robot.base2.local_planner_status = "idle"
#        self.robot.base2.plan = plan
#
#        return "execute"

class NavigateWithConstraints(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self,outcomes=['arrived','unreachable','goal_not_defined'])
        self.robot = robot

        with self:

            smach.StateMachine.add('CHECK_GOAL_POSITION_CONSTRAINT',    checkGoalPositionConstraint(self.robot),
                transitions={'unreachable'                          :   'unreachable',
                             'goal_not_defined'                     :   'goal_not_defined',
                             'goal_ok'                              :   'PREPARE_NAVIGATION'})

            smach.StateMachine.add('PREPARE_NAVIGATION',                prepareNavigation(self.robot),
                transitions={'succeeded'                            :   'EXECUTE_PLAN' })

            smach.StateMachine.add('EXECUTE_PLAN',                      executePlan(self.robot),
                transitions={'arrived'                              :   'arrived',
                             'blocked'                              :   'unreachable'})
