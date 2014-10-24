#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_smach_states')
import rospy
import smach
import geometry_msgs.msg
import time
import ed

from math import cos, sin
from geometry_msgs.msg import *
from cb_planner_msgs_srvs.srv import *
from cb_planner_msgs_srvs.msg import *

import math
from psi import Term, Compound, Conjunction
import actionlib
from random import choice

class getPlan(smach.State):
    def __init__(self, robot, position_constraint, orientation_constraint):
        smach.State.__init__(self,outcomes=['unreachable','goal_not_defined','goal_ok'])
        self.robot = robot 

        self.position_constraint = position_constraint
        self.orientation_constraint = orientation_constraint

    def execute(self, userdata):
        # Perform some typechecks
        if not isinstance(self.position_constraint, PositionConstraint) or not isinstance(self.orientation_constraint, OrientationConstraint):
            rospy.loginfo("Invalid constraints given to getPlan()")
            return "goal_not_defined"

        plan = self.robot.base.global_planner.getPlan(self.position_constraint)

        if not plan:
            return "goal_not_defined"

        if (len(plan) == 0):
            return "unreachable"

        # Constraints and plan seem to be valid, so set the plan
        self.robot.base.local_planner.setPlan(plan, self.position_constraint, self.orientation_constraint)

        self.robot.speech.speak(choice(["I'm on my way!","Getting there!","I will go there right away!"]))

        return "goal_ok"

class executePlan(smach.State):
    def __init__(self, robot, blocked_timeout = 4):
        smach.State.__init__(self,outcomes=['arrived','blocked'])
        self.robot = robot 
        self.t_last_free = None     
        self.blocked_timeout = blocked_timeout   

    def execute(self, userdata):

        self.t_last_free = rospy.Time.now()

        # Cancel head goal, we need it for navigation :)
        if self.robot.head.getGoal():
                self.robot.head.cancelGoal()

        while True:
            rospy.Rate(1.0).sleep() # 1hz

            status = self.robot.base.local_planner.getStatus()

            if status == "arrived":
                return "arrived"      
            elif self.isBlocked(status):
                return "blocked"

    def isBlocked(self, status):
        if status == "blocked":
            if (rospy.Time.now() - self.t_last_free) > rospy.Duration(self.blocked_timeout):
                return True
        else:
            self.t_last_free = rospy.Time.now()
        return False

class determineBlocked(smach.State):
   def __init__(self, robot, timeout=5):
       smach.State.__init__(self,outcomes=['blocked','blocked_human', 'free'])
       self.robot = robot 
       self.timeout = timeout

   def execute(self, userdata):

        r = rospy.Rate(1.0) # 1hz
        t_start = rospy.Time.now()  

        while self.robot.base.local_planner.getStatus() == "blocked":
            entity = self.robot.ed.getClosestEntity(center_point=self.robot.base.local_planner.getObstaclePoint())

            # Look at the entity
            if entity:
                self.robot.head.setLookAtGoal(entity.id)

            if (rospy.Time.now() - t_start) > rospy.Duration(self.timeout):
                if not entity or entity.type != "human":
                    return "blocked"
                else:
                    return "blocked_human" 

            r.sleep()

        return "free"    

class planBlocked(smach.State):
   def __init__(self, robot, timeout = 1):
       smach.State.__init__(self,outcomes=['replan','free'])
       self.robot = robot 
       self.timeout = timeout

   def execute(self, userdata):

        r = rospy.Rate(1.0) # 1hz
        t_start = rospy.Time.now()

        self.robot.speech.speak(choice(["An obstacle is blocking my plan!","I cannot get through here!"]))

        while self.robot.base.local_planner.getStatus() == "blocked":
            if (rospy.Time.now() - t_start) > rospy.Duration(self.timeout):
                return "replan"
            r.sleep()

        return "free"

class planBlockedHuman(smach.State):
   def __init__(self, robot, timeout = 10):
       smach.State.__init__(self,outcomes=['replan','free'])
       self.robot = robot 
       self.timeout = timeout

   def execute(self, userdata):

        r = rospy.Rate(2.0) # 1hz
        t_start = rospy.Time.now()

        while self.robot.base.local_planner.getStatus() == "blocked":
            self.robot.speech.speak(choice(["Please, get out of my way!","Excuse me, I can't get through!","Please step aside!"]))
            if (rospy.Time.now() - t_start) > rospy.Duration(self.timeout):
                return "replan"
            r.sleep()

        self.robot.speech.speak(choice(["Thank you!","Thanks a lot bro!","Delightful!"]))

        return "free"

class NavigateWithConstraints(smach.StateMachine):
    def __init__(self, robot, position_constraint, orientation_constraint):
        smach.StateMachine.__init__(self,outcomes=['arrived','unreachable','goal_not_defined'])
        self.robot = robot

        rospy.loginfo("{0},{1}".format(position_constraint,orientation_constraint))

        with self:

            smach.StateMachine.add('GET_PLAN',                          getPlan(self.robot, position_constraint, orientation_constraint),
                transitions={'unreachable'                          :   'unreachable',
                             'goal_not_defined'                     :   'goal_not_defined',
                             'goal_ok'                              :   'EXECUTE_PLAN'})

            smach.StateMachine.add('EXECUTE_PLAN',                      executePlan(self.robot),
                transitions={'arrived'                              :   'arrived',
                             'blocked'                              :   'DETERMINE_BLOCKED'})

            smach.StateMachine.add('DETERMINE_BLOCKED',                 determineBlocked(self.robot),
                transitions={'blocked_human'                        :   'PLAN_BLOCKED_HUMAN',
                             'blocked'                              :   'PLAN_BLOCKED',
                             'free'                                 :   'EXECUTE_PLAN'})

            smach.StateMachine.add('PLAN_BLOCKED_HUMAN',                planBlockedHuman(self.robot),
                transitions={'replan'                               :   'GET_PLAN',
                             'free'                                 :   'EXECUTE_PLAN'})

            smach.StateMachine.add('PLAN_BLOCKED',                      planBlocked(self.robot),
                transitions={'replan'                               :   'GET_PLAN',
                             'free'                                 :   'EXECUTE_PLAN'})

################ TESTS ##################

def testNavigateWithConstraints(robot, constraint="x^2+y^2<1", frame="/map"):
    p = PositionConstraint()
    p.constraint = constraint
    p.frame = frame

    o = OrientationConstraint()   
    o.frame = frame

    nwc = NavigateWithConstraints(robot, p, o)
    nwc.execute()