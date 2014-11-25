#! /usr/bin/env python
import roslib; 
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

# ----------------------------------------------------------------------------------------------------

class getPlan(smach.State):
    def __init__(self, robot, constraint_function):
        smach.State.__init__(self,
            outcomes=['unreachable','goal_not_defined','goal_ok'])
        self.robot = robot 
        self.constraint_function = constraint_function

    def execute(self, userdata):    

        constraint = self.constraint_function()

        # Perform some typechecks
        if not constraint:
            rospy.logwarn("Invalid constraints given to getPlan()")
            return "goal_not_defined"

        pc, oc = constraint

        plan = self.robot.base.global_planner.getPlan(pc)

        if not plan or len(plan) == 0:
            self.robot.base.local_planner.cancelCurrentPlan()
            return "unreachable"

        # Constraints and plan seem to be valid, so set the plan
        self.robot.base.local_planner.setPlan(plan, pc, oc)

        self.robot.speech.speak(choice(["I'm on my way!","Getting there!","I will go there right away!"]))

        return "goal_ok"

# ----------------------------------------------------------------------------------------------------

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

# ----------------------------------------------------------------------------------------------------

class determineBlocked(smach.State):
   def __init__(self, robot):
       smach.State.__init__(self,outcomes=['blocked','blocked_human', 'free'])
       self.robot = robot 

   def execute(self, userdata):

        # Look at the entity
        p=self.robot.base.local_planner.getObstaclePoint()
        p.z = 1.6
        self.robot.head.setLookAtGoal("/map", p)
        rospy.sleep(5.0)
        self.robot.head.cancelGoal()

        if self.robot.base.local_planner.getStatus() == "blocked":
            if len(self.robot.ed.getEntities(type="human", center_point=self.robot.base.local_planner.getObstaclePoint(), radius=1)) > 0:
                return "blocked_human"
            else:
                return "blocked" 

        return "free"    

# ----------------------------------------------------------------------------------------------------

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

# ----------------------------------------------------------------------------------------------------

class planBlockedHuman(smach.State):
   def __init__(self, robot, timeout = 10):
       smach.State.__init__(self,outcomes=['replan','free'])
       self.robot = robot 
       self.timeout = timeout

   def execute(self, userdata):

        r = rospy.Rate(.5) # 1hz
        t_start = rospy.Time.now()

        while self.robot.base.local_planner.getStatus() == "blocked":
            self.robot.speech.speak(choice(["Please, get out of my way!","Excuse me, I can't get through!","Please step aside!"]))
            if (rospy.Time.now() - t_start) > rospy.Duration(self.timeout):
                return "replan"
            r.sleep()

        self.robot.speech.speak(choice(["Thank you!","Thanks a lot bro!","Delightful!"]))

        return "free"

# ----------------------------------------------------------------------------------------------------

class resetWorldModel(smach.State):
   def __init__(self, robot):
       smach.State.__init__(self,outcomes=['succeeded'])
       self.robot = robot 

   def execute(self, userdata):
        self.robot.ed.reset()
        return "succeeded"

# # ----------------------------------------------------------------------------------------------------

# class NavigateWithConstraintsOnce(smach.StateMachine):
#     def __init__(self, robot, position_constraint, orientation_constraint):
#         smach.StateMachine.__init__(self,outcomes=['arrived','unreachable','goal_not_defined'])
#         self.robot = robot

#         with self:

#             smach.StateMachine.add('GET_PLAN',                          getPlan(self.robot),
#                 transitions={'unreachable'                          :   'RESET_WORLD_MODEL',
#                              'goal_not_defined'                     :   'goal_not_defined',
#                              'goal_ok'                              :   'EXECUTE_PLAN'})

#             smach.StateMachine.add('EXECUTE_PLAN',                      executePlan(self.robot),
#                 transitions={'arrived'                              :   'arrived',
#                              'blocked'                              :   'DETERMINE_BLOCKED'})

#             smach.StateMachine.add('DETERMINE_BLOCKED',                 determineBlocked(self.robot),
#                 transitions={'blocked_human'                        :   'PLAN_BLOCKED_HUMAN',
#                              'blocked'                              :   'PLAN_BLOCKED',
#                              'free'                                 :   'EXECUTE_PLAN'})

#             smach.StateMachine.add('PLAN_BLOCKED_HUMAN',                planBlockedHuman(self.robot),
#                 transitions={'replan'                               :   'GET_PLAN',
#                              'free'                                 :   'EXECUTE_PLAN'})

#             smach.StateMachine.add('PLAN_BLOCKED',                      planBlocked(self.robot),
#                 transitions={'replan'                               :   'GET_PLAN',
#                              'free'                                 :   'EXECUTE_PLAN'})

#             smach.StateMachine.add('RESET_WORLD_MODEL',                 resetWorldModel(self.robot),
#                 transitions={'succeeded'                            :   'unreachable'})

# # ----------------------------------------------------------------------------------------------------

# class NavigateWithConstraints(smach.Sequence):
#     def __init__(self, robot, position_constraint, orientation_constraint):
#         smach.Sequence.__init__(self,outcomes=['arrived','unreachable','goal_not_defined'], connector_outcome = 'unreachable')
#         self.robot = robot

#         with self:
#             smach.Sequence.add('NAVIGATE_TRY_1', NavigateWithConstraintsOnce(self.robot, position_constraint, orientation_constraint))
#             smach.Sequence.add('NAVIGATE_TRY_2', NavigateWithConstraintsOnce(self.robot, position_constraint, orientation_constraint))
#             smach.Sequence.add('NAVIGATE_TRY_3', NavigateWithConstraintsOnce(self.robot, position_constraint, orientation_constraint))

# # ----------------------------------------------------------------------------------------------------

class NavigateTo(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self,outcomes=['arrived','unreachable','goal_not_defined'])
        self.robot = robot

        with self:

            smach.StateMachine.add('GET_PLAN',                          getPlan(self.robot, self.generateConstraint),
                transitions={'unreachable'                          :   'RESET_WORLD_MODEL',
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

            smach.StateMachine.add('RESET_WORLD_MODEL',                 resetWorldModel(self.robot),
                transitions={'succeeded'                            :   'unreachable'})

    def generateConstraint(self):
        pass

    def breakOut(self):
        return False


class NavigateToPose(NavigateTo):
    def __init__(self, robot, x, y, rz, radius = 0.15):
        super(NavigateToPose, self).__init__(robot)

        self.x = x
        self.y = y
        self.rz = rz
        self.radius = radius

    def generateConstraint(self):
        pc = PositionConstraint(constraint="(x-%f)^2+(y-%f)^2 < %f^2"%(self.x, self.y, self.radius), frame="/map")
        oc = OrientationConstraint(look_at=Point(self.x+1, self.y, 0.0), angle_offset=self.rz, frame="/map")

        return pc, oc

    def breakOut(self):
        if bla:
            return True
        else:
            return False

class NavigateToWaypoint(NavigateTo):
    def __init__(self, robot, waypoint, radius = 0.15):
        super(NavigateToWaypoint, self).__init__(robot)

        self.robot    = robot
        self.waypoint = waypoint
        self.radius   = radius

    def generateConstraint(self):
        e = self.robot.ed.getEntity(id=self.waypoint)

        print e

        if not e:
            rospy.logerr("No such entity")
            return None

        try:
            pose = e.data["pose"]
            x = pose["x"]
            y = pose["y"]
            rz= pose["z"]
        except KeyError:
            rospy.logerr(KeyError)
            return None

        rospy.logwarn("Should Sjoerd check the newest model data in???")

        pc = PositionConstraint(constraint="(x-%f)^2+(y-%f)^2 < %f^2"%(x, y, self.radius), frame="/map")
        oc = OrientationConstraint(look_at=Point(x+1, y, 0.0), angle_offset=rz, frame="/map")

        return pc, oc

    def breakOut(self):
        if bla:
            return True
        else:
            return False                       

# class NavigateTo(smach.StateMachine):
#     def __init__(self, robot, constraint_args={}, break_out_args={}):
#         smach.StateMachine.__init__(sm, outcomes=['arrived','unreachable','goal_not_defined'])

#         self.robot = robot
#         self.constraint_args = constraint_args
#         self.break_out_args  = break_out_args

#     @smach.cb_interface(input_keys=['']
#                         output_keys=['position_constraint','orientation_constraint'],
#                         outcomes=['succeeded','failed'])
#     def generateConstraint(self, userdata):
#         return 'failed'

#     def breakOut(self):
#         return False

#     with sm:
#         smach.StateMachine.add('DETERMINE_CONSTRAINT', CBState(self.generateConstraint,
#                                                         cb_kwargs=self.constraint_args),
#                                 transitions={'succeeded'        : 'arrived',
#                                              'failed'           : 'goal_not_defined'})

#         smach.StateMachine.add('NAVIGATE', NavigateWithConstraintsOnce,
#                                 transitions={'arrived'          : 'arrived',
#                                              'unreachable'      : 'unreachable',
#                                              'goal_not_defined' : 'goal_not_defined'})


# class NavigateToPose(NavigateTo):
#     def __init__(self, robot, x, y, rz):
#         NavigateTo.__init__(robot, constraint_args={'x': x, 'y': y, 'rz': rz})

#     def generateConstraint(self, userdata):
#         userdata.position_constraint = PositionConstraint(constraint="(x-%d)+(x-%d)^2 < %d"%(x,y,radius), frame="/map")
#         userdata.orientation_constraint = OrientationConstraint(look_at=Point(x+1,y), angle_offset=rz, frame="/map")

#         return 'succeeded'

#     def breakOut(self):
#         if bla:
#             return True
#         else:
#             return False
















# ----------------------------------------------------------------------------------------------------

# ToDo: move up
def generateWaypointConstraint(robot, entityId):
    #robot.ed.do_useful_stuff

    rospy.loginfo("Defaulting navigation to 1, 3, 0")

    position_constraint = PositionConstraint

    return position_constraint

class constraintGenerator(smach.State):
    def __init__(self):
        smach.State.__init__(outcomes=['succeeded','failed'],
                            input_keys=['position_constraint', 'orientation_constraint'],
                            output_keys=['position_constraint', 'orientation_constraint'])

    def execute(self, userdata):
        return 'failed'

class Navigate(smach.StateMachine):
    """Look at an object. Depending on its geometry, several viewpoints are taken and iterated over"""

    def __init__(self, robot, baseConstraintGenerator=None, finishedChecker=None):
        """@param robot the robot with which to perform this action
        @param entityId the entity or item to observe.
        @param baseConstraintGenerator a function func(robot, entityInfo) that returns a (PositionConstraint, OrientationConstraint)-tuple for cb_navigation. 
            entityInfo is a ed/EntityInfo message. 
        @param finishedChecker a function(robot) that checks whether the item if observed to your satisfaction. """
        smach.StateMachine.__init__(self, outcomes=['arrived','unreachable','preempted','goal_not_defined'])

# ----------------------------------------------------------------------------------------------------

################ TESTS ##################

def testNavigateWithConstraints(robot, constraint="x^2+y^2<1", frame="/map"):
    p = PositionConstraint()
    p.constraint = constraint
    p.frame = frame

    o = OrientationConstraint()   
    o.frame = frame

    nwc = NavigateWithConstraints(robot, p, o)
    nwc.execute()
