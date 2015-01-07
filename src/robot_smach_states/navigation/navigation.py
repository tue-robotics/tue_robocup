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

class StartAnalyzer(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,outcomes=['done'])
        self.robot = robot
        
    def execute(self, userdata):
        self.robot.base.analyzer.start_measurement(self.robot.base.get_location())
        return 'done'

# ----------------------------------------------------------------------------------------------------
        
class StopAnalyzer(smach.State):
    def __init__(self, robot, result):
        smach.State.__init__(self,outcomes=['done'])
        self.robot  = robot
        self.result = result
        
    def execute(self, userdata):
        self.robot.base.analyzer.stop_measurement(self.robot.base.get_location(), self.result)
        return 'done'

# ----------------------------------------------------------------------------------------------------

class AbortAnalyzer(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,outcomes=['done'])
        self.robot  = robot
        
    def execute(self, userdata):
        self.robot.base.analyzer.abort_measurement()
        return 'done'

# ----------------------------------------------------------------------------------------------------

class getPlan(smach.State):
    def __init__(self, robot, constraint_function):
        smach.State.__init__(self,
            outcomes=['unreachable','goal_not_defined','goal_ok','preempted'])
        self.robot = robot 
        self.constraint_function = constraint_function

    def execute(self, userdata):

        if self.preempt_requested():
            rospy.loginfo('Get plan: preempt_requested')
            return 'preempted'  

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
        smach.State.__init__(self,outcomes=['arrived','blocked','preempted'])
        self.robot = robot 
        self.t_last_free = None     
        self.blocked_timeout = blocked_timeout   

    def execute(self, userdata):

        self.t_last_free = rospy.Time.now()

        # Cancel head goal, we need it for navigation :)
        if self.robot.head.getGoal():
                self.robot.head.cancelGoal()

        while not rospy.is_shutdown():
            rospy.Rate(1.0).sleep() # 1hz

            if self.preempt_requested():
                self.robot.base.local_planner.cancelCurrentPlan()
                rospy.loginfo('execute: preempt_requested')
                return 'preempted'

            status = self.robot.base.local_planner.getStatus()

            if status == "arrived":
                return "arrived"      

# ----------------------------------------------------------------------------------------------------

class determineBlocked(smach.State):
   def __init__(self, robot):
       smach.State.__init__(self,outcomes=['blocked','blocked_human', 'free'])
       self.robot = robot 

   def execute(self, userdata):

        rospy.logwarn("Plan blocked");

        # Look at the entity
        p=self.robot.base.local_planner.getObstaclePoint()
        p.z = 1.6
        self.robot.head.setLookAtGoal("/map", p)

        # Wait for 5 seconds but continue if the path is free
        wait_start = rospy.Time.now()
        while ( (rospy.Time.now() - wait_start) < rospy.Duration(5.0) ):
            rospy.sleep(0.5)
            if not self.robot.base.local_planner.getStatus == "blocked":
                self.robot.head.cancelGoal()
                rospy.logwarn("Plan free again")
                return "free"
        
        # Else: take other actions
        self.robot.head.cancelGoal()

        if self.robot.base.local_planner.getStatus() == "blocked":
            if len(self.robot.ed.get_entities(type="human", center_point=self.robot.base.local_planner.getObstaclePoint(), radius=1)) > 0:
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

class breakOutState(smach.State):
    """docstring for breakOutState"""
    def __init__(self, robot, breakout_function):
        smach.State.__init__(self,outcomes=['preempted','passed'])
        self.robot = robot
        self.breakout_function = breakout_function

    def execute(self, userdata):

        breakout = False
        while (not rospy.is_shutdown() and not breakout and not self.preempt_requested()):
            rospy.sleep(rospy.Duration(0.5))
            breakout = self.breakout_function()

        if breakout:
            return 'preempted'

        return 'passed'
        

# # ----------------------------------------------------------------------------------------------------

class NavigateTo(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['arrived','unreachable','goal_not_defined'])
        self.robot = robot

        with self:

            # Create the sub SMACH state machine
            sm_nav = smach.StateMachine(outcomes=['unreachable','goal_not_defined','preempted'])

            with sm_nav:

                smach.StateMachine.add('GET_PLAN',                          getPlan(self.robot, self.generateConstraint),
                    transitions={'unreachable'                          :   'RESET_WORLD_MODEL',
                                 'goal_not_defined'                     :   'goal_not_defined',
                                 'goal_ok'                              :   'EXECUTE_PLAN'})

                smach.StateMachine.add('EXECUTE_PLAN',                      executePlan(self.robot),
                    transitions={'arrived'                              :   'GET_PLAN',
                                 'blocked'                              :   'DETERMINE_BLOCKED',
                                 'preempted'                            :   'preempted'})

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

            # Create the concurrent state machine
            # gets called when ANY child state terminates
            def child_term_cb(outcome_map):
                if outcome_map['MONITOR'] == 'preempted':
                    return True

                if outcome_map['MONITOR'] == 'passed':
                    return False

                if outcome_map['NAVIGATE'] == 'unreachable':
                    return True

            sm_con = smach.Concurrence( outcomes=['arrived','unreachable','goal_not_defined','preempted'],
                                        default_outcome='arrived',
                                        outcome_map={   'arrived'           :   {   'NAVIGATE'  : 'preempted',
                                                                                    'MONITOR'   : 'preempted'},
                                                        'unreachable'       :   {   'NAVIGATE'  : 'unreachable',
                                                                                    'MONITOR'   : 'passed'},
                                                        'goal_not_defined'  :   {   'NAVIGATE'  : 'goal_not_defined',
                                                                                    'MONITOR'   : 'passed'},
                                                        'preempted'         :   {   'MONITOR'   : 'preempted'}},
                                        child_termination_cb = child_term_cb)

            with sm_con:
                smach.Concurrence.add('NAVIGATE', sm_nav)
                smach.Concurrence.add('MONITOR' , breakOutState(self.robot, self.breakOut))

            smach.StateMachine.add('START_ANALYSIS', StartAnalyzer(self.robot),
                transitions={'done'                                 :'MONITORED_NAVIGATE'})

            smach.StateMachine.add('MONITORED_NAVIGATE', sm_con,
                transitions={'arrived'                              : 'STOP_ANALYSIS_SUCCEED',
                             'unreachable'                          : 'STOP_ANALYSIS_UNREACHABLE',
                             'goal_not_defined'                     : 'ABORT_ANALYSIS_NOT_DEFINED',
                             'preempted'                            : 'STOP_ANALYSIS_SUCCEED'})

            smach.StateMachine.add('STOP_ANALYSIS_SUCCEED', StopAnalyzer(self.robot, 'succeeded'),
                transitions={'done'                                 : 'arrived'})
                
            smach.StateMachine.add('STOP_ANALYSIS_UNREACHABLE', StopAnalyzer(self.robot, 'unreachable'),
                transitions={'done'                                 : 'unreachable'})
                
            smach.StateMachine.add('ABORT_ANALYSIS_NOT_DEFINED', AbortAnalyzer(self.robot),
                transitions={'done'                                 : 'goal_not_defined'})

    def generateConstraint(self):
        pass

    def breakOut(self):
        ''' 
        Default breakout function: makes sure things go to 'arrived' if robot arrives at goal 
        DO NOT OVERLOAD THIS IF NOT NECESSARY
        '''
        status = self.robot.base.local_planner.getStatus()

        if status == "arrived":
            return True      
        
        return False

# ----------------------------------------------------------------------------------------------------              

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
