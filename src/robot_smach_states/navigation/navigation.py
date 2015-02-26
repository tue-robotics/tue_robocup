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
import actionlib
from random import choice
import robot_skills.util.msg_constructors as msgs

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

        # Sleep for 0.1 s (breakOut sleep) to prevent synchronization errors between monitor state and nav state
        rospy.sleep(rospy.Duration(0.1))

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

        self.robot.speech.speak(choice(["I'm on my way!","Getting there!","I will go there right away!"]), block=False)

        return "goal_ok"

# ----------------------------------------------------------------------------------------------------

class executePlan(smach.State):
    def __init__(self, robot, blocked_timeout = 4):
        smach.State.__init__(self,outcomes=['arrived','blocked','preempted'])
        self.robot = robot
        self.t_last_free = None
        self.blocked_timeout = blocked_timeout

    def execute(self, userdata):
        # ToDo: check for alternative plans???

        self.t_last_free = rospy.Time.now()

        # Cancel head goal, we need it for navigation :)
        if self.robot.head.getGoal():
                self.robot.head.cancelGoal()

        while not rospy.is_shutdown():
            rospy.Rate(1.0).sleep() # 1hz

            # Check for the presence of a better plan
            self.checkBetterPlan()

            if self.preempt_requested():
                self.robot.base.local_planner.cancelCurrentPlan()
                rospy.loginfo('execute: preempt_requested')
                return 'preempted'

            status = self.robot.base.local_planner.getStatus()

            if status == "arrived":
                return "arrived"
            elif status == "blocked":
                return "blocked"

    def checkBetterPlan(self):
        # Get alternative plan
        pc = self.robot.base.global_planner.getCurrentPositionConstraint()
        if pc:
            plan = self.robot.base.global_planner.getPlan(pc)

        # Compare plan
        if (plan and len(plan) > 0):
            dtgcp = self.robot.base.local_planner.getDistanceToGoal()           # Distance To Goal Current Plan
            if dtgcp == None:
                return

            dtgap = self.robot.base.global_planner.computePathLength(plan)      # Distance To Goal Alternative Plan
            rospy.logdebug('Distance original = {0}, distance alternative = {1}'.format(dtgcp, dtgap))

            # Path should be at least 20% and 1 m shorter
            if (dtgap/dtgcp < 0.8 and (dtgcp - dtgap) > 1.0):
                rospy.logwarn("Executing alternative path!")
                oc = self.robot.base.local_planner.getCurrentOrientationConstraint()

                if oc:
                    # Constraints and plan seem to be valid, so set the plan
                    self.robot.base.local_planner.setPlan(plan, pc, oc)
                else:
                    rospy.logerr("Cannot execute alternative plan due to absence of orientation constraint")


# ----------------------------------------------------------------------------------------------------

class determineBlocked(smach.State):
   def __init__(self, robot):
       smach.State.__init__(self,outcomes=['blocked','blocked_human', 'free'])
       self.robot = robot

   def execute(self, userdata):

        rospy.loginfo("Plan blocked");

        # ToDo: move this to execute???

        # Look at the entity
        ps = msgs.PointStamped(point=self.robot.base.local_planner.getObstaclePoint(), frame_id="/map")
        ps.point.z = 1.6
        self.robot.head.setLookAtGoal(ps)

        # Wait for 5 seconds but continue if the path is free
        wait_start = rospy.Time.now()
        while ( (rospy.Time.now() - wait_start) < rospy.Duration(5.0) ):
            rospy.sleep(0.5)

            # Get alternative plan
            pc = self.robot.base.global_planner.getCurrentPositionConstraint()
            if pc:
                plan = self.robot.base.global_planner.getPlan(pc)

            # Compare plan
            if (plan and len(plan) > 0):
                dtgap = self.robot.base.global_planner.computePathLength(plan)     # Distance To Goal Alternative Plan
                rospy.loginfo('Distance original = {0}, distance alternative = {1}'.format(self.robot.base.local_planner.getDistanceToGoal(), dtgap))

                dtgcp = self.robot.base.local_planner.getDistanceToGoal()

                if (dtgcp == None):
                    rospy.logdebug("Current distance to goal not available")
                elif (dtgap/dtgcp < 1.2):
                    rospy.loginfo("Executing alternative path!")
                    oc = self.robot.base.local_planner.getCurrentOrientationConstraint()

                    if oc:
                        # Constraints and plan seem to be valid, so set the plan
                        self.robot.base.local_planner.setPlan(plan, pc, oc)
                        return "free"
                    else:
                        rospy.logerr("Cannot execute alternative plan due to absence of orientation constraint")

                else:
                    rospy.loginfo("Alternative path too long...")

            if not self.robot.base.local_planner.getStatus() == "blocked":
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

        rospy.loginfo("Robot is properly blocked, trying to recover")

        self._reset()

        r = rospy.Rate(1.0) # 1hz

        ''' Determine possible recovery behaviors:
            -   replan
            -   people: ask them to step aside
            -   object: push or remove (not yet implemented)
            -   wait
            -   clearing costmap
            -   resetting ED
            -   giving up...
        '''
        self._init_dummy()
        self._init_replan()
        self._init_human()
        #self._reset()
        #self._init_move_object()

        ''' Order possible recovery behaviors '''
        self._options = sorted(self._options, key=lambda k: k['cost'])

        ''' Loop through the options to see what we can really do '''
        for option in self._options:
            if option['method']():
                return "free"

        ''' If nothing else works: wait a little longer and ask for a replan
            Note that returning 'replan' may result in an updated position constraint, which is different from the replanning considered here!
        '''
        self.robot.speech.speak(choice(["An obstacle is blocking my plan!","I cannot get through here!"]))

        while self.robot.base.local_planner.getStatus() == "blocked":
            if (rospy.Time.now() - self._t_start) > rospy.Duration(self.timeout):
                return "replan"
            r.sleep()

        return "free"

    def _reset(self):

        ''' Resets all variables '''
        # Cost is estimated time in seconds to execute
        self._options = []
        self._t_start = rospy.Time.now()

        self._pc = None
        self._plan = None

    # ToDo: put in separate classes (don't screw around with method...)

    def _init_dummy(self):
        self._options.append({'name': 'dummy', 'cost': 0.01, 'method': self._execute_dummy})

    def _execute_dummy(self):
        rospy.loginfo('Recovery: executing dummy')
        return False

    def _init_replan(self):
        # Get alternative plan
        self._pc = self.robot.base.global_planner.getCurrentPositionConstraint()
        if self._pc:
            self._plan = self.robot.base.global_planner.getPlan(self._pc)

        # Compare plan
        if (self._plan and len(self._plan) > 0):
            dtgap = self.robot.base.global_planner.computePathLength(self._plan)     # Distance To Goal Alternative Plan
            cost = dtgap/0.5 # Time is distance/velocity

            self._options.append({'name': 'replan', 'cost': cost, 'method': self._execute_replan})

    def _execute_replan(self):
        rospy.loginfo('Recovery: executing replan')
        oc = self.robot.base.local_planner.getCurrentOrientationConstraint()

        if oc:
            # Constraints and plan seem to be valid, so set the plan
            self.robot.base.local_planner.setPlan(self._plan, self._pc, oc)
            return True
        else:
            rospy.logerr("Cannot execute alternative plan due to absence of orientation constraint")
            return False


    def _init_human(self):
        if len(self.robot.ed.get_entities(type="human", center_point=self.robot.base.local_planner.getObstaclePoint(), radius=1)) > 0:
            self._options.append({'name': 'human', 'cost': 5.0, 'method': self._execute_human}) # Estimated time to step aside is 5 seconds

    def _execute_human(self):
        rospy.loginfo('Recovery: asking person to step aside')
        r = rospy.Rate(.5) # 1hz
        t_start = rospy.Time.now()

        while self.robot.base.local_planner.getStatus() == "blocked":
            self.robot.speech.speak(choice(["Please, get out of my way!",
                                            "Please step aside",
                                            "Please let me through",
                                            "Would you please get out of my way",
                                            "Would you please step aside",
                                            "Would you please let me through",
                                            "Can I please get through",
                                            "Excuse me, I can't get through!",
                                            "Excuse me, I would like to pass through here",
                                            "Excuse me, can I go through",
                                            ]), block=False)
            if (rospy.Time.now() - t_start) > rospy.Duration(self.timeout):
                return False
            r.sleep()

        self.robot.speech.speak(choice(["Thank you!","Thanks a lot bro!","Delightful!"]))
        return True

    def _init_reset(self):
        rospy.loginfo('Reset costmap not yet implemented')

    def _init_move_object(self):
        rospy.loginfo('Moving object not yet implemented as recovery behavior')



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
            self.robot.speech.speak(choice(["Please, get out of my way!",
                                            "Please step aside",
                                            "Please let me through",
                                            "Would you please get out of my way",
                                            "Would you please step aside",
                                            "Would you please let me through",
                                            "Can I please get through",
                                            "Excuse me, I can't get through!",
                                            "Excuse me, I would like to pass through here",
                                            "Excuse me, can I go through",
                                            ]), block=False)
            if (rospy.Time.now() - t_start) > rospy.Duration(self.timeout):
                return "replan"
            r.sleep()

        self.robot.speech.speak(choice(["Thank you!","Thanks a lot bro!","Delightful!"]))

        return "free"


# ----------------------------------------------------------------------------------------------------

class breakOutState(smach.State):
    """docstring for breakOutState"""
    def __init__(self, robot, breakout_function):
        smach.State.__init__(self,outcomes=['preempted','passed'])
        self.robot = robot
        self.breakout_function = breakout_function

    def execute(self, userdata):

        # Wait for some time before checking
        t_start = rospy.Time.now()
        while (not rospy.is_shutdown() and (rospy.Time.now() - t_start) < rospy.Duration(0.5)):
          rospy.sleep(0.1)
        rospy.logwarn("Starting breakout check")

        breakout = False
        while (not rospy.is_shutdown() and not breakout and not self.preempt_requested()):
            rospy.sleep(rospy.Duration(0.1))
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
                    transitions={'unreachable'                          :   'unreachable',
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


            # Create the concurrent state machine
            # gets called when ANY child state terminates
            def child_term_cb(outcome_map):
                if outcome_map['MONITOR'] == 'preempted':
                    return True

                if outcome_map['MONITOR'] == 'passed':
                    return False

                if outcome_map['NAVIGATE'] == 'unreachable':
                    return True

                # if outcome_map['NAVIGATE'] == 'preempted':
                #     rospy.logwarn("Is this a good idea???")
                #     # Does not help...
                #     return False

                if outcome_map['NAVIGATE'] == 'goal_not_defined':
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
                transitions={'done'                                 :'RESET_SM_NAV'})

            @smach.cb_interface(outcomes=['done'],
                                input_keys=[],
                                output_keys=[])
            def reset_sm_nav(userdata, con_state):
                # Recall preempt on concurrency container and all children
                children = con_state.get_children()
                for state in children:
                    children[state].recall_preempt()

                con_state.recall_preempt()
                return 'done'

            smach.StateMachine.add('RESET_SM_NAV', smach.CBState(reset_sm_nav,
                                    cb_kwargs={'con_state': sm_nav}),
                                    transitions={   'done':'MONITORED_NAVIGATE' })

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
        goal_handle = self.robot.base.local_planner.getGoalHandle()

        # if hasattr(self, 'breakout_goal_handle'):
        #     rospy.logwarn("Status = {0}, goal_handle = {1}, stored = {2}".format(status, goal_handle, self.breakout_goal_handle))
        # else:
        #     rospy.logwarn("Status = {0}, goal_handle = {1}".format(status, goal_handle))

        if status == "arrived":

            if not hasattr(self, 'breakout_goal_handle'):

                # If no breakout goal handle exists (hasn't arrived yet), make one and return breakout
                rospy.logwarn("Breaking out for the first time")
                self.breakout_goal_handle = goal_handle
                return True


            elif self.breakout_goal_handle != goal_handle:

                # If the existing goal handle is unequal to the current one, the local planner has arrived at a new goal
                # Hence, store this one and return true
                rospy.logwarn("Breaking out!!")
                self.breakout_goal_handle = goal_handle
                return True

            else:

                # Breakout function has already returned true on this goalhandle
                rospy.logerr("Executing should never be here. If this is the case, this function can be made a lot simpler!")
                return False

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
