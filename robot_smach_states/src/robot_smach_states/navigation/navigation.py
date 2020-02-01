# System
from random import choice

# ROS
import rospy
import smach


class StartAnalyzer(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,outcomes=['done'])
        self.robot = robot

    def execute(self, userdata=None):
        self.robot.base.analyzer.start_measurement(self.robot.base.get_location().frame)
        return 'done'


class StopAnalyzer(smach.State):
    def __init__(self, robot, result):
        smach.State.__init__(self,outcomes=['done'])
        self.robot  = robot
        self.result = result

    def execute(self, userdata=None):
        self.robot.base.analyzer.stop_measurement(self.robot.base.get_location().frame, self.result)
        return 'done'


class AbortAnalyzer(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,outcomes=['done'])
        self.robot  = robot

    def execute(self, userdata=None):
        self.robot.base.analyzer.abort_measurement()
        return 'done'


class getPlan(smach.State):
    def __init__(self, robot, constraint_function, speak=True):
        smach.State.__init__(self,
            outcomes=['unreachable','goal_not_defined','goal_ok','preempted'])
        self.robot = robot
        self.constraint_function = constraint_function
        self.speak = speak

    def execute(self, userdata=None):

        # Sleep for 0.1 s (breakOut sleep) to prevent synchronization errors between monitor state and nav state
        rospy.sleep(rospy.Duration(0.1))

        if self.preempt_requested():
            rospy.loginfo('Get plan: preempt_requested')
            return 'preempted'

        constraint = self.constraint_function()

        # Perform some typechecks
        if not constraint:
            rospy.logwarn("Invalid constraints given to getPlan(). constraint: {}".format(constraint))
            return "goal_not_defined"

        pc, oc = constraint

        plan = self.robot.base.global_planner.getPlan(pc)

        if not plan or len(plan) == 0:
            self.robot.base.local_planner.cancelCurrentPlan()
            return "unreachable"

        # Constraints and plan seem to be valid, so set the plan
        self.robot.base.local_planner.setPlan(plan, pc, oc)

        if self.speak:
            self.robot.speech.speak(choice(["Affirmative!","I'm on my way!","Getting there!","I will be there in a sec!", "I'm coming!"]), block=False)

        return "goal_ok"


class executePlan(smach.State):
    def __init__(self, robot, breakout_function, blocked_timeout = 4, reset_head=True):
        smach.State.__init__(self,outcomes=['succeeded','arrived','blocked','preempted'])
        self.robot = robot
        self.t_last_free = None
        self.blocked_timeout = blocked_timeout
        self.breakout_function = breakout_function
        self.reset_head = reset_head

    def execute(self, userdata=None):
        """
        Possible outcomes (when overloading)
        - 'breakout': when a condition has been met and navigation should stop because the goal has succeeded
        - 'checking': the condition has not been met. Upon arrival at a goal, the statemachine will return to 'GetPlan' to get the next goal_not_defined
        - 'passed'  : checking a condition is not necessary. Upon arrival at the current goal, the state machine will return 'succeeded'
        """

        self.t_last_free = rospy.Time.now()

        # Cancel head goal, we need it for navigation :)
        if self.reset_head:
            self.robot.head.close()

        while not rospy.is_shutdown():
            rospy.Rate(10).sleep() # 10hz

            ''' If the breakoutfunction returns preempt,
                navigation has succeeded and the robot can stop'''
            breakout_status = self.breakout_function()
            if breakout_status == "breakout":
                self.robot.base.local_planner.cancelCurrentPlan()
                return "succeeded"

            # Check for the presence of a better plan
            #self.checkBetterPlan()

            if self.preempt_requested():
                self.robot.base.local_planner.cancelCurrentPlan()
                rospy.loginfo('execute: preempt_requested')
                return 'preempted'

            status = self.robot.base.local_planner.getStatus()

            if status == "arrived" and breakout_status == "checking":
                return "arrived"
            elif status == "arrived" and breakout_status == "passed":
                return "succeeded"
            elif status == "blocked":
                return "blocked"

    def checkBetterPlan(self):
        # Get alternative plan
        pc = self.robot.base.global_planner.getCurrentPositionConstraint()
        if pc:
            plan = self.robot.base.global_planner.getPlan(pc)

        # Compare plan
        if plan and len(plan) > 0:
            dtgcp = self.robot.base.local_planner.getDistanceToGoal()  # Distance To Goal Current Plan
            if dtgcp is None:
                return

            dtgap = self.robot.base.global_planner.computePathLength(plan)  # Distance To Goal Alternative Plan
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


class planBlocked(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,outcomes=['blocked', 'free'])
        self.robot = robot

    def execute(self, userdata=None):

        rospy.loginfo("Plan blocked")

        # Wait for 5 seconds but continue if the path is free
        wait_start = rospy.Time.now()

        while (rospy.Time.now() - wait_start) < rospy.Duration(3.0) and not rospy.is_shutdown():
            rospy.sleep(0.5)

            # Look at the entity
            # ps = msgs.PointStamped(point=self.robot.base.local_planner.getObstaclePoint(), frame_id="/map")
            # self.robot.head.look_at_point(kdl_vector_stamped_from_point_stamped_msg(ps))

            if not self.robot.base.local_planner.getStatus() == "blocked":
                self.robot.head.cancel_goal()
                rospy.loginfo("Plan free again")
                return "free"

        # Else: replan with same constraints
        # Get alternative plan
        pc = self.robot.base.global_planner.getCurrentPositionConstraint()
        oc = self.robot.base.local_planner.getCurrentOrientationConstraint()

        if pc and oc:
            plan = self.robot.base.global_planner.getPlan(pc)

            if plan and len(plan) > 0:
                self.robot.base.local_planner.setPlan(plan, pc, oc)

                # Give local planner time to get started; this shouldn't be neccesary
                rospy.sleep(0.5)

                return 'free'

        return 'blocked'


class NavigateTo(smach.StateMachine):
    def __init__(self, robot, reset_head=True, speak=True, input_keys=[], output_keys=[]):
        smach.StateMachine.__init__(self, outcomes=['arrived', 'unreachable', 'goal_not_defined'],
                                    input_keys=input_keys, output_keys=output_keys)
        self.robot = robot
        self.speak = speak

        with self:

            # Create the sub SMACH state machine
            sm_nav = smach.StateMachine(outcomes=['arrived', 'unreachable', 'goal_not_defined', 'preempted'])

            with sm_nav:

                smach.StateMachine.add('GET_PLAN',                          getPlan(self.robot, self.generateConstraint, self.speak),
                    transitions={'unreachable'                          :   'unreachable',
                                 'goal_not_defined'                     :   'goal_not_defined',
                                 'goal_ok'                              :   'EXECUTE_PLAN'})

                smach.StateMachine.add('EXECUTE_PLAN',                      executePlan(self.robot, self.breakOut, reset_head=reset_head),
                    transitions={'succeeded'                            :   'arrived',
                                 'arrived'                              :   'GET_PLAN',
                                 'blocked'                              :   'PLAN_BLOCKED',
                                 'preempted'                            :   'preempted'})

                smach.StateMachine.add('PLAN_BLOCKED',                      planBlocked(self.robot),
                    transitions={'blocked'                              :   'GET_PLAN',
                                 'free'                                 :   'EXECUTE_PLAN'})

            smach.StateMachine.add('START_ANALYSIS', StartAnalyzer(self.robot),
                transitions={'done'                                 :'NAVIGATE'})


            smach.StateMachine.add('NAVIGATE', sm_nav,
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
        raise NotImplementedError("generateConstraint must be implemented by subclasses of NavigateTo")

    def breakOut(self):
        """
        Default breakout function: makes sure things go to 'succeeded' if robot arrives at goal
        DO NOT OVERLOAD THIS IF NOT NECESSARY

        Possible outcomes (when overloading)
        - 'breakout': when a condition has been met and navigation should stop because the goal has succeeded
        - 'checking': the condition has not been met. Upon arrival at a goal, the statemachine will return to 'GetPlan' to get the next goal_not_defined
        - 'passed'  : checking a condition is not necessary. Upon arrival at the current goal, the state machine will return 'succeeded'
        """

        return 'passed'

        # status = self.robot.base.local_planner.getStatus()
        # goal_handle = self.robot.base.local_planner.getGoalHandle()

        # # if hasattr(self, 'breakout_goal_handle'):
        # #     rospy.logwarn("Status = {0}, goal_handle = {1}, stored = {2}".format(status, goal_handle, self.breakout_goal_handle))
        # # else:
        # #     rospy.logwarn("Status = {0}, goal_handle = {1}".format(status, goal_handle))

        # if status == "arrived":

        #     if not hasattr(self, 'breakout_goal_handle'):

        #         # If no breakout goal handle exists (hasn't arrived yet), make one and return breakout
        #         rospy.logwarn("Breaking out for the first time")
        #         self.breakout_goal_handle = goal_handle
        #         return True


        #     elif self.breakout_goal_handle != goal_handle:

        #         # If the existing goal handle is unequal to the current one, the local planner has arrived at a new goal
        #         # Hence, store this one and return true
        #         rospy.logwarn("Breaking out!!")
        #         self.breakout_goal_handle = goal_handle
        #         return True

        #     else:

        #         # Breakout function has already returned true on this goalhandle
        #         rospy.logerr("Executing should never be here. If this is the case, this function can be made a lot simpler!")
        #         return False

        # return False


class ForceDrive(smach.State):
    """ Force drives... """
    def __init__(self, robot, vx, vy, vth, duration):
        """
        Constructor

        :param robot: robot object
        :param vx: velocity in x-direction (m/s)
        :param vy: velocity in y-direction (m/s)
        :param vth: yaw-velocity (rad/s)
        :param duration: float indicating how long to drive (seconds)
        """
        smach.State.__init__(self, outcomes=['done'])
        self._robot = robot
        self._vx = vx
        self._vy = vy
        self._vth = vth
        self._duration = duration

    def execute(self, userdata=None):
        """ Executes the state """
        self._robot.base.force_drive(self._vx, self._vy, self._vth, self._duration)
        return 'done'
