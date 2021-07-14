from __future__ import absolute_import

# System
from random import choice

# ROS
import rospy
import smach
import math
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist


class StartAnalyzer(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot

    def execute(self, userdata=None):
        self.robot.base.analyzer.start_measurement(self.robot.base.get_location().frame)
        return 'done'


class LethalPlanner(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['in_free_space', 'in_lethal_zone'])
        self.robot = robot
        self.front_bumper_sub = rospy.Subscriber("hero/base_f_bumper_sensor", Bool, self.front_callback)
        self.front_bumper_active = False
        self.back_bumper_sub = rospy.Subscriber("hero/base_b_bumper_sensor", Bool, self.back_callback)
        self.back_bumper_active = False
        self.global_costmap_sub = rospy.Subscriber("/hero/global_planner/global_costmap/costmap", OccupancyGrid,
                                                   self.global_costmap_callback)
        self._cmd_vel = rospy.Publisher("/hero/base/references", Twist)
        self.costmap_info = None
        self.costmap_data = None

    def front_callback(self, msg):
        self.front_bumper_active = msg.data

    def back_callback(self, msg):
        self.back_bumper_active = msg.data

    def get_costmap_at(self, x, y):
        return self.costmap_data[x + y * self.costmap_info.width]

    def global_costmap_callback(self, msg):
        self.costmap_info = msg.info
        self.costmap_data = msg.data

    def start_value(self, x, y):
        x_grid = math.floor((x - self.costmap_info.origin.position.x) / self.costmap_info.resolution)
        y_grid = math.floor((y - self.costmap_info.origin.position.y) / self.costmap_info.resolution)
        # Calculate the x and y grid coordinates by
        # first subtracting the x and y coordinates of the origin
        # from the x and y coordinates of HERO's start position.
        # Then that value is divided by the resolution of the costmap.
        # Lastly the value is rounded down to get an integer value of the grid coordinates.
        return int(x_grid), int(y_grid)

    def free_space_finder(self, x, y):
        search_range = int(round((0.24 + 0.2 - 0.5 * 0.05) / (self.costmap_info.resolution)))
        d_max_grid = (x + search_range) ** 2 + (y + search_range) ** 2
        x_free_grid = None
        y_free_grid = None
        for i in range(x - search_range, x + search_range + 1):
            for j in range(y - search_range, y + search_range + 1):
                if self.get_costmap_at(i, j) == 0:
                    # We are looking for an free space which corresponds with a value of 0
                    # A value of 65 corresponds with the border of the lethal zone
                    # A value of 99 corresponds with cells in the lethal zone
                    # A value of 100 corresponds with a cell that contains a object
                    d = (i - x) ** 2 + (j - y) ** 2
                    # Distance between the free grid cell and HERO's is calculated
                    if d < d_max_grid:
                        # If the calculated distance is smaller than the previously
                        # calculated distance the grid coordinates are saved
                        d_max_grid = d
                        x_free_grid = i
                        y_free_grid = j
        return x_free_grid, y_free_grid, d_max_grid

    def execute(self, userdata=None):
        while self.costmap_data is None:
            rospy.sleep(0.1)
        robot_frame = self.robot.base.get_location()
        x = robot_frame.frame.p.x()
        y = robot_frame.frame.p.y()
        # Get coordinates from HERO's current location
        rospy.loginfo("x: {}, y: {}".format(x, y))
        x_grid, y_grid = self.start_value(x, y)
        # Grid coordinates of HERO's start position
        rospy.loginfo("x_grid: {}, y_grid: {}".format(x_grid, y_grid))
        i_data = x_grid + y_grid * self.costmap_info.width
        rospy.loginfo("i_data: {}".format(self.costmap_data[i_data]))
        if self.costmap_data[i_data] > 98:

            x_free_grid, y_free_grid, d_max_grid = self.free_space_finder(x_grid, y_grid)
            rospy.loginfo("grid: x_free: {}, y_free: {}, at d {}".format(x_free_grid, y_free_grid, d_max_grid))
            # Get grid coordinates of the free space from the free_space_finder function
            if x_free_grid is not None:

                x_free = (x_free_grid * self.costmap_info.resolution) + self.costmap_info.origin.position.x
                y_free = (y_free_grid * self.costmap_info.resolution) + self.costmap_info.origin.position.y
                # Convert grid coordinates to regular coordinates

                d_max = math.sqrt(d_max_grid) * self.costmap_info.resolution
                # Convert the value of d_max_grid to the actual distance to the free space in meters

                _, _, theta_h = robot_frame.frame.M.GetRPY()
                # Get rotation of HERO with respect to the world coordinate system

                vx = (math.cos(theta_h) * (x_free - x) + math.sin(theta_h) * (y_free - y)) * 0.1 / d_max
                vy = (math.cos(theta_h) * (y_free - y) - math.sin(theta_h) * (x_free - x)) * 0.1 / d_max
                vth = 0
                # Calculate the velocities in the x and y with respect to HERO's coordinate system
                duration = d_max / 0.1
                rospy.loginfo("vx: {}, vy {}, time{}".format(vx, vy, duration))

                # start driving
                # Cancel the local planner goal
                self.robot.base.local_planner.cancelCurrentPlan()

                v = Twist()  # Initialize velocity
                t_start = rospy.Time.now()
                t_end = t_start + rospy.Duration.from_sec(duration)

                # Define loop parameters
                loop_rate = 30  # publish rate for velocities
                rate = rospy.Rate(loop_rate)

                x_current = x
                y_current = y
                theta_current = theta_h
                xi_current = x_grid
                yi_current = y_grid

                # Loop while publishing velocities.
                # in this example loop will continue driving until we have gone past a certain time. If you want to
                #change that you can change the condition in the while statement: e.g. while self.costmap[xi_current, yi_current] will wait until the robot is at a free space.

                while rospy.Time.now() < t_end:  # e.g. while self.get_costmap_at(xi_current, yi_current):
                    # retrieve robot position
                    robot_frame_current = self.robot.base.get_location()
                    x_current = robot_frame_current.frame.p.x() # position in [m] in map frame
                    y_current = robot_frame_current.frame.p.y() # position in [m] in map frame
                    _, _, theta_current = robot_frame_current.frame.M.GetRPY()
                    xi_current = round((x_current * self.costmap_info.resolution) + self.costmap_info.origin.position.x) # position in grids in gridmap frame
                    yi_current = round((y_current * self.costmap_info.resolution) + self.costmap_info.origin.position.y) # position in grids in gridmap frame

                    v.linear.x = vx
                    v.linear.y = vy
                    v.angular.z = vth
                    self._cmd_vel.publish(v)
                    rate.sleep()

                # driving done
                return 'in_free_space'
            else:

                rospy.loginfo("No free space found")
                return 'in_lethal_zone'
        else:

            rospy.loginfo("HERO already in free space")
            return 'in_free_space'


class StopAnalyzer(smach.State):
    def __init__(self, robot, result):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot
        self.result = result

    def execute(self, userdata=None):
        self.robot.base.analyzer.stop_measurement(self.robot.base.get_location().frame, self.result)
        return 'done'


class AbortAnalyzer(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])
        self.robot  = robot

    def execute(self, userdata=None):
        self.robot.base.analyzer.abort_measurement()
        return 'done'


class getPlan(smach.State):
    def __init__(self, robot, constraint_function, speak=True):
        smach.State.__init__(self, outcomes=['unreachable', 'goal_not_defined', 'goal_ok', 'preempted'])
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
    def __init__(self, robot, breakout_function, blocked_timeout=4, reset_head=True, reset_pose=True):
        smach.State.__init__(self, outcomes=['succeeded', 'arrived', 'blocked', 'preempted'])
        self.robot = robot
        self.t_last_free = None
        self.blocked_timeout = blocked_timeout
        self.breakout_function = breakout_function
        self.reset_head = reset_head
        self.reset_pose = reset_pose

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

        # Move the robot to a suitable driving pose
        if self.reset_pose and self.robot.base.global_planner.path_length > 0.5:
            self.robot.go_to_driving_pose()

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
        smach.State.__init__(self, outcomes=['blocked', 'free'])
        self.robot = robot

    def execute(self, userdata=None):

        rospy.loginfo("Plan blocked")

        # Wait for 5 seconds but continue if the path is free
        wait_start = rospy.Time.now()

        while (rospy.Time.now() - wait_start) < rospy.Duration(3.0) and not rospy.is_shutdown():
            rospy.sleep(0.5)

            # Look at the entity
            # ps = msgs.PointStamped(point=self.robot.base.local_planner.getObstaclePoint(), frame_id="map")
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
    """
    Move the robot to a specified location.

    :param robot: Robot object
    :param constraint_function: function resolving to a tuple(PositionConstraint, OrientationConstraint)
        telling the robot where to drive to.
    :param reset_head: Whether or not the head should be used for obstacle avoidance during navigation.
    :param speak: Whether or not the robot should speak during navigation
    :param reset_pose: Whether or not the robot is allowed to change its pose for navigation.
    """
    def __init__(self, robot, constraint_function, reset_head=True, speak=True, reset_pose=True):
        smach.StateMachine.__init__(self, outcomes=['arrived', 'unreachable', 'goal_not_defined'])
        self.robot = robot
        self.speak = speak

        with self:

            # Create the sub SMACH state machine
            sm_nav = smach.StateMachine(outcomes=['arrived', 'unreachable', 'goal_not_defined', 'preempted'])

            with sm_nav:

                smach.StateMachine.add('LETHAL_ROUTE_PLANNING_ACTIVATE', LethalPlanner(self.robot),
                                       transitions={'in_free_space': 'GET_PLAN',
                                                    'in_lethal_zone': 'unreachable'})

                smach.StateMachine.add('GET_PLAN', getPlan(self.robot, constraint_function, self.speak),
                                       transitions={'unreachable': 'unreachable',
                                                    'goal_not_defined': 'goal_not_defined',
                                                    'goal_ok': 'EXECUTE_PLAN'})

                smach.StateMachine.add('EXECUTE_PLAN', executePlan(self.robot, self.breakOut,
                                                                   reset_head=reset_head, reset_pose=reset_pose),
                                       transitions={'succeeded': 'arrived',
                                                    'arrived': 'GET_PLAN',
                                                    'blocked': 'PLAN_BLOCKED',
                                                    'preempted': 'preempted'})

                smach.StateMachine.add('PLAN_BLOCKED', planBlocked(self.robot),
                                       transitions={'blocked': 'GET_PLAN',
                                                    'free': 'EXECUTE_PLAN'})

            smach.StateMachine.add('START_ANALYSIS', StartAnalyzer(self.robot),
                                   transitions={'done': 'NAVIGATE'})

            smach.StateMachine.add('NAVIGATE', sm_nav,
                                   transitions={'arrived': 'STOP_ANALYSIS_SUCCEED',
                                                'unreachable': 'STOP_ANALYSIS_UNREACHABLE',
                                                'goal_not_defined': 'ABORT_ANALYSIS_NOT_DEFINED',
                                                'preempted': 'STOP_ANALYSIS_SUCCEED'})

            smach.StateMachine.add('STOP_ANALYSIS_SUCCEED', StopAnalyzer(self.robot, 'succeeded'),
                                   transitions={'done': 'arrived'})

            smach.StateMachine.add('STOP_ANALYSIS_UNREACHABLE', StopAnalyzer(self.robot, 'unreachable'),
                                   transitions={'done': 'unreachable'})

            smach.StateMachine.add('ABORT_ANALYSIS_NOT_DEFINED', AbortAnalyzer(self.robot),
                                   transitions={'done': 'goal_not_defined'})

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
