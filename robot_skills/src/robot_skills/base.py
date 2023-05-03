#
#  Rein Appeldoorn
#  October '14
#

from typing import List, Optional

# System
import math

# ROS
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Twist
import PyKDL as kdl
from pykdl_ros import FrameStamped
import rospy
import tf2_ros
# noinspection PyUnresolvedReferences
import tf2_geometry_msgs
# noinspection PyUnresolvedReferences
import tf2_pykdl_ros


# TU/e Robotics
from cb_base_navigation_msgs.msg import LocalPlannerAction, LocalPlannerGoal, OrientationConstraint, PositionConstraint
from cb_base_navigation_msgs.srv import CheckPlan, GetPlan
from numpy import sign

from robot_skills.robot_part import RobotPart
from robot_skills.util import nav_analyzer, transformations


def _abs_max(value, max_value):
    """
    Robust max function, which ignores the sign of the max_value. And can handle inf/NaN as max_value.

    >>> _abs_max(1.0, 2.0)
    1.0
    >>> _abs_max(1.0, -2.0)
    1.0
    >>> _abs_max(1.0, 1.0)
    1.0
    >>> _abs_max(1.0, -1.0)
    1.0
    >>> _abs_max(1.0, 0.5)
    0.5
    >>> _abs_max(1.0, -0.5)
    0.5
    >>> _abs_max(1.0, 0.0)
    0.0
    >>> _abs_max(1.0, -0.0)
    0.0
    >>> _abs_max(-1.0, 2.0)
    -1.0
    >>> _abs_max(-1.0, -2.0)
    -1.0
    >>> _abs_max(-1.0, 1.0)
    -1.0
    >>> _abs_max(-1.0, -1.0)
    -1.0
    >>> _abs_max(-1.0, 0.5)
    -0.5
    >>> _abs_max(-1.0, -0.5)
    -0.5
    >>> _abs_max(-1.0, 0.0)
    -0.0
    >>> _abs_max(-1.0, -0.0)
    -0.0

    >>> _abs_max(0.0, 1.0)
    0.0
    >>> _abs_max(0.0, -1.0)
    0.0
    >>> _abs_max(0.0, 0.0)
    0.0
    >>> _abs_max(0.0, -0.0)
    0.0

    >>> _abs_max(-0.0, 1.0)
    0.0
    >>> _abs_max(-0.0, -1.0)
    0.0
    >>> _abs_max(-0.0, 0.0)
    0.0
    >>> _abs_max(-0.0, -0.0)
    0.0

    >>> _abs_max(1.0, float("inf"))
    1.0
    >>> _abs_max(-1.0, float("inf"))
    -1.0
    >>> _abs_max(1.0, float("-inf"))
    1.0
    >>> _abs_max(-1.0, float("-inf"))
    -1.0
    >>> _abs_max(1.0, float("NaN"))
    1.0
    >>> _abs_max(-1.0, float("NaN"))
    -1.0
    >>> _abs_max(1.0, float("-NaN"))
    1.0
    >>> _abs_max(-1.0, float("-NaN"))
    -1.0
    """
    # max_value should be 2nd arg, so in case of inf/NaN `min` returns `value`
    return sign(value) * min(abs(value), abs(max_value))


class LocalPlanner(RobotPart):
    def __init__(self, robot_name, tf_buffer, analyzer):
        super(LocalPlanner, self).__init__(robot_name=robot_name, tf_buffer=tf_buffer)
        self.analyzer = analyzer
        self._action_client = self.create_simple_action_client('/' + robot_name + '/local_planner/action_server',
                                                               LocalPlannerAction)

        self._position_constraint = None
        self._orientation_constraint = None

        # Public members!
        self._status = "idle"  # idle, controlling, blocked, arrived
        self._obstacle_point = None
        self._dtg = None
        self._plan = None
        self._goal_handle = None

    def setPlan(self, plan, position_constraint, orientation_constraint):
        goal = LocalPlannerGoal()
        goal.plan = plan
        goal.orientation_constraint = orientation_constraint
        self._orientation_constraint = orientation_constraint
        self._action_client.send_goal(goal, done_cb=self.__doneCallback, feedback_cb=self.__feedbackCallback)
        self._goal_handle = self._action_client.gh
        rospy.logdebug("Goal handle = {0}".format(self._goal_handle))
        self.__setState("controlling", None, None, plan)

    def cancelCurrentPlan(self):
        if self._goal_handle:
            state = self._action_client.get_state()
            # Only cancel goal when pending or active
            if state == GoalStatus.PENDING or state == GoalStatus.ACTIVE:
                self._action_client.cancel_goal()
                self.__setState("idle")

    def selfreset(self):
        self._action_client.cancel_all_goals()
        # cancel_all_goals doesn't return anything useful to check, so we just return True
        # https://docs.ros.org/kinetic/api/actionlib/html/simple__action__client_8py_source.html#l00192
        return True

    def getGoalHandle(self):
        return self._goal_handle

    def getStatus(self):
        return self._status

    def getDistanceToGoal(self):
        return self._dtg

    def getObstaclePoint(self):
        return self._obstacle_point

    def getCurrentOrientationConstraint(self):
        return self._orientation_constraint

    def __feedbackCallback(self, feedback):
        if feedback.blocked:
            self.__setState("blocked", feedback.point_blocked, feedback.dtg, self._plan)
        else:
            self.__setState("controlling", None, feedback.dtg, self._plan)

    def __doneCallback(self, terminal_state, result):
        self.__setState("arrived")

    def __setState(self, status, obstacle_point=None, dtg=None, plan=None):
        self._status = status
        self._obstacle_point = obstacle_point
        self._dtg = dtg
        self._plan = plan


class GlobalPlanner(RobotPart):
    def __init__(self, robot_name, tf_buffer, analyzer):
        super(GlobalPlanner, self).__init__(robot_name=robot_name, tf_buffer=tf_buffer)
        self.analyzer = analyzer
        self._position_constraint = None
        self._get_plan_client = self.create_service_client("/" + robot_name + "/global_planner/get_plan_srv", GetPlan)
        self._check_plan_client = self.create_service_client("/" + robot_name + "/global_planner/check_plan_srv", CheckPlan)

    def getPlan(
        self, position_constraint: PositionConstraint, start_pose: Optional[FrameStamped] = None
    ) -> List[PoseStamped]:
        """
        Get a global plan from start(optional) to a goal constrained by position_constraint

        :param position_constraint: Goal position constraints
        :param start_pose: optional start pose. If this is not provided, the current position is used.
        :return: If No path was found, this list is empty. If the planner service fails,
            'None' is returned.
        """

        self._position_constraint = position_constraint

        if start_pose is None:
            start_pose = get_location(self.robot_name, self.tf_buffer)

        pcs = [position_constraint]

        start_time = rospy.Time.now()

        start_pose_msg = tf2_ros.convert(start_pose, PoseStamped)

        try:
            resp = self._get_plan_client(start_pose_msg, pcs)
        except Exception as e:
            rospy.logerr("Could not get plan from global planner via service call: {}".format(e))
            return None

        if not resp.succes:
            rospy.logerr("Global planner couldn't plan a path to the specified constraints. Are the constraints you specified valid?: {}".format(pcs))
            return None

        end_time = rospy.Time.now()
        plan_time = (end_time-start_time).to_sec()

        self.path_length = computePathLength(resp.plan)

        return resp.plan

    def checkPlan(self, plan):
        try:
            resp = self._check_plan_client(plan)
        except Exception as e:
            rospy.logerr("Could not check plan: {}".format(e))
            return False

        return resp.valid

    def getCurrentPositionConstraint(self):
        return self._position_constraint


class Base(RobotPart):
    def __init__(self, robot_name, tf_buffer, cmd_vel_topic=None, initial_pose_topic=None):
        super(Base, self).__init__(robot_name=robot_name, tf_buffer=tf_buffer)
        if cmd_vel_topic is None:
            cmd_vel_topic = '/' + robot_name + '/base/references'
        if initial_pose_topic is None:
            initial_pose_topic = '/' + robot_name + '/initialpose'
        self._cmd_vel = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self._initial_pose_publisher = rospy.Publisher(initial_pose_topic, PoseWithCovarianceStamped, queue_size=10)

        self.analyzer = nav_analyzer.NavAnalyzer(robot_name)

        # The planners
        self.global_planner = GlobalPlanner(robot_name, tf_buffer, self.analyzer)
        self.local_planner = LocalPlanner(robot_name, tf_buffer, self.analyzer)

        self.subscribe_hardware_status('base')

    def wait_for_connections(self, timeout, log_failing_connections=True):
        """ Waits for the connections until they are connected

        :param timeout: timeout in seconds
        :param log_failing_connections: (bool) whether to log errors if not connected. This is useful when checking
            multiple robot parts in a loop
        :return: bool indicating whether all connections are connected
        """
        return self.global_planner.wait_for_connections(
            timeout=timeout, log_failing_connections=log_failing_connections) and \
            self.local_planner.wait_for_connections(timeout=timeout, log_failing_connections=log_failing_connections)

    def move(self, position_constraint_string, frame):
        p = PositionConstraint()
        p.constraint = position_constraint_string
        p.frame = frame
        plan = self.global_planner.getPlan(p)
        if plan:
            o = OrientationConstraint()
            o.frame = frame
            self.local_planner.setPlan(plan, p, o)

        return plan

    def turn_towards(self, x, y, frame, offset=0.0):
        current_pose = self.get_location()
        p = PositionConstraint()
        p.constraint = "(x-%f)^2+(y-%f)^2 < %f^2" % (current_pose.frame.p.x(), current_pose.frame.p.y(), 0.1)
        p.frame = current_pose.header.frame_id
        plan = self.global_planner.getPlan(p)
        o = OrientationConstraint(look_at=Point(x, y, 0.0), angle_offset=offset)
        o.frame = frame
        self.local_planner.setPlan(plan, p, o)

        return plan

    def force_drive(self, vx, vy, vth, timeout, loop_rate=10, stop=True, ax=float('inf'), ay=float('inf'), ath=float('inf')):
        """ Forces the robot to drive by sending a command velocity directly to the base controller. N.B.: all collision
        avoidance is bypassed.

        :param vx: forward velocity [m/s]
        :param vy: sideways velocity [m/s]
        :param vth: rotational velocity in [rad/s]
        :param timeout: duration for this motion [s]
        :param loop_rate: Rate of sending twist messages [Hz]
        :param stop: send stop message afterwards
        :param ax: forward acceleration [m/s^2]
        :param ay: sideways acceleration [m/s^2]
        :param ath: rotational acceleration [rad/s^2]
        """
        # Cancel the local planner goal
        self.local_planner.cancelCurrentPlan()

        v = Twist()  # Initialize velocity
        t_start = rospy.Time.now()
        t_end = t_start + rospy.Duration.from_sec(timeout)

        # Drive
        rate = rospy.Rate(loop_rate)
        while rospy.Time.now() < t_end:
            seconds_from_start = rospy.Time.now().to_sec() - t_start.to_sec()

            # 0 * inf =  NaN. This can be the case when 'seconds_from_start' is close to zero.
            v.linear.x = _abs_max(vx, ax * seconds_from_start)
            v.linear.y = _abs_max(vy, ay * seconds_from_start)
            v.angular.z = _abs_max(vth, ath * seconds_from_start)
            self._cmd_vel.publish(v)
            rate.sleep()

        if stop:
            # Stop driving
            v.linear.x = 0.0
            v.linear.y = 0.0
            v.angular.z = 0.0
            self._cmd_vel.publish(v)

        return True

    def get_location(self) -> FrameStamped:
        """ Returns a FrameStamped with the robot pose

        :return: FrameStamped with robot pose
        """
        return get_location(self.robot_name, self.tf_buffer)

    def set_initial_pose(self, x, y, phi):

        initial_pose = PoseWithCovarianceStamped()

        initial_pose.header.frame_id = "map"

        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation = transformations.euler_z_to_quaternion(phi)
        initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.06853891945200942, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        #rospy.loginfo("initalpose = {0}".format(initial_pose))

        # We have to do this twice for some reason, somewhere in ed_localization
        self._initial_pose_publisher.publish(initial_pose)
        rospy.sleep(0.5)
        self._initial_pose_publisher.publish(initial_pose)

        return True

    def wait_for_motion_done(self, timeout=10.0, cancel=False):
        """
        Waits until local planner is done

        :param timeout: timeout in seconds; in case 0.0, no sensible output is provided, just False
        :param cancel: bool specifying whether goals should be cancelled
            if timeout is exceeded
        :return: bool indicates whether motion was done (True if reached, False otherwise)
        """
        #TODO implement navigation using an actionserver
        starttime = rospy.Time.now()
        r = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():
            if self.local_planner.getStatus() == "arrived":
                return True
            passed_time = (rospy.Time.now() - starttime).to_sec()
            if passed_time > timeout:
                if cancel:
                    self.local_planner.cancelCurrentPlan()
                return False
            r.sleep()
        return False

    def selfreset(self):
        loc = self.local_planner.reset()
        glob = self.global_planner.reset()
        return loc and glob

    ########################################################
    ###### Are the following functions deprecated ??? ######
    ########################################################
    def go(self, x, y, phi, frame="map", timeout=0):
        rospy.logwarn("[base.py] Function 'go' of 'Base' is obsolete.")
        return True

    def reset_costmap(self):
        rospy.logwarn("[base.py] Function 'reset_costmap' of 'Base' is obsolete.")
        return True

    def cancel_goal(self):
        rospy.logwarn("[base.py] Function 'cancel_goal' of 'Base' is obsolete.")
        return True

    ########################################################
    ########################################################
    ########################################################


def get_location(robot_name, tf_buffer) -> FrameStamped:

    time = rospy.Time()
    target_frame_stamped = FrameStamped(kdl.Frame(), time, f"{robot_name}/base_link")
    try:
        target_frame_stamped = tf_buffer.transform(target_frame_stamped, "map", timeout=rospy.Duration(20))
        return target_frame_stamped

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
        rospy.logerr("tf2 request failed!, {}".format(e))
        return FrameStamped(kdl.Frame(), time, "map")


def computePathLength(path):
    distance = 0.0
    for index, pose in enumerate(path):
        if not index == 0:
            dx = path[index].pose.position.x - path[index-1].pose.position.x
            dy = path[index].pose.position.y - path[index-1].pose.position.y
            distance += math.sqrt(dx*dx + dy*dy)
    return distance
