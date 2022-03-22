# System
import collections
import math
import typing

# ROS
import actionlib
import geometry_msgs.msg as geom_msgs
import PyKDL as kdl
import rospy
import smach
# noinspection PyUnresolvedReferences
import tf2_geometry_msgs
import tf2_ros

# TU/e Robotics
import pykdl_ros
# noinspection PyUnresolvedReferences
import tf2_pykdl_ros
from cb_base_navigation_msgs.msg import PositionConstraint, OrientationConstraint
from people_recognition_msgs.msg import TrackOperatorAction, TrackOperatorFeedback, TrackOperatorGoal
from robot_skills.robot import Robot


def planar_distance(p1: geom_msgs.PoseStamped, p2: geom_msgs.PoseStamped):
    """
    Compute the distance between two PoseStamped msgs. N.B.: the z coordinate is not taken into account!

    :param p1:
    :param p2:
    :return:
    """
    assert p1.header.frame_id == p2.header.frame_id, \
        f"Frame IDs of p1 ({p1.header.frame_id}) and p2 ({p2.header.frame_id}) are not equal"
    pos1 = p1.pose.position
    pos2 = p2.pose.position
    return math.hypot(pos2.x - pos1.x, pos2.y - pos1.y)


def trail_to_path(
    robot_pose: geom_msgs.PoseStamped,
    trail: typing.List[geom_msgs.PoseStamped],
    reference_operator_distance: float,
) -> typing.List[geom_msgs.PoseStamped]:
    """

    :param robot_pose: current robot pose
    :param trail: trail of breadcrumbs
    :param reference_operator_distance: distance the robot is supposed to keep to the operator
    :return: interpolated path
    """
    plan = [robot_pose] + [p for p in trail if planar_distance(trail[-1], p) > reference_operator_distance]
    return plan


def operator_moving(
    trail: typing.List[geom_msgs.PoseStamped],
    threshold: float = 0.4,
    timeout: float = 3.0,
) -> bool:
    """
    Checks if the operator is moving, i.e., if the breadcrumbs move

    :param trail:
    :param threshold:
    :param timeout
    :return:
    """
    operator_pose = trail[-1]
    for crumb in reversed(trail):
        if planar_distance(operator_pose, crumb) > threshold:
            return True
        elif (operator_pose.header.stamp.to_sec() - crumb.header.stamp.to_sec()) > timeout:
            break
    return False


class SelectOperator(smach.State):
    def __init__(self, robot: Robot):
        """
        Selects the operator to follow and starts the tracking action

        :param robot: robot API object
        """
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self._robot = robot
        # ToDo: move to robot_skills
        self._ac = actionlib.SimpleActionClient(f"/{robot.robot_name}/track_operator", TrackOperatorAction)
        self._ac_cb = None

    def execute(self, _):
        self._robot.speech.speak(
            "I am selecting the operator, still assuming he or she is right in front of me", block=False
        )
        robot_pose_stamped = self._robot.base.get_location()
        operator_pose = robot_pose_stamped.frame * kdl.Frame(kdl.Vector(1.0, 0.0, 0.0))
        operator_pose_stamped = pykdl_ros.FrameStamped(operator_pose, rospy.Time.now(), "map")
        goal = TrackOperatorGoal()
        goal.start_pose = tf2_ros.convert(operator_pose_stamped, geom_msgs.PoseStamped)
        self._ac.send_goal(goal=goal, feedback_cb=self._track_operator_cb)
        return "succeeded"

    def register_callback(self, callback: callable):
        """
        Registers a callback method that will be passed to the action client

        :param callback: callable
        """
        self._ac_cb = callback

    def _track_operator_cb(self, feedback_msg: TrackOperatorFeedback):
        """
        Default callback method. Simply echos the position. Will be overridden if another callback is registered

        :param feedback_msg: feedback_msg
        """
        if self._ac_cb is not None:
            self._ac_cb(feedback_msg.current_pose)
        else:
            rospy.loginfo(f"Received operator position feedback: {feedback_msg.current_pose.pose.position}")


class MotionState:
    def __init__(self, timeout):
        """
        Holds the 'state' of the motion with some convenience methods
        """
        self._timeout = timeout
        self._stamp = None

    @property
    def has_moved(self):
        return self._stamp is not None

    @property
    def is_moving(self):
        if self._stamp is None:
            return False
        return (rospy.Time.now() - self._stamp).to_sec() < self._timeout

    def tick(self):
        """
        Registers that the robot is actually moving
        """
        self._stamp = rospy.Time.now()


class OperatorState:
    NEW = "NEW"
    MOVING = "MOVING"
    STOPPED = "STOPPED"
    LOST = "LOST"

    def __init__(self, lost_timeout):
        """
        Holds the breadcrumb trail along with some convenience functions
        """
        self._trail = []
        self._has_moved = False

    def get_state(self, stamp):
        if self._state == self.NEW:
            return self.NEW
        # If latest stamp in trail too old: operator is lost
        # elif
        # If crumbs are at the same position (within threshold) for a certain duration, operator not moving
        # elif
        # Else: we're moving!
        return self.MOVING

    @property
    def trail(self):
        # ToDo: need copy? Less efficient, less error-prone
        return self._trail

    def process_queue(self, queue):
        """
        Processes the queue that is passed to this function and adds the entries to the internal trail
        :param queue:
        :return:
        """
        while True:
            try:
                self._trail.append(queue.popleft())
                self._has_moved = True
            except IndexError:
                break

    def prune_trail(self, robot_pose: geom_msgs.PoseStamped):
        """
        Prunes the trail based on the robot pose

        :param robot_pose:
        :return:
        """
        min_idx = 0
        min_dist = float('inf')
        for idx, crumb in enumerate(self._trail):
            dist = planar_distance(robot_pose, crumb)
            if dist < min_dist:
                min_idx = idx
                min_dist = dist
        self._trail = self._trail[min_idx:]


class FollowBreadcrumb(smach.State):
    def __init__(self, robot: Robot):
        """
        Smach state to follow a breadcrumb provided by the input queue(

        :param robot:
        """
        smach.State.__init__(self, outcomes=["within_reach", "lost"])
        # Props
        self._robot = robot
        self._reference_operator_distance = 0.8  # Robot tries to get within this distance of the operator
        self._motion_timeout = 5.0  # If the robot has not moved for this time, it'll stop

        # State
        self._buffer = collections.deque()  # Use a deque for thread safety
        self._motion_state = MotionState(self._motion_timeout)

    def execute(self, _):
        self._buffer.clear()  # Clear the buffer to make sure we don't have any old data
        self._robot.speech.speak("I am trying to follow the breadcrumb", block=False)
        r = rospy.Rate(1.0)
        # trail = []
        operator_state = OperatorState()
        self._motion_state = MotionState(self._motion_timeout)

        while self.is_following(operator_state.trail):
            robot_pose_kdl = self._robot.base.get_location()
            robot_pose = tf2_ros.convert(robot_pose_kdl, geom_msgs.PoseStamped)
            operator_state.process_queue(self._buffer)
            operator_state.prune_trail(robot_pose)
            if operator_state.trail:
                self._send_base_goal(operator_state.trail)
            # Check if we still receive breadcrumbs
            # ToDo
            # ToDo: make a clear distinction between receiving operator behavior (moving, stationary, lost)
            # and robot behavior
            r.sleep()

        self._robot.base.local_planner.cancelCurrentPlan()
        return "within_reach"

    def is_following(self, trail: typing.List[geom_msgs.PoseStamped]) -> bool:
        """
        Checks if the robot is still actively following

        :param trail:
        :return:
        """
        # Robot is standing still
        if self._robot.base.local_planner.getStatus() in ["controlling", "blocked"]:
            rospy.loginfo("Local planner still active")
            self._motion_state.tick()
            return True

        # Check last motion
        if not self._motion_state.has_moved:
            rospy.loginfo("Robot has not moved yet")
            return True

        # if self._motion_state.is_moving:
        #     rospy.loginfo("Robot is still moving")
        #     return True

        # Operator within reach
        robot_pose_kdl = self._robot.base.get_location()
        robot_pose = tf2_ros.convert(robot_pose_kdl, geom_msgs.PoseStamped)
        operator_distance = planar_distance(robot_pose, trail[-1])
        if operator_distance > (self._reference_operator_distance + 0.3):
            rospy.loginfo(f"Operator too far away at {operator_distance}")
            return True

        # Operator standing still for a certain period?
        if operator_moving(trail):
            rospy.loginfo("Operator still moving")
            return True

        rospy.loginfo("Seems I'm not following anymore")
        return False

    def _process_queue(self, trail: typing.List[geom_msgs.PoseStamped]) -> typing.List[geom_msgs.PoseStamped]:
        """
        Adds the poses in the buffer to the trail

        :param trail: current trail to which the crumbs will be added
        :return: updated trail
        """
        while True:
            try:
                trail.append(self._buffer.popleft())
            except IndexError:
                break
        return trail

    def _prune_trail(self, trail: typing.List[geom_msgs.PoseStamped]) -> typing.List[geom_msgs.PoseStamped]:
        """
        Removes crumbs from the trail where the robot has already been. This is relevant because the robot pose will
        be pre-pended to the plan so that the local planner can deal with it.

        :param trail: current trail
        :return: pruned trail
        """
        # ToDo: make a decent implementation. This assumes the local planner is smart enough
        robot_pose_kdl = self._robot.base.get_location()
        robot_pose = tf2_ros.convert(robot_pose_kdl, geom_msgs.PoseStamped)
        min_idx = 0
        min_dist = float('inf')
        for idx, crumb in enumerate(trail):
            dist = planar_distance(robot_pose, crumb)
            if dist < min_dist:
                min_idx = idx
                min_dist = dist
        return trail[min_idx:]

    def _send_base_goal(self, trail: typing.List[geom_msgs.PoseStamped]):
        """
        Sends a goal to the base local planner

        :param trail: trail that the operator left
        """
        robot_pose_kdl = self._robot.base.get_location()
        robot_pose = tf2_ros.convert(robot_pose_kdl, geom_msgs.PoseStamped)
        plan = trail_to_path(robot_pose, trail, self._reference_operator_distance)
        if len(plan) <= 1:  # Plan always contains the robot pose so should be > 1 to continue
            rospy.loginfo("Plan is still empty")
            return
        operator_pos = trail[-1].pose.position
        pc = PositionConstraint()
        pc.constraint = f"(x-{operator_pos.x})^2 + (y-{operator_pos.y})^2 < {self._reference_operator_distance + 0.1}^2"
        oc = OrientationConstraint()
        oc.frame = "map"
        oc.look_at.x = operator_pos.x
        oc.look_at.y = operator_pos.y
        self._robot.base.local_planner.setPlan(plan, pc, oc)

    def add_breadcrumb(self, operator_pose: geom_msgs.PoseStamped):
        """
        Adds a breadcrumb. Typically used as a callback for the tracker.

        :param operator_pose: pose of the operator to follow
        """
        rospy.loginfo(f"FollowBreadCrumb: Received operator position: {operator_pose.pose.position}")
        self._buffer.append(operator_pose)


class WithinReach(smach.State):
    def __init__(self, robot: Robot):
        """
        Smach state that is executed when the operator is within reach of the robot.
        Here, it is decided whether we keep tracking or are done.

        :param robot: robot API object
        """
        smach.State.__init__(self, outcomes=["continue", "done"])
        self._robot = robot

    def execute(self, _):
        self._robot.speech.speak("I am done here")
        return "done"


class Recover(smach.State):
    def __init__(self, robot: Robot):
        """
        Smach state that is executed when the operator is lost. Here, it is decided whether
        to start again or to go to failed

        :param robot: robot API object
        """
        smach.State.__init__(self, outcomes=["found", "lost"])
        self._robot = robot

    def execute(self, _):
        self._robot.speech.speak("I have lost my buddy")
        return "lost"


@smach.cb_interface(
    outcomes=["done", "lost"]
)
def cleanup(_: smach.UserData, robot: Robot, result: str) -> str:
    """
    Preempts the action

    :param _: Smach userdata
    :param robot: robot API object
    :param result: result to return. Passing this in eases reuse
    :return: result
    """
    robot.speech.speak("Now I should cancel the tracking")
    return result


class FollowOperator(smach.StateMachine):
    def __init__(self, robot: Robot):
        """
        Class to follow an operator

        :param robot: robot API object
        """
        smach.StateMachine.__init__(self, outcomes=["done", "lost"])

        select_operator = SelectOperator(robot)
        follow_breadcrumb = FollowBreadcrumb(robot)
        select_operator.register_callback(follow_breadcrumb.add_breadcrumb)

        with self:
            smach.StateMachine.add(
                "SELECT_OPERATOR", select_operator, transitions={
                    "succeeded": "FOLLOW",
                    "failed": "lost",
                }
            )

            smach.StateMachine.add(
                "FOLLOW", follow_breadcrumb, transitions={
                    "within_reach": "WITHIN_REACH",
                    "lost": "RECOVER",
                }
            )

            smach.StateMachine.add(
                "WITHIN_REACH", WithinReach(robot), transitions={
                    "continue": "FOLLOW",
                    "done": "CLEANUP_DONE",
                }
            )

            smach.StateMachine.add(
                "RECOVER", Recover(robot), transitions={
                    "found": "FOLLOW",
                    "lost": "CLEANUP_LOST",
                }
            )

            smach.StateMachine.add(
                "CLEANUP_DONE", smach.CBState(cleanup, cb_kwargs={"robot": robot, "result": "done"}), transitions={
                    "done": "done",
                    "lost": "lost",
                }
            )

            smach.StateMachine.add(
                "CLEANUP_LOST", smach.CBState(cleanup, cb_kwargs={"robot": robot, "result": "lost"}), transitions={
                    "done": "done",
                    "lost": "lost",
                }
            )
