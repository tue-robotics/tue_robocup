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


class FollowBreadcrumb(smach.State):
    def __init__(self, robot: Robot):
        """
        Smach state to follow a breadcrumb provided by the input queue(

        :param robot:
        """
        smach.State.__init__(self, outcomes=["within_reach", "lost"])
        self._robot = robot
        self._buffer = collections.deque()  # Use a deque for thread safety
        self._reference_operator_distance = 0.8  # Robot tries to get within this distance of the operator

    def execute(self, _):
        self._buffer.clear()  # Clear the buffer to make sure we don't have any old data
        self._robot.speech.speak("I am trying to follow the breadcrumb", block=False)
        start_stamp = rospy.Time.now()  # ToDo: remove after implementing is_following
        r = rospy.Rate(1.0)
        trail = []
        while self.is_following(start_stamp):
            trail = self._process_queue(trail=trail)
            trail = self._prune_trail(trail=trail)
            if trail:
                self._send_base_goal(trail)
            r.sleep()
        return "within_reach"

    @staticmethod
    def is_following(start_stamp):
        # ToDo: make decent implementation
        duration = (rospy.Time.now() - start_stamp).to_sec()
        if duration < 5.0:
            return True
        else:
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

    @staticmethod
    def _prune_trail(trail: typing.List[geom_msgs.PoseStamped]) -> typing.List[geom_msgs.PoseStamped]:
        """
        Removes crumbs from the trail where the robot has already been

        :param trail: current trail
        :return: pruned trail
        """
        # ToDo: make a decent implementation. This assumes the local planner is smart enough
        return trail

    def _send_base_goal(self, trail: typing.List[geom_msgs.PoseStamped]):
        """
        Sends a goal to the base local planner

        :param trail: trail that the operator left
        """
        plan = [p for p in trail if planar_distance(trail[-1], p) > self._reference_operator_distance]
        if len(plan) == 0:
            rospy.loginfo("Plan is still empty")
            return
        operator_pos = trail[-1].pose.position
        pc = PositionConstraint()
        pc.constraint = f"(x-{operator_pos.x})^2 + (y-{operator_pos.y})^2 < {self._reference_operator_distance + 0.1}^2"
        oc = OrientationConstraint()
        oc.frame = "map"  # ToDo: check!!!
        rospy.loginfo(f"Sending goal to planner: {plan}, {pc}, {oc}")
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
