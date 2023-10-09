from typing import List, Optional, Union

# System
import copy
from functools import partial
import math

# ROS
import genpy
from geometry_msgs.msg import PointStamped, PoseStamped
import PyKDL as kdl
from pykdl_ros import FrameStamped, VectorStamped
import smach
import rospy
import sys
import tf2_ros

# noinspection PyUnresolvedReferences
import tf2_geometry_msgs

# noinspection PyUnresolvedReferences
import tf2_pykdl_ros
from visualization_msgs.msg import Marker

# TU/e Robotics
from cb_base_navigation_msgs.msg import PositionConstraint, OrientationConstraint
from ed.entity import Entity
from hmi import TimeoutException
from robot_skills.simulation.sim_mode import is_sim_mode
from ..util.startup import startup
from ..utility import RateSleep, WriteDesignator
import robot_smach_states.util.designators as ds


def vector_stampeds_to_point_stampeds(vector_stampeds):
    return map(partial(tf2_ros.convert, b_type=PointStamped), vector_stampeds)


def frame_stampeds_to_pose_stampeds(frame_stampeds):
    return map(partial(tf2_ros.convert, b_type=PoseStamped), frame_stampeds)


class RegisterOperator(smach.State):
    """
    Robots looks at the operator and asks whether the operator should follow.
    """

    def __init__(
        self,
        robot,
        operator_des,
        operator_id_hint: Union[str, ds.Designator[str]] = ds.VariableDesignator(resolve_type=str),
        operator_timeout: float = 20,
        ask_follow: bool = True,
        learn_face: bool = True,
    ):
        """
        Constructor

        :param robot: robot object
        :param operator_des: Designator that resolves to the operator entity
        :param operator_id_hint: Designator that resolves to the operator id
        :param ask_follow: Whether to ask the operator to follow
        :param learn_face: Whether to learn the operator's face
        """
        smach.State.__init__(self, outcomes=["succeeded", "failed", "no_operator"])
        self._robot = robot

        ds.is_writeable(operator_des)
        ds.check_type(operator_id_hint, str)
        ds.check_type(operator_timeout, float, int)
        ds.check_type(ask_follow, bool)
        ds.check_type(learn_face, bool)

        self._operator_des = operator_des
        self._operator_id_hint = operator_id_hint
        self._operator_timeout = operator_timeout
        self._ask_follow = ask_follow
        self._learn_face = learn_face

    def execute(self, ud=None):
        start_time = rospy.Time.now()

        self._robot.head.look_at_standing_person(distance=1.5)

        # Try to get an operator entity
        operator_id_hint = ds.value_or_resolve(self._operator_id_hint)
        if operator_id_hint:
            # Can still result in None if the operator is not in ED
            operator = self._robot.ed.get_entity(uuid=operator_id_hint)
        else:
            operator = None

        # Try to get an operator entity
        while not operator:
            if self.preempt_requested():
                self.service_preempt()
                return "failed"

            if (rospy.Time.now() - start_time).to_sec() > self._operator_timeout:
                return "no_operator"

            if self._ask_follow:
                sentence = "Should I follow you?"
                self._robot.speech.speak(sentence, block=True)
                try:
                    if is_sim_mode():
                        answer = self._robot.hmi.query(sentence, "T -> yes | no", "T")
                    else:
                        answer = self._robot.picovoice.get_intent("yesOrNo")

                except TimeoutException:
                    self._robot.speech.speak("I did not hear you!")
                    rospy.sleep(1)
                else:
                    if (is_sim_mode() and answer.sentence == "yes") or (
                        not is_sim_mode() and "yes" in answer.semantics
                    ):
                        operator = self._robot.ed.get_closest_laser_entity(
                            radius=1,
                            center_point=VectorStamped.from_xyz(1.5, 0, 1, rospy.Time(), self._robot.base_link_frame),
                            ignore_z=True,
                        )
                        rospy.loginfo(f"registering operator: {operator}")
                        if not operator:
                            self._robot.speech.speak("Please stand in front of me")
                        else:
                            if self._learn_face:
                                self._robot.speech.speak(
                                    "Please look at me while I learn to recognize you.", block=True
                                )
                                self._robot.speech.speak("Just in case...", block=False)
                                self._robot.head.look_at_standing_person()
                                learn_person_start_time = rospy.Time.now()
                                learn_person_timeout = 10.0  # TODO: Parameterize
                                num_detections = 0
                                while num_detections < 5:
                                    if self._robot.perception.learn_person(self._operator_name):
                                        num_detections += 1
                                    elif (rospy.Time.now() - learn_person_start_time).to_sec() > learn_person_timeout:
                                        self._robot.speech.speak("Please stand in front of me and look at me")
                                        operator = None
                                        break
                    else:
                        # Operator said no
                        return "no_operator"
            else:
                # Not asking for the operator, just going for the closest person
                operator = self._robot.ed.get_closest_laser_entity(
                    radius=1,
                    center_point=VectorStamped.from_xyz(1.5, 0, 1, rospy.Time(), self._robot.base_link_frame),
                    ignore_z=True,
                )
                rospy.loginfo(f"registering operator without asking: {operator}")
                if not operator:
                    rospy.sleep(1)

        rospy.loginfo(f"We have a new operator: {operator.uuid}")
        self._robot.speech.speak("Gotcha! I will follow you!", block=False)
        self._operator_des.write(operator)

        # ToDo: Do this in a separate state
        # self._operator_id = operator.uuid
        # self._breadcrumbs.append(operator)

        self._robot.head.close()
        self._robot.head.wait_for_motion_done(timeout=5)

        return "succeeded"


class TrackingOperatorRate(smach.StateMachine):
    def __init__(
        self,
        robot,
        operator_id: ds.Designator[str],
        operator: ds.Designator[Entity],
        last_operator: ds.Designator[Entity],
        rate: float = 2,
    ):
        super().__init__(outcomes=["succeeded", "failed", "aborted"])

        ds.check_resolve_type(operator_id, str)
        ds.check_resolve_type(operator, Entity)
        ds.is_writeable(operator)
        ds.check_resolve_type(last_operator, Entity)
        ds.is_writeable(last_operator)

        ds.check_type(rate, float, int)

        with self:
            self.add("SLEEP", RateSleep(rate), transitions={"slept": "TRACKING_OPERATOR"})  # Start to reset the rate
            self.add(
                "TRACKING_OPERATOR",
                TrackingOperator(robot, operator_id, operator, last_operator),
                transitions={"succeeded": "SLEEP", "failed": "TRACKING_OPERATOR", "aborted": "TRACKING_OPERATOR"},
            )


class TrackingOperator(smach.State):
    def __init__(
        self,
        robot,
        operator_id: ds.Designator[str],
        operator: ds.Designator[Entity],
        last_operator: ds.Designator[Entity],
    ):
        super().__init__(self, outcomes=["tracking", "failed", "aborted"])

        ds.check_resolve_type(operator_id, str)
        ds.is_writeable(operator_id)  # ToDo: This is needed when recovery is done
        ds.check_resolve_type(operator, Entity)
        ds.is_writeable(operator)
        ds.check_resolve_type(last_operator, Entity)
        ds.is_writeable(last_operator)

        self._robot = robot
        self._operator_id = operator_id
        self._operator = operator
        self._last_operator = last_operator

    def execute(self, ud=None):
        """
        Sets self._operator_distance if we have an operator and otherwise set self._operator_distance to the
        distance to the last operator
        """
        operator_id = ds.value_or_resolve(self._operator_id)
        if operator_id:
            self._operator.write(self._robot.ed.get_entity(uuid=operator_id))
        else:
            self._operator.reset()

        if operator := self._operator.resolve():
            # Speak when last entity detection is old
            # ToDo: configure speaking period
            if (rospy.Time.now() - operator.last_update_time).to_sec() > 10:
                self._robot.speech.speak("Not so fast!")

            # If the operator is still tracked, it is also the last_operator
            self._last_operator.write(operator)

            operator_pos = PointStamped()
            operator_pos.header.stamp = rospy.Time.now()
            operator_pos.header.frame_id = self._operator_id
            operator_pos.point.x = 0.0
            operator_pos.point.y = 0.0
            operator_pos.point.z = 0.0
            self._operator_pub.publish(operator_pos)

            f = self._robot.base.get_location().frame
            self._operator_distance = operator.distance_to_2d(f.p)

            return True
        else:
            if self._last_operator.resolve():
                if self._backup_register():
                    # If the operator is still tracked, it is also the last_operator
                    self._last_operator.write(self._operator.resolve())

                    operator_pos = PointStamped()
                    operator_pos.header.stamp = rospy.Time.now()
                    operator_pos.header.frame_id = self._operator_id
                    operator_pos.point.x = 0.0
                    operator_pos.point.y = 0.0
                    operator_pos.point.z = 0.0
                    self._operator_pub.publish(operator_pos)

                    f = self._robot.base.get_location().frame
                    self._operator_distance = self._last_operator.distance_to_2d(f.p)

                    return True
                else:
                    self._robot.speech.speak("I'm sorry, but I couldn't find a person to track")

            f = self._robot.base.get_location().frame
            self._operator_distance = self._last_operator.distance_to_2d(f.p)
            # If the operator is lost, check if we still have an ID
            if self._operator_id:
                # At the moment when the operator is lost, tell him to slow down and clear operator ID
                self._operator_id = None
                rospy.loginfo(f"Operator ID is reset to {self._operator_id}")
                self._robot.speech.speak("Stop! I lost you! Until I find you again, please wait there.", block=False)
            return False


# ToDo: This should be a state machine with different arguments
class NavigateToOperator(smach.State):
    def __init__(self, robot, operator, operator_radius):
        smach.State.__init__(self, outcomes=["succeeded", "failed", "aborted"])


# ToDo: This should be a state machine with different arguments
class MonitorFollowing(smach.State):
    def __init__(self, robot, operator, replan_active, lost_operator_timeout):
        smach.State.__init__(self, outcomes=["stopped", "recover_operator", "lost_operator"])

        """
        Check we have met any end criteria to stop following the operator

        :return: None if no end criteria met, otherwise a string describing the end criteria
        """

        self._robot = robot

        ds.check_resolve_type(operator, Entity)
        self._operator = operator

        ds.check_resolve_type(replan_active, bool)
        self._replan_active = replan_active

        ds.check_type(lost_operator_timeout, float)
        self._lost_operator_timeout = lost_operator_timeout

        self._lost_operator_start = None

    def execute(self, ud=None):
        # Check if we still have an operator
        operator = self._operator.resolve()
        lost_operator = operator is None

        rospy.loginfo("Checking end criteria")

        replan_active = self._replan_active.resolve()
        if not replan_active:
            if lost_operator:
                if self._lost_operator_start is None:
                    self._lost_operator_start = rospy.Time.now()
                if (rospy.Time.now() - self._lost_operator_start).to_sec() > self._lost_operator_timeout:
                    return "lost_operator"
                return "recover_operator"
            if len(self._robot.base.global_planner.getPlan(self._replan_pc)) < self._replan_done_limit:
                self._replan_active.write(False)
                if lost_operator and not self._recover_operator():
                    return "lost_operator"

        # Try to recover operator if lost and reached last seen operator position
        rospy.loginfo(f"Operator is at {self._operator_distance:.2f} meters distance")
        if lost_operator and self._operator_distance < self._lookat_radius and self._standing_still_for_x_seconds(self._standing_still_timeout):
            rospy.loginfo(f"Lost operator and within lookat radius and standing still for {self._standing_still_timeout} seconds")
            if not self._recover_operator():
                self._robot.base.local_planner.cancelCurrentPlan()
                self._robot.speech.speak("I am unable to recover you")
                return "lost_operator"

        # Check are standing still long
        if self._standing_still_for_x_seconds(self._standing_still_timeout):
            # Navigation stuck! One of the following possibilities
            # - Following an operator, operator is still correct, corner is cut or path is otherwise invalid:
            # (path should not have been cut off) replan with global planner and wait for the local planner to get us
            # out of here
            # - Following an operator, operator is still correct, local planner is in local minimum:
            # wait for the local planner to get us out of here (at least 10 s)
            # - Following an operator, operator is not correct, 'operator' is unreachable:
            # try a global plan and wait for the local planner to get us out of here
            # - Not following an operator, planner is in local minimum: try a global plan and wait for
            # the local planner to get us out of here
            self._robot.base.local_planner.cancelCurrentPlan()
            if self._replan_allowed:
                if self._replan_attempts < self._max_replan_attempts:
                    if (rospy.Time.now() - self._replan_time).to_sec() > self._replan_timeout:
                        self._replan()
                else:
                    if not self._recover_operator():
                        return "lost_operator"
            elif not self._recover_operator():
                return "lost_operator"

        else:
            self._replan_attempts = 0

        # Check if we are already there (in operator radius and operator standing still long enough)
        rospy.loginfo("Checking if done following")
        if self._operator_distance < self._operator_radius and self._operator_standing_still_for_x_seconds(self._operator_standing_still_timeout):
            rospy.loginfo(f"I'm close enough to the operator and he's been standing there for {self._operator_standing_still_timeout} seconds")
            rospy.loginfo("Checking if we pass the start timeout")
            if (rospy.Time.now() - self._time_started).to_sec() > self._start_timeout:
                rospy.loginfo("Start timeout has passed")
                self._operator_id_des.writeable.write(self._operator_id)
                self._robot.base.local_planner.cancelCurrentPlan()
                return "stopped"
            else:
                rospy.loginfo("Start timeout not yet passed")
        else:
            rospy.loginfo("Apparently not done following")

        # No end criteria met
        return None


@smach.cb_interface(outcomes=["done"])
def _stop_following(robot, userdata=None):
    robot.base.local_planner.cancelCurrentPlan()
    return "done"


class _reset_timers(smach.State):
    def __init__(
        self,
        time_started: ds.Designator[genpy.Time],
        replan_time: ds.Designator[genpy.Time],
        replan_timeout: Union[float, ds.Designator[float]],
    ):
        super().__init__(outcomes=["done"])

        ds.check_resolve_type(time_started, genpy.Time)
        ds.check_resolve_type(replan_time, genpy.Time)
        ds.check_type(replan_timeout, float, int)

        ds.is_writeable(time_started)
        ds.is_writeable(replan_time)

        self._time_started = time_started
        self._replan_time = replan_time
        self._replan_timeout = replan_timeout

    def execute(self, ud=None):
        self._time_started.write = rospy.Time.now()
        if self._replan_time.resolve() is None:
            self._replan_time.write(
                self._time_started.resolve() - rospy.Duration(ds.value_or_resolve(self._replan_timeout))
            )

        return "done"


class FollowOperator(smach.StateMachine):
    def __init__(
        self,
        robot,
        ask_follow=True,
        learn_face=True,
        operator_radius=1,
        lookat_radius=1.2,
        start_timeout=10,
        operator_timeout=20,
        lost_timeout=60,
        lost_distance=0.8,
        operator_id_des=ds.VariableDesignator(resolve_type=str),
        standing_still_timeout=20,
        operator_standing_still_timeout=3.0,
        replan=False,
        update_period=0.5,
    ):
        """
        Constructor

        :param robot: robot object
        :param ask_follow:
        :param learn_face:
        :param operator_radius:
        :param lookat_radius:
        :param start_timeout:
        :param operator_timeout: Timeout for finding an operator
        :param lost_timeout: How long to look for the operator when we lost him/her?
        :param lost_distance:
        :param operator_id_des: Designator that resolves to the operator id to start following
        :param standing_still_timeout:
        :param operator_standing_still_timeout:
        :param replan:
        :param update_period: Time period for tracking updates in seconds
        """
        smach.StateMachine.__init__(self, outcomes=["stopped", "lost_operator", "no_operator"])
        self._robot = robot
        self._time_started: Optional[genpy.Time] = None
        self._operator: ds.VariableDesignator[Entity] = ds.VariableDesignator(resolve_type=Entity)
        self._last_operator: ds.VariableDesignator[Entity] = ds.VariableDesignator(resolve_type=Entity)
        self._operator_id: ds.VariableDesignator[str] = ds.VariableDesignator(resolve_type=str)
        self._operator_name: str = "operator"
        self._operator_radius: float = operator_radius
        self._lookat_radius: float = lookat_radius
        self._start_timeout: float = start_timeout
        self._breadcrumbs: ds.Designator[List[Entity]] = ds.VariableDesignator(
            resolve_type=[Entity]
        )  # List of Entity's
        self._breadcrumb_distance: float = 0.1  # meters between dropped breadcrumbs
        self._operator_timeout: float = operator_timeout
        self._ask_follow: bool = ask_follow
        self._learn_face: bool = learn_face
        self._lost_timeout: float = lost_timeout
        self._lost_distance: float = lost_distance
        self._standing_still_timeout: float = standing_still_timeout
        self._operator_standing_still_timeout: float = operator_standing_still_timeout
        self._operator_id_des = operator_id_des
        self._operator_distance: Optional[float] = None
        self._operator_pub = rospy.Publisher(
            f"/{robot.robot_name}/follow_operator/operator_position", PointStamped, queue_size=10
        )
        self._plan_marker_pub = rospy.Publisher(
            f"/{robot.robot_name}/global_planner/visualization/markers/global_plan", Marker, queue_size=10
        )
        self._breadcrumb_pub = rospy.Publisher(
            f"/{robot.robot_name}/follow_operator/breadcrumbs", Marker, queue_size=10
        )
        self._face_pos_pub = rospy.Publisher(
            f"/{robot.robot_name}/follow_operator/operator_detected_face", PointStamped, queue_size=10
        )

        self._last_robot_fs: Optional[FrameStamped] = None  # Used by _standing_still_for_x_seconds
        self._last_pose_stamped_time = None
        # self._last_operator_fs: Optional[FrameStamped] = None  # Used by _operator_standing_still_for_x_seconds
        self._replan_active: bool = False
        self._replan_done_limit: int = 10  # Upper limit of points in the plan before we consider the replan done
        self._replan_allowed = replan
        self._replan_timeout = 10  # seconds before another replan is allowed
        self._replan_time: Optional[genpy.Time] = None
        self._replan_attempts: int = 0
        self._max_replan_attempts: int = 3
        self._update_period: float = update_period

        @smach.cb_interface(outcomes=["done"])
        def _prepare_following(userdata=None):
            # Reset robot and operator last pose
            self._last_robot_fs = None
            # self._last_operator_fs = None
            self._last_operator = None
            self._breadcrumbs.resolve().clear()

            if self._operator_id_des:
                operator_id = self._operator_id_des.resolve()
                if operator_id:
                    self._operator_id = operator_id
                    rospy.loginfo(f"Operator ID is: {self._operator_id}")

            self._robot.head.close()

            if self._robot.robot_name == "amigo":
                self._robot.torso.send_goal("reset", timeout=4.0)

            return "done"

        def _child_term_cb(outcome_map):
            if "MONITOR" not in outcome_map:
                rospy.logerr("FollowOperator: No MONITOR outcome in outcome map")
                return False

            return True

        with self:
            self.add("PREPARE_FOLLOWING", smach.CBState(_prepare_following), transitions={"done": "REGISTER_OPERATOR"})

            self.add(
                "REGISTER_OPERATOR",
                RegisterOperator(
                    robot=self._robot,
                    operator_des=self._operator.writeable,
                    operator_id_hint=self._operator_id_des,
                    ask_follow=self._ask_follow,
                    learn_face=self._learn_face,
                ),
                transitions={
                    "succeeded": "WRITE_OPERATOR_ID",
                    "failed": "NO_OPERATOR_STOP",
                    "no_operator": "NO_OPERATOR_STOP",
                },
            )
            self.add(
                "WRITE_OPERATOR_ID",
                WriteDesignator(
                    self._operator_id.writeable, ds.AttrDesignator(self._operator, "uuid", resolve_type=str)
                ),
                transitions={"written": "RESET_TIMERS"},
            )

            self.add(
                "RESET_TIMERS",
                _reset_timers(self._time_started, self._replan_time, self._replan_timeout),
                transitions={"done": "FOLLOWING_MONITORED"},
            )

            cc = smach.Concurrence(
                outcomes=["stopped", "recover_operator", "lost_operator"],
                default_outcome="stopped",
                outcome_map={
                    "stopped": {"TRACKING_OPERATOR": "operator_stopped", "BC_NAVIGATION": "end"},
                    "recover_operator": {"TRACKING_OPERATOR": "lost_operator", "BC_NAVIGATION": "end"},
                    "lost_operator": {"TRACKING_OPERATOR": "lost_operator", "BC_NAVIGATION": "blocked"},
                },
            )
            with cc:
                # ToDo: replace with correct states/arguments
                cc.add(
                    "TRACKING_OPERATOR",
                    TrackingOperatorRate(
                        self._robot,
                        self._operator_id.writeable,
                        self._operator.writeable,
                        self._last_operator.writeable,
                    ),
                )
                cc.add("BC_NAVIGATION", NavigateToOperator(self._robot, self._operator, self._operator_radius))

            cc2 = smach.Concurrence(
                outcomes=["stopped", "recover_operator", "lost_operator"],
                default_outcome="stopped",
                child_termination_cb=_child_term_cb,
                outcome_map={
                    "stopped": {"MONITOR": "stopped", "FOLLOWING": "end"},
                    "recover_operator": {"MONITOR": "recover_operator"},
                    "lost_operator": {"MONITOR": "lost_operator", "FOLLOWING": "blocked"},
                },
            )
            with cc2:
                cc2.add("MONITOR", MonitorFollowing(self._robot, self._operator, self._operator_radius))
                cc2.add("FOLLOWING", cc)

            self.add(
                "FOLLOWING_MONITORED",
                cc2,
                transitions={
                    "stopped": "stopped",
                    "recover_operator": "RECOVER_OPERATOR",
                    "lost_operator": "LOST_OPERATOR_STOP",
                },
            )

            self.add(
                "NO_OPERATOR_STOP",
                smach.CBState(_stop_following, cb_kwargs={"robot": self._robot}),
                transitions={"done": "no_operator"},
            )
            self.add(
                "LOST_OPERATOR_STOP",
                smach.CBState(_stop_following, cb_kwargs={"robot": self._robot}),
                transitions={"done": "lost_operator"},
            )


class FollowOperatorOld(smach.State):
    def __init__(self, robot, ask_follow=True, learn_face=True, operator_radius=1, lookat_radius=1.2,
                 start_timeout=10, operator_timeout=20, lost_timeout=60, lost_distance=0.8,
                 operator_id_des=ds.VariableDesignator(resolve_type=str), standing_still_timeout=20,
                 operator_standing_still_timeout=3.0, replan=False, update_period=0.5):
        """
        Constructor

        :param robot: robot object
        :param ask_follow:
        :param learn_face:
        :param operator_radius:
        :param lookat_radius:
        :param start_timeout:
        :param operator_timeout: Timeout for finding an operator
        :param lost_timeout: How long to look for the operator when we lost him/her?
        :param lost_distance:
        :param operator_id_des: Designator that resolves to the operator id to start following
        :param standing_still_timeout:
        :param operator_standing_still_timeout:
        :param replan:
        :param update_period: Time period for tracking updates in seconds
        """
        smach.State.__init__(self, outcomes=["stopped", 'lost_operator', "no_operator"])
        self._robot = robot
        self._time_started: Optional[genpy.Time] = None
        self._operator: Optional[Entity] = None
        self._operator_id: Optional[str] = None
        self._operator_name: str = "operator"
        self._operator_radius: float = operator_radius
        self._lookat_radius: float = lookat_radius
        self._start_timeout: float = start_timeout
        self._breadcrumbs: List[Entity] = []  # List of Entity's
        self._breadcrumb_distance: float = 0.1  # meters between dropped breadcrumbs
        self._operator_timeout: float = operator_timeout
        self._ask_follow: bool = ask_follow
        self._learn_face: bool = learn_face
        self._lost_timeout: float = lost_timeout
        self._lost_distance: float = lost_distance
        self._standing_still_timeout: float = standing_still_timeout
        self._operator_standing_still_timeout: float = operator_standing_still_timeout
        self._operator_id_des = operator_id_des
        self._operator_distance: Optional[float] = None
        self._operator_pub = rospy.Publisher(
            f"/{robot.robot_name}/follow_operator/operator_position", PointStamped, queue_size=10
        )
        self._plan_marker_pub = rospy.Publisher(
            f"/{robot.robot_name}/global_planner/visualization/markers/global_plan", Marker, queue_size=10
        )
        self._breadcrumb_pub = rospy.Publisher(
            f"/{robot.robot_name}/follow_operator/breadcrumbs", Marker, queue_size=10
        )
        self._face_pos_pub = rospy.Publisher(
            f"/{robot.robot_name}/follow_operator/operator_detected_face", PointStamped, queue_size=10
        )

        self._last_robot_fs: Optional[FrameStamped] = None  # Used by _standing_still_for_x_seconds
        self._last_pose_stamped_time = None
        self._last_operator_fs: Optional[FrameStamped] = None  # Used by _operator_standing_still_for_x_seconds
        self._replan_active: bool = False
        self._replan_done_limit: int = 10  # Upper limit of points in the plan before we consider the replan done
        self._last_operator: Optional[Entity] = None
        self._replan_allowed = replan
        self._replan_timeout = 10  # seconds before another replan is allowed
        self._replan_time: Optional[genpy.Time] = None
        self._replan_attempts: int = 0
        self._max_replan_attempts: int = 3
        self._update_period: float = update_period

    def _operator_standing_still_for_x_seconds(self, timeout, cartesian_limit: float = 0.15):
        """
        Check whether the operator is standing still for X seconds

        :param timeout: how many seconds must the operator be standing still before returning True
        :type timeout: float
        :return: bool indicating whether the operator has been standing still for longer than timeout seconds
        """
        if not self._operator:
            return False

        operator_current_fs = FrameStamped(self._operator.pose.frame, rospy.Time.now(), "map")
        # rospy.loginfo("Operator position: %s" % self._operator.pose.position)

        if not self._last_operator_fs:
            self._last_operator_fs = operator_current_fs
        else:
            # Compare the pose with the last pose and update if difference is larger than x
            if (operator_current_fs.frame.p - self._last_operator_fs.frame.p).Norm() > cartesian_limit:
                # Update the last pose
                self._last_operator_fs = operator_current_fs
            else:
                time_passed = (operator_current_fs.header.stamp - self._last_operator_fs.header.stamp).to_sec()
                rospy.loginfo(f"Operator is standing still for {time_passed} seconds")
                # Check whether we passed the timeout
                if time_passed > timeout:
                    return True

        return False

    def _standing_still_for_x_seconds(self, timeout: float, cartesian_limit: float = 0.05, angular_limit: float = 0.3):
        """
        Check whether the robot is standing still for X seconds

        :param timeout: how many seconds must the robot be standing still before returning True
        :return: bool indicating whether the robot has been standing still for longer than timeout seconds
        """
        current_frame = self._robot.base.get_location().frame
        now = rospy.Time.now()

        if not self._last_robot_fs:
            self._last_robot_fs = current_frame
            self._last_pose_stamped_time = now
        else:
            current_yaw = current_frame.M.GetRPY()[2]  # Get the Yaw
            last_yaw = self._last_robot_fs.M.GetRPY()[2]  # Get the Yaw

            # Compare the pose with the last pose and update if difference is larger than x
            if kdl.diff(current_frame.p, self._last_robot_fs.p).Norm() > cartesian_limit or abs(current_yaw - last_yaw) > angular_limit:
                # Update the last pose
                self._last_robot_fs = current_frame
                self._last_pose_stamped_time = rospy.Time.now()
            else:
                time_passed = (now - self._last_pose_stamped_time).to_sec()
                rospy.loginfo(f"Robot dit not move for {time_passed} seconds")

                # Check whether we passed the timeout
                if time_passed > timeout:
                    return True
        return False

    def _register_operator(self):
        """
        Robots looks at the operator and asks whether the operator should follow.
        If he says yes, then set self._operator.
        Also adds the operator to the breadcrumb list
        """
        start_time = rospy.Time.now()

        self._robot.head.look_at_standing_person(distance=1.5)

        if self._operator_id:
            # Can still result in None if the operator is not in ED
            operator = self._robot.ed.get_entity(uuid=self._operator_id)
        else:
            operator = None

        # Try to get an operator entity
        while not operator:
            if self.preempt_requested():
                return False

            if (rospy.Time.now() - start_time).to_sec() > self._operator_timeout:
                return False

            if self._ask_follow:
                sentence = "Should I follow you?"
                self._robot.speech.speak(sentence, block=True)
                try:
                    if is_sim_mode():
                        answer = self._robot.hmi.query(sentence, "T -> yes | no", "T")
                    else:
                        answer = self._robot.picovoice.get_intent("yesOrNo")

                except TimeoutException:
                    self._robot.speech.speak("I did not hear you!")
                    rospy.sleep(2)
                else:
                    if (is_sim_mode() and answer.sentence == "yes") or (not is_sim_mode() and "yes" in answer.semantics):
                        operator = self._robot.ed.get_closest_laser_entity(
                            radius=1,
                            center_point=VectorStamped.from_xyz(1.5, 0, 1, rospy.Time(), self._robot.base_link_frame),
                            ignore_z=True)
                        rospy.loginfo(f"registering operator: {operator}")
                        if not operator:
                            self._robot.speech.speak("Please stand in front of me")
                        else:
                            if self._learn_face:
                                self._robot.speech.speak("Please look at me while I learn to recognize you.",
                                                         block=True)
                                self._robot.speech.speak("Just in case...",
                                                         block=False)
                                self._robot.head.look_at_standing_person()
                                learn_person_start_time = rospy.Time.now()
                                learn_person_timeout = 10.0  # TODO: Parameterize
                                num_detections = 0
                                while num_detections < 5:
                                    if self._robot.perception.learn_person(self._operator_name):
                                        num_detections += 1
                                    elif (rospy.Time.now() - learn_person_start_time).to_sec() > learn_person_timeout:
                                        self._robot.speech.speak("Please stand in front of me and look at me")
                                        operator = None
                                        break
                    else:
                        return False
            else:
                # Not asking for the operator, just going for the closest person
                operator = self._robot.ed.get_closest_laser_entity(
                    radius=1,
                    center_point=VectorStamped.from_xyz(1.5, 0, 1, rospy.Time(), self._robot.base_link_frame),
                    ignore_z=True)
                rospy.loginfo(f"registering operator without asking: {operator}")
                if not operator:
                    rospy.sleep(1)

        rospy.loginfo("We have a new operator: %s" % operator.uuid)
        self._robot.speech.speak("Gotcha! I will follow you!", block=False)
        self._operator_id = operator.uuid
        self._operator = operator
        self._breadcrumbs.append(operator)

        self._robot.head.close()

        rospy.loginfo("NOW!!!")
        rospy.sleep(1)

        return True

    def _update_breadcrumb_path(self):
        """
        If the last breadcrumb is less than a threshold away, replace
        the last breadcrumb with the latest operator position; otherwise
        just add it.
        In case a breadcrumb is 'reached', remove all breadcrumbs up to that point
        """
        if self._operator:
            if self._breadcrumbs:
                if self._breadcrumbs[-1].distance_to_2d(self._operator.pose.frame.p) < self._breadcrumb_distance:
                    self._breadcrumbs[-1] = self._operator
                else:
                    self._breadcrumbs.append(self._operator)
            else:
                self._breadcrumbs.append(self._operator)

        # Remove 'reached' breadcrumbs from breadcrumb path
        robot_position = self._robot.base.get_location().frame
        # robot_yaw = transformations.euler_z_from_quaternion(self._robot.base.pose.orientation)
        temp_crumbs = []
        for crumb in self._breadcrumbs:
            if crumb.distance_to_2d(robot_position.p) > self._lookat_radius + 0.1:
                temp_crumbs.append(crumb)
            else:
                temp_crumbs = []

        self._breadcrumbs = temp_crumbs

        self._visualize_breadcrumbs(self._breadcrumbs)

    def _backup_register(self):
        """This only happens when the operator was just registered, and never tracked"""
        rospy.loginfo("Operator already lost. Getting closest possible person entity at 1.5 m in front, radius = 1")
        self._operator = self._robot.ed.get_closest_laser_entity(
            radius=1,
            center_point=VectorStamped.from_xyz(1.5, 0, 1, rospy.Time(), self._robot.base_link_frame),
            ignore_z=True
        )
        if self._operator:
            return True
        else:
            rospy.loginfo("Operator still lost. Getting closest possible laser entity at 1.5 m in front, radius = 1")
            self._operator = self._robot.ed.get_closest_laser_entity(
                radius=1,
                center_point=VectorStamped.from_xyz(1.5, 0, 1, rospy.Time(), self._robot.base_link_frame),
                ignore_z=True
            )

        if self._operator:
            return True
        else:
            rospy.loginfo("Trying to register operator again")
            self._robot.speech.speak("Oops, let's try this again...", block=False)
            self._register_operator()
            self._operator = self._robot.ed.get_entity(uuid=self._operator_id)

        if self._operator:
            self._last_operator = self._operator
            return True

        return False

    def _track_operator(self):
        """
        Sets self._operator_distance if we have an operator and otherwise set self._operator_distance to the
        distance to the last operator
        """
        if self._operator_id:
            self._operator = self._robot.ed.get_entity(uuid=self._operator_id)
        else:
            self._operator = None

        if self._operator:
            if (rospy.Time.now() - self._operator.last_update_time).to_sec() > self._update_period:
                self._robot.speech.speak("Not so fast!")

            # If the operator is still tracked, it is also the last_operator
            self._last_operator = self._operator

            operator_pos = PointStamped()
            operator_pos.header.stamp = rospy.Time.now()
            operator_pos.header.frame_id = self._operator_id
            operator_pos.point.x = 0.0
            operator_pos.point.y = 0.0
            operator_pos.point.z = 0.0
            self._operator_pub.publish(operator_pos)

            f = self._robot.base.get_location().frame
            self._operator_distance = self._last_operator.distance_to_2d(f.p)

            return True
        else:
            if not self._last_operator:
                if self._backup_register():
                    # If the operator is still tracked, it is also the last_operator
                    self._last_operator = self._operator

                    operator_pos = PointStamped()
                    operator_pos.header.stamp = rospy.Time.now()
                    operator_pos.header.frame_id = self._operator_id
                    operator_pos.point.x = 0.0
                    operator_pos.point.y = 0.0
                    operator_pos.point.z = 0.0
                    self._operator_pub.publish(operator_pos)

                    f = self._robot.base.get_location().frame
                    self._operator_distance = self._last_operator.distance_to_2d(f.p)

                    return True
                else:
                    self._robot.speech.speak("I'm sorry, but I couldn't find a person to track")

            f = self._robot.base.get_location().frame
            self._operator_distance = self._last_operator.distance_to_2d(f.p)
            # If the operator is lost, check if we still have an ID
            if self._operator_id:
                # At the moment when the operator is lost, tell him to slow down and clear operator ID
                self._operator_id = None
                rospy.loginfo(f"Operator ID is reset to {self._operator_id}")
                self._robot.speech.speak("Stop! I lost you! Until I find you again, please wait there.", block=False)
            return False

    def _visualize_breadcrumbs(self, breadcrumbs):
        breadcrumbs_msg = Marker()
        breadcrumbs_msg.type = Marker.POINTS
        breadcrumbs_msg.scale.x = 0.05
        breadcrumbs_msg.header.stamp = rospy.get_rostime()
        breadcrumbs_msg.header.frame_id = "map"
        breadcrumbs_msg.color.a = 1
        breadcrumbs_msg.color.r = 0
        breadcrumbs_msg.color.g = 1
        breadcrumbs_msg.color.b = 1
        breadcrumbs_msg.lifetime = rospy.Time(1.0)
        breadcrumbs_msg.id = 0
        breadcrumbs_msg.action = Marker.ADD

        for crumb in breadcrumbs:
            breadcrumbs_msg.points.append(tf2_ros.convert(crumb.pose, PoseStamped).pose.position)

        self._breadcrumb_pub.publish(breadcrumbs_msg)

    def _visualize_plan(self, path):
        line_strip = Marker()
        line_strip.type = Marker.LINE_STRIP
        line_strip.scale.x = 0.05
        line_strip.header.frame_id = "map"
        line_strip.header.stamp = rospy.Time.now()
        line_strip.color.a = 1
        line_strip.color.r = 0
        line_strip.color.g = 1
        line_strip.color.b = 1
        line_strip.id = 0
        line_strip.action = Marker.ADD

        # Push back all pnts
        for pose_stamped in path:
            line_strip.points.append(pose_stamped.pose.position)

        self._plan_marker_pub.publish(line_strip)

    def _update_navigation(self):
        """
        Set the navigation plan to match the breadcrumbs collected into self._breadcrumbs.
        This list has all the Entity's of where the operator has been
        """
        self._robot.head.cancel_goal()

        robot_position = self._robot.base.get_location().frame.p
        operator_position = self._last_operator.pose.frame.p

        """Define end goal constraint, solely based on the (old) operator position"""
        pc = PositionConstraint()
        pc.constraint = f"(x-{operator_position.x()})^2 + (y-{operator_position.y()})^2 < {self._operator_radius}^2"

        oc = OrientationConstraint()
        # ToDo: should we check if the operator ID still exists in ED, before using it?
        if self._operator_id and self._robot.ed.get_entity(uuid=self._operator_id):
            oc.frame = self._operator_id
        else:
            oc.frame = "map"
            oc.look_at = tf2_ros.convert(self._last_operator.pose, PoseStamped).pose.position

        """Calculate global plan from robot position, through breadcrumbs, to the operator"""
        res = 0.05  # ToDo: magic number
        kdl_plan = []
        previous_point = robot_position

        if self._operator:
            breadcrumbs = self._breadcrumbs + [self._operator]
        else:
            breadcrumbs = self._breadcrumbs + [self._last_operator]
        for crumb in breadcrumbs:
            assert isinstance(crumb, Entity)
            diff = crumb.pose.frame.p - previous_point
            dx, dy = diff.x(), diff.y()

            length = crumb.distance_to_2d(previous_point)

            if length != 0:
                dx_norm = dx / length
                dy_norm = dy / length
                yaw = math.atan2(dy, dx)

                start = 0
                end = int(length / res)

                for i in range(start, end):
                    x = previous_point.x() + i * dx_norm * res
                    y = previous_point.y() + i * dy_norm * res
                    kdl_plan.append(FrameStamped.from_xyz_rpy(x, y, 0, 0, 0, yaw, rospy.Time.now(), "map"))

            previous_point = copy.deepcopy(crumb.pose.frame.p)

        # Delete the last elements from the plan within the operator radius from the operator, to keep some distance
        cutoff = int(self._operator_radius/(2.0*res))
        if len(kdl_plan) > cutoff:
            del kdl_plan[-cutoff:]

        ros_plan = [*frame_stampeds_to_pose_stampeds(kdl_plan)]
        # Check if plan is valid. If not, remove invalid points from the path
        if not self._robot.base.global_planner.checkPlan(ros_plan):
            rospy.loginfo("Breadcrumb plan is blocked, removing blocked points")
            # Go through plan from operator to robot and pick the first unoccupied point as goal point and the points
            # before that as the plan
            ros_plan = [point for point in ros_plan if self._robot.base.global_planner.checkPlan([point])]

        self._visualize_plan(ros_plan)
        self._robot.base.local_planner.setPlan(ros_plan, pc, oc)

    def _recover_operator(self, look_distance: float = 2.0):
        if not self._learn_face:
            return False
        rospy.loginfo("Trying to recover the operator")
        self._robot.head.look_at_standing_person()
        self._robot.speech.speak(f"{self._operator_name}, please look at me while I am looking for you", block=False)

        look_angles = [0.0,
                       math.pi/6,
                       math.pi/4,
                       math.pi/2.3,
                       0.0,
                       -math.pi/6,
                       -math.pi/4,
                       -math.pi/2.3]
        head_goals = [VectorStamped.from_xyz(look_distance*math.cos(angle), look_distance*math.sin(angle), 1.7,
                                             rospy.Time.now(), self._robot.base_link_frame)
                      for angle in look_angles]

        # Wait for the operator and find his/her face
        start_time = rospy.Time.now()
        i = 0
        while (rospy.Time.now() - start_time).to_sec() < self._lost_timeout:
            if self.preempt_requested():
                return False

            self._robot.head.look_at_point(head_goals[i])
            i += 1
            if i == len(head_goals):
                i = 0

            self._robot.head.wait_for_motion_done()

            # raw_detections is a list of Recognitions
            # a recognition contains a CategoricalDistribution
            # a CategoricalDistribution is a list of CategoryProbabilities
            # a CategoryProbability has a label and a float
            raw_detections, _ = self._robot.perception.detect_faces()
            best_detection = self._robot.perception.get_best_face_recognition(raw_detections, "operator")

            rospy.loginfo(f"{best_detection=}")
            if best_detection:

                # rospy.loginfo("Best detection: {}".format(best_detection))
                roi = best_detection.roi

                try:
                    operator_pos_kdl = self._robot.perception.project_roi(roi=roi, frame_id="map")
                except Exception as e:
                    rospy.logerr(f"head.project_roi failed: {e}")
                    return False
                operator_pos_ros = tf2_ros.convert(operator_pos_kdl, PointStamped)

                self._face_pos_pub.publish(operator_pos_ros)

                recovered_operator = self._robot.ed.get_closest_laser_entity(radius=self._lost_distance,
                                                                             center_point=operator_pos_kdl,
                                                                             ignore_z=True)

                if recovered_operator:
                    rospy.loginfo("Found one!")
                    self._operator_id = recovered_operator.uuid
                    rospy.loginfo(f"Recovered operator id: {self._operator_id}")
                    self._operator = recovered_operator
                    self._robot.speech.speak("There you are! Go ahead, I'll follow you again", block=False)
                    self._robot.head.close()
                    self._time_started = rospy.Time.now()
                    return True
                else:
                    rospy.loginfo(f"Could not find an entity {self._lost_distance} meter near {operator_pos_kdl}")

        self._robot.head.close()
        self._turn_towards_operator()
        self._update_navigation()
        rospy.sleep(2.0)
        return False

    def _turn_towards_operator(self):
        robot_position = self._robot.base.get_location().frame.p
        operator_position = self._last_operator.pose.frame.p

        pc = PositionConstraint()
        pc.constraint = f"(x-{operator_position.x()})^2 + (y-{operator_position.y()})^2 < {self._operator_radius}^2"

        oc = OrientationConstraint()
        if self._operator_id and self._robot.ed.get_entity(uuid=self._operator_id):
            oc.frame = self._operator_id
        else:
            oc.frame = 'map'
            oc.look_at = tf2_ros.convert(self._last_operator.pose, PoseStamped).pose.position

        dx = operator_position.x() - robot_position.x()
        dy = operator_position.y() - robot_position.y()

        yaw = math.atan2(dy, dx)
        fs = FrameStamped.from_xyz_rpy(robot_position.x(), robot_position.y(), 0, 0, 0, yaw, rospy.Time.now(), "map")
        ps = tf2_ros.convert(fs, PoseStamped)

        self._robot.base.local_planner.setPlan([ps], pc, oc)

    def _replan(self):
        self._replan_attempts += 1
        rospy.loginfo("Trying to get a global plan")
        operator_position = self._last_operator.pose.frame.p
        # Define end goal constraint, solely based on the (old) operator position
        self._replan_pc = PositionConstraint()
        self._replan_pc.constraint = (
            f"(x-{operator_position.x()})^2 + (y-{operator_position.y()})^2 < {self._operator_radius}^2"
        )
        ros_plan = self._robot.base.global_planner.getPlan(self._replan_pc)
        if not ros_plan or not self._robot.base.global_planner.checkPlan(ros_plan):
            rospy.loginfo("No global plan possible")
        else:
            self._robot.speech.speak("Just a sec, let me try this way.")
            rospy.loginfo("Found a global plan, sending it to local planner")
            self._replan_time = rospy.Time.now()
            self._replan_active = True
            oc = self._robot.base.local_planner.getCurrentOrientationConstraint()
            self._visualize_plan(ros_plan)
            self._robot.base.local_planner.setPlan(ros_plan, self._replan_pc, oc)
            self._breadcrumbs.clear()

    def _check_end_criteria(self) -> Optional[str]:
        """
        Check we have met any end criteria to stop following the operator

        :return: None if no end criteria met, otherwise a string describing the end criteria
        """
        # Check if we still have an operator
        lost_operator = self._operator is None

        rospy.loginfo("Checking end criteria")

        if self._replan_active:
            if len(self._robot.base.global_planner.getPlan(self._replan_pc)) < self._replan_done_limit:
                self._replan_active = False
                if lost_operator and not self._recover_operator():
                    return "lost_operator"

        # Try to recover operator if lost and reached last seen operator position
        rospy.loginfo(f"Operator is at {self._operator_distance:.2f} meters distance")
        if lost_operator and self._operator_distance < self._lookat_radius and self._standing_still_for_x_seconds(self._standing_still_timeout):
            rospy.loginfo(f"Lost operator and within lookat radius and standing still for {self._standing_still_timeout} seconds")
            if not self._recover_operator():
                self._robot.base.local_planner.cancelCurrentPlan()
                self._robot.speech.speak("I am unable to recover you")
                return "lost_operator"

        # Check are standing still long
        if self._standing_still_for_x_seconds(self._standing_still_timeout):
            # Navigation stuck! One of the following possibilities
            # - Following an operator, operator is still correct, corner is cut or path is otherwise invalid:
            # (path should not have been cut off) replan with global planner and wait for the local planner to get us
            # out of here
            # - Following an operator, operator is still correct, local planner is in local minimum:
            # wait for the local planner to get us out of here (at least 10 s)
            # - Following an operator, operator is not correct, 'operator' is unreachable:
            # try a global plan and wait for the local planner to get us out of here
            # - Not following an operator, planner is in local minimum: try a global plan and wait for
            # the local planner to get us out of here
            self._robot.base.local_planner.cancelCurrentPlan()
            if self._replan_allowed:
                if self._replan_attempts < self._max_replan_attempts:
                    if (rospy.Time.now() - self._replan_time).to_sec() > self._replan_timeout:
                        self._replan()
                else:
                    if not self._recover_operator():
                        return "lost_operator"
            elif not self._recover_operator():
                return "lost_operator"

        else:
            self._replan_attempts = 0

        # Check if we are already there (in operator radius and operator standing still long enough)
        rospy.loginfo("Checking if done following")
        if self._operator_distance < self._operator_radius and self._operator_standing_still_for_x_seconds(self._operator_standing_still_timeout):
            rospy.loginfo(f"I'm close enough to the operator and he's been standing there for {self._operator_standing_still_timeout} seconds")
            rospy.loginfo("Checking if we pass the start timeout")
            if (rospy.Time.now() - self._time_started).to_sec() > self._start_timeout:
                rospy.loginfo("Start timeout has passed")
                self._operator_id_des.writeable.write(self._operator_id)
                self._robot.base.local_planner.cancelCurrentPlan()
                return "stopped"
            else:
                rospy.loginfo("Start timeout not yet passed")
        else:
            rospy.loginfo("Apparently not done following")

        # No end criteria met
        return None

    def execute(self, userdata=None):
        # Reset robot and operator last pose
        self._last_robot_fs = None
        self._last_operator_fs = None
        self._breadcrumbs.clear()

        if self._operator_id_des:
            operator_id = self._operator_id_des.resolve()
            if operator_id:
                self._operator_id = operator_id
                rospy.loginfo(f"Operator ID is: {self._operator_id}")

        self._robot.head.close()

        if self._robot.robot_name == "amigo":
            self._robot.torso.send_goal('reset', timeout=4.0)

        if not self._register_operator():
            # ToDo: Why do we cancel the plan? We didn't start anything yet
            self._robot.base.local_planner.cancelCurrentPlan()
            return "no_operator"

        self._time_started = rospy.Time.now()

        if self._replan_time is None:
            self._replan_time = self._time_started - rospy.Duration(self._replan_timeout)

        rate = rospy.Rate(1/self._update_period)
        while not rospy.is_shutdown():

            if self.preempt_requested():
                return 'lost_operator'

            # 1) Track operator
            self._track_operator()

            # 2) Keep track of operator history
            self._update_breadcrumb_path()

            # 3) Check end criteria
            result = self._check_end_criteria()
            if result:
                return result

            # 4) Action
            if not self._operator_standing_still_for_x_seconds(self._operator_standing_still_timeout) and self._operator_distance < self._lookat_radius:
                rospy.loginfo("Operator within self._lookat_radius")
                self._turn_towards_operator()
            else:
                # Only update navigation if there is something to update: operator must have moved
                if self._replan_allowed:
                    # If replanned: if recently replanned, only update navigation if not standing still for too long
                    # (to make sure that local planner reaches align state) or just started following
                    rospy.loginfo("Replan=True, so check if we replanned...")
                    if self._replan_time > self._time_started:
                        rospy.loginfo("We did replan at least once")
                        if self._replan_active:
                            rospy.loginfo("and this plan is still active, so I'll give the global planner a chance")
                        else:
                            rospy.loginfo("but we reached that goal at some point, so we can safely update navigation")
                            self._update_navigation()
                    else:
                        rospy.loginfo("We never replanned so far, so we can safely update navigation")
                        self._update_navigation()
                    # else:
                    #     rospy.loginfo("Updating navigation")
                    #     self._update_navigation()
                else:
                    self._update_navigation()
                    rospy.loginfo("Updating navigation.")

            rate.sleep()


def setup_statemachine(robot):
    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])
    with sm:
        smach.StateMachine.add('TEST', FollowOperator(robot), transitions={"stopped": "TEST",
                                                                           'lost_operator': "TEST",
                                                                           "no_operator": "TEST"})
        return sm


if __name__ == "__main__":
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        rospy.loginfo("Please provide robot name as argument.")
        exit(1)

    rospy.init_node('test_follow_operator')
    startup(setup_statemachine, robot_name=robot_name)
