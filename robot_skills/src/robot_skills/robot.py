#! /usr/bin/env python

from typing import Optional

from collections import OrderedDict
from collections.abc import Sequence, Set

# ROS
import geometry_msgs
import rospy
import tf2_ros
# noinspection PyUnresolvedReferences
import tf2_geometry_msgs
# noinspection PyUnresolvedReferences
import tf2_kdl
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String

# Robot skills
from .arm import arms
from .functionalities.add_functionalities import add_functionalities

DEFAULT_CONNECTION_TIMEOUT = 10.0  # Timeout: all ROS connections must be alive within this duration


class Robot:
    """
    Interface to all parts of the robot.
    """
    def __init__(
        self,
        robot_name: str = "",
        tf_buffer: Optional[tf2_ros.Buffer] = None,
        base_link_frame: Optional[str] = None,
        connection_timeout: float = DEFAULT_CONNECTION_TIMEOUT
    ):
        """
        Constructor

        :param robot_name: Name of the robot
        :param tf_buffer: tf2_ros.Buffer object
        :param base_link_frame: Frame id of the base_link (default: {robot_name}/base_link
        :param connection_timeout: timeout to wait for ROS connections
        """

        self.robot_name = robot_name
        if tf_buffer is None:
            self.tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        else:
            self.tf_buffer = tf_buffer

        if base_link_frame is None:
            base_link_frame = f"{robot_name}/base_link"

        self.base_link_frame = base_link_frame

        self._connection_timeout = float(connection_timeout)

        self.configured = False

        # Body parts
        self.parts = dict()

        # Ensuring arms have a fixed order of iteration.
        self._arms = OrderedDict()  # type: OrderedDict[arms.Arm]

        # Ignore diagnostics: parts that are not present in the real robot
        self._ignored_parts = []

        # Miscellaneous
        self.pub_target = rospy.Publisher("/target_location", geometry_msgs.msg.Pose2D, queue_size=10)

        self.image_pub = rospy.Publisher("/" + self.robot_name + '/image_from_ros', Image, queue_size=1)
        self.message_pub = rospy.Publisher("/" + self.robot_name + '/message_from_ros', String, queue_size=1)

        # Check hardware status
        self._hardware_status_sub = rospy.Subscriber(
            "/" + self.robot_name + "/hardware_status", DiagnosticArray, self.handle_hardware_status
        )

        self.laser_topic = "/"+self.robot_name+"/base_laser/scan"

    def get_joint_states(self):
        msg = rospy.wait_for_message("/{}/joint_states".format(self.robot_name), JointState)
        return dict(zip(msg.name, msg.position))

    def add_body_part(self, partname, bodypart):
        """
        Add a bodypart to the robot. This is added to the parts dict and set as an attribute

        :param partname: name of the bodypart
        :param bodypart: bodypart object
        """
        self.parts[partname] = bodypart
        setattr(self, partname, bodypart)

    def add_arm_part(self, arm_name, arm_part):
        """
        Add an arm part to the robot. This is added to the parts dictionary as well.

        :param arm_name: Name of the arm part.
        :param arm_part: Arm part object
        """
        # Don't add the arm to the robot object to avoid direct access from challenge code.
        self.parts[arm_name] = arm_part
        self._arms[arm_name] = arm_part

    @property
    def arms(self):
        # ToDo: remove this property per September 2020
        rospy.logwarn('Arms should be private and therefore not be called directly. The only interface that should be '
                      'used is the get_arm function. Change your code, you are invading private property!')
        return self._arms

    def configure(self):
        """
        This should be run at the end of the constructor of a child class.
        """

        add_functionalities(self)  # at the end of robot construction add functionalities

        if self._connection_timeout <= 0:
            rospy.loginfo("Not waiting for ROS connections as timeout is 0")
            self.configured = True
            return

        # Wait for connections
        connected = False
        s = rospy.Time.now()
        r = rospy.Rate(1.0)
        rospy.loginfo("Waiting for ROS connections")

        def remaining_time():
            return self._connection_timeout - (rospy.Time.now() - s).to_sec()

        while not connected and remaining_time() > 0:
            connected_hypot = True
            for bodypart in self.parts.values():
                connected_hypot = connected_hypot and bodypart.wait_for_connections(0.1, log_failing_connections=False)
            if connected_hypot:
                connected = True
                break

            loop_remaining_time = r.remaining().to_sec()
            if loop_remaining_time+0.01 < remaining_time():
                r.sleep()
                rospy.loginfo("Will wait for another {} seconds".format(
                    round(remaining_time(), 2)))
            else:
                rospy.sleep(loop_remaining_time)

        # If connected: log how low it took
        if connected:
            e = rospy.Time.now()
            rospy.logdebug("Connecting took {} seconds".format((e-s).to_sec()))
        else:  # Else: check again but now do log the errors
            for bodypart in self.parts.values():
                bodypart.wait_for_connections(0.1, log_failing_connections=True)

        if not self.operational:
            not_operational_parts = [name for name, part in self.parts.items() if not part.operational]
            rospy.logwarn("Not all hardware operational: {parts}".format(parts=not_operational_parts))

        self.configured = True

    def reset(self):
        results = {}
        for partname, bodypart in self.parts.items():
            rospy.logdebug("Resetting {}".format(partname))
            if self.robot_name == 'hero' and partname == 'torso':
                rospy.logwarn("Skipping reset of %s", partname)
            else:
                bodypart.reset()
        return all(results.values())

    def reset_all_arms(self):
        """
        Reset all arms of the robot, including their parts.
        """
        for arm in self._arms.values():
            arm.reset()

    def publish_target(self, x, y):
        self.pub_target.publish(geometry_msgs.msg.Pose2D(x, y, 0))

    def get_arm(self, required_gripper_types=None, desired_gripper_types=None,
                required_goals=None, desired_goals=None,
                required_trajectories=None, desired_trajectories=None,
                required_arm_name=None,
                force_sensor_required=False,
                required_objects=None, desired_objects=None):
        """
        Find an arm that has the needed and hopefully the desired properties.
        Does not give an arm if all needed properties cannot be satisfied. An
        arm returned by the call will not be able to do anything you didn't ask
        for, except for objects held by it.

        :param required_gripper_types: Collection of gripper types that must all be
                available. None means grippers are not needed.
        :param desired_gripper_types: Collection of gripper types from GripperTypes,
                where one or more may be selected. None means no grippers are desired.

        :param required_goals: Collection of joint goals that must all be available.
                None means no joint goals are needed.
        :param desired_goals: Collection of joint goals where one or more may be
                selected. None means no joint goals are desired.

        :param required_trajectories: Collection of joint trajectories that must all
                be available. None means no joint trajectories are needed.
        :param desired_trajectories: Collection of joint trajectories where one or
                more may be selected. None means no joint trajectories are desired.

        :param required_arm_name: Name of the arm that is needed. If set, no
                other arm will be considered. None means any arm will do.

        :param force_sensor_required: Bool specifying whether a force_sensor is needed or not.

        :param required_objects: Collection of objects that the arm must have. Special
                pseudo-objects PseudoObjects.ANY and PseudoObjects.EMPTY may be used
                too in the collection, although they do not make much sense when used
                together with other objects. None means there are no required objects.
        :param desired_objects: Collection of objects that the arm may have. None
                means there are no desired objects.

        :return: An Arm of the robot with the requested properties, or None.
                Note that the arm will never do more than you requested.
        """
        discarded_reasons = []  # Reasons why arms are discarded.

        # Check that collection arguments are really a collection of objects, but not strings.
        # Because then you might accidentally pass a GripperType instead of a [GripperType], which is a List
        def seq_set_or_none(obj):
            return not isinstance(obj, str) and (isinstance(obj, Sequence) or isinstance(obj, Set) or obj is None)
        assert seq_set_or_none(required_gripper_types)
        assert seq_set_or_none(desired_gripper_types)
        assert seq_set_or_none(required_goals)
        assert seq_set_or_none(desired_goals)
        assert seq_set_or_none(required_trajectories)
        assert seq_set_or_none(desired_trajectories)
        assert isinstance(force_sensor_required, bool)
        assert seq_set_or_none(required_objects)
        assert seq_set_or_none(desired_objects)

        for arm_name, arm in self._arms.items():
            if not arm.operational:
                discarded_reasons.append((arm_name, "not operational"))
                continue

            # Name
            if required_arm_name is not None and arm_name != required_arm_name:
                discarded_reasons.append((arm_name, "required arm name failed"))
                continue

            # Grippers
            matching_grippers = set()
            if required_gripper_types is not None:
                matches = [arm.collect_gripper_types(req_type) for req_type in required_gripper_types]
                all_matched = all(match_list for match_list in matches)
                if not all_matched:
                    discarded_reasons.append((arm_name, "required gripper type failed"))
                    continue

                for match in matches:
                    matching_grippers.update(match)

            if desired_gripper_types is not None:
                matches = [arm.collect_gripper_types(des_type) for des_type in desired_gripper_types]
                for match in matches:
                    matching_grippers.update(match)

            # Goals
            matching_goals = _collect_needs_desires(required_goals, desired_goals, arm.has_joint_goal)
            if matching_goals is None:
                discarded_reasons.append((arm_name, "required goals failed"))
                continue

            # Trajectories
            matching_trajectories = _collect_needs_desires(required_trajectories, desired_trajectories,
                                                           arm.has_joint_trajectory)
            if matching_trajectories is None:
                discarded_reasons.append((arm_name, "required trajectories failed"))
                continue

            # Force sensor availability
            if force_sensor_required and not hasattr(arm, "force_sensor"):
                discarded_reasons.append((arm_name, "should have a force sensor but hasn't"))
                continue

            # Objects
            if not self._check_required_obj(arm, required_objects):
                discarded_reasons.append((arm_name, "required objects failed"))
                continue
            # Desired objects not checked currently.

            # Unlike the other properties, it is likely not exactly known what objects will be used.
            # For this reason, it seems better not to limit objects to the set specified in required
            # _objects and desired_objects.
            # uses_objects = (required_objects is not None or desired_objects is not None)

            # ToDO: HACK for not specifying any requirements:
            # We often want to use the arm for object manipulation, so this is always needed
            uses_objects = True  # (required_objects is not None or desired_objects is not None)

            # Success!
            if not matching_grippers:
                default_gripper = None
            else:
                default_gripper = next(iter(matching_grippers))
            return arms.PublicArm(arm, matching_grippers, default_gripper, uses_objects,
                                  force_sensor_required, matching_goals, matching_trajectories)

        # No arm matched. Dump why.
        msg = ("Failed to find a matching arm, reasons: " +
               " ".join("{}:{}".format(name, reason) for name, reason in discarded_reasons))
        rospy.logwarn(msg)
        return None

    @staticmethod
    def _check_required_obj(arm, obj_collection):
        """
        Check the object requirement.

        :param arm: Arm to check.
        :param obj_collection: Objects to find. None means the empty requirement
                (arm may have anything, including nothng).
        :return: Whether the requirement holds.
        """
        if obj_collection is None:
            return True

        cur_obj = arm.gripper.occupied_by
        for obj in obj_collection:
            if obj == arms.PseudoObjects.ANY:  # Any object, but not empty.
                if cur_obj is None:
                    return False
            elif obj == arms.PseudoObjects.EMPTY:  # Arm must be empty.
                if cur_obj is not None:
                    return False
            elif obj != cur_obj:
                return False
        return True

    def close(self):
        for partname, bodypart in self.parts.items():
            try:
                bodypart.close()
            except Exception:
                pass

    @property
    def operational(self):
        """
        :return: if all parts are operational
        """
        return all(bodypart.operational for bodypart in self.parts.values())

    def handle_hardware_status(self, diagnostic_array):
        """
        hardware_status callback to determine if the bodypart is operational

        :param diagnostic_array: diagnostic_msgs.msg.DiagnosticArray
        :return: no return
        """

        diagnostic_dict = {diagnostic_status.name: diagnostic_status for diagnostic_status in diagnostic_array.status}

        for name, part in self.parts.items():
            # Pass a dict mapping the name to the item.
            # Bodypart.handle_hardware_status needs to find the element relevant to itself
            # iterating over the array would be done be each bodypart, but with a dict they can just look theirs up.
            if name not in self._ignored_parts:
                part.process_hardware_status(diagnostic_dict)

    def move_to_inspect_pose(self, inspection_position):
        """
        This poses the robot for an inspect.

        :param inspection_position: kdl.Frame with the pose of the entity to be inspected.
        :return: boolean, false if something went wrong.
        """
        rospy.logdebug("move_to_inspect_pose() not implemented for {} object".format(self.robot_name))
        return True

    def move_to_pregrasp_pose(self, arm, grasp_position):
        """
        This poses the robot for an inspect.

        :param arm: PublicArm to use for grasping the target
        :param grasp_position: kdl.Frame with the pose of the entity to be grasp.
        :return: boolean, false if something went wrong.
        """
        rospy.logdebug("move_to_grasp_pose() not implemented for {} object".format(self.robot_name))
        return True

    def move_to_hmi_pose(self):
        """
        This poses the robot for conversations.

        :return: None
        """
        rospy.logdebug("move_to_hmi_pose() not implemented for {} object".format(self.robot_name))
        pass

    def go_to_driving_pose(self):
        """
        This poses the robot for driving.

        :return: None
        """
        rospy.logdebug("go_to_driving_pose() not implemented for {} object".format(self.robot_name))
        pass

    def __enter__(self):
        pass

    def __exit__(self, exception_type, exception_val, trace):
        if any((exception_type, exception_val, trace)):
            rospy.logerr("Robot exited with {0},{1},{2}".format(exception_type, exception_val, trace))
        self.close()


def _collect_needs_desires(needs, desires, test_func):
    """
    :param needs: Collection of needed values. None means nothing is needed.
    :param desires: Collection of desired values, None means nothing is desired.
    :param test_func: Function that takes a value and returns whether the value is available.
    :return: Needed and subset of the desired values, or None if the needs can't be met.
    """
    founds = set()
    if needs is not None:
        founds.update(_collect_available(needs, test_func))
        if len(founds) != len(needs):  # All needs must be met.
            return None

    if desires is not None:
        founds.update(_collect_available(desires, test_func))

    return founds


def _collect_available(values, test_func):
    """
    :param values: Collection of values that must be tested.
    :param test_func: Function that takes a value and returns whether the value is available.
    :return: Subset of values that is found to be available.
    """
    founds = set()
    for value in values:
        if test_func(value):
            founds.add(value)
    return founds
