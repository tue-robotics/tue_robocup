#! /usr/bin/env python

# ROS
import rospy

# Body parts
import base
import torso
import arms
import head
import perception
import ssl

# Human Robot Interaction
import speech
import api
import ears
import ebutton
import lights

# tf
import tf

# Reasoning/world modeling
import world_model_ed

# Misc: do we need this???
import geometry_msgs
from collections import OrderedDict, Sequence

# Check hardware status
from diagnostic_msgs.msg import DiagnosticArray

from robot_skills.util import decorators

class Robot(object):
    """
    Interface to all parts of the robot.
    """
    def __init__(self, robot_name="", wait_services=False):

        self.robot_name = robot_name
        self.tf_listener = tf.TransformListener()

        # Construct body parts
        self.base = base.Base(self.robot_name, self.tf_listener)
        self.torso = torso.Torso(self.robot_name, self.tf_listener)

        # Don't expose specific arms in the robot, they are not generic enough.
        leftArm = arms.Arm(self.robot_name, self.tf_listener, side="left")
        rightArm = arms.Arm(self.robot_name, self.tf_listener, side="right")
        self.arms = OrderedDict(left=leftArm, right=rightArm)

        self.head = head.Head(self.robot_name, self.tf_listener)
        self.perception = perception.Perception(self.robot_name, self.tf_listener)
        self.ssl = ssl.SSL(self.robot_name, self.tf_listener)

        # Human Robot Interaction
        self.lights = lights.Lights(self.robot_name, self.tf_listener)
        self.speech = speech.Speech(self.robot_name, self.tf_listener,
                                             lambda: self.lights.set_color_colorRGBA(lights.SPEAKING),
                                             lambda: self.lights.set_color_colorRGBA(lights.RESET))
        self.hmi = api.Api(self.robot_name, self.tf_listener,
                                    lambda: self.lights.set_color_colorRGBA(lights.LISTENING),
                                    lambda: self.lights.set_color_colorRGBA(lights.RESET))
        self.ears = ears.Ears(self.robot_name, self.tf_listener,
                                       lambda: self.lights.set_color_colorRGBA(lights.LISTENING),
                                       lambda: self.lights.set_color_colorRGBA(lights.RESET))
        self.ears._hmi = self.hmi

        self.ebutton = ebutton.EButton(self.robot_name, self.tf_listener)

        # Reasoning/world modeling
        self.ed = world_model_ed.ED(self.robot_name, self.tf_listener)

        # Construct a map with all robot parts.
        # Arms are copied from the self.arms dict.
        self.parts = {
            'base': self.base,
            'torso': self.torso,

            'head': self.head,
            'perception': self.perception,
            'ssl': self.ssl,

            'lights': self.lights,
            'speech': self.speech,
            'hmi': self.hmi,
            'ears': self.ears,

            'ebutton': self.ebutton,

            'ed': self.ed,
        }
        for arm_name, arm_part in self.arms.iteritems():
            self.parts[arm_name + 'Arm'] = arm_part

        # Ignore diagnostics: parts that are not present in the real robot
        self._ignored_parts = []

        # Miscellaneous
        self.pub_target = rospy.Publisher("/target_location", geometry_msgs.msg.Pose2D, queue_size=10)
        self.base_link_frame = "/"+self.robot_name+"/base_link"

        # Check hardware status
        self._hardware_status_sub = rospy.Subscriber("/" + self.robot_name + "/hardware_status", DiagnosticArray, self.handle_hardware_status)

        # Grasp offsets
        go = rospy.get_param("/"+self.robot_name+"/skills/arm/offset/grasp_offset")
        self.grasp_offset = geometry_msgs.msg.Point(go.get("x"), go.get("y"), go.get("z"))

        # Wait for connections
        s = rospy.Time.now()
        for partname, bodypart in self.parts.iteritems():
            bodypart.wait_for_connections(1.0)
        e = rospy.Time.now()
        rospy.logdebug("Connecting took {} seconds".format((e-s).to_sec()))

        if not self.operational:
            not_operational_parts = [name for name, part in self.parts.iteritems() if not part.operational]
            rospy.logwarn("Not all hardware operational: {parts}".format(parts=not_operational_parts))

        self.laser_topic = "/"+self.robot_name+"/base_laser/scan"

    @property
    @decorators.deprecated_replace_with('robot.get_arm')
    def leftArm(self):
        return self.arms['left']

    @property
    @decorators.deprecated_replace_with('robot.get_arm')
    def rightArm(self):
        return self.arms['right']

    def reset(self):
        results = {}
        for partname, bodypart in self.parts.iteritems():
            rospy.logdebug("Resetting {}".format(partname))
            bodypart.reset()
        return all(results.values())

    def standby(self):
        if not self.robot_name == 'amigo':
            rospy.logerr('Standby only works for amigo')
            return

        for arm in self.arms.itervalues():
            arm.reset()
        for arm in self.arms.itervalues():
            arm.send_gripper_goal('close')

        self.head.look_down()
        self.torso.low()
        self.lights.set_color(0, 0, 0)

    def publish_target(self, x, y):
        self.pub_target.publish(geometry_msgs.msg.Pose2D(x, y, 0))

    def tf_transform_pose(self, ps, frame):
        output_pose = geometry_msgs.msg.PointStamped
        self.tf_listener.waitForTransform(frame, ps.header.frame_id, rospy.Time(), rospy.Duration(2.0))
        output_pose = self.tf_listener.transformPose(frame, ps)
        return output_pose

    def get_arm(self, required_gripper_types=None, desired_gripper_types=None,
                      required_goals=None, desired_goals=None,
                      required_trajectories=None, desired_trajectories=None,
                      required_arm_name=None,
                      required_objects=None, desired_objects=None):
        """
        Find an arm that has the needed and hopefully the desired properties.
        Does not give an arm if all needed properties cannot be satisfied. An
        arm returned by the call will not be able to do anything you didn't ask
        for, except for objects held by it.

        :param required_gripper_types: Collection of gripper types that must all be
                available. None means grippers are not needed.
        :param desired_gripper_types: Collection of gripper types where one or more
                may be selected. None means no grippers are desired.

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

        :param required_objects: Collection of objects that the arm must have. Special
                pseudo-objects arms.ANY_OBJECT and arms.NO_OBJECT may be used
                too in the collection, although they do not make much sense when used
                together with other objects. None means there are no required objects.
        :param desired_objects: Collection of objects that the arm may have. None
                means there are no desired objects.

        :return: An Arm of the robot with the requested properties, or None.
                Note that the arm will never do more than you requested.
        """
        discarded_reasons = [] # Reasons why arms are discarded.

        # Check that collection arguments are really a collection of objects, but not strings.
        # Because then you might accidentally pass a GripperType instead of a [GripperType], which is a List
        def seq_or_none(obj): 
            return not isinstance(obj, str) and (isinstance(obj, Sequence) or obj is None)
        assert seq_or_none(required_gripper_types)
        assert seq_or_none(desired_gripper_types)
        assert seq_or_none(required_goals)
        assert seq_or_none(desired_goals)
        assert seq_or_none(required_trajectories)
        assert seq_or_none(desired_trajectories)
        assert seq_or_none(required_objects)
        assert seq_or_none(desired_objects)

        for arm_name, arm in self.arms.iteritems():
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
                matching_grippers.update(arm.collect_gripper_types(des_type) for des_type in desired_gripper_types)

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

            # Objects
            if not self._check_required_obj(arm, required_objects):
                discarded_reasons.append((arm_name, "required objects failed"))
                continue
            # Desired objects not checked currently.

            # Unlike the other properties, it is likely not exactly known what objects will be used.
            # For this reason, it seems better not to limit objects to the set specified in required
            # _objects and desired_objects.
            uses_objects = (required_objects is not None or desired_objects is not None)

            # Success!
            if not matching_grippers:
                default_gripper = None
            else:
                default_gripper = next(iter(matching_grippers))
            return arms.PublicArm(arm, matching_grippers, default_gripper, uses_objects,
                                  matching_goals, matching_trajectories)

        # No arm matched. Dump why.
        msg = ("Failed to find a matching arm, reasons: " +
               " ".join("{}:{}".format(name, reason) for name, reason in discarded_reasons))
        rospy.logwarn(msg)
        return None

    def _check_required_obj(self, arm, obj_collection):
        """
        Check the object requirement.

        :param arm: Arm to check.
        :param obj_collection: Objects to find. None means the empty requirement
                (arm may have anything, including nothng).
        :return: Whether the requirement holds.
        """
        if obj_collection is None:
            return True

        cur_obj = arm.occupied_by
        for obj in obj_collection:
            if obj == arms.PseudoObjects.ANY: # Any object, but not empty.
                if cur_obj is None:
                    return False
            elif obj == arms.PseudoObjects.EMPTY: # Arm must be empty.
                if cur_obj is not None:
                    return False
            elif obj != cur_obj:
                return False
        return True

    def close(self):
        for partname, bodypart in self.parts.iteritems():
            try:
                bodypart.close()
            except:
                pass

    @property
    def operational(self):
        """
        :returns if all parts are operational"""
        return all(bodypart.operational for bodypart in self.parts.values())

    def handle_hardware_status(self, diagnostic_array):
        """
        hardware_status callback to determine if the bodypart is operational
        :param msg: diagnostic_msgs.msg.DiagnosticArray
        :return: no return
        """

        diagnostic_dict = {diagnostic_status.name:diagnostic_status for diagnostic_status in diagnostic_array.status}

        for name, part in self.parts.iteritems():
            # Pass a dict mapping the name to the item.
            # Bodypart.handle_hardware_status needs to find the element relevant to itself
            # iterating over the array would be done be each bodypart, but with a dict they can just look theirs up.
            if name not in self._ignored_parts:
                part.process_hardware_status(diagnostic_dict)

    def __enter__(self):
        pass

    def __exit__(self, exception_type, exception_val, trace):
        if any((exception_type, exception_val, trace)):
            rospy.logerr("Robot exited with {0},{1},{2}".format(exception_type, exception_val, trace))
        self.close()

def _collect_needs_desires(needs, desires, test_func):
    """
    :param needs: Collection of needed values. None means nothing is needed.
    :param desireds: Collection of desired values, None means nothing is desired.
    :param test_func: Function that takes a value and returns whether the value is available.
    :return: Needed and subset of the desired values, or None if the needs cannor be met.
    """
    founds = set()
    if needs is not None:
        founds.update(_collect_available(needs, test_func))
        if len(founds) != len(needs): # All needs must be met.
            return False

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

if __name__ == "__main__":
    rospy.init_node("robot")

    import doctest
    doctest.testmod()
