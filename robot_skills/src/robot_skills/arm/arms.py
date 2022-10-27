from __future__ import print_function, annotations
from typing import Optional, Union, List, Set, Callable

import math
import time

import PyKDL as kdl
# ROS
import rospy
import visualization_msgs.msg
from actionlib import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pykdl_ros import FrameStamped

# TU/e Robotics
from tue_manipulation_msgs.msg import GraspPrecomputeAction, GraspPrecomputeGoal

from robot_skills.arm.handover_detector import HandoverDetector
from robot_skills.arm.gripper import ParrallelGripper, Gripper, GripperState
from robot_skills.robot_part import RobotPart
from ed.entity import Entity


# Constants for arm requirements. Note that "don't care at all" is not here, as
# it can be expressed by not imposing a requirement (set it to None).


# Commonly used values when sending goals to a joint
MAX_JOINT_VEL = 0.7
JOINT_TIMEOUT = 5.0


# Specific types of gripper.
class GripperTypes(object):
    # Concrete types of gripper.
    PINCH = "gripper-type-pinch"
    PARALLEL = "gripper-type-parallel"
    SUCTION = "gripper-type-suction"

    # Pseudo gripper types.
    GRASPING = "pseudo-gripper-type-any-grasping-will-do"  # Either pinch or parallel
    NONE = "pseudo-gripper-type-no-gripper"


# Pseudo objects for 'any object' or 'no object'.
class PseudoObjects(object):
    ANY = "pseudo-object-saying-any-object-will-do"
    EMPTY = "pseudo-object-saying-lack-of-object-will-do"


class PublicArm(object):
    """
    Public arm interface, also checks a challenge doesn't try to use more than it asked for.

    THE ARM SHOULD NEVER ALLOW ANYTHING THAT WAS NOT ASKED FOR IN THE CONSTRUCTOR!


    :ivar _arm: Private link to the real arm.
    :vartype _arm: Arm

    :ivar default_gripper_type: Gripper type to use if the user didn't provide one.
    :vartpe default_gripper_type: str or None

    :ivar _available_gripper_types: Gripper types that may be used.
    :vartype _available_gripper_types: set of str (the GripperTypes.* constants)

    :ivar _has_occupied_by: Whether the arm supports 'occupied_by' calls.
    :vartype _has_occupied_by: bool

    :ivar _allow_force_sensor: Whether use of the force sensor is allowed.
    :vartype _allow_force_sensor: bool

    :ivar _available_joint_goals: Joint goals that may be used.
    :vartype _available_joint_goals: set of str

    :ivar _available_joint_trajectories: Joint trajectories that may be used.
    :vartype _available_joint_trajectories: set of str

    """
    def __init__(self, arm: Arm, available_gripper_types: Set[str], default_gripper_type: Optional[str],
                 has_occupied_by: bool, allow_force_sensor: bool, available_joint_goals: Set[str],
                 available_joint_trajectories: Set[str]) -> None:
        self._arm = arm
        self.default_gripper_type = default_gripper_type
        self._available_gripper_types = available_gripper_types
        self._has_occupied_by = has_occupied_by
        self._allow_force_sensor = allow_force_sensor
        self._available_joint_goals = available_joint_goals
        self._available_joint_trajectories = available_joint_trajectories

    # Occupied by
    def has_occupied_by(self) -> bool:
        """
        Test whether the arm supports 'occupied_by' calls.
        """
        return self._has_occupied_by

    @property
    def occupied_by(self) -> Entity:
        """
        !Deprecated: use arm.gripper.occupied_by instead!
        Query the object currently held by the arm.
        """
        rospy.logwarn("Deprecation Warning: publicarm.occupied_by is deprecated, use publicarm.gripper.occupied_by instead!")
        self._test_die(self._has_occupied_by, "occupied_by",
                       "Specify get_arm(..., required_objects=[PseudoObjects.EMPTY]) or get_arm(..., required_objects="
                       "[PseudoObjects.ANY]) or get_arm(..., required_objects=[Entity(...)])")
        self._test_die(hasattr(self._arm, 'gripper'), "This arm does not have a gripper")
        return self._arm.gripper.occupied_by

    @occupied_by.setter
    def occupied_by(self, value: Entity) -> None:
        """
        !Deprecated: use arm.gripper.occupied_by instead!
        Set the object currently held by the arm,
        """
        rospy.logwarn("Deprecation Warning: publicarm.occupied_by is deprecated, use publicarm.gripper.occupied_by instead!")
        self._test_die(self._has_occupied_by, "occupied_by",
                       "Specify get_arm(..., required_objects=[PseudoObjects.EMPTY]) or get_arm(..., required_objects="
                       "[PseudoObjects.ANY]) or get_arm(..., required_objects=[Entity(...)])")
        self._test_die(hasattr(self._arm, 'gripper'), "This arm does not have a gripper")
        self._arm.gripper.occupied_by = value

    # Joint goals
    def has_joint_goal(self, configuration: str) -> bool:
        """
        Query whether the provided joint goal exists for the arm.
        """
        return configuration in self._available_joint_goals

    def send_joint_goal(self, configuration: str, timeout: float = JOINT_TIMEOUT,
                        max_joint_vel: Union[int, float, List[int], List[float]] = MAX_JOINT_VEL) -> bool:
        self._test_die(configuration in self._available_joint_goals, 'joint-goal ' + configuration,
                       "Specify get_arm(..., required_goals=['{}'])".format(configuration))
        return self._arm.send_joint_goal(configuration, timeout=timeout,
                                         max_joint_vel=max_joint_vel)

    # Joint trajectories
    def has_joint_trajectory(self, configuration: str) -> bool:
        """
        Query whether the provided joint trajectory exists for the arm.
        """
        return configuration in self._available_joint_trajectories

    def send_joint_trajectory(self, configuration: str, timeout: float = JOINT_TIMEOUT,
                              max_joint_vel: Union[int, float, List[int], List[float]] = MAX_JOINT_VEL) -> bool:
        self._test_die(configuration in self._available_joint_trajectories, 'joint-goal ' + configuration,
                       "Specify get_arm(..., required_trajectories=['{}'])".format(configuration))
        return self._arm.send_joint_trajectory(configuration, timeout=timeout,
                                               max_joint_vel=max_joint_vel)

    def send_goal(self, framestamped: FrameStamped, timeout: float = 30.0, pre_grasp: bool = False,
                  first_joint_pos_only: bool = False, allowed_touch_objects: Optional[List[str]] = None) -> bool:
        if allowed_touch_objects is None:
            allowed_touch_objects = list()
        return self._arm.send_goal(framestamped, timeout, pre_grasp, first_joint_pos_only, allowed_touch_objects)

    # Gripper
    @property
    def gripper(self) -> Gripper:
        self._test_die(hasattr(self._arm, 'gripper'), "This arm does not have a gripper")
        return self._arm.gripper

    # handover
    @property
    def handover_detector(self) -> HandoverDetector:
        self._test_die(hasattr(self._arm, 'handover_detector'), "This arm does not have a handover_detector")
        return self._arm.handover_detector

    def has_gripper_type(self, gripper_type: Optional[str] = None) -> bool:
        """
        Query whether the arm has the provided specific type of gripper.

        :param gripper_type: Optional type of gripper to test.
        """
        if gripper_type is None:
            gripper_type = self.default_gripper_type

        return gripper_type is not None and gripper_type in self._available_gripper_types

    @property
    def has_force_sensor(self) -> bool:
        # Check that the user enabled force sensor access.
        self._test_die(self._allow_force_sensor, 'allow_force_sensor=' + str(self._allow_force_sensor),
                       "Specify get_arm(..., force_sensor_required=True)")
        return hasattr(self._arm, "force_sensor")

    def move_down_until_force_sensor_edge_up(self, timeout: float = 10, retract_distance: float = 0.01,
                                             distance_move_down: Optional[float] = None) -> bool:
        self._test_die(self.has_force_sensor, 'has_force_sensor=' + str(self.has_force_sensor),
                       "Specify get_arm(..., force_sensor_required=True)")
        return self._arm.move_down_until_force_sensor_edge_up(timeout=timeout,
                                                              retract_distance=retract_distance,
                                                              distance_move_down=distance_move_down)

    # salvaged deprecated functionality
    def send_gripper_goal(self, state: str, timeout: float = 5.0, gripper_type: Optional[str] = None,
                          max_torque: float = 0.1) -> bool:
        """
        Tell the gripper to perform a motion.
        :param state: New state (GripperState) of the gripper.
        :param timeout: Amount of time available to reach the goal, default is 5
        :param gripper_type: Optional type of gripper to perform the action.
        :param max_torque: How much torque [Nm] to apply
        :return: success
        """
        rospy.logwarn("Deprication warning: publicarm.send_gripper_goal is deprecated, use publicarm.gripper.send_goal instead!")
        if gripper_type is None:
            gripper_type = self.default_gripper_type

        self._test_die(gripper_type in self._available_gripper_types, 'gripper type ' + str(gripper_type),
                "Specify get_arm(..., required_gripper_types=[GripperTypes.X])")
        # Specified type of gripper currently not used.
        self._test_die(hasattr(self._arm, 'gripper'), "This arm does not have a gripper")
        return self._arm.gripper.send_goal(state, timeout, max_torque=max_torque)

    def handover_to_human(self, timeout: float = 10.0, gripper_type: Optional[str] = None) -> bool:
        rospy.logwarn(
            "Deprication warning: publicarm.handover_to_human is deprecated, use publicarm.handover_detector.handover_to_human instead!")
        if gripper_type is None:
            gripper_type = self.default_gripper_type

        self._test_die(gripper_type in self._available_gripper_types, 'gripper type ' + str(gripper_type),
                "Specify get_arm(..., required_gripper_types=[GripperTypes.X])")
        self._test_die(hasattr(self._arm, 'handover_detector'), "This arm does not have a handover_detector")
        return self._arm.handover_detector.handover_to_human(timeout)

    def handover_to_robot(self, timeout: float = 10, gripper_type: Optional[str] = None) -> bool:
        rospy.logwarn(
            "Deprication warning: publicarm.handover_to_robot is deprecated, use publicarm.handover_detector.handover_to_robot instead!")
        if gripper_type is None:
            gripper_type = self.default_gripper_type

        self._test_die(gripper_type in self._available_gripper_types, 'gripper type ' + str(gripper_type),
               "Specify get_arm(..., required_gripper_types=[GripperTypes.X])")
        self._test_die(hasattr(self._arm, 'handover_detector'), "This arm does not have a handover_detector")
        return self._arm.handover_detector.handover_to_robot(timeout)

    def wait_for_motion_done(self, timeout: float = 10.0, cancel: bool = False,
                             gripper_type: Optional[str] = None) -> bool:
        # Provided gripper type currently ignored.
        return self._arm.wait_for_motion_done(timeout, cancel)

    def cancel_goals(self) -> None:
        """
        Cancels the currently active grasp-precompute and joint-trajectory-action goals
        """
        return self._arm.cancel_goals()

    def close(self) -> None:
        self._arm.close()

    def reset(self) -> bool:
        return self._arm.reset()

    def selfreset(self) -> bool:
        return self._arm.selfreset()

    @property
    def base_offset(self) -> kdl.Vector:
        """
        Retrieves the 'optimal' position of an object w.r.t. the base link of a
        robot for this arm to grasp it.

        :return: Position of an object w.r.t. the base link of a robot.
        """
        return self._arm.base_offset

    def _test_die(self, cond: bool, feature: str, hint: str = ''):
        """
        Test the condition, if it fails, die with an assertion error explaining what is wrong.
        """
        if not cond:
            msg = "get_arm for '{}' arm did not request '{}' access. Hint: {}"
            raise AssertionError(msg.format(self._arm.name, feature, hint))

    def __repr__(self) -> str:
        return "PublicArm(arm={arm})".format(arm=self._arm)


class Arm(RobotPart):
    """
    A kinematic chain ending in an end_effector. Can be controlled using either joint goals or a goal to reach with
    the end_effector described in carthesian coordinates.
    """
    def __init__(self, robot_name: str, tf_buffer: str, get_joint_states: Callable, name: str) -> None:
        """
        constructor

        :param robot_name: robot_name
        :param tf_buffer: tf2_ros.Buffer
        :param get_joint_states: get_joint_states function for getting the last joint states
        :param name: string used to identify the arm
        """
        super(Arm, self).__init__(robot_name=robot_name, tf_buffer=tf_buffer)
        self.name = name

        self._operational = True  # In simulation, there will be no hardware cb

        # Get stuff from the parameter server
        offset = self.load_param('skills/gripper/grasp_offset/')
        self.offset = kdl.Frame(kdl.Rotation.RPY(offset["roll"], offset["pitch"], offset["yaw"]),
                                kdl.Vector(offset["x"], offset["y"], offset["z"]))

        self.marker_to_grippoint_offset = self.load_param('skills/gripper/marker_to_grippoint')

        # Grasp offsets
        go = self.load_param('skills/' + self.name + '/base_offset')
        self._base_offset = kdl.Vector(go["x"], go["y"], go["z"])

        self.joint_names = self.load_param('skills/' + self.name + '/joint_names')
        self.torso_joint_names = self.load_param('skills/torso/joint_names')

        self.default_configurations = self.load_param('skills/' + self.name + '/default_configurations')
        self.default_trajectories = self.load_param('skills/' + self.name + '/default_trajectories')

        self.grasp_frame = self.load_param('skills/gripper/grasp_frame')  #TODO remove gripper specific parameters

        # listen to the hardware status to determine if the arm is available
        self.subscribe_hardware_status(self.name)

        # Init grasp precompute actionlib
        self._ac_grasp_precompute = self.create_simple_action_client(
            "/" + robot_name + "/" + self.name + "/grasp_precompute", GraspPrecomputeAction)

        # Init joint trajectory action server
        self._ac_joint_traj = self.create_simple_action_client(
            "/" + robot_name + "/body/joint_trajectory_action", FollowJointTrajectoryAction)

        # Init marker publisher
        self._marker_publisher = rospy.Publisher(
            "/" + robot_name + "/" + self.name + "/grasp_target",
            visualization_msgs.msg.Marker, queue_size=10)

        self.get_joint_states = get_joint_states

    def collect_gripper_types(self, gripper_type: str) -> List[str]:
        """
        Query the arm for having the proper gripper type and collect the types that fulfill the
        requirement.

        :param gripper_type: Wanted type of the gripper. May be a pseudo gripper type.
        :return: Collection gripper types at the arm that meet the requirements.
        """
        #TODO move this function outside of arms.py
        if gripper_type == GripperTypes.NONE:
            # There are no arms without a gripper.
            return []
        if gripper_type == GripperTypes.GRASPING:
            return (self._has_specific_gripper_types(GripperTypes.PINCH) +
                    self._has_specific_gripper_types(GripperTypes.PARALLEL))
        return self._has_specific_gripper_types(gripper_type)

    def _has_specific_gripper_types(self, gripper_type: str) -> List[str]:
        """
        Verify whether the arm as the given type of specific gripper.

        :param gripper_type: Type of gripper to check for. Must not be a pseudo gripper type.
        :return: Gripper types that match the requirement.
        """
        #TODO move this function outside of arms.py
        if not hasattr(self, 'gripper'):
            rospy.logerr("This arm does not have a 'gripper' ")

        # TODO: Extend grippers to have knowledge about the gripper type that it has.
        if gripper_type == GripperTypes.PINCH and isinstance(self.gripper, ParrallelGripper):
            return [GripperTypes.PARALLEL]
        elif gripper_type == GripperTypes.PARALLEL and isinstance(self.gripper, ParrallelGripper):
            return [GripperTypes.PARALLEL]
        elif gripper_type == GripperTypes.SUCTION:
            return []
        else:
            return []  # Arm has no unknown types of grippers,

    def has_joint_goal(self, configuration: str) -> bool:
        """
        Query the arm for having a given joint goal.

        :param configuration: name of jint goal to check.
        :return: Whether the joint goal is available.
        """
        return configuration in self.default_configurations

    def has_joint_trajectory(self, configuration: str) -> bool:
        """
        Query the arm for having a given joint trajectory.

        :param configuration: name of jint trajectory to check.
        :return: Whether the joint trajectory is available.
        """
        return configuration in self.default_trajectories

    def cancel_goals(self) -> None:
        """
        Cancels the currently active grasp-precompute and joint-trajectory-action goals
        """
        self._ac_grasp_precompute.cancel_all_goals()
        self._ac_joint_traj.cancel_all_goals()

    def close(self) -> None:
        """
        Cancels all active goals for the arm and the gripper
        """
        try:
            rospy.loginfo("{0} arm cancelling all goals on all arm-related ACs on close".format(self.name))
        except AttributeError:
            print("{0} arm cancelling all goals on all arm-related ACs on close. rospy is already deleted.".
                  format(self.name))

        self._ac_grasp_precompute.cancel_all_goals()
        self._ac_joint_traj.cancel_all_goals()

    def send_goal(self, frameStamped: FrameStamped, timeout: float = 30.0, pre_grasp: bool = False,
                  first_joint_pos_only: bool = False, allowed_touch_objects: List[str] = None) -> bool:
        """
        Send a arm to a goal:

        Using a combination of position and orientation: a kdl.Frame. A time
        out time_out. pre_grasp means go to an offset that is normally needed
        for things such as grasping. You can also specify the frame_id which
        defaults to base_link

        :param frameStamped: A FrameStamped to move the arm's end effector to
        :param timeout: timeout in seconds; In case of 0.0, goal is executed without feedback and waiting
        :param pre_grasp: Bool to use pre_grasp or not
        :param first_joint_pos_only: Bool to only execute first joint position of whole trajectory
        :param allowed_touch_objects: List of object names in the worldmodel, which are allowed to be touched
        :return: True of False
        """
        if allowed_touch_objects is None:
            allowed_touch_objects = list()

        # save the arguments for debugging later
        myargs = locals()

        # Convert to baselink, which is needed because the offset is defined in the base_link frame
        frame_in_baselink = self.tf_buffer.transform(frameStamped, self.robot_name + "/base_link")

        self._publish_marker(frameStamped, [1, 0, 0], "grasp_point")

        end_effector_frame = frame_in_baselink.frame * self.offset

        # TODO: Get rid of this custom message type
        # Create goal:
        grasp_precompute_goal = GraspPrecomputeGoal()
        grasp_precompute_goal.goal.header.frame_id = frame_in_baselink.header.frame_id
        grasp_precompute_goal.goal.header.stamp = rospy.Time.now()

        grasp_precompute_goal.PERFORM_PRE_GRASP = pre_grasp
        grasp_precompute_goal.FIRST_JOINT_POS_ONLY = first_joint_pos_only

        grasp_precompute_goal.allowed_touch_objects = allowed_touch_objects

        grasp_precompute_goal.goal.x = end_effector_frame.p.x()
        grasp_precompute_goal.goal.y = end_effector_frame.p.y()
        grasp_precompute_goal.goal.z = end_effector_frame.p.z()

        roll, pitch, yaw = end_effector_frame.M.GetRPY()
        grasp_precompute_goal.goal.roll = roll
        grasp_precompute_goal.goal.pitch = pitch
        grasp_precompute_goal.goal.yaw = yaw

        time.sleep(0.001)   # This is necessary: the rtt_actionlib in the hardware seems
                            # to only have a queue size of 1 and runs at 1000 hz. This
                            # means that if two goals are send approximately at the same
                            # time (e.g. an arm goal and a torso goal), one of the two
                            # goals probably won't make it. This sleep makes sure the
                            # goals will always arrive in different update hooks in the
                            # hardware TrajectoryActionLib server.

        # Send goal:

        if timeout == 0.0:
            self._ac_grasp_precompute.send_goal(grasp_precompute_goal)
            return True
        else:
            result = self._ac_grasp_precompute.send_goal_and_wait(
                grasp_precompute_goal,
                execute_timeout=rospy.Duration(timeout))
            if result == GoalStatus.SUCCEEDED:
                result_pose = self.tf_buffer.lookup_transform(self.robot_name + "/base_link",
                                                              self.grasp_frame,
                                                              rospy.Time(0))
                dx = frame_in_baselink.frame.p.x() - result_pose.transform.translation.x
                dy = frame_in_baselink.frame.p.y() - result_pose.transform.translation.y
                dz = frame_in_baselink.frame.p.z() - result_pose.transform.translation.z

                if abs(dx) > 0.005 or abs(dy) > 0.005 or abs(dz) > 0.005:
                    rospy.logwarn("Grasp-precompute error too large: [{}, {}, {}]".format(
                                  dx, dy, dz))
                return True
            else:
                # failure
                rospy.logerr('grasp precompute goal failed: \n%s', repr(myargs))
                return False

    def send_joint_goal(self, configuration: str, timeout: float = JOINT_TIMEOUT,
                        max_joint_vel: Union[int, float, List[int], List[float]] = MAX_JOINT_VEL) -> bool:
        """
        Send a named joint goal (pose) defined in the parameter default_configurations to the arm
        :param configuration: Name of configuration, configuration should be loaded as parameter
        :param timeout: Timeout in seconds
        :param max_joint_vel: Speed the robot can have when getting to the desired configuration
        :return: True or False, False in case of nonexistent configuration or failed execution
        """
        if configuration in self.default_configurations:
            return self._send_joint_trajectory([self.default_configurations[configuration]],
                                               timeout=rospy.Duration.from_sec(timeout),
                                               max_joint_vel=max_joint_vel)
        else:
            rospy.logwarn('Default configuration {0} does not exist'.format(configuration))
            return False

    def send_joint_trajectory(self, configuration: str, timeout: float = JOINT_TIMEOUT,
                              max_joint_vel: Union[int, float, List[int], List[float]] = MAX_JOINT_VEL) -> bool:
        """
        Send a named joint trajectory (sequence of poses) defined in the default_trajectories to the arm

        :param configuration: Name of configuration, configuration should be loaded as parameter
        :param timeout: Timeout in seconds
        :param max_joint_vel: Speed the robot can have when getting to the desired configuration
        :return: True or False, False in case of nonexistent configuration or failed execution
        """
        if configuration in self.default_trajectories:
            return self._send_joint_trajectory(self.default_trajectories[configuration],
                                               timeout=rospy.Duration.from_sec(timeout),
                                               max_joint_vel=max_joint_vel)
        else:
            rospy.logwarn('Default trajectories {0} does not exist'.format(configuration))
            return False

    def selfreset(self) -> bool:
        """
        Put the arm into the 'reset' pose

        :return: True or False
        """
        return self.send_joint_goal('reset', timeout=0.0)

    def _send_joint_trajectory(self, joints_references: List[str],
                               max_joint_vel: Union[int, float, List[int], List[float]] = MAX_JOINT_VEL,
                               timeout: float = JOINT_TIMEOUT) -> bool:
        """
        Low level method that sends a array of joint references to the arm.

        If timeout is defined, it will wait for timeout*len(joints_reference) seconds for the
        completion of the actionlib goal. It will return True as soon as possible when the goal
        succeeded. On timeout, it will return False.

        :param joints_references: List of joint configurations,
            which should be a list of the length equal to the number of joints to be moved
        :param max_joint_vel: Speed the robot can have when getting to the desired
            configuration. A single value can be given, which will be used for all joints, or a list of values can be
            given in which the order has to agree with the joints according to the joints_references.
        :param timeout: Timeout for each joint configuration in rospy.Duration(seconds); timeout of 0.0 is not
            allowed
        :return: True or False
        """
        timeout = rospy.Duration(timeout)

        if not joints_references:
            return False

        if len(joints_references[0]) == len(self.joint_names) + len(self.torso_joint_names):
            joint_names = self.torso_joint_names + self.joint_names
        else:
            joint_names = self.joint_names

        if isinstance(max_joint_vel, (float, int)):
            max_joint_vel = [max_joint_vel]*len(joint_names)

        if isinstance(max_joint_vel, list):
            if isinstance(max_joint_vel[0], (int, float)):
                if len(max_joint_vel) is not len(joint_names):
                    rospy.logerr("The length of 'max_joint_vel' is {} and the length of 'joint_names' is {}. \n"
                                 "Please give the velocities for the following joints (in the correct order!): {}"
                                 .format(len(max_joint_vel), len(joint_names), joint_names))

        ps = []
        time_from_start = 0.0
        start_joint_state = self.get_joint_states()
        prev_joint_ref = [start_joint_state[jn] for jn in joint_names]
        for joints_reference in joints_references:
            max_diff = [abs(prev - new) for prev, new in zip(prev_joint_ref, joints_reference)]
            if len(joints_reference) != len(joint_names):
                rospy.logwarn('Please use the correct {} number of joint references (current = {})'
                              .format(len(joint_names), len(joints_references)))
            time_from_start += max(x/y for x, y in zip(max_diff, max_joint_vel))
            ps.append(JointTrajectoryPoint(
                positions=joints_reference,
                time_from_start=rospy.Duration.from_sec(time_from_start)))

        joint_trajectory = JointTrajectory(joint_names=joint_names,
                                           points=ps)
        goal = FollowJointTrajectoryGoal(trajectory=joint_trajectory, goal_time_tolerance=timeout)

        rospy.logdebug("Send {0} arm to jointcoords \n{1}".format(self.name, ps))

        import time; time.sleep(0.001)  # This is necessary: the rtt_actionlib in the hardware seems
                                        # to only have a queue size of 1 and runs at 1000 hz. This
                                        # means that if two goals are send approximately at the same
                                        # time (e.g. an arm goal and a torso goal), one of the two
                                        # goals probably won't make it. This sleep makes sure the
                                        # goals will always arrive in different update hooks in the
                                        # hardware TrajectoryActionLib server.
        self._ac_joint_traj.send_goal(goal)
        if timeout != rospy.Duration(0):
            done = self._ac_joint_traj.wait_for_result(timeout*len(joints_references))
            if not done:
                rospy.logwarn("Cannot reach joint goal {0}".format(goal))
            return done
        else:
            return False

    def wait_for_motion_done(self, timeout: float = 10.0, cancel: bool = False) -> bool:
        """
        Waits until all action clients are done

        :param timeout: timeout in seconds; in case 0.0, no sensible output is provided, just False
        :param cancel: bool specifying whether goals should be cancelled
            if timeout is exceeded
        :return: bool indicates whether motion was done (True if reached, False otherwise)
        """
        # rospy.loginfo('Waiting for ac_joint_traj')
        starttime = rospy.Time.now()
        if self._ac_joint_traj.gh:
            if not self._ac_joint_traj.wait_for_result(rospy.Duration(timeout)):
                if cancel:
                    rospy.loginfo("Arms: cancelling all goals (1)")
                    self.cancel_goals()

        passed_time = (rospy.Time.now() - starttime).to_sec()
        if passed_time > timeout:
            return False

        # rospy.loginfo('Waiting for ac_grasp_precompute')
        if self._ac_grasp_precompute.gh:
            if not self._ac_grasp_precompute.wait_for_result(rospy.Duration(timeout-passed_time)):
                if cancel:
                    rospy.loginfo("Arms: cancelling all goals (2)")
                    self.cancel_goals()
        return True

    @property
    def base_offset(self) -> kdl.Vector:
        """
        Retrieves the 'optimal' position of an object w.r.t. the base link of a
        robot for this arm to grasp it.

        :return: Position of an object w.r.t. the base link of a robot.
        """
        return self._base_offset

    def _publish_marker(self, goal: FrameStamped, color: List[float], ns: str = "") -> None:
        """
        Publish markers for visualisation
        :param goal: frame_stamped
        :param color: list of rgb colors (0.0-1.0)
        :param ns: namespace
        """
        marker = visualization_msgs.msg.Marker()
        marker.header.frame_id = goal.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.type = 2
        marker.pose.position.x = goal.frame.p.x()
        marker.pose.position.y = goal.frame.p.y()
        marker.pose.position.z = goal.frame.p.z()
        marker.lifetime = rospy.Duration(20.0)
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.ns = ns

        marker.color.a = 1
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]

        self._marker_publisher.publish(marker)

    def __repr__(self):
        return "Arm(name='{}')".format(self.name)
