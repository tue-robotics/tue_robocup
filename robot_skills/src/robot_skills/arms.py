import time

# ROS
import rospy
import std_msgs.msg
import PyKDL as kdl

import visualization_msgs.msg
from actionlib import GoalStatus
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from robot_skills.force_sensor import ForceSensor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# TU/e Robotics
from tue_manipulation_msgs.msg import GraspPrecomputeGoal, GraspPrecomputeAction
from tue_manipulation_msgs.msg import GripperCommandGoal, GripperCommandAction
from tue_msgs.msg import GripperCommand

from robot_skills.robot_part import RobotPart

# Constants for arm requirements. Note that "don't care at all" is not here, as
# it can be expressed by not imposing a requirement (set it to None).


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

    :ivar _arm: Private link to the real arm.
    :vartype _arm: Arm

    :ivar default_gripper_type: Gripper type to use if the user didn't provide one.
    :vartpe default_gripper_type: str or None

    :ivar _available_gripper_types: Gripper types that may be used.
    :vartype _available_gripper_types: set of str (the GripperTypes.* constants)

    :ivar _has_occupied_by: Whether the arm supports 'occupied_by' calls.
    :vartype _has_occupied_by: bool

    :ivar _available_joint_goals: Joint goals that may be used.
    :vartype _available_joint_goals: set of str

    :ivar _available_joint_trajectories: Joint trajectories that may be used.
    :vartype _available_joint_trajectories: set of str

    """
    def __init__(self, arm, available_gripper_types, default_gripper_type,
                 has_occupied_by, available_joint_goals, available_joint_trajectories):
        self._arm = arm
        self.default_gripper_type = default_gripper_type
        self._available_gripper_types = available_gripper_types
        self._has_occupied_by = has_occupied_by
        self._available_joint_goals = available_joint_goals
        self._available_joint_trajectories = available_joint_trajectories

    # Occupied by
    def has_occupied_by(self):
        """
        Test whether the arm supports 'occupied_by' calls.
        """
        return self._has_occupied_by

    @property
    def occupied_by(self):
        """
        Query the object currently held by the arm.
        """
        self._test_die(self._has_occupied_by, "occupied_by",
                       "Specify get_arm(..., required_objects=[PseudoObjects.EMPTY]) or get_arm(..., required_objects="
                       "[PseudoObjects.ANY]) or get_arm(..., required_objects=[Entity(...)])")
        return self._arm.occupied_by

    @occupied_by.setter
    def occupied_by(self, value):
        """
        Set the object currently held by the arm,
        """
        self._test_die(self._has_occupied_by, "occupied_by",
                       "Specify get_arm(..., required_objects=[PseudoObjects.EMPTY]) or get_arm(..., required_objects="
                       "[PseudoObjects.ANY]) or get_arm(..., required_objects=[Entity(...)])")
        self._arm.occupied_by = value

    # Joint goals
    def has_joint_goal(self, configuration):
        """
        Query whether the provided joint goal exists for the arm.
        """
        return configuration in self._available_joint_goals

    def send_joint_goal(self, configuration, timeout=5.0, max_joint_vel=0.7):
        self._test_die(configuration in self._available_joint_goals, 'joint-goal ' + configuration,
                       "Specify get_arm(..., required_goals=['{}'])".format(configuration))
        return self._arm.send_joint_goal(configuration, timeout=timeout,
                                         max_joint_vel=max_joint_vel)

    # Joint trajectories
    def has_joint_trajectory(self, configuration):
        """
        Query whether the provided joint trajectory exists for the arm.
        """
        return configuration in self._available_joint_trajectories

    def send_joint_trajectory(self, configuration, timeout=5, max_joint_vel=0.7):
        self._test_die(configuration in self._available_joint_trajectories, 'joint-goal ' + configuration,
                       "Specify get_arm(..., required_trajectories=['{}'])".format(configuration))
        return self._arm.send_joint_trajectory(configuration, timeout=timeout,
                                               max_joint_vel=max_joint_vel)

    def send_goal(self, frameStamped, timeout=30, pre_grasp=False, first_joint_pos_only=False,
                  allowed_touch_objects=None):
        if allowed_touch_objects is None:
            allowed_touch_objects = list()
        return self._arm.send_goal(frameStamped, timeout, pre_grasp, first_joint_pos_only, allowed_touch_objects)

    # Gripper
    def has_gripper_type(self, gripper_type=None):
        """
        Query whether the arm has the provided specific type of gripper.

        :param gripper_type: Optional type of gripper to test.
        :type  gripper_type: str or None
        """
        if gripper_type is None:
            gripper_type = self.default_gripper_type

        return gripper_type is not None and gripper_type in self._available_gripper_types

    def send_gripper_goal(self, state, timeout=5.0, gripper_type=None, max_torque=0.1):
        """
        Tell the gripper to perform a motion.

        :param state: New state of the gripper.
        :type state: str (GripperState)
        :param timeout: Amount of time available to reach the goal, default is 5
        :type timeout: float
        :param gripper_type: Optional type of gripper to perform the action.
        :type gripper_type: str
        :param max_torque: How much torque [Nm] to apply
        :return: succes
        :rtype: bool
        """
        if gripper_type is None:
            gripper_type = self.default_gripper_type

        self._test_die(gripper_type in self._available_gripper_types, 'gripper type ' + str(gripper_type),
                       "Specify get_arm(..., required_gripper_types=[GripperTypes.X])")
        # Specified type of gripper currently not used.
        return self._arm.send_gripper_goal(state, timeout, max_torque=max_torque)

    @property
    def has_force_sensor(self):
        return hasattr(self._arm, "force_sensor")

    def move_down_until_force_sensor_edge_up(self, timeout=10, retract_distance=0.01):
        self._test_die(self.has_force_sensor, 'available_force_sensor=' + str(self.has_force_sensor),
                       "Specify get_arm(..., available_force_sensor=True")
        return self._arm.move_down_until_force_sensor_edge_up(timeout=timeout, retract_distance=retract_distance)

    def handover_to_human(self, timeout=10, gripper_type=None):
        if gripper_type is None:
            gripper_type = self.default_gripper_type

        self._test_die(gripper_type in self._available_gripper_types, 'gripper type ' + str(gripper_type),
                       "Specify get_arm(..., required_gripper_types=[GripperTypes.X])")
        return self._arm.handover_to_human(timeout)

    def handover_to_robot(self, timeout=10, gripper_type=None):
        if gripper_type is None:
            gripper_type = self.default_gripper_type

        self._test_die(gripper_type in self._available_gripper_types, 'gripper type ' + str(gripper_type),
                       "Specify get_arm(..., required_gripper_types=[GripperTypes.X])")
        return self._arm.handover_to_robot(timeout)

    def wait_for_motion_done(self, timeout=10.0, cancel=False, gripper_type=None):
        # Provided gripper type currently ignored.
        return self._arm.wait_for_motion_done(timeout, cancel)

    def cancel_goals(self):
        """
        Cancels the currently active grasp-precompute and joint-trajectory-action goals
        :return: no return
        """
        return self._arm.cancel_goals()

    def close(self):
        self._arm.close()

    def reset(self, timeout=0.0):
        return self._arm.reset(timeout)

    @property
    def base_offset(self):
        """
        Retrieves the 'optimal' position of an object w.r.t. the base link of a
        robot for this arm to grasp it.

        :return: Position of an object w.r.t. the base link of a robot.
        :rtype: kdl Vector
        """
        return self._arm.base_offset

    def _test_die(self, cond, feature, hint=''):
        """
        Test the condition, if it fails, die with an assertion error explaining what is wrong.
        """
        if not cond:
            msg = "get_arm for '{}' arm did not request '{}' access. Hint: {}"
            raise AssertionError(msg.format(self._arm.side, feature, hint))

    def __repr__(self):
        return "PublicArm(arm={arm})".format(arm=self._arm)


class GripperMeasurement(object):
    """
    Class holding measurements from the distance sensor on the grippers
    """
    EMPTY = -1
    UNKNOWN = 0
    HOLDING = 1

    def __init__(self, distance):
        """
        Constructor

        :param distance: float with measured distance
        """
        self._distance = distance
        self._stamp = rospy.Time.now()

        # If the grasp sensor distance is smaller than this value, the gripper is holding an object
        self.GRASP_SENSOR_THRESHOLD = rospy.get_param("skills/arm/grasp_sensor/threshold", 0.1)
        self.GRASP_SENSOR_TIMEOUT = rospy.get_param("skills/arm/grasp_sensor/timeout", 0.5)
        self.GRASP_SENSOR_LIMITS = tuple(rospy.get_param("skills/arm/grasp_sensor/limits", [0.0025, 0.18]))

    def _is_recent(self):
        """
        Checks if the sensor data is recent

        :return: bool True if recent, i.e., measurement is less than GRASP_SENSOR_TIMEOUT old, False otherwise
        """
        return (rospy.Time.now() - self._stamp).to_sec() < self.GRASP_SENSOR_TIMEOUT

    @property
    def distance(self):
        """
        Returns the measured distance. If the measurement is too old or the distance is outside of of the provided
        limits, NaN is returned

        :return: float with distance if valid, NaN otherwise
        """
        # Check if data is recent
        if not self._is_recent():
            return float('nan')
        elif not self.GRASP_SENSOR_LIMITS[0] < self._distance < self.GRASP_SENSOR_LIMITS[1]:
            return float('nan')
        else:
            return self._distance

    @property
    def is_holding(self):
        """
        Returns if the gripper is holding anything based on the measurement, i.e., if the measurement is recent and
        the value is between the lower limit and the sensor threshold

        :return: bool if holding
        """
        return self._is_recent() and self.GRASP_SENSOR_LIMITS[0] < self._distance < self.GRASP_SENSOR_THRESHOLD

    @property
    def is_unknown(self):
        """
        Returns if the state is unknown, i.e., either the measurement is outdated or the distance is less than the
        limit

        :return: bool if unknown
        """
        return not self._is_recent() or self._distance < self.GRASP_SENSOR_LIMITS[0]

    @property
    def is_empty(self):
        """
        Returns if the gripper is empty, i.e., the measurement is recent and the value is greater than the threshold

        :return: bool if holding
        """
        return self._is_recent() and self._distance > self.GRASP_SENSOR_THRESHOLD

    def __repr__(self):
        return "Distance: {}, is_holding: {}, is_unknown: {}, " \
               "is_empty: {}".format(self.distance, self.is_holding, self.is_unknown, self.is_empty)


class GripperState(object):
    """
    Specifies a State either OPEN or CLOSE
    """
    OPEN = "open"
    CLOSE = "close"


class Arm(RobotPart):
    """
    A single arm can be either left or right, extends Arms:
    Use left or right to get arm while running from the python console

    Examples:
    >>> left.send_goal(0.265, 1, 0.816, 0, 0, 0, 60)  # doctest: +SKIP
    or Equivalently:
    >>> left.send_goal(px=0.265, py=1, pz=0.816, yaw=0, pitch=0, roll=0, time_out=60, pre_grasp=False, frame_id='/amigo/base_link')  # doctest: +SKIP

    #To open left gripper
    >>> left.send_gripper_goal_open(10)  # doctest: +SKIP
    """
    def __init__(self, robot_name, tf_listener, get_joint_states, side):
        """
        constructor

        :param robot_name: robot_name
        :param tf_listener: tf_server.TFClient()
        :param get_joint_states: get_joint_states function for getting the last joint states
        :param side: left or right
        """
        super(Arm, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self.side = side

        self._occupied_by = None

        self._operational = True  # In simulation, there will be no hardware cb

        # Get stuff from the parameter server
        offset = self.load_param('skills/arm/' + self.side + '/grasp_offset/')
        self.offset = kdl.Frame(kdl.Rotation.RPY(offset["roll"], offset["pitch"], offset["yaw"]),
                                kdl.Vector(offset["x"], offset["y"], offset["z"]))

        self.marker_to_grippoint_offset = self.load_param('skills/arm/' + self.side + '/marker_to_grippoint')

        # Grasp offsets
        go = self.load_param('skills/arm/' + self.side + '/base_offset')
        self._base_offset = kdl.Vector(go["x"], go["y"], go["z"])

        self.joint_names = self.load_param('skills/arm/joint_names')
        self.joint_names = [name + "_" + self.side for name in self.joint_names]
        self.torso_joint_names = self.load_param('skills/torso/joint_names')

        self.default_configurations = self.load_param('skills/arm/default_configurations')
        self.default_trajectories   = self.load_param('skills/arm/default_trajectories')

        # listen to the hardware status to determine if the arm is available
        self.subscribe_hardware_status(self.side + '_arm')

        # Init gripper actionlib
        self._ac_gripper = self.create_simple_action_client(
            "/" + robot_name + "/" + self.side + "_arm/gripper/action", GripperCommandAction)

        # Init grasp precompute actionlib
        self._ac_grasp_precompute = self.create_simple_action_client(
            "/" + robot_name + "/" + self.side + "_arm/grasp_precompute", GraspPrecomputeAction)

        # Init joint trajectory action server
        self._ac_joint_traj = self.create_simple_action_client(
            "/" + robot_name + "/body/joint_trajectory_action", FollowJointTrajectoryAction)

        # Init grasp sensor subscriber
        self._grasp_sensor_state = GripperMeasurement(0.0)
        rospy.Subscriber("/" + self.robot_name + "/" + self.side + "_arm/proximity_sensor",
                         std_msgs.msg.Float32MultiArray, self._grasp_sensor_callback)

        # Init marker publisher
        self._marker_publisher = rospy.Publisher(
            "/" + robot_name + "/" + self.side + "_arm/grasp_target",
            visualization_msgs.msg.Marker, queue_size=10)

        self.get_joint_states = get_joint_states

    def collect_gripper_types(self, gripper_type):
        """
        Query the arm for having the proper gripper type and collect the types that fulfill the
        requirement.

        :param gripper_type: Wanted type of the gripper. May be a pseudo gripper type.
        :return: Collection gripper types at the arm that meet the requirements.
        """
        if gripper_type == GripperTypes.NONE:
            # There are no arms without a gripper.
            return []
        if gripper_type == GripperTypes.GRASPING:
            return (self._has_specific_gripper_types(GripperTypes.PINCH) +
                    self._has_specific_gripper_types(GripperTypes.PARALLEL))
        return self._has_specific_gripper_types(gripper_type)

    @staticmethod
    def _has_specific_gripper_types(gripper_type):
        """
        Verify whether the arm as the given type of specific gripper.

        :param gripper_type: Type of gripper to check for. Must not be a pseudo gripper type.
        :return: Gripper types that match the requirement.
        """
        # TODO: Extend arm to have knowledge about the gripper type that it has.
        if gripper_type == GripperTypes.PINCH:
            return [GripperTypes.PINCH]
        elif gripper_type == GripperTypes.PARALLEL:
            return [GripperTypes.PARALLEL]
        elif gripper_type == GripperTypes.SUCTION:
            return []
        else:
            return []  # Arm has no unknown types of grippers,

    def has_joint_goal(self, configuration):
        """
        Query the arm for having a given joint goal.

        :param configuration: name of jint goal to check.
        :return: Whether the joint goal is available.
        """
        return configuration in self.default_configurations

    def has_joint_trajectory(self, configuration):
        """
        Query the arm for having a given joint trajectory.

        :param configuration: name of jint trajectory to check.
        :return: Whether the joint trajectory is available.
        """
        return configuration in self.default_trajectories

    def cancel_goals(self):
        """
        Cancels the currently active grasp-precompute and joint-trajectory-action goals
        :return: no return
        """
        self._ac_grasp_precompute.cancel_all_goals()
        self._ac_joint_traj.cancel_all_goals()

    def close(self):
        """
        Cancels all active goals for the arm and the gripper
        :return: no return
        """
        try:
            rospy.loginfo("{0} arm cancelling all goals on all arm-related ACs on close".format(self.side))
        except AttributeError:
            print("{0} arm cancelling all goals on all arm-related ACs on close. rospy is already deleted.".
                  format(self.side))

        self._ac_gripper.cancel_all_goals()
        self._ac_grasp_precompute.cancel_all_goals()
        self._ac_joint_traj.cancel_all_goals()

    def send_goal(self, frameStamped, timeout=30, pre_grasp=False, first_joint_pos_only=False,
                  allowed_touch_objects=None):
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

        # If necessary, prefix frame_id
        if frameStamped.frame_id.find(self.robot_name) < 0:
            frameStamped.frame_id = "/"+self.robot_name+"/"+frameStamped.frame_id
            rospy.loginfo("Grasp precompute frame id = {0}".format(frameStamped.frame_id))

        # Convert to baselink, which is needed because the offset is defined in the base_link frame
        frame_in_baselink = frameStamped.projectToFrame("/"+self.robot_name+"/base_link", self.tf_listener)

        # TODO: Get rid of this custom message type
        # Create goal:
        grasp_precompute_goal = GraspPrecomputeGoal()
        grasp_precompute_goal.goal.header.frame_id = frame_in_baselink.frame_id
        grasp_precompute_goal.goal.header.stamp = rospy.Time.now()

        grasp_precompute_goal.PERFORM_PRE_GRASP = pre_grasp
        grasp_precompute_goal.FIRST_JOINT_POS_ONLY = first_joint_pos_only

        grasp_precompute_goal.allowed_touch_objects = allowed_touch_objects

        grasp_precompute_goal.goal.x = frame_in_baselink.frame.p.x()
        grasp_precompute_goal.goal.y = frame_in_baselink.frame.p.y()
        grasp_precompute_goal.goal.z = frame_in_baselink.frame.p.z()

        roll, pitch, yaw = frame_in_baselink.frame.M.GetRPY()
        grasp_precompute_goal.goal.roll  = roll
        grasp_precompute_goal.goal.pitch = pitch
        grasp_precompute_goal.goal.yaw   = yaw

        self._publish_marker(grasp_precompute_goal, [1, 0, 0], "grasp_point")

        # Add tunable parameters
        offset_frame = frame_in_baselink.frame * self.offset

        grasp_precompute_goal.goal.x = offset_frame.p.x()
        grasp_precompute_goal.goal.y = offset_frame.p.y()
        grasp_precompute_goal.goal.z = offset_frame.p.z()

        roll, pitch, yaw = frame_in_baselink.frame.M.GetRPY()
        grasp_precompute_goal.goal.roll  = roll
        grasp_precompute_goal.goal.pitch = pitch
        grasp_precompute_goal.goal.yaw   = yaw

        # rospy.loginfo("Arm goal: {0}".format(grasp_precompute_goal))

        self._publish_marker(grasp_precompute_goal, [0, 1, 0], "grasp_point_corrected")

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

                result_pose = self.tf_listener.lookupTransform(self.robot_name + "/base_link",
                                                               self.robot_name + "/grippoint_{}".format(self.side),
                                                               rospy.Time(0))
                dx = grasp_precompute_goal.goal.x - result_pose[0][0]
                dy = grasp_precompute_goal.goal.y - result_pose[0][1]
                dz = grasp_precompute_goal.goal.z - result_pose[0][2]

                if abs(dx) > 0.005 or abs(dy) > 0.005 or abs(dz) > 0.005:
                    rospy.logwarn("Grasp-precompute error too large: [{}, {}, {}]".format(
                                  dx, dy, dz))
                return True
            else:
                # failure
                rospy.logerr('grasp precompute goal failed: \n%s', repr(myargs))
                return False

    def send_joint_goal(self, configuration, timeout=5.0, max_joint_vel=0.7):
        """
        Send a named joint goal (pose) defined in the parameter default_configurations to the arm
        :param configuration:(str) name of configuration, configuration should be loaded as parameter
        :param timeout:(secs) timeout in seconds
        :param max_joint_vel:(int,float,[int]) speed the robot can have when getting to the desired configuration
        :return: True or False, False in case of nonexistent configuration or failed execution
        """
        if configuration in self.default_configurations:
            return self._send_joint_trajectory([self.default_configurations[configuration]],
                                               timeout=rospy.Duration.from_sec(timeout),
                                               max_joint_vel=max_joint_vel)
        else:
            rospy.logwarn('Default configuration {0} does not exist'.format(configuration))
            return False

    def send_joint_trajectory(self, configuration, timeout=5.0, max_joint_vel=0.7):
        """
        Send a named joint trajectory (sequence of poses) defined in the default_trajectories to the arm

        :param configuration:(str) name of configuration, configuration should be loaded as parameter
        :param timeout:(secs) timeout in seconds
        :param max_joint_vel:(int,float,[int]) speed the robot can have when getting to the desired configuration
        :return: True or False, False in case of nonexistent configuration or failed execution
        """
        if configuration in self.default_trajectories:
            return self._send_joint_trajectory(self.default_trajectories[configuration],
                                               timeout=rospy.Duration.from_sec(timeout),
                                               max_joint_vel=max_joint_vel)
        else:
            rospy.logwarn('Default trajectories {0} does not exist'.format(configuration))
            return False

    def reset(self, timeout=0.0):
        """
        Put the arm into the 'reset' pose

        :param timeout: timeout in seconds
        :return: True or False
        """
        return self.send_joint_goal('reset', timeout=timeout) and self.send_gripper_goal('close', 0.0)

    @property
    def occupied_by(self):
        """
        The 'occupied_by' property will return the current entity that is in the gripper of this arm.

        :return: robot_skills.util.entity, ED entity
        """

        return self._occupied_by

    @occupied_by.setter
    def occupied_by(self, value):
        """
        Set the entity which occupies the arm.

        :param value: robot_skills.util.entity, ED entity
        :return: no return
        """
        self._occupied_by = value

    def send_gripper_goal(self, state, timeout=5.0, max_torque=0.1):
        """
        Send a GripperCommand to the gripper of this arm and wait for finishing

        :param state: open or close
        :type state: str (GripperState)
        :param timeout: timeout in seconds; timeout of 0.0 is not allowed
        :type timeout: float
        :param max_torque: How much torque [Nm] to apply, only applied when closing the gripper
        :return: True of False
        :rtype: bool
        """
        goal = GripperCommandGoal()

        if state == GripperState.OPEN:
            goal.command.direction = GripperCommand.OPEN
        elif state == GripperState.CLOSE:
            goal.command.direction = GripperCommand.CLOSE
            goal.command.max_torque = max_torque
        else:
            rospy.logerr('State shoulde be open or close, now it is {0}'.format(state))
            return False

        self._ac_gripper.send_goal(goal)

        if state == GripperState.OPEN:
            if self.occupied_by is not None:
                rospy.logerr("send_gripper_goal open is called but there is still an entity with id '%s' \
                occupying the gripper, please update the world model and remove this entity" % self.occupied_by.id)
            self.occupied_by = None

        goal_status = GoalStatus.SUCCEEDED
        if timeout != 0.0:
            self._ac_gripper.wait_for_result(rospy.Duration(timeout))
            goal_status = self._ac_gripper.get_state()

        return goal_status == GoalStatus.SUCCEEDED

    def handover_to_human(self, timeout=10):
        """
        Handover an item from the gripper to a human.

        Feels if user slightly pulls or pushes (the item in) the arm. On timeout, it will return False.
        :param timeout: timeout in seconds
        :return: True or False
        """
        pub = rospy.Publisher('/'+self.robot_name+'/handoverdetector_'+self.side+'/toggle_robot2human',
                              std_msgs.msg.Bool, queue_size=1, latch=True)
        pub.publish(std_msgs.msg.Bool(True))

        try:
            rospy.wait_for_message('/'+self.robot_name+'/handoverdetector_'+self.side+'/result', std_msgs.msg.Bool,
                                   timeout)
            # print('/'+self.robot_name+'/handoverdetector_'+self.side+'/result')
            return True
        except rospy.ROSException as e:
            rospy.logerr(e)
            return False

    def handover_to_robot(self, timeout=10):
        """
        Handover an item from a human to the robot.

        Feels if user slightly pushes an item in the gripper. On timeout, it will return False.
        :param timeout: timeout in seconds
        :return: True or False
        """
        pub = rospy.Publisher('/'+self.robot_name+'/handoverdetector_'+self.side+'/toggle_human2robot',
                              std_msgs.msg.Bool, queue_size=1, latch=True)
        pub.publish(std_msgs.msg.Bool(True))

        try:
            rospy.wait_for_message('/'+self.robot_name+'/handoverdetector_'+self.side+'/result', std_msgs.msg.Bool,
                                   timeout)
            # print('/'+self.robot_name+'/handoverdetector_'+self.side+'/result')
            return True
        except rospy.ROSException as e:
            rospy.logerr(e)
            return False

    def _send_joint_trajectory(self, joints_references, max_joint_vel=0.7, timeout=rospy.Duration(5)):
        """
        Low level method that sends a array of joint references to the arm.

        If timeout is defined, it will wait for timeout*len(joints_reference) seconds for the
        completion of the actionlib goal. It will return True as soon as possible when the goal
        succeeded. On timeout, it will return False.

        :param joints_references:[str] list of joint configurations,
            which should be a list of the length equal to the number of joints to be moved
        :param max_joint_vel:(int,float,[int], [float]) speed the robot can have when getting to the desired
            configuration. A single value can be given, which will be used for all joints, or a list of values can be given
            in which the order has to agree with the joints according to the joints_references.
        :param timeout:(secs) timeout for each joint configuration in rospy.Duration(seconds); timeout of 0.0 is not
            allowed
        :return: True or False
        """
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

        rospy.logdebug("Send {0} arm to jointcoords \n{1}".format(self.side, ps))

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

    def wait_for_motion_done(self, timeout=10.0, cancel=False):
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

        passed_time = (rospy.Time.now() - starttime).to_sec()
        if passed_time > timeout:
            return False

        # rospy.loginfo('Waiting for ac_gripper')
        if self._ac_gripper.gh:
            rospy.logdebug('Not waiting for gripper action')
            # return self._ac_gripper.wait_for_result(rospy.Duration(timeout - passed_time))
            return True

    @property
    def object_in_gripper_measurement(self):
        """
        Returns whether the gripper is empty, holding an object or if this is unknown

        :return: latest GripperMeasurement
        """
        return self._grasp_sensor_state

    @property
    def grasp_sensor_distance(self):
        """
        Returns the sensor distance. If no recent measurement is available or the measurement is outside bounds
        and hence unreliable, NaN is returned

        :return: float with distance
        """
        return self._grasp_sensor_state.distance

    def _grasp_sensor_callback(self, msg):
        """
        Callback function for grasp sensor messages

        :param msg: std_msgs.msg.Float32MultiArray
        """
        self._grasp_sensor_state = GripperMeasurement(msg.data[0])

    @property
    def base_offset(self):
        """
        Retrieves the 'optimal' position of an object w.r.t. the base link of a
        robot for this arm to grasp it.

        :return: Position of an object w.r.t. the base link of a robot.
        :rtype: kdl Vector
        """
        return self._base_offset

    def _publish_marker(self, goal, color, ns=""):
        """
        Publish markers for visualisation
        :param goal: tue_manipulation_msgs.msg.GraspPrecomputeGoal
        :param color: list of rgb colors (0.0-1.0)
        :param ns: namespace
        :return: no return
        """
        marker = visualization_msgs.msg.Marker()
        marker.header.frame_id = goal.goal.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.type = 2
        marker.pose.position.x = goal.goal.x
        marker.pose.position.y = goal.goal.y
        marker.pose.position.z = goal.goal.z
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
        return "Arm(side='{side}')".format(side=self.side)


class ForceSensingArm(Arm):
    def __init__(self, robot_name, tf_listener, get_joint_states, side):
        """
        constructor

        :todo: Make the Arm class similar to the robot, such that it can be composed from parts

        :param robot_name: robot_name
        :param tf_listener: tf_server.TFClient()
        :param get_joint_states: get_joint_states function for getting the last joint states
        :param side: left or right
        """
        super(ForceSensingArm, self).__init__(robot_name, tf_listener, get_joint_states, side)

        self.force_sensor = ForceSensor(robot_name, tf_listener, "/" + robot_name + "/wrist_wrench/raw")

    def move_down_until_force_sensor_edge_up(self, timeout=10, retract_distance=0.01):
        """
        Move down the arm (hero specific, only joint arm_lift_joint) until the force sensor detects an edge up

        A 'force_sensor.TimeOutException' will be raised if no edge up is detected within timeout

        :param timeout: Max duration for edge up detection
        :param retract_distance: How much to retract if we have reached a surface
        """
        # Fill with required joint names (desired in hardware / gazebo impl)
        current_joint_state = self.get_joint_states()
        current_joint_state['arm_lift_joint'] = 0
        self._ac_joint_traj.send_goal(self._make_goal(current_joint_state, timeout))

        self.force_sensor.wait_for_edge_up(timeout)
        self.cancel_goals()

        current_joint_state = self.get_joint_states()
        current_joint_state['arm_lift_joint'] += retract_distance
        self._ac_joint_traj.send_goal(self._make_goal(current_joint_state, 0.5))

    def _make_goal(self, current_joint_state, timeout):
        positions = [current_joint_state[n] for n in self.joint_names]
        points = [JointTrajectoryPoint(positions=positions,
                                       time_from_start=rospy.Duration.from_sec(timeout))]
        trajectory = JointTrajectory(joint_names=self.joint_names, points=points)
        return FollowJointTrajectoryGoal(trajectory=trajectory)


class FakeArm(RobotPart):
    def __init__(self, robot_name, tf_listener, side):
        super(FakeArm, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self.side = side
        if (self.side is "left") or (self.side is "right"):
            pass
        else:
            raise Exception("Side should be either: left or right")

        self._operational = False

        # Get stuff from the parameter server
        offset = self.load_param('skills/arm/offset/' + self.side)
        self.offset = kdl.Frame(kdl.Rotation.RPY(offset["roll"], offset["pitch"], offset["yaw"]),
                                kdl.Vector(offset["x"], offset["y"], offset["z"]))

        self.marker_to_grippoint_offset = self.load_param('skills/arm/offset/marker_to_grippoint')

        self.joint_names = self.load_param('skills/arm/joint_names')
        self.joint_names = [name + "_" + self.side for name in self.joint_names]
        self.torso_joint_names = self.load_param('skills/torso/joint_names')

        self.default_configurations = self.load_param('skills/arm/default_configurations')
        self.default_trajectories   = self.load_param('skills/arm/default_trajectories')

    @property
    def operational(self):
        return False

    def cancel_goals(self):
        pass

    def close(self):
        pass

    def send_goal(self, frameStamped, timeout=30, pre_grasp=False, first_joint_pos_only=False,
                  allowed_touch_objects=[]):
        return False

    def send_joint_goal(self, configuration, max_joint_vel=0.7, timeout=5.0):
        return False

    def send_joint_trajectory(self, configuration, max_joint_vel=0.7, timeout=5):
        return False

    def _send_joint_trajectory(self, joints_references, max_joint_vel=0.7, timeout=rospy.Duration(5)):
        rospy.logwarn("_send_joint_trajectory called on FakeArm.")
        return False

    def reset(self, timeout=0.0):
        return False

    @property
    def occupied_by(self):
        return None

    @occupied_by.setter
    def occupied_by(self, value):
        pass

    def send_gripper_goal(self, state, timeout=5.0):
        return False

    def handover_to_human(self, timeout=10):
        return False

    def handover_to_robot(self, timeout=10):
        return False

    def wait_for_motion_done(self, timeout=10.0, cancel=False):
        return False

    @property
    def object_in_gripper_measurement(self):
        return GripperMeasurement(0.0)

    def __repr__(self):
        return "FakeArm(side='{side}')".format(side=self.side)
