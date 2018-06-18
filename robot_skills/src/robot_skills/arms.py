# ROS
import rospy
import std_msgs.msg
import PyKDL as kdl

import visualization_msgs.msg
from actionlib import GoalStatus
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# TU/e Robotics
from tue_manipulation_msgs.msg import GraspPrecomputeGoal, GraspPrecomputeAction
from tue_manipulation_msgs.msg import GripperCommandGoal, GripperCommandAction
from tue_msgs.msg import GripperCommand

from robot_part import RobotPart
from util.entity import Entity

# If the grasp sensor distance is smaller than this value, the gripper is holding an object

GRASP_SENSOR_THRESHOLD = rospy.get_param("skills/arm/grasp_sensor/threshold", 0.1)
GRASP_SENSOR_TIMEOUT = rospy.get_param("skills/arm/grasp_sensor/timeout", 0.5)
GRASP_SENSOR_LIMITS = tuple(rospy.get_param("skills/arm/grasp_sensor/limits", [0.02, 0.18]))
# Temporary: this should be approximately 0.02 once the sensor is correctly setup
GRASP_SENSOR_LIMITS = tuple(rospy.get_param("skills/arm/grasp_sensor/limits", [0.0025, 0.18]))


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

    def _is_recent(self):
        """
        Checks if the sensor data is recent

        :return: bool True if recent, i.e., measurement is less than GRASP_SENSOR_TIMEOUT old, False otherwise
        """
        return (rospy.Time.now() - self._stamp).to_sec() < GRASP_SENSOR_TIMEOUT

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
        elif not GRASP_SENSOR_LIMITS[0] < self._distance < GRASP_SENSOR_LIMITS[1]:
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
        return self._is_recent() and GRASP_SENSOR_LIMITS[0] < self._distance < GRASP_SENSOR_THRESHOLD

    @property
    def is_unknown(self):
        """
        Returns if the state is unknown, i.e., either the measurement is outdated or the distance is less than the
        limit

        :return: bool if unknown
        """
        return not self._is_recent() or self._distance < GRASP_SENSOR_LIMITS[0]

    @property
    def is_empty(self):
        """
        Returns if the gripper is empty, i.e., the measurement is recent and the value is greater than the threshold

        :return: bool if holding
        """
        return self._is_recent() and self._distance > GRASP_SENSOR_THRESHOLD

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
    >>> left.send_goal(0.265, 1, 0.816, 0, 0, 0, 60)
    or Equivalently:
    >>> left.send_goal(px=0.265, py=1, pz=0.816, yaw=0, pitch=0, roll=0, time_out=60, pre_grasp=False, frame_id='/amigo/base_link')

    #To open left gripper
    >>> left.send_gripper_goal_open(10)
    """
    def __init__(self, robot_name, tf_listener, side, world_model):
        """
        constructor
        :param robot_name: robot_name
        :param tf_listener: tf_server.TFClient()
        :param side: left or right
        """
        super(Arm, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self.side = side
        if (self.side is "left") or (self.side is "right"):
            pass
        else:
            raise Exception("Side should be either: left or right")

        self._world_model = world_model

        self._occupied_by = None
        self.__occupied_by_param = self.robot_name + '/' + self.side + '/occupied_by'

        self._operational = True  # In simulation, there will be no hardware cb

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

        # listen to the hardware status to determine if the arm is available
        self.subscribe_hardware_status(self.side + '_arm')

        # Init gripper actionlib
        self._ac_gripper = self.create_simple_action_client(
            "/" + robot_name + "/" + self.side + "_arm/gripper/action", GripperCommandAction)

        # Init graps precompute actionlib
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
            print "{0} arm cancelling all goals on all arm-related ACs on close. Rospy is already deleted.".format(self.side)

        self._ac_gripper.cancel_all_goals()
        self._ac_grasp_precompute.cancel_all_goals()
        self._ac_joint_traj.cancel_all_goals()

    def send_goal(self, frameStamped, timeout=30, pre_grasp=False, first_joint_pos_only=False,
                  allowed_touch_objects=[]):
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

        import time; time.sleep(0.001)  # This is necessary: the rtt_actionlib in the hardware seems
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
                execute_timeout=rospy.Duration(timeout)
            )
            if result == GoalStatus.SUCCEEDED:

                result_pose = self.tf_listener.lookupTransform(self.robot_name + "/base_link", self.robot_name + "/grippoint_{}".format(self.side), rospy.Time(0))
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

    def send_joint_goal(self, configuration, timeout=5.0):
        """
        Send a named joint goal (pose) defined in the parameter default_configurations to the arm
        :param configuration: name of configuration, configuration should be loaded as parameter
        :param timeout: timeout in seconds
        :return: True or False, False in case of nonexistent configuration or failed execution
        """
        if configuration in self.default_configurations:
            return self._send_joint_trajectory(
                [self.default_configurations[configuration]],
                timeout=rospy.Duration(timeout))
        else:
            rospy.logwarn('Default configuration {0} does not exist'.format(configuration))
            return False

    def send_joint_trajectory(self, configuration, timeout=5):
        """
        Send a named joint trajectory (sequence of poses) defined in the default_trajectories to
        the arm
        :param configuration: name of configuration, configuration should be loaded as parameter
        :param timeout: timeout in seconds
        :return: True or False, False in case of nonexistent configuration or failed execution
        """
        if configuration in self.default_trajectories:
            return self._send_joint_trajectory(self.default_trajectories[configuration], timeout=rospy.Duration(timeout))
        else:
            rospy.logwarn('Default trajectories {0} does not exist'.format(configuration))
            return False

    def reset(self, timeout=0.0):
        """
        Put the arm into the 'reset' pose
        :param timeout: timeout in seconds
        :return: True or False
        """
        return self.send_joint_goal('reset', timeout=timeout)

    @property
    def occupied_by(self):
        """
        The 'occupied_by' property will return the current entity that is in the gripper of this arm.
        :return: robot_skills.util.entity, ED entity
        """

        if self._occupied_by:
            return self._occupied_by
        else:
            occupied_by = rospy.get_param(self.__occupied_by_param, 'unoccupied')
            rospy.logwarn("Get {}.occupied_by = {} from rosparam".format(self, occupied_by))
            if occupied_by != "unoccupied":
                entity = self._world_model.get_entity(id=occupied_by)
                if not entity:
                    rospy.logwarn("No entity with id '{}'".format(occupied_by))
                return entity
            else:
                return None

    @occupied_by.setter
    def occupied_by(self, value):
        """
        Set the entity which occupies the arm.
        :param value: robot_skills.util.entity, ED entity
        :return: no return
        """
        rospy.loginfo("Set {}.occupied_by = {}".format(self, value))
        self._occupied_by = value

        if value:
            assert isinstance(value, Entity)
            rospy.set_param(self.__occupied_by_param, value.id)
        else:
            rospy.set_param(self.__occupied_by_param, "unoccupied")

    def send_gripper_goal(self, state, timeout=5.0):
        """
        Send a GripperCommand to the gripper of this arm and wait for finishing
        :param state: open or close
        :param timeout: timeout in seconds; timeout of 0.0 is not allowed
        :return: True of False
        """
        goal = GripperCommandGoal()

        if state == GripperState.OPEN:
            goal.command.direction = GripperCommand.OPEN
        elif state == GripperState.CLOSE:
            goal.command.direction = GripperCommand.CLOSE
        else:
            rospy.logerr('State shoulde be open or close, now it is {0}'.format(state))
            return False

        self._ac_gripper.send_goal(goal)

        if state == GripperState.OPEN:
            if self.occupied_by is not None:
                rospy.logerr("send_gripper_goal open is called but there is still an entity with id '%s' occupying the gripper, please update the world model and remove this entity" % self.occupied_by.id)
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
        pub = rospy.Publisher('/'+self.robot_name+'/handoverdetector_'+self.side+'/toggle_robot2human', std_msgs.msg.Bool, queue_size=1, latch=True)
        pub.publish(std_msgs.msg.Bool(True))

        try:
            rospy.wait_for_message('/'+self.robot_name+'/handoverdetector_'+self.side+'/result', std_msgs.msg.Bool, timeout)
            print '/'+self.robot_name+'/handoverdetector_'+self.side+'/result'
            return True
        except rospy.ROSException, e:
            rospy.logerr(e)
            return False

    def handover_to_robot(self, timeout=10):
        """
        Handover an item from a human to the robot.

        Feels if user slightly pushes an item in the gripper. On timeout, it will return False.
        :param timeout: timeout in seconds
        :return: True or False
        """
        pub = rospy.Publisher('/'+self.robot_name+'/handoverdetector_'+self.side+'/toggle_human2robot', std_msgs.msg.Bool, queue_size=1, latch=True)
        pub.publish(std_msgs.msg.Bool(True))

        try:
            rospy.wait_for_message('/'+self.robot_name+'/handoverdetector_'+self.side+'/result', std_msgs.msg.Bool, timeout)
            print '/'+self.robot_name+'/handoverdetector_'+self.side+'/result'
            return True
        except rospy.ROSException, e:
            rospy.logerr(e)
            return False

    def _send_joint_trajectory(self, joints_references, timeout=rospy.Duration(5), joint_names = None):
        """
        Low level method that sends a array of joint references to the arm.

        If timeout is defined, it will wait for timeout*len(joints_reference) seconds for the
        completion of the actionlib goal. It will return True as soon as possible when the goal
        succeeded. On timeout, it will return False.
        :param joints_references: list of joint configurations,
        which should be a list of the length equal to the number of joints to be moved
        :param timeout: timeout for each joint configuration in rospy.Duration(seconds); timeout of 0.0 is not allowed
        :param joint_names: joint names, which need to me moved
        :return: True or False
        """
        if not joints_references:
            return False

        if not joint_names:
            if len(joints_references[0]) == len(self.joint_names) + len(self.torso_joint_names):
                joint_names = self.torso_joint_names + self.joint_names
            else:
                joint_names = self.joint_names

        time_from_start = rospy.Duration()
        ps = []
        for joints_reference in joints_references:
            if len(joints_reference) != len(joint_names):
                rospy.logwarn('Please use the correct %d number of joint references (current = %d'
                              % (len(joint_names), len(joints_references)))
            ps.append(JointTrajectoryPoint(
                positions=joints_reference,
                time_from_start=time_from_start))

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
        :return bool indicates whether motion was done (True if reached,
        False otherwise)
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

    def _publish_marker(self, goal, color, ns = ""):
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

    def send_joint_goal(self, configuration, timeout=5.0):
        return False

    def send_joint_trajectory(self, configuration, timeout=5):
        return False

    def _send_joint_trajectory(self, joints_references, timeout=rospy.Duration(5), joint_names = None):
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
