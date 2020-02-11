#
# Copyright (c) 2019, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import math
from collections import namedtuple

# ROS
from geometry_msgs.msg import PoseStamped, Twist, Vector3
import rospy
import smach
import tf2_geometry_msgs
import tf2_ros
from tf.transformations import euler_from_quaternion

# TU/e Robotics
from robot_smach_states.util.geometry_helpers import wrap_angle_pi
from robot_smach_states.util.designators.checks import check_type

_ = tf2_geometry_msgs  # tf2_geometry_msgs must be declared here for it to be imported


def _clamp(abs_value, value):
    return max(-abs_value, min(abs_value, value))


def _get_yaw_from_quaternion_msg(msg):
    """
    Returns the yaw angle from a rotation in quaternion representation (msg)

    :param msg: The quaternion msg
    :return: (float) Yaw angle in rad
    """
    orientation_list = [msg.x, msg.y, msg.z, msg.w]
    _, _, yaw = euler_from_quaternion(orientation_list)
    return yaw


class ControlParameters(namedtuple('ControlParameters', [
    'position_gain',
    'rotation_gain',
    'abs_vx',
    'abs_vy',
    'abs_vyaw',
    'goal_position_tolerance',
    'goal_rotation_tolerance'
])):
    """
    position_gain: (float) Tunable parameter to increase or decrease the change in position per time
    rotation_gain: (float) Tunable parameter to increase or decrease the change in yaw per time
    abs_vx: (float) Absolute velocity in x [m/s]
    abs_vy: (float) Absolute velocity in y [m/s]
    abs_vyaw: (float) Absolute velocity in yaw [rad/s]
    goal_position_tolerance: (float) Tolerance in position [m]
    goal_rotation_tolerance: (float) Tolerance in yaw [rad]
    """


class ControlToPose(smach.State):
    def __init__(self, robot, goal_pose, control_parameters, rate=10):
        """
        State that allows the tuning of robot navigation to a specific goal through custom speeds, gains and tolerances

        :param robot: (Robot) api object
        :param goal_pose: (PoseStamped or Designator to PoseStamped) Position the robot needs to go to
        :param control_parameters: (namedtuple, ControlParameters) Parameters that specify how the robot should reach
        :param rate: (float, int) Control rate [Hz]
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        assert all([isinstance(p, (float, int)) for p in control_parameters]), "Control parameters are invalid"

        self.robot = robot
        check_type(goal_pose, PoseStamped)
        self.goal_pose = goal_pose
        self.params = control_parameters

        self._rate = rate

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    def execute(self, userdata=None):
        goal_pose = self.goal_pose.resolve() if hasattr(self.goal_pose, "resolve") else self.goal_pose
        if self._goal_reached(*self._get_target_delta_in_robot_frame(goal_pose)):
            rospy.loginfo("We are already there")
            return 'succeeded'

        rate = rospy.Rate(self._rate)
        rospy.loginfo("Starting alignment ....")
        with self.robot.base.get_cmd_vel_publisher() as pub_cmd_vel:
            while not rospy.is_shutdown():
                dx, dy, dyaw = self._get_target_delta_in_robot_frame(goal_pose)

                if self._goal_reached(dx, dy, dyaw):
                    rospy.loginfo("Goal reached")
                    return 'succeeded'

                rospy.logdebug_throttle(0.1, "Aligning .. Delta = {} {} {}".format(dx, dy, dyaw))

                pub_cmd_vel(vx=_clamp(self.params.abs_vx, self.params.position_gain * dx),
                            vy=_clamp(self.params.abs_vy, self.params.position_gain * dy),
                            vth=_clamp(self.params.abs_vyaw, self.params.rotation_gain * dyaw),)

                rate.sleep()

        return 'failed'

    def _get_target_delta_in_robot_frame(self, goal_pose):
        """
        Transfers the goal pose to robot frame

        :param goal_pose: (PoseStamped) Position the robot needs to go to
        :return: (float) x position of goal in robot frame,
                 (float) y position of goal in robot frame,
                 (float) yaw of goal in robot frame
        """
        goal_pose.header.stamp = rospy.Time.now()
        pose = self._tf_buffer.transform(goal_pose, self.robot.base_link_frame.lstrip("/"), rospy.Duration(1.0))
        yaw = _get_yaw_from_quaternion_msg(pose.pose.orientation)
        return pose.pose.position.x, pose.pose.position.y, wrap_angle_pi(yaw)

    def _goal_reached(self, dx, dy, dyaw):
        """
        Checks if the goal pose is reached

        :param dx: (float)[m] Margin between goal_pose and robot in x
        :param dy: (float)[m] Margin between goal_pose and robot in y
        :param dyaw: (float)[rad] Margin between goal_pose and robot in yaw
        :return: (bool) Boolean indicating whether the margins are reached
        """
        return math.hypot(dx, dy) < self.params.goal_position_tolerance and abs(
            dyaw) < self.params.goal_rotation_tolerance
