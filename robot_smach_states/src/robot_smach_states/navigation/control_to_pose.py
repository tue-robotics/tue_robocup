#
# Copyright (c) 2019, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import math
from collections import namedtuple
from robot_smach_states.util.geometry_helpers import wrap_angle_pi
import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import Twist, Vector3
from smach import State
from tf.transformations import euler_from_quaternion

_ = tf2_geometry_msgs


def _clamp(abs_value, value):
    return max(-abs_value, min(abs_value, value))


def _get_yaw_from_quaternion_msg(msg):
    """
    Returns the yaw angle from a rotation in quaternion representation (msg)
    :param msg: The quaternion msg
    :return: Yaw angle
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
    abs_vx: (float) Absolute velocity in x
    abs_vy: (float) Absolute velocity in y
    abs_vyaw: (float) Absolute velocity in yaw
    goal_position_tolerance: (float) Tolerance in position
    goal_rotation_tolerance: (float) Tolerance in yaw
    """


class ControlToPose(State):
    def __init__(self, robot, goal_pose, control_parameters, rate=10):
        """
        State that allows the tuning of robot navigation to a specific goal through custom speeds, gains and tolerances

        :param robot: (Robot) api object
        :param goal_pose: (PoseStamped) Position the robot needs to go to
        :param control_parameters: (namedtuple, ControlParameters) Parameters that specify how the robot should reach
        the goal_pose
        """
        State.__init__(self, outcomes=['succeeded', 'failed'])

        assert all([isinstance(p, (float, int)) for p in control_parameters]), "Control parameters are invalid"

        self.robot = robot
        self.goal_pose = goal_pose
        self.params = control_parameters

        self._rate = rospy.Rate(rate)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._cmd_vel_publisher = rospy.Publisher("/" + self.robot.robot_name + "/base/references", Twist, queue_size=1)
        rospy.sleep(0.5)

    def execute(self, userdata=None):
        if self._goal_reached(*self._get_target_delta_in_robot_frame(self.goal_pose)):
            rospy.loginfo("We are already there")
            return 'succeeded'

        rospy.loginfo("Starting alignment ....")
        while not rospy.is_shutdown():
            dx, dy, dyaw = self._get_target_delta_in_robot_frame(self.goal_pose)

            if self._goal_reached(dx, dy, dyaw):
                rospy.loginfo("Goal reached")
                return 'succeeded'

            rospy.logdebug_throttle(0.1, "Aligning .. Delta = {} {} {}".format(dx, dy, dyaw))

            self._cmd_vel_publisher.publish(Twist(
                linear=Vector3(
                    x=_clamp(self.params.abs_vx, self.params.position_gain * dx),
                    y=_clamp(self.params.abs_vy, self.params.position_gain * dy)
                ),
                angular=Vector3(z=_clamp(self.params.abs_vyaw, self.params.rotation_gain * dyaw))
            ))

            self._rate.sleep()

        return 'failed'

    def _get_target_delta_in_robot_frame(self, goal_pose):
        """
        Transfers the goal pose to robot frame

        :param goal_pose: (PoseStamped) Position the robot needs to go to
        :return: (float) x position of goal in robot frame, (float) y position of goal in robot frame,
        (float) yaw of goal in robot frame
        """
        goal_pose.header.stamp = rospy.Time.now()
        pose = self._tf_buffer.transform(goal_pose, self.robot.robot_name + '/base_link', rospy.Duration(1.0))
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
