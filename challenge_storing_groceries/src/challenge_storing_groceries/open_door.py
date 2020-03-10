# System
import os

# ROS
import math
import rospy
import smach
import tf2_ros
import tf2_geometry_msgs  # required for transforms
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Quaternion
from robot_skills.amigo import Amigo
# TU/e
from robot_skills.util.kdl_conversions import VectorStamped

# Challenge storing groceries
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import config

import robot_smach_states as states

import robot_smach_states.util.designators as ds
from robot_smach_states.util.geometry_helpers import wrap_angle_pi


def _clamp(abs_value, value):
    """
    Clamps the value to be between -abs_value and +abs_value

    :param abs_value: limit of the value in both positive and negative direction
    :param value: value to be clamped...
    :return: The -abs_value when value is smaller than -abs_value OR +abs_value when value is more than +abs_value
    """
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



class UpdateCabinetPose(smach.State):
    def __init__(self, robot, cabinet, cabinet_inspect_area):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.robot = robot
        self.cabinet = cabinet
        self.cabinet_inspect_area = cabinet_inspect_area

    def execute(self, userdata=None):
        self.robot.torso.send_goal('upper_limit')
        self.robot.head.look_at_ground_in_front_of_robot(2.0)

        rospy.sleep(1)
        # Now update the pose of the cabinet
        self.robot.ed.update_kinect("{} {}".format(self.cabinet_inspect_area, self.cabinet.id_))

        return 'succeeded'


class OpenDoor(smach.State):
    def __init__(self, robot, cabinet):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.robot = robot
        self.cabinet = cabinet

        self.arm = self.robot.leftArm  # Joint goals are tuned for this arm only

        self._rate = rospy.Rate(10)
        self._goal_position_tolerance = 0.01
        self._goal_rotation_tolerance = 0.1

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._cmd_vel_publisher = rospy.Publisher("/" + robot.robot_name + "/base/references", Twist, queue_size=1)

    def _get_target_delta_in_robot_frame(self, goal_pose):
        goal_pose.header.stamp = rospy.Time.now()
        pose = self._tf_buffer.transform(goal_pose, self.robot.robot_name + '/base_link', rospy.Duration(1.0))
        yaw = _get_yaw_from_quaternion_msg(pose.pose.orientation)
        return pose.pose.position.x, pose.pose.position.y, wrap_angle_pi(yaw)

    def _goal_reached(self, dx, dy, dyaw):
        return math.hypot(dx, dy) < self._goal_position_tolerance and abs(dyaw) < self._goal_rotation_tolerance

    def _control_to_pose(self, goal_pose, position_gain, rotation_gain, abs_vx, abs_vy, abs_vyaw):
        if self._goal_reached(*self._get_target_delta_in_robot_frame(goal_pose)):
            rospy.loginfo("We are already there")
            return

        rospy.loginfo("Starting alignment ....")
        while not rospy.is_shutdown():
            dx, dy, dyaw = self._get_target_delta_in_robot_frame(goal_pose)

            if self._goal_reached(dx, dy, dyaw):
                break

            rospy.loginfo_throttle(1.0, "Aligning .. Delta = {} {} {}".format(dx, dy, dyaw))

            self._cmd_vel_publisher.publish(Twist(
                linear=Vector3(
                    x=_clamp(abs_vx, position_gain * dx),
                    y=_clamp(abs_vy, position_gain * dy)
                ),
                angular=Vector3(z=_clamp(abs_vyaw, rotation_gain * dyaw))
            ))

            self._rate.sleep()

        rospy.loginfo("Goal reached")

    def _align_with_cabinet(self):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = self.cabinet.id_
        goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.pi - 0.05))
        goal_pose.pose.position.x = 0.6
        self._control_to_pose(goal_pose, 0.5, 1.0, 0.3, 0.3, 0.3)

    def _return_from_cabinet(self):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = self.cabinet.id_
        goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.pi))
        goal_pose.pose.position.x = 1.0
        self._control_to_pose(goal_pose, 0.5, 1.0, 0.5, 0.5, 0.5)

    def _move_arm_in_cabinet(self):
        self.arm._send_joint_trajectory([[-0.101, 0.119, 0.200, 1.363, 0.230, 0.688, 0.280]])
        self.arm._send_joint_trajectory([[-0.574, 0.802, 0.691, 1.021, 0.750, 0.535, 0.378]])
        #self.arm._send_joint_trajectory([[-0.042, 0.720, 0.846, 1.039, 0.849, 0.534, 0.377]])
        self.arm._send_joint_trajectory([[-0.04446312115056816, 0.9908744970312501, 0.9101328212542089, 0.9240762995414635, 1.0067001691414637, 0.3949460482219828, 0.1621731308728449]])

    def _drive_to_open_cabinet(self):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = self.cabinet.id_
        goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.pi - 0.8))
        goal_pose.pose.position.x = 0.75
        goal_pose.pose.position.y = -0.02
        self._control_to_pose(goal_pose, 1.0, 1.0, 0.15, 0.075, 0.1)

    def execute(self, userdata=None):
        self._align_with_cabinet()
        self._move_arm_in_cabinet()
        self._drive_to_open_cabinet()
        self.arm.reset()
        rospy.sleep(1)
        self._return_from_cabinet()

        return 'succeeded'


class OpenDoorMachine(smach.StateMachine):
    def __init__(self, robot, cabinet_id, cabinet_navigate_area, cabinet_inspect_area):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        self.cabinet = ds.EntityByIdDesignator(robot=robot, id=cabinet_id)

        with self:
            smach.StateMachine.add("NAVIGATE_TO_CABINET",
                                   states.NavigateToSymbolic(robot, {self.cabinet: cabinet_navigate_area}, self.cabinet),
                                   transitions={'arrived': 'UPDATE_CABINET_POSE',
                                                'unreachable': 'failed',
                                                'goal_not_defined': 'failed'})

            smach.StateMachine.add("UPDATE_CABINET_POSE",
                                   UpdateCabinetPose(robot, self.cabinet, cabinet_inspect_area),
                                   transitions={'succeeded': 'OPEN_DOOR',
                                                'failed': 'failed'})

            smach.StateMachine.add("OPEN_DOOR",
                                   OpenDoor(robot, self.cabinet),
                                   transitions={'succeeded': 'succeeded',
                                                'failed': 'failed'})


if __name__ == '__main__':
    rospy.init_node('test_open_door')

    robot = Amigo()
    robot.ed.reset()
    robot.leftArm.reset()
    robot.torso.reset()

    # Nothing at shelf 6 but we are only interest in update of the pose
    sm = OpenDoorMachine(robot, 'cupboard', 'in_front_of', 'shelf6')
    sm.execute()
