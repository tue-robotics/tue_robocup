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

# Challenge storing groceries
from tf.transformations import euler_from_quaternion, quaternion_from_euler


import robot_smach_states as states

import robot_smach_states.util.designators as ds

from robot_skills.util.kdl_conversions import frame_stamped, VectorStamped


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


def _wrap_angle_pi(angle):
    """
    Wraps between -pi and +pi
    :param angle: Input angle
    :return: Wrapped angle
    """
    angle = angle % (2 * math.pi)
    if angle > math.pi:
        return angle - 2 * math.pi
    elif angle < -math.pi:
        return angle + 2 * math.pi
    return angle


class OpenDishwasher(smach.State):
    def __init__(self, robot, dishwasher_id):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.robot = robot
        self.dishwasher_id = dishwasher_id

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
        return pose.pose.position.x, pose.pose.position.y, _wrap_angle_pi(yaw)

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

            rospy.loginfo_throttle(0.1, "Aligning .. Delta = {} {} {}".format(dx, dy, dyaw))

            self._cmd_vel_publisher.publish(Twist(
                linear=Vector3(
                    x=_clamp(abs_vx, position_gain * dx),
                    y=_clamp(abs_vy, position_gain * dy)
                ),
                angular=Vector3(z=_clamp(abs_vyaw, rotation_gain * dyaw))
            ))

            self._rate.sleep()

        rospy.loginfo("Goal reached")

    def _align_with_dishwasher(self):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = self.dishwasher_id
        goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.pi))
        goal_pose.pose.position.x = 0.85
        self._control_to_pose(goal_pose, 0.5, 1.0, 0.3, 0.3, 0.3)

    def _pre_grab_handle(self):
        self.robot.leftArm.send_gripper_goal("open", timeout=0)
        self.robot.leftArm._send_joint_trajectory([[0, 0.2519052373022729913, 0.7746500794619434, 1.3944848321343395, -1.829999276180074, 0.6947045024700284, 0.1889253710114966]], timeout=rospy.Duration(0))

    def _grab_handle(self):
        self.robot.leftArm.wait_for_motion_done()
        self.robot.speech.speak('I hope this goes right!', block=False)
        self.robot.speech.speak('Ah, at least it is worth a try for 50 points, here we go!', block=False)
        self.robot.head.look_at_point(VectorStamped(y=100, frame_id="/"+self.robot.robot_name+"/base_link"))
        fs = frame_stamped("dishwasher", 0.41, 0, 0.83, roll=-math.pi / 2, pitch=0, yaw=math.pi)
        self.robot.leftArm.send_goal(fs.projectToFrame(self.robot.robot_name + "/base_link", self.robot.tf_listener))
        self.robot.leftArm.send_gripper_goal("close")
        self.robot.head.reset()
        self.robot.speech.speak('Okay, now what!')

    def _drive_to_open_dishwasher(self):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = self.dishwasher_id
        goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.pi))
        goal_pose.pose.position.x = 1.5

        self._control_to_pose(goal_pose, 1.0, 1.0, 0.15, 0.075, 0.1)

    def execute(self, userdata=None):
        self._pre_grab_handle()
        self._align_with_dishwasher()
        self._grab_handle()
        self._drive_to_open_dishwasher()

        self.robot.leftArm.reset()
        self.robot.torso.reset()

        return 'succeeded'


class TestOpenDishwasher(smach.StateMachine):
    def __init__(self, robot, dishwasher_id, dishwasher_navigate_area):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        dishwasher = ds.EdEntityDesignator(robot=robot, id=dishwasher_id)

        with self:
            smach.StateMachine.add("NAVIGATE_TO_DISHWASHER",
                                   states.NavigateToSymbolic(robot, {dishwasher: dishwasher_navigate_area}, dishwasher),
                                   transitions={'arrived': 'OPEN_DISHWASHER',
                                                'unreachable': 'failed',
                                                'goal_not_defined': 'failed'})

            smach.StateMachine.add("OPEN_DISHWASHER",
                                   OpenDishwasher(robot, dishwasher_id),
                                   transitions={'succeeded': 'succeeded',
                                                'failed': 'failed'})


if __name__ == '__main__':
    rospy.init_node('test_open_dishwasher')

    robot = Amigo()
    robot.ed.reset()
    robot.leftArm.reset()
    robot.torso.reset()

    sm = TestOpenDishwasher(robot, 'dishwasher', 'in_front_of')
    sm.execute()
