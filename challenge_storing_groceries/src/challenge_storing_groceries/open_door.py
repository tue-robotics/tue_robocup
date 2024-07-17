# System
import os

# ROS
import math
import rospy
import smach
import tf2_ros
# noinspection PyUnresolvedReferences
import tf2_geometry_msgs  # required for transforms
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Quaternion

# Challenge storing groceries
from tf.transformations import euler_from_quaternion, quaternion_from_euler

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
        self.robot.torso.send_goal("reset")
        self.robot.head.look_at_ground_in_front_of_robot(2.0)

        rospy.sleep(1)
        # Now update the pose of the cabinet
        self.robot.ed.update_kinect(f"{self.cabinet_inspect_area} {self.cabinet.uuid}")

        return "succeeded"


class OpenDoor(smach.State):
    def __init__(self, robot, cabinet):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.robot = robot
        self.cabinet = cabinet

        self.arm = self.robot.get_arm()  # hotfix for hero
        # self.arm = self.robot.leftArm  # Joint goals are tuned for this arm only

        self._rate = rospy.Rate(10)
        self._goal_position_tolerance = 0.025
        self._goal_rotation_tolerance = 0.1

        self._tf_buffer = self.robot.tf_buffer
        self._cmd_vel_publisher = rospy.Publisher(f"/{robot.robot_name}/base/references", Twist, queue_size=1)

    def _get_target_delta_in_robot_frame(self, goal_pose):
        goal_pose.header.stamp = rospy.Time.now()
        pose = self._tf_buffer.transform(goal_pose, 'base_link', rospy.Duration(1.0))
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

            rospy.loginfo_throttle(1.0, f"Aligning .. Delta = {dx} {dy} {dyaw}")

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
        goal_pose.header.frame_id = self.cabinet.uuid
        goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.pi - 0.4))
        goal_pose.pose.position.x = 0.8
        goal_pose.pose.position.y = 0.2
        self._control_to_pose(goal_pose, 0.5, 1.0, 0.3, 0.3, 0.3)

    def _return_from_cabinet(self):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = self.cabinet.uuid
        goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.pi))
        goal_pose.pose.position.x = 1.0
        self._control_to_pose(goal_pose, 0.5, 1.0, 0.5, 0.5, 0.5)

    def _move_arm_to_open_cabinet_position(self):
        self.arm._arm._send_joint_trajectory([[0.0, 0.0, -1.572, -1.572, 0.0]])
        self.arm._arm._send_joint_trajectory([[0.0, 0.0, 1.572, -1.572, 0.0]])
        self.arm._arm._send_joint_trajectory([[0.0, -1.572, 1.572, -1.572, 0.0]])
        self.arm.gripper.send_goal("close")

    def _move_arm_in_cabinet(self):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = self.cabinet.uuid
        goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.pi - 0.1))
        goal_pose.pose.position.x = 0.50
        goal_pose.pose.position.y = 0.27
        self._control_to_pose(goal_pose, 0.5, 1.0, 0.3, 0.3, 0.3)

    def _move_arm_in_cabinet2(self):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = self.cabinet.uuid
        goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.pi - 0.1))
        goal_pose.pose.position.x = 0.50
        goal_pose.pose.position.y = 0.17
        self._control_to_pose(goal_pose, 0.5, 1.0, 0.3, 0.3, 0.3)


    def _drive_to_open_cabinet(self):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = self.cabinet.uuid
        goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.pi + 0.15))
        goal_pose.pose.position.x = 0.55
        goal_pose.pose.position.y = 0.17
        self._control_to_pose(goal_pose, 1.0, 1.0, 0.15, 0.075, 0.1)

    def _drive_to_open_cabinet2(self):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = self.cabinet.uuid
        goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.pi + 0.5))
        goal_pose.pose.position.x = 0.8
        goal_pose.pose.position.y = 0.17
        self._control_to_pose(goal_pose, 1.0, 1.0, 0.15, 0.075, 0.1)

    def execute(self, userdata=None):
        self._align_with_cabinet()
        self._move_arm_to_open_cabinet_position()
        self._move_arm_in_cabinet()
        self._move_arm_in_cabinet2()
        self._drive_to_open_cabinet()
        self._drive_to_open_cabinet2()
        self.arm.reset()
        rospy.sleep(1)
        self._return_from_cabinet()

        return 'succeeded'


class OpenDoorMachine(smach.StateMachine):
    def __init__(self, robot, shelf_designator, cabinet_navigate_area: str = "in_front_of", cabinet_inspect_area: str = "on_top_of"):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        with self:
            smach.StateMachine.add("NAVIGATE_TO_CABINET",
                                   states.navigation.NavigateToSymbolic(robot, {shelf_designator: cabinet_navigate_area}, shelf_designator),
                                   transitions={'arrived': 'UPDATE_CABINET_POSE',
                                                'unreachable': 'failed',
                                                'goal_not_defined': 'failed'})

            smach.StateMachine.add("UPDATE_CABINET_POSE",
                                   UpdateCabinetPose(robot, shelf_designator, cabinet_inspect_area),
                                   transitions={'succeeded': 'SAY_OPEN_DOOR',
                                                'failed': 'failed'})

            smach.StateMachine.add("SAY_OPEN_DOOR",
                                   states.human_interaction.Say(robot,
                                                                "I will now try to open the left door of the cabinet",
                                                                block=False),
                                   transitions={'spoken': 'OPEN_DOOR'})

            smach.StateMachine.add("OPEN_DOOR",
                                   OpenDoor(robot, shelf_designator),
                                   transitions={'succeeded': 'succeeded',
                                                'failed': 'failed'})


if __name__ == '__main__':
    rospy.init_node('test_open_door')

    from robot_skills.get_robot import get_robot
    robot = get_robot("hero")
    robot.ed.reset()
    robot.leftArm.reset()
    robot.torso.reset()

    # Nothing at shelf 6 but we are only interest in update of the pose
    sm = OpenDoorMachine(robot, 'cupboard', 'in_front_of', 'shelf6')
    sm.execute()
