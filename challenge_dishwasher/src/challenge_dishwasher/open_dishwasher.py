import math
from collections import namedtuple

import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Quaternion
from robot_skills.amigo import Amigo
from robot_skills.util.kdl_conversions import frame_stamped, VectorStamped
from robot_smach_states import NavigateToSymbolic
from robot_smach_states.util.designators import EdEntityDesignator
from robot_smach_states.util.geometry_helpers import wrap_angle_pi
from smach import StateMachine, State, cb_interface, CBState
from tf.transformations import euler_from_quaternion, quaternion_from_euler

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



ControlParameters = namedtuple('ControlParameters', [
    'position_gain',
    'rotation_gain',
    'abs_vx',
    'abs_vy',
    'abs_vyaw',
    'goal_position_tolerance',
    'goal_rotation_tolerance'
])


class ControlToPose(State):
    def __init__(self, robot, goal_pose, control_parameters):
        State.__init__(self, outcomes=['succeeded', 'failed'])
        self.robot = robot
        self.goal_pose = goal_pose
        self.params = control_parameters

        self._rate = rospy.Rate(10)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._cmd_vel_publisher = rospy.Publisher("/" + self.robot.robot_name + "/base/references", Twist, queue_size=1)
        rospy.sleep(0.5)

    def execute(self, ud):
        if self._goal_reached(*self._get_target_delta_in_robot_frame(self.goal_pose)):
            rospy.loginfo("We are already there")
            return

        rospy.loginfo("Starting alignment ....")
        while not rospy.is_shutdown():
            dx, dy, dyaw = self._get_target_delta_in_robot_frame(self.goal_pose)

            if self._goal_reached(dx, dy, dyaw):
                break

            rospy.logdebug_throttle(0.1, "Aligning .. Delta = {} {} {}".format(dx, dy, dyaw))

            self._cmd_vel_publisher.publish(Twist(
                linear=Vector3(
                    x=_clamp(self.params.abs_vx, self.params.position_gain * dx),
                    y=_clamp(self.params.abs_vy, self.params.position_gain * dy)
                ),
                angular=Vector3(z=_clamp(self.params.abs_vyaw, self.params.rotation_gain * dyaw))
            ))

            self._rate.sleep()

        rospy.loginfo("Goal reached")

    def _get_target_delta_in_robot_frame(self, goal_pose):
        goal_pose.header.stamp = rospy.Time.now()
        pose = self._tf_buffer.transform(goal_pose, self.robot.robot_name + '/base_link', rospy.Duration(1.0))
        yaw = _get_yaw_from_quaternion_msg(pose.pose.orientation)
        return pose.pose.position.x, pose.pose.position.y, wrap_angle_pi(yaw)

    def _goal_reached(self, dx, dy, dyaw):
        return math.hypot(dx, dy) < self.params.goal_position_tolerance and abs(dyaw) < self.params.goal_rotation_tolerance


class OpenDishwasher(StateMachine):
    def __init__(self, robot, dishwasher_id):
        StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        base_y_position_door = 0
        base_y_position_rack = -0.75
        base_y_position_rack2 = -0.65
        base_yaw_position_rack = 3.7

        @cb_interface(outcomes=['done'])
        def _pre_grab_handle(ud):
            robot.rightArm.send_gripper_goal("open", timeout=0)
            robot.rightArm._send_joint_trajectory([[0, 0.2519052373022729913, 0.7746500794619434, 1.3944848321343395,
                                                   -1.829999276180074, 0.6947045024700284, 0.1889253710114966]],
                                                 timeout=rospy.Duration(0))
            robot.rightArm.wait_for_motion_done()
            return 'done'

        @cb_interface(outcomes=['done'])
        def _grab_handle(ud):
            robot.rightArm.wait_for_motion_done()
            robot.speech.speak('I hope this goes right!', block=False)
            fs = frame_stamped("dishwasher", 0.42, 0, 0.8, roll=math.pi / 2, pitch=0, yaw=math.pi)
            robot.rightArm.send_goal(fs.projectToFrame(robot.robot_name + "/base_link", robot.tf_listener))
            robot.rightArm.send_gripper_goal("close")
            return 'done'

        @cb_interface(outcomes=['done'])
        def _align_with_dishwasher(ud):
            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = dishwasher_id
            goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.pi))
            goal_pose.pose.position.x = 0.85
            goal_pose.pose.position.y = base_y_position_door
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.01, 0.1)).execute({})
            return 'done'

        @cb_interface(outcomes=['done'])
        def _drive_to_open_dishwasher(ud):
            # Open the dishwasher
            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = dishwasher_id
            goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.pi))
            goal_pose.pose.position.x = 1.4
            goal_pose.pose.position.y = base_y_position_door
            robot.torso.low()
            ControlToPose(robot, goal_pose, ControlParameters(0.8, 1.0, 0.15, 0.1, 0.1, 0.05, 0.5)).execute({})

            return 'done'

        @cb_interface(outcomes=['done'])
        def _fully_open_dishwasher_door(ud):
            # Stand in front of the door and rotate
            robot.torso.high()
            robot.rightArm._send_joint_trajectory([[0, 0, 0, 0, 0, 0, 0]], timeout=rospy.Duration(0))
            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = dishwasher_id
            goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, - math.pi / 2))
            goal_pose.pose.position.x = 1.4
            goal_pose.pose.position.y = base_y_position_door
            ControlToPose(robot, goal_pose, ControlParameters(0.8, 1.0, 0.5, 0.5, 0.5, 0.05, 0.1)).execute({})
            robot.torso.wait_for_motion_done()
            robot.rightArm.wait_for_motion_done()

            # Move arm to the other side of the door
            robot.rightArm._send_joint_trajectory([[0, 1.57, 0, 0, 0, 0, 0]], timeout=rospy.Duration(0))
            robot.rightArm.wait_for_motion_done()
            robot.rightArm._send_joint_trajectory([[-1.5, 1.57, 0, 0, 0, 0, 0]], timeout=rospy.Duration(0))
            robot.rightArm.wait_for_motion_done()
            robot.rightArm._send_joint_trajectory([[-1.5, 0, 0, 0, 0, 0, 0]], timeout=rospy.Duration(0))
            robot.rightArm.wait_for_motion_done()

            # Go down
            robot.torso._send_goal([0.1])
            robot.torso.wait_for_motion_done()

            robot.rightArm._send_joint_trajectory([[-0.8, 0, 0, 0, 0, 0, 0]], timeout=rospy.Duration(0))
            robot.rightArm.wait_for_motion_done()

            # Drive away from the door so we open the door with our stretched arm
            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = dishwasher_id
            goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, - math.pi / 2))
            goal_pose.pose.position.x = 1.9
            goal_pose.pose.position.y = base_y_position_door
            ControlToPose(robot, goal_pose, ControlParameters(0.8, 1.0, 0.1, 0.1, 0.1, 0.05, 0.1)).execute({})

            robot.rightArm.reset()
            robot.rightArm.wait_for_motion_done()

            return 'done'

        @cb_interface(outcomes=['done'])
        def _align_with_dishwasher_to_open_rack(ud):
            robot.torso.low()

            # Drive sideways around the open door
            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = dishwasher_id
            goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.pi))
            goal_pose.pose.position.x = 1.4
            goal_pose.pose.position.y = base_y_position_rack - 0.15
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.1, 0.1, 0.1, 0.02, 0.2)).execute({})

            # Arm in the right position so we can drive in the dishwasher with the arm
            robot.rightArm._send_joint_trajectory([[-1.48, 0, 0, 0.5, 1.57, 0, -0.1]], timeout=rospy.Duration(0))
            robot.rightArm.wait_for_motion_done()
            robot.torso.wait_for_motion_done()

            # Drive aside the open door, with arm in the dishwasher
            goal_pose.pose.position.x = 0.8
            goal_pose.pose.position.y = base_y_position_rack2
            goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, base_yaw_position_rack))
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 0.5, 0.1, 0.1, 0.1, 0.02, 0.2)).execute({})
            return 'done'

        @cb_interface(outcomes=['done'])
        def _arm_in_rack(ud):
            robot.rightArm._send_joint_trajectory([[-1.4, 0, 0, 0.45, 1.57, 0.8, -0.1]], timeout=rospy.Duration(0))
            robot.rightArm.wait_for_motion_done()
            return 'done'

        @cb_interface(outcomes=['done'])
        def _drive_to_open_rack(ud):
            # Open the dishwasher rack
            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = dishwasher_id
            goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, base_yaw_position_rack))
            goal_pose.pose.position.x = 1.25
            goal_pose.pose.position.y = base_y_position_rack2
            ControlToPose(robot, goal_pose, ControlParameters(0.8, 1.0, 0.15, 0.1, 0.1, 0.05, 0.2)).execute({})

            return 'done'

        @cb_interface(outcomes=['done'])
        def _retract_arm_from_rack(ud):
            robot.torso.reset()
            robot.torso.wait_for_motion_done()

            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = dishwasher_id
            goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0.9 * math.pi))
            goal_pose.pose.position.x = 1.25
            goal_pose.pose.position.y = base_y_position_rack2
            ControlToPose(robot, goal_pose, ControlParameters(0.8, 1.0, 0.15, 0.1, 0.4, 0.1, 0.2)).execute({})

            robot.rightArm.reset()
            robot.rightArm.wait_for_motion_done()
            return 'done'

        with self:
            self.add_auto('PRE_GRAB_HANDLE', CBState(_pre_grab_handle), ['done'])
            self.add_auto('ALIGN_WITH_DISHWASHER', CBState(_align_with_dishwasher), ['done'])
            self.add_auto('GRAB_HANDLE', CBState(_grab_handle), ['done'])
            self.add_auto('DRIVE_TO_OPEN_DISHWASHER', CBState(_drive_to_open_dishwasher), ['done'])
            self.add_auto('FULLY_OPEN_DISHWASHER_DOOR', CBState(_fully_open_dishwasher_door), ['done'])
            self.add_auto('ALIGN_WITH_DISHWASHER_TO_OPEN_RACK', CBState(_align_with_dishwasher_to_open_rack), ['done'])
            self.add_auto('ARM_IN_RACK', CBState(_arm_in_rack), ['done'])
            self.add_auto('DRIVE_TO_OPEN_RACK', CBState(_drive_to_open_rack), ['done'])
            self.add('RETRACT_ARM_FROM_RACK', CBState(_retract_arm_from_rack), transitions={'done': 'succeeded'})


class TestOpenDishwasher(StateMachine):
    def __init__(self, robot, dishwasher_id, dishwasher_navigate_area):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        dishwasher = EdEntityDesignator(robot=robot, id=dishwasher_id)

        with self:
            StateMachine.add("NAVIGATE_TO_DISHWASHER",
                             NavigateToSymbolic(robot, {dishwasher: dishwasher_navigate_area}, dishwasher),
                             transitions={'arrived': 'OPEN_DISHWASHER',
                                          'unreachable': 'failed',
                                          'goal_not_defined': 'failed'})

            StateMachine.add("OPEN_DISHWASHER",
                             OpenDishwasher(robot, dishwasher_id),
                             transitions={'succeeded': 'succeeded',
                                          'failed': 'failed'})


if __name__ == '__main__':
    rospy.init_node('test_open_dishwasher')

    robot = Amigo()
    robot.ed.reset()
    robot.rightArm.reset()
    robot.torso.reset()

    sm = TestOpenDishwasher(robot, 'dishwasher', 'in_front_of')
    sm.execute()
