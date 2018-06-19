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


ControlParameters = namedtuple('ControlParameters', [
    'position_gain',
    'rotation_gain',
    'abs_vx',
    'abs_vy',
    'abs_vyaw'
])


class ControlToPose(State):
    def __init__(self, robot, goal_pose, control_parameters):
        State.__init__(self, outcomes=['succeeded', 'failed'])
        self.robot = robot
        self.goal_pose = goal_pose
        self.params = control_parameters

        self._rate = rospy.Rate(10)
        self._goal_position_tolerance = 0.01
        self._goal_rotation_tolerance = 0.1

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
        return pose.pose.position.x, pose.pose.position.y, _wrap_angle_pi(yaw)

    def _goal_reached(self, dx, dy, dyaw):
        return math.hypot(dx, dy) < self._goal_position_tolerance and abs(dyaw) < self._goal_rotation_tolerance


class OpenDishwasher(StateMachine):
    def __init__(self, robot, dishwasher_id):
        StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        @cb_interface(outcomes=['done'])
        def _pre_grab_handle(ud):
            robot.leftArm.send_gripper_goal("open", timeout=0)
            robot.leftArm._send_joint_trajectory([[0, 0.2519052373022729913, 0.7746500794619434, 1.3944848321343395,
                                                   -1.829999276180074, 0.6947045024700284, 0.1889253710114966]],
                                                 timeout=rospy.Duration(0))
            return 'done'

        @cb_interface(outcomes=['done'])
        def _grab_handle(ud):
            robot.leftArm.wait_for_motion_done()
            robot.speech.speak('I hope this goes right!', block=False)
            robot.speech.speak('Ah, at least it is worth a try for 50 points, here we go!', block=False)
            robot.head.look_at_point(VectorStamped(y=100, frame_id="/" + robot.robot_name + "/base_link"))
            fs = frame_stamped("dishwasher", 0.41, 0, 0.83, roll=-math.pi / 2, pitch=0, yaw=math.pi)
            robot.leftArm.send_goal(fs.projectToFrame(robot.robot_name + "/base_link", robot.tf_listener))
            robot.leftArm.send_gripper_goal("close")
            robot.head.reset()
            robot.speech.speak('Okay, now what!', block=False)
            return 'done'

        @cb_interface(outcomes=['done'])
        def _align_with_dishwasher(ud):
            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = dishwasher_id
            goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.pi))
            goal_pose.pose.position.x = 0.85
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3)).execute({})
            return 'done'

        @cb_interface(outcomes=['done'])
        def _drive_to_open_dishwasher(ud):
            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = dishwasher_id
            goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.pi))
            goal_pose.pose.position.x = 1.4
            goal_pose.pose.position.y = 0.2

            robot.torso.low()
            ControlToPose(robot, goal_pose, ControlParameters(0.8, 1.0, 0.5, 0.1, 0.1)).execute({})

            goal_pose.pose.position.x = 1.6
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.15, 0.1, 0.1)).execute({})

            robot.leftArm.reset()
            robot.torso.reset()
            return 'done'

        with self:
            self.add_auto('PRE_GRAB_HANDLE', CBState(_pre_grab_handle), ['done'])
            self.add_auto('ALIGN_WITH_DISHWASHER', CBState(_align_with_dishwasher), ['done'])
            self.add_auto('GRAB_HANDLE', CBState(_grab_handle), ['done'])
            self.add('DRIVE_TO_OPEN_DISHWASHER', CBState(_drive_to_open_dishwasher), transitions={'done': 'succeeded'})


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
    robot.leftArm.reset()
    robot.torso.reset()

    sm = TestOpenDishwasher(robot, 'dishwasher', 'in_front_of')
    sm.execute()
