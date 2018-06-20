import rospy
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Quaternion
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from robot_skills.amigo import Amigo
from robot_smach_states import NavigateToSymbolic
from robot_smach_states.util.designators import EdEntityDesignator, Designator
from smach import StateMachine, cb_interface, CBState
from challenge_dishwasher.open_dishwasher import ControlToPose, ControlParameters


_ = tf2_geometry_msgs


class CustomPlace(StateMachine):
    def __init__(self, robot, dishwasher_id, arm):
        StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        @cb_interface(outcomes=['done'])
        def pre_place_pose(ud):
            arm.resolve()._send_joint_trajectory([[-0.92, 0.044, 0.80, 0.297, 0.934, -0.95, 0.4]],
                                                 timeout=rospy.Duration(0))
            arm.resolve().wait_for_motion_done()
            return 'done'

        @cb_interface(outcomes=['done'])
        def move_closer(ud):
            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = dishwasher_id
            goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 1.9656259917))
            goal_pose.pose.position.x = -4.55638664735
            goal_pose.pose.position.y = 4.99959353359
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.01, 0.1)).execute({})

            arm.resolve()._send_joint_trajectory([[-0.92, 0.044, 0.80, 0.497, 0.934, -0.95, 0.4]],
                                                 timeout=rospy.Duration(0))
            robot.torso._send_goal([0.35])
            robot.torso.wait_for_motion_done()
            return 'done'

        @cb_interface(outcomes=['done'])
        def place_pose(ud):
            return 'done'

        @cb_interface(outcomes=['done'])
        def open_gripper(ud):
            arm.resolve().send_gripper_goal("open", timeout=0)
            return 'done'

        @cb_interface(outcomes=['done'])
        def move_away(ud):
            return 'done'

        @cb_interface(outcomes=['done'])
        def reset_arms(ud):
            arm.resolve().reset()
            return 'done'

        with self:
            self.add_auto('PRE_PLACE_POSE', CBState(pre_place_pose), ['done'])
            self.add_auto('MOVE_CLOSER', CBState(move_closer), ['done'])
            self.add_auto('PLACE_POSE', CBState(place_pose), ['done'])
            self.add_auto('OPEN_GRIPPER', CBState(open_gripper), ['done'])
            self.add_auto('PRE_PLACE_POSE2', CBState(pre_place_pose), ['done'])
            self.add_auto('MOVE_AWAY', CBState(move_away), ['done'])
            self.add('RESET_ARMS', CBState(reset_arms), transitions={'done': 'succeeded'})


class TestCustomPlace(StateMachine):
    def __init__(self, robot, dishwasher_id, dishwasher_navigate_area):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        dishwasher = EdEntityDesignator(robot=robot, id=dishwasher_id)
        arm = Designator(robot.leftArm)

        with self:
            StateMachine.add("NAVIGATE_TO_DISHWASHER",
                             NavigateToSymbolic(robot, {dishwasher: dishwasher_navigate_area}, dishwasher),
                             transitions={'arrived': 'OPEN_DISHWASHER',
                                          'unreachable': 'failed',
                                          'goal_not_defined': 'failed'})

            StateMachine.add("OPEN_DISHWASHER",
                             CustomPlace(robot, dishwasher_id, arm),
                             transitions={'succeeded': 'succeeded',
                                          'failed': 'failed'})


if __name__ == '__main__':
    rospy.init_node('test_open_dishwasher')

    robot = Amigo()
    robot.ed.reset()
    robot.leftArm.reset()
    robot.torso.reset()

    sm = TestCustomPlace(robot, 'dishwasher', 'to_the_side_of')
    sm.execute()
