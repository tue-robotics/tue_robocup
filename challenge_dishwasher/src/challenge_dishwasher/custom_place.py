import rospy
import tf2_geometry_msgs
from robot_skills.amigo import Amigo
from robot_smach_states import NavigateToSymbolic
from robot_smach_states.util.designators import EdEntityDesignator, Designator
from smach import StateMachine, cb_interface, CBState


_ = tf2_geometry_msgs


class CustomPlace(StateMachine):
    def __init__(self, robot, dishwasher_id, arm):
        StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        @cb_interface(outcomes=['done'])
        def pre_place_pose(ud):
            arm.resolve()._send_joint_trajectory([[0, 0.2519052373022729913, 0.7746500794619434, 1.3944848321343395,
                                                   -1.829999276180074, 0.6947045024700284, 0.1889253710114966]],
                                                 timeout=rospy.Duration(0))
            return 'done'

        @cb_interface(outcomes=['done'])
        def move_closer(ud):
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

    sm = TestCustomPlace(robot, 'dishwasher', 'in_front_of')
    sm.execute()
