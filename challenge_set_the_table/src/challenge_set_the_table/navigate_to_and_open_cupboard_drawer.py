#
# Copyright (c) 2019, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import math
import os

import rospy
from robot_smach_states.navigation.control_to_pose import ControlParameters, ControlToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from robot_skills import Hero
from robot_smach_states import NavigateToSymbolic
from robot_smach_states.util.designators import EdEntityDesignator
from smach import StateMachine, cb_interface, CBState
from tf.transformations import quaternion_from_euler


class OpenCupboard(StateMachine):
    def __init__(self, robot, cupboard_id):
        StateMachine.__init__(self, outcomes=['succeeded', 'failed'])
        arm = robot.get_arm()._arm

        def send_joint_goal(position_array, wait_for_motion_done=True):
            arm._send_joint_trajectory([position_array], timeout=rospy.Duration(0))
            if wait_for_motion_done:
                arm.wait_for_motion_done()

        def send_gripper_goal(open_close_string):
            arm.send_gripper_goal(open_close_string)
            rospy.sleep(1.0)  # Does not work with motion_done apparently

        @cb_interface(outcomes=['done'])
        def _pre_grab_handle(_):
            arm.send_gripper_goal("open", timeout=0)
            send_joint_goal([0, -0, 0, -1.57, 1.57])
            return 'done'

        @cb_interface(outcomes=['done'])
        def _align_with_cupboard(_):
            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = cupboard_id
            goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.pi))
            goal_pose.pose.position.x = 0.85
            goal_pose.pose.position.y = 0
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute({})
            return 'done'

        @cb_interface(outcomes=['done'])
        def _grab_handle(_):
            robot.speech.speak('I hope this goes right!', block=False)
            send_joint_goal([0, -0.5, 0, -1.07, 1.57])
            send_gripper_goal("close")
            return 'done'

        @cb_interface(outcomes=['done'])
        def _drive_to_open_cupboard(_):
            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = cupboard_id
            goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.pi))
            goal_pose.pose.position.x = 1.4
            goal_pose.pose.position.y = 0
            ControlToPose(robot, goal_pose, ControlParameters(0.8, 1.0, 0.15, 0.1, 0.1, 0.05, 0.5)).execute({})

            arm.send_joint_goal("carrying_pose")

            return 'done'

        with self:
            self.add_auto('PRE_GRAB_HANDLE', CBState(_pre_grab_handle), ['done'])
            self.add_auto('ALIGN_WITH_CUPBOARD', CBState(_align_with_cupboard), ['done'])
            self.add_auto('GRAB_HANDLE', CBState(_grab_handle), ['done'])
            self.add('DRIVE_TO_OPEN_CUPBOARD', CBState(_drive_to_open_cupboard), transitions={'done': 'succeeded'})


class NavigateToAndOpenCupboard(StateMachine):
    def __init__(self, robot, cupboard_id, cupboard_navigation_area):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        cupboard = EdEntityDesignator(robot=robot, id=cupboard_id)

        with self:
            StateMachine.add("NAVIGATE_TO_CUPBOARD",
                             NavigateToSymbolic(robot, {cupboard: cupboard_navigation_area}, cupboard),
                             transitions={'arrived': 'failed',
                                          'unreachable': 'failed',
                                          'goal_not_defined': 'failed'})

            StateMachine.add("OPEN_CUPBOARD", OpenCupboard(robot, cupboard_id),
                             transitions={'succeeded': 'succeeded',
                                          'failed': 'failed'})


if __name__ == '__main__':
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    hero = Hero()
    hero.reset()
    NavigateToAndOpenCupboard(hero, 'cabinet', 'in_front_of').execute()
