#
# Copyright (c) 2019, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import math
import os
import sys

import PyKDL
import rospy
from geometry_msgs.msg import PoseStamped
from robot_skills import Hero
from robot_smach_states import NavigateToSymbolic, Say, WaitTime, ForceDrive
from robot_smach_states.navigation.control_to_pose import ControlParameters, ControlToPose
from robot_smach_states.util.designators import EdEntityDesignator
from smach import StateMachine, cb_interface, CBState
from tf_conversions import toMsg


class PlaceItemOnTable(StateMachine):
    def __init__(self, robot, table_id):
        StateMachine.__init__(self, outcomes=['succeeded', 'failed'], input_keys=["item_picked"])
        arm = robot.get_arm()._arm

        def send_joint_goal(position_array, wait_for_motion_done=True):
            arm._send_joint_trajectory([position_array], timeout=rospy.Duration(0))
            if wait_for_motion_done:
                arm.wait_for_motion_done()

        def send_gripper_goal(open_close_string):
            arm.send_gripper_goal(open_close_string)
            rospy.sleep(1.0)  # Does not work with motion_done apparently

        @cb_interface(outcomes=['done'])
        def _pre_place(_):
            send_joint_goal([0.69, 0, 0, 0, 0])
            send_joint_goal([0.69, -1.2, 0, -1.57, 0])
            return 'done'

        @cb_interface(outcomes=['done'], input_keys=["item_picked"])
        def _align_with_table(user_data):
            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = table_id

            frame = PyKDL.Frame(
                PyKDL.Rotation.RPY(0, 0, -math.pi / 2),
                PyKDL.Vector(0, 0.75, 0)
            )

            item_vector_dict = {
                "plate": PyKDL.Vector(0, 0, 0),
                "cup": PyKDL.Vector(0.2, -0.2, 0),
                "knife": PyKDL.Vector(0, -0.2, 0),
                "fork": PyKDL.Vector(0, 0.2, 0),
                "spoon": PyKDL.Vector(0, -0.25, 0)
            }

            item_placement_vector = item_vector_dict[user_data["item_picked"]]
            item_frame = frame
            item_frame.T = frame * item_placement_vector
            rospy.loginfo("Placing {i} at frame ({f}) * item_placement_vector ({ipv}) = {ift}".format(
                i=user_data["item_picked"],
                f=frame,
                ipv=item_placement_vector,
                ift=item_frame.T))

            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = table_id
            goal_pose.pose = toMsg(item_frame)
            rospy.loginfo("Placing {} at {}".format(user_data["item_picked"], goal_pose))
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute({})
            return 'done'

        @cb_interface(outcomes=['done'])
        def _place_and_retract(_):
            send_joint_goal([0.64, -1.57, 0, -1.57, 0])
            send_gripper_goal("open")
            send_joint_goal([0.69, 0, -1.57, 0, 0])
            send_gripper_goal("close")
            arm.send_joint_goal("carrying_pose")
            return 'done'

        @cb_interface(outcomes=['done'], input_keys=["item_picked"])
        def _ask_user(user_data):
            send_joint_goal([0, 0, 0, 0, 0])

            item_name = user_data["item_picked"]

            robot.speech.speak("Please put the {} on the table.".format(item_name))
            robot.speech.speak("Watch out, I will open my gripper now")
            send_gripper_goal("open")
            rospy.sleep(5.0)
            robot.speech.speak("Thanks for that!", block=False)
            send_gripper_goal("open")
            arm.send_joint_goal("carrying_pose", timeout=0)
            return 'done'

        with self:
            self.add_auto('PRE_PLACE', CBState(_pre_place), ['done'])
            self.add_auto('ALIGN_WITH_TABLE', CBState(_align_with_table), ['done'])
            self.add('PLACE_AND_RETRACT', CBState(_place_and_retract), transitions={'done': 'succeeded'})
            # self.add('ASK_USER', PLACE_AND_RETRACT(_ask_user), transitions={'done': 'succeeded'})


class NavigateToAndPlaceItemOnTable(StateMachine):
    def __init__(self, robot, table_id, table_navigation_area, table_close_navigation_area):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"], input_keys=["item_picked"])

        table = EdEntityDesignator(robot=robot, id=table_id)

        with self:
            StateMachine.add("NAVIGATE_TO_TABLE",
                             NavigateToSymbolic(robot, {table: table_navigation_area}, table),
                             transitions={'arrived': 'NAVIGATE_TO_TABLE_CLOSE',
                                          'unreachable': 'failed',
                                          'goal_not_defined': 'failed'})

            StateMachine.add("NAVIGATE_TO_TABLE_CLOSE",
                             NavigateToSymbolic(robot, {table: table_close_navigation_area}, table),
                             transitions={'arrived': 'PLACE_ITEM_ON_TABLE',
                                          'unreachable': 'FORCE_DRIVE',
                                          'goal_not_defined': 'failed'})

            StateMachine.add("FORCE_DRIVE",
                             ForceDrive(robot, -0.1, 0, 0, 5.0),
                             transitions={'done': 'SAY_PICK_AWAY_THE_CHAIR'})

            StateMachine.add("SAY_PICK_AWAY_THE_CHAIR",
                             Say(robot,
                                 "Please pick away the chair, I cannot get close enough to the {}".format(table_id)),
                             transitions={'spoken': 'WAIT_FOR_PICK_AWAY_CHAIR'})

            StateMachine.add('WAIT_FOR_PICK_AWAY_CHAIR',
                             WaitTime(robot, 5),
                             transitions={'waited': 'SAY_THANKS', 'preempted': 'failed'})

            StateMachine.add('SAY_THANKS',
                             Say(robot, "Thank you darling"),
                             transitions={'spoken': 'NAVIGATE_TO_TABLE_CLOSE'})

            StateMachine.add("PLACE_ITEM_ON_TABLE", PlaceItemOnTable(robot, table_id),
                             transitions={'succeeded': 'succeeded',
                                          'failed': 'failed'})


if __name__ == '__main__':
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    hero = Hero()
    hero.reset()
    state_machine = NavigateToAndPlaceItemOnTable(hero, 'kitchen_table', 'right_of', 'right_of_close')
    state_machine.userdata['item_picked'] = sys.argv[1]
    state_machine.execute()
