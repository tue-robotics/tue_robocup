#
# Copyright (c) 2019, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

from __future__ import print_function

import math
import os
import sys

import PyKDL
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA
from tf_conversions import toMsg

from challenge_set_the_table.knowledge import TABLE_ID, TABLE_NAVIGATION_AREA, PLACEMENT_HEIGHT
from robot_skills import get_robot
from robot_smach_states.human_interaction import Say
from robot_smach_states.navigation import NavigateToSymbolic, ForceDrive
from robot_smach_states.navigation.control_to_pose import ControlParameters, ControlToPose
from robot_smach_states.util.designators import EdEntityDesignator
from robot_smach_states.utility import WaitTime
from smach import StateMachine, cb_interface, CBState

item_vector_dict = {
    "plate": PyKDL.Vector(0.05, 0.2, 0),
    "cup": PyKDL.Vector(0.2, -0.2, 0),
    "knife": PyKDL.Vector(0.0, -0.2, 0),
    "fork": PyKDL.Vector(0.0, 0.25, 0),
    "spoon": PyKDL.Vector(0.0, -0.2, 0),
    "bowl": PyKDL.Vector(0.05, -0.05, 0),  # Must go on top of the plate
    "napkin": PyKDL.Vector(0.05, 0.35, 0)  # besides the fork
}

color_dict = {
    "plate": ColorRGBA(1, 0, 0, 1),
    "cup": ColorRGBA(0, 1, 0, 1),
    "knife": ColorRGBA(0, 0, 1, 1),
    "fork": ColorRGBA(1, 1, 0, 1),
    "spoon": ColorRGBA(1, 0, 1, 1),
    "bowl": ColorRGBA(0, 1, 1, 1),
    "napkin": ColorRGBA(0.5, 0, 0.5, 1)
}

pboven = [0.69, -1.5, -1.4, -1.5, -0.3]
pleg = [0.62, -1.75, -1.4, -1.5, 0.3]
pweg = [0.70, -1.45, -0.2, -1.5, 0.3]


def item_vector_to_item_frame(item_vector):
    frame = PyKDL.Frame(
        PyKDL.Rotation.RPY(0, 0, -math.pi / 2),
        PyKDL.Vector(-0.05, 0.75, 0)
    )

    item_placement_vector = item_vector
    item_frame = frame
    item_frame.p = frame * item_placement_vector
    rospy.loginfo("Placing at frame ({f}) * item_placement_vector ({ipv}) = {itf}".format(
        f=frame,
        ipv=item_placement_vector,
        itf=item_frame))

    return item_frame


def item_frame_to_pose(item_frame, frame_id):
    goal_pose = PoseStamped()
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.header.frame_id = frame_id
    goal_pose.pose = toMsg(item_frame)

    return goal_pose


class PlaceItemOnTable(StateMachine):
    def __init__(self, robot, table_id, placement_height):
        StateMachine.__init__(self, outcomes=['succeeded', 'failed'], input_keys=["item_picked"])
        # noinspection PyProtectedMember
        arm = robot.get_arm()._arm
        self.placement_height = placement_height

        def send_joint_goal(position_array, wait_for_motion_done=True):
            # noinspection PyProtectedMember
            arm._send_joint_trajectory([position_array], timeout=rospy.Duration(0))
            if wait_for_motion_done:
                arm.wait_for_motion_done()

        def send_gripper_goal(open_close_string):
            arm.gripper.send_goal(open_close_string)
            rospy.sleep(1.0)  # Does not work with motion_done apparently

        @cb_interface(outcomes=['done'], input_keys=["item_picked"])
        def _pre_place(user_data):
            rospy.loginfo("Preplacing...")
            robot.head.look_up()
            robot.head.wait_for_motion_done()

            item_name = user_data["item_picked"]
            send_joint_goal([0.69, 0, 0, 0, 0])

            robot.speech.speak(f"I wanna {item_name} on the table")

            if item_name in ['plate', 'napkin']:
                send_joint_goal(pboven)
            else:
                send_joint_goal([0.69, -1.2, 0, -1.57, 0])
            return 'done'

        @cb_interface(outcomes=['done'], input_keys=["item_picked"])
        def _align_with_table(user_data):
            item_placement_vector = item_vector_dict[user_data["item_picked"]]
            item_frame = item_vector_to_item_frame(item_placement_vector)

            goal_pose = item_frame_to_pose(item_frame, table_id)
            rospy.loginfo("Placing {} at {}".format(user_data["item_picked"], goal_pose))
            robot.head.look_down()
            robot.head.wait_for_motion_done()
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute({})
            return 'done'

        @cb_interface(outcomes=['done'], input_keys=["item_picked"])
        def _place_and_retract(user_data):
            rospy.loginfo("Placing...")
            item_name = user_data["item_picked"]
            if item_name in ['plate', 'napkin']:
                # TODO: Do a different joint goal/trajectory
                send_joint_goal(pleg)
            else:
                send_joint_goal([self.placement_height, -1.57, 0, -1.57, 0])

            rospy.loginfo("Dropping...")
            send_gripper_goal("open")
            send_joint_goal(pweg)
            robot.head.look_up()
            robot.head.wait_for_motion_done()

            if item_name == 'napkin':
                robot.base.force_drive(0, 0, -0.5, 0.1)
                robot.base.force_drive(0, 0, 0.5, 0.1)
                robot.base.force_drive(0, 0, -0.5, 0.1)
                robot.base.force_drive(0, 0, 0.5, 0.1)
                robot.base.force_drive(0, 0, -0.5, 0.1)
                robot.base.force_drive(0, 0, 0.5, 0.1)

            rospy.loginfo("Retract...")
            send_joint_goal([0.69, 0, -1.57, 0, 0])
            send_gripper_goal("close")
            robot.base.force_drive(-0.1, 0, 0, 3)  # Drive backwards at 0.1m/s for 3s, so 30cm
            arm.send_joint_goal("carrying_pose")
            return 'done'

        @cb_interface(outcomes=['done'], input_keys=["item_picked"])
        def _ask_user(user_data):
            robot.head.look_up()
            robot.head.wait_for_motion_done()
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


class NavigateToAndPlaceItemOnTable(StateMachine):
    def __init__(self, robot, table_id, table_navigation_area):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"], input_keys=["item_picked"])

        table = EdEntityDesignator(robot=robot, uuid=table_id)

        with self:
            StateMachine.add("NAVIGATE_TO_TABLE_CLOSE",
                             NavigateToSymbolic(robot, {table: table_navigation_area}, table),
                             transitions={'arrived': 'PLACE_ITEM_ON_TABLE',
                                          'unreachable': 'SAY_PICK_AWAY_THE_CHAIR',
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
                             transitions={'spoken': 'ROTATE'})

            StateMachine.add(
                "ROTATE",
                ForceDrive(robot, 0.0, 0, 0.5, math.pi / 0.5),
                transitions={"done": "NAVIGATE_TO_TABLE_CLOSE"},
            )

            StateMachine.add("PLACE_ITEM_ON_TABLE", PlaceItemOnTable(robot, table_id, PLACEMENT_HEIGHT),
                             transitions={'succeeded': 'succeeded',
                                          'failed': 'failed'})


if __name__ == '__main__':
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])

    item_frames = {k: item_vector_to_item_frame(v) for k, v in item_vector_dict.items()}
    item_poses = {k: item_frame_to_pose(v, 'kitchen_table') for k, v in item_frames.items()}

    robot_instance = get_robot("hero")

    state_machine = NavigateToAndPlaceItemOnTable(robot_instance, TABLE_ID, TABLE_NAVIGATION_AREA)
    state_machine.userdata['item_picked'] = sys.argv[1]
    state_machine.execute()
