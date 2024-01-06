#
# Copyright (c) 2022, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import copy
import os
import sys

import rospy
import visualization_msgs.msg
from geometry_msgs.msg import PoseStamped, Vector3
from pykdl_ros import FrameStamped

from challenge_serve_breakfast.tuning import (
    get_item_place_pose,
    ITEM_VECTOR_DICT,
    item_vector_to_item_frame,
    item_frame_to_pose,
    COLOR_DICT,
    REQUIRED_ITEMS,
)
from robot_skills import get_robot
from robot_smach_states.human_interaction import Say
from robot_smach_states.navigation import NavigateToSymbolic, ForceDrive
from robot_smach_states.navigation.control_to_pose import ControlParameters, ControlToPose
from robot_smach_states.util.designators import EdEntityDesignator
from robot_smach_states.utility import WaitTime
from smach import StateMachine, cb_interface, CBState


class PlaceItemOnTable(StateMachine):
    def __init__(self, robot, table_id, retract = True):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"], input_keys=["item_picked"])
        # noinspection PyProtectedMember
        arm = robot.get_arm()._arm
        self.retract = retract
        def send_goal(pose_goal, wait_for_motion_done=True):
            arm.send_goal(pose_goal, timeout=0.0)
            if wait_for_motion_done:
                arm.wait_for_motion_done()

        def send_gripper_goal(open_close_string, wait_for_motion_done=True):
            arm.gripper.send_goal(open_close_string)

        @cb_interface(outcomes=["done"], input_keys=["item_picked"])
        def _place(user_data):
            item_name = user_data["item_picked"]
            rospy.loginfo(f"Preplacing {item_name}...")

            robot.speech.speak(f"I am going to place the {item_name}", block=False)

            robot.head.look_up()
            robot.head.wait_for_motion_done()

            item_place_pose = get_item_place_pose(user_data["item_picked"])
            place_fs = FrameStamped(item_place_pose, rospy.Time(0), table_id)

            pre_place_fs = copy.deepcopy(place_fs)
            pre_place_fs.frame.p.z(place_fs.frame.p.z() + 0.1)

            post_place_fs = copy.deepcopy(place_fs)
            post_place_fs.frame.p.z(place_fs.frame.p.z() + 0.2)

            rospy.loginfo("Pre Placing...")
            send_goal(pre_place_fs)
            robot.head.look_down()
            robot.head.look_up()
            robot.head.wait_for_motion_done()

            rospy.loginfo("Placing...")
            send_goal(place_fs)
            send_gripper_goal("open")

            if item_name != "cereal_box" or self.retract:
                rospy.loginfo("Retracting...")
                send_goal(post_place_fs)
                robot.base.force_drive(-0.1, 0, 0, 3)  # Drive backwards at 0.1m/s for 3s, so 30cm
                send_gripper_goal("close", wait_for_motion_done=False)
                arm.send_joint_goal("carrying_pose")

            #if item_name in ["milk_carton", "cereal_box"]:
            #    send_joint_goal(JOINTS_PRE_PLACE_HORIZONTAL)
            #else:
            #    send_joint_goal(JOINTS_PRE_PLACE_VERTICAL)

            #rospy.loginfo("Placing...")
            #item_name = user_data["item_picked"]
            #if item_name in ["milk_carton"]:
            #    send_joint_goal(JOINTS_PLACE_HORIZONTAL_MILK)
            #elif item_name in ["milk_carton", "cereal_box"]:
            #    send_joint_goal(JOINTS_PLACE_HORIZONTAL)
            #else:
            #    send_joint_goal(JOINTS_PLACE_VERTICAL)

            robot.head.reset()

            return "done"

        with self:
            self.add("PLACE", CBState(_place), {"done": "succeeded"})


class NavigateToAndPlaceItemOnTable(StateMachine):
    def __init__(self, robot, table_id, table_close_navigation_area):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"], input_keys=["item_picked"])

        item_frames = {k: item_vector_to_item_frame(v) for k, v in ITEM_VECTOR_DICT.items()}
        item_poses = {k: item_frame_to_pose(v, table_id) for k, v in item_frames.items()}

        marker_array_pub = rospy.Publisher('/markers', visualization_msgs.msg.MarkerArray, queue_size=1, latch=True)

        table = EdEntityDesignator(robot=robot, uuid=table_id)

        with self:
            StateMachine.add_auto(
                "PUBLISH_MARKERS", CBState(_publish_item_poses, cb_args=[marker_array_pub, item_poses]), ["done"]
            )
            StateMachine.add(
                "NAVIGATE_TO_TABLE_CLOSE",
                NavigateToSymbolic(robot, {table: table_close_navigation_area}, table, speak=False),
                transitions={
                    "arrived": "PLACE_ITEM_ON_TABLE",
                    "unreachable": "SAY_PICK_AWAY_THE_CHAIR",
                    "goal_not_defined": "failed",
                },
            )

            StateMachine.add(
                "SAY_PICK_AWAY_THE_CHAIR",
                Say(robot, "Please pick away the chair, I cannot get close enough to the {}".format(table_id)),
                transitions={"spoken": "WAIT_FOR_PICK_AWAY_CHAIR"},
            )

            StateMachine.add(
                "WAIT_FOR_PICK_AWAY_CHAIR",
                WaitTime(robot, 5),
                transitions={"waited": "SAY_THANKS", "preempted": "failed"},
            )

            StateMachine.add(
                "SAY_THANKS", Say(robot, "Thank you darling"), transitions={"spoken": "NAVIGATE_TO_TABLE_CLOSE"}
            )

            StateMachine.add(
                "PLACE_ITEM_ON_TABLE",
                PlaceItemOnTable(robot, table_id, retract=False),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )

@cb_interface(outcomes=["done"])
def _publish_item_poses(user_data, marker_array_pub, items):
    """
    Publishes item poses as a visualization marker array

    :param items: (dict) ...
    """
    array_msg = visualization_msgs.msg.MarkerArray()

    marker_id = 1234
    for k, posestamped in items.items():
        posestamped = posestamped  # type: PoseStamped

        marker_id += 1
        marker_msg = visualization_msgs.msg.Marker()
        marker_msg.header.frame_id = posestamped.header.frame_id
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.id = marker_id
        marker_msg.type = visualization_msgs.msg.Marker.SPHERE
        marker_msg.action = 0
        marker_msg.pose = posestamped.pose
        marker_msg.pose.position.z = 1.0
        marker_msg.scale = Vector3(0.05, 0.05, 0.05)
        marker_msg.color = COLOR_DICT[k]
        array_msg.markers.append(marker_msg)

        marker_id += 1
        marker_msg2 = copy.deepcopy(marker_msg)
        marker_msg2.id = marker_id
        marker_msg2.type = visualization_msgs.msg.Marker.TEXT_VIEW_FACING
        marker_msg2.pose.position.z += 0.1
        marker_msg2.text = k
        array_msg.markers.append(marker_msg2)

    marker_array_pub.publish(array_msg)

    return "done"


if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] not in REQUIRED_ITEMS:
        print(f"Please specify an item, e.g {REQUIRED_ITEMS}")
        sys.exit(1)

    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])

    robot_instance = get_robot("hero")
    state_machine = NavigateToAndPlaceItemOnTable(robot_instance, "kitchen_table", "in_front_of")
    state_machine.userdata["item_picked"] = sys.argv[1]
    state_machine.execute()
