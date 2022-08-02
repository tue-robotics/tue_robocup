#
# Copyright (c) 2022, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import copy
import math
import os
import sys

import PyKDL
import rospy
import visualization_msgs.msg
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import ColorRGBA
from tf_conversions import toMsg

from robot_skills import get_robot
from robot_smach_states.human_interaction import Say
from robot_smach_states.navigation import NavigateToSymbolic, ForceDrive
from robot_smach_states.navigation.control_to_pose import ControlParameters, ControlToPose
from robot_smach_states.util.designators import EdEntityDesignator
from robot_smach_states.utility import WaitTime
from smach import StateMachine, cb_interface, CBState

item_vector_dict = {
    "spoon": PyKDL.Vector(0, -0.25, 0),
    "bowl": PyKDL.Vector(0, 0, 0),
    "milk_carton": PyKDL.Vector(0, -0.2, 0),
    "cereal_box": PyKDL.Vector(0, 0.15, 0),
}

color_dict = {
    "spoon": ColorRGBA(1, 0, 1, 1),
    "bowl": ColorRGBA(0, 1, 1, 1),
    "milk_carton": ColorRGBA(0, 0, 1, 1),
    "cereal_box": ColorRGBA(1, 1, 0, 1),
}

p_pre_place_horizontal = [0.69, -1.2, 0, -1.57, 0]
p_place_horizontal = [0.58, -1.75, -1.4, -1.5, 0.3]
p_post_place_horizontal = [0.69, -1.75, -1.0, -1.5, 0.3]

p_pre_place_vertical = [0.69, -1.2, 0, -1.57, 0.3]
p_place_vertical = [0.58, -1.75, -1.4, -1.5, 0.3]
p_post_place_vertical = [0.69, -1.75, -1.0, -1.5, 0.3]

p_retract = [0.69, 0, -1.57, 0, 0]


def item_vector_to_item_frame(item_vector):
    frame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, -math.pi / 2), PyKDL.Vector(-0.05, 0.75, 0))

    item_placement_vector = item_vector
    item_frame = frame
    item_frame.p = frame * item_placement_vector
    rospy.loginfo(
        "Placing at frame ({f}) * item_placement_vector ({ipv}) = {itf}".format(
            f=frame, ipv=item_placement_vector, itf=item_frame
        )
    )

    return item_frame


def item_frame_to_pose(item_frame, frame_id):
    goal_pose = PoseStamped()
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.header.frame_id = frame_id
    goal_pose.pose = toMsg(item_frame)

    return goal_pose


class PlaceItemOnTable(StateMachine):
    def __init__(self, robot, table_id):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"], input_keys=["item_picked"])
        # noinspection PyProtectedMember
        arm = robot.get_arm()._arm

        def send_joint_goal(position_array, wait_for_motion_done=True):
            # noinspection PyProtectedMember
            arm._send_joint_trajectory([position_array], timeout=rospy.Duration(0))
            if wait_for_motion_done:
                arm.wait_for_motion_done()

        def send_gripper_goal(open_close_string):
            arm.gripper.send_goal(open_close_string)
            rospy.sleep(1.0)  # Does not work with motion_done apparently

        @cb_interface(outcomes=["done"], input_keys=["item_picked"])
        def _pre_place(user_data):
            item_name = user_data["item_picked"]
            rospy.loginfo(f"Preplacing {item_name}...")
            robot.head.look_up()
            robot.head.wait_for_motion_done()

            if item_name in ["milk_carton", "cereal_box"]:
                send_joint_goal(p_pre_place_horizontal)
            else:
                send_joint_goal(p_pre_place_vertical)

            return "done"

        @cb_interface(outcomes=["done"], input_keys=["item_picked"])
        def _align_with_table(user_data):
            item_placement_vector = item_vector_dict[user_data["item_picked"]]
            item_frame = item_vector_to_item_frame(item_placement_vector)

            goal_pose = item_frame_to_pose(item_frame, table_id)
            rospy.loginfo("Placing {} at {}".format(user_data["item_picked"], goal_pose))
            robot.head.look_down()
            robot.head.wait_for_motion_done()
            ControlToPose(robot, goal_pose, ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.02, 0.1)).execute({})
            return "done"

        @cb_interface(outcomes=["done"], input_keys=["item_picked"])
        def _place_and_retract(user_data):
            rospy.loginfo("Placing...")
            item_name = user_data["item_picked"]
            if item_name in ["milk_carton", "cereal_box"]:
                send_joint_goal(p_place_horizontal)
            else:
                send_joint_goal(p_place_vertical)

            rospy.loginfo("Dropping...")
            send_gripper_goal("open")
            send_joint_goal(p_post_place_vertical)
            robot.head.look_up()
            robot.head.wait_for_motion_done()

            rospy.loginfo("Retract...")
            send_joint_goal(p_retract)
            send_gripper_goal("close")
            robot.base.force_drive(-0.1, 0, 0, 3)  # Drive backwards at 0.1m/s for 3s, so 30cm
            arm.send_joint_goal("carrying_pose")
            return "done"

        with self:
            self.add_auto("PRE_PLACE", CBState(_pre_place), ["done"])
            self.add_auto("ALIGN_WITH_TABLE", CBState(_align_with_table), ["done"])
            self.add("PLACE_AND_RETRACT", CBState(_place_and_retract), transitions={"done": "succeeded"})


class NavigateToAndPlaceItemOnTable(StateMachine):
    def __init__(self, robot, table_id, table_close_navigation_area):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"], input_keys=["item_picked"])

        table = EdEntityDesignator(robot=robot, uuid=table_id)

        with self:
            StateMachine.add(
                "NAVIGATE_TO_TABLE_CLOSE",
                NavigateToSymbolic(robot, {table: table_close_navigation_area}, table),
                transitions={
                    "arrived": "PLACE_ITEM_ON_TABLE",
                    "unreachable": "SAY_PICK_AWAY_THE_CHAIR",
                    "goal_not_defined": "failed",
                },
            )

            StateMachine.add(
                "FORCE_DRIVE", ForceDrive(robot, -0.1, 0, 0, 5.0), transitions={"done": "SAY_PICK_AWAY_THE_CHAIR"}
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
                PlaceItemOnTable(robot, table_id),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )


def _publish_item_poses(robot, items):
    """
    Publishes item poses as a visualization marker array

    :param robot: (Robot) robot API object
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
        marker_msg.pose.position.z += 1.0
        marker_msg.scale = Vector3(0.05, 0.05, 0.05)
        marker_msg.color = color_dict[k]
        marker_msg.lifetime = rospy.Duration(30.0)
        array_msg.markers.append(marker_msg)

        marker_id += 1
        marker_msg2 = copy.deepcopy(marker_msg)
        marker_msg2.id = marker_id
        marker_msg2.type = visualization_msgs.msg.Marker.TEXT_VIEW_FACING
        marker_msg2.pose.position.z += 0.1
        marker_msg2.text = k
        array_msg.markers.append(marker_msg2)

    robot.marker_array_pub.publish(array_msg)


if __name__ == "__main__":
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])

    item_frames = {k: item_vector_to_item_frame(v) for k, v in item_vector_dict.items()}
    import pprint

    print("Item frames:")
    pprint.pprint(item_frames)

    item_poses = {k: item_frame_to_pose(v, "kitchen_table") for k, v in item_frames.items()}
    print("Item poses:")
    pprint.pprint(item_poses)

    robot_instance = get_robot("hero")
    _publish_item_poses(robot_instance, item_poses)
    robot_instance.reset()

    state_machine = NavigateToAndPlaceItemOnTable(robot_instance, "kitchen_table", "right_of_close")
    state_machine.userdata["item_picked"] = sys.argv[1]
    state_machine.execute()
