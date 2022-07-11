#
# Copyright (c) 2022, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import math

import PyKDL
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA
from tf_conversions import toMsg

REQUIRED_ITEMS = ["spoon", "bowl", "milk_carton", "cereal_box"]

ITEM_VECTOR_DICT = {
    "spoon": PyKDL.Vector(0, -0.15, 0),
    "bowl": PyKDL.Vector(0, 0, 0),
    "milk_carton": PyKDL.Vector(0, 0.3, 0),
    "cereal_box": PyKDL.Vector(0, -0.3, 0),
}

COLOR_DICT = {
    "spoon": ColorRGBA(1, 0, 1, 1),
    "bowl": ColorRGBA(0, 1, 1, 1),
    "milk_carton": ColorRGBA(0, 0, 1, 1),
    "cereal_box": ColorRGBA(1, 1, 0, 1),
}

PICK_ROTATION = 3.14

JOINTS_HANDOVER = [0.4, -0.2, 0.0, -1.37, 0]

JOINTS_PRE_PRE_PLACE = [0.69, 0, 0, -0.7, 0]

JOINTS_PRE_PLACE_HORIZONTAL = [0.8, -1.2, 0, 0, 0]
JOINTS_PLACE_HORIZONTAL = [0.7, -1.75, 0, 0, 0]

JOINTS_PRE_PLACE_VERTICAL = [0.8, -1.2, 0, -1.57, 0]
JOINTS_PLACE_VERTICAL = [0.7, -1.57, 0, -1.57, 0]

JOINTS_RETRACT = [0.7, 0, 0, -1.57, 0]

JOINTS_POST_PICK = [0.7, -1.2, 0, 0, 0]

JOINTS_PRE_POUR = [0.7, -1.2, -1.5, 0, 0]

JOINTS_POUR = [0.7, -1.2, -1.8, 0, 0]

POUR_OFFSET_X = 0
POUR_OFFSET_Y = 0.2


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
