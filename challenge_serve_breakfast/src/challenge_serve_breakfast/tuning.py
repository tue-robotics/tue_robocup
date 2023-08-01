#
# Copyright (c) 2022, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import copy
import math

import PyKDL
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA
from tf_conversions import toMsg

REQUIRED_ITEMS = ["spoon", "bowl", "milk_carton", "cereal_box"]

# pose of the breakfast on the table
BREAKFAST_POSE = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, math.pi), PyKDL.Vector(0.7, 0, 0.76))

# vectors of the items with respect to the breakfast frame
ITEM_VECTOR_DICT = {
    "spoon": PyKDL.Vector(0.0, -0.1, 0),
    "bowl": PyKDL.Vector(0.0, 0.0, 0),
    "milk_carton": PyKDL.Vector(0.0, 0.15, 0),
    "cereal_box": PyKDL.Vector(0.0, -0.2, 0),
}

# frame indicating the pose of the hand with respect to the vector in ITEM_VECTOR_DICT
ITEM_OFFSET_DICT = {
    "spoon": PyKDL.Frame(PyKDL.Rotation.RPY(0, 0.5*math.pi, 0), PyKDL.Vector(0.0, 0.0, 0.1)),
    "bowl": PyKDL.Frame(PyKDL.Rotation.RPY(0.5*math.pi, 0.25*math.pi, 0.0), PyKDL.Vector(-0.08, 0.0, 0.07)),
    "milk_carton": PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0.0, 0.0, 0.07)),
    "cereal_box": PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0.0, 0.0, 0.07)),
}

POUR_OFFSET_DICT = {
    "bowl": [PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0.0, -0.07, 0.20)),
             PyKDL.Frame(PyKDL.Rotation.RPY(-0.5*math.pi, 0, 0), PyKDL.Vector(0.0, -0.07, 0.20)),
             PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0.0, -0.07, 0.20))]
}

COLOR_DICT = {
    "spoon": ColorRGBA(1, 0, 1, 1),
    "bowl": ColorRGBA(0, 1, 1, 1),
    "milk_carton": ColorRGBA(0, 0, 1, 1),
    "cereal_box": ColorRGBA(1, 1, 0, 1),
}

JOINTS_HANDOVER = [0.4, -0.2, 0.0, -1.37, 0]

JOINTS_POST_PICK = [0.7, -1.2, 0, 0, 0]

def item_vector_to_item_frame(item_vector):
    frame = copy.deepcopy(BREAKFAST_POSE)

    item_placement_vector = item_vector
    item_frame = frame
    item_frame.p = frame * item_placement_vector
    rospy.loginfo(
        "Placing at frame ({f}) * item_placement_vector ({ipv}) = {itf}".format(
            f=frame, ipv=item_placement_vector, itf=item_frame
        )
    )

    return item_frame


def get_item_place_pose(item_name):
    item_vector = ITEM_VECTOR_DICT[item_name]

    item_frame = BREAKFAST_POSE
    item_frame.p = BREAKFAST_POSE * item_vector

    item_place_offset = ITEM_OFFSET_DICT[item_name]
    item_place_pose = item_frame * item_place_offset
    rospy.loginfo(f"Placing at frame {item_frame} with place pose {item_place_pose}")

    return item_place_pose

def get_item_pour_poses(item_name):
    item_vector = ITEM_VECTOR_DICT[item_name]

    item_frame = BREAKFAST_POSE
    item_frame.p = BREAKFAST_POSE * item_vector

    item_pour_offsets = POUR_OFFSET_DICT[item_name]
    item_pour_poses = []
    for offset in item_pour_offsets:
        item_pour_poses.append(item_frame * offset)
    return item_pour_poses


def item_frame_to_pose(item_frame, frame_id):
    goal_pose = PoseStamped()
    goal_pose.header.stamp = rospy.Time()
    goal_pose.header.frame_id = frame_id
    goal_pose.pose = toMsg(item_frame)

    return goal_pose
