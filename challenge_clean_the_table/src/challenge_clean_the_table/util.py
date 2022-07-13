#
# Copyright (c) 2022, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import math

import PyKDL
import rospy
from geometry_msgs.msg import PoseStamped
from tf_conversions import toMsg


def item_vector_to_item_frame(item_vector):
    frame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, math.pi))

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
