from __future__ import absolute_import

from typing import Optional

# TU/e Robotics
from ed.entity import Entity
from pykdl_ros import FrameStamped
from .navigation import NavigateTo
from .constraint_functions import arms_reach_constraint, room_constraint, combine_constraints
from ..util.designators import  check_resolve_type
from robot_smach_states.util.designators.core import Designator


class NavigateToPlace(NavigateTo):
    def __init__(self, robot, place_pose_designator, arm_designator, room: Optional[Designator[Entity]] = None, reset_head=True, speak=True, reset_pose=True):
        """
        Navigate so that the arm can reach the place point

        :param robot: robot object
        :param place_pose_designator designator that resolves to a FrameStamped
        :param arm_designator: which arm to eventually place with
        :param room: (Optional) Designator to the room you want to stay in
        :param reset_head: Whether or not the head should be used for obstacle avoidance during navigation.
        :param speak: Whether or not the robot should speak during navigation
        :param reset_pose: Whether or not the robot is allowed to change its pose for navigation.
        """
        check_resolve_type(place_pose_designator, FrameStamped)

        constraint_list = [lambda: arms_reach_constraint(pose_designator=place_pose_designator,
                                                         arm_designator=arm_designator,
                                                         look=True)]
        if room is not None:
            check_resolve_type(room, Entity)
            constraint_list.append(lambda: room_constraint(robot, room))

        super(NavigateToPlace, self).__init__(robot,
                                              lambda: combine_constraints(constraint_list),
                                              reset_head=reset_head,
                                              speak=speak,
                                              reset_pose=reset_pose)
