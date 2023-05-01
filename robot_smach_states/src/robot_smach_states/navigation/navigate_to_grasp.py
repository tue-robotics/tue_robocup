from __future__ import absolute_import

# TU/e Robotics
from ed.entity import Entity
from pykdl_ros import FrameStamped
from .navigation import NavigateTo
from .constraint_functions import arms_reach_constraint
from ..util.designators import check_resolve_type, AttrDesignator


class NavigateToGrasp(NavigateTo):
    def __init__(self, robot, arm_designator, entity_designator, room=None, reset_head=True, speak=True, reset_pose=True):
        """"
        Navigate so that the arm can easily grasp the entity

        :param robot: robot object
        :param arm_designator: which arm to eventually grasp with?
        :param entity_designator: designator that resolves to an Ed Entity
        :param room: (Optional) Designator to the room you want to stay in
        :param reset_head: Whether or not the head should be used for obstacle avoidance during navigation.
        :param speak: Whether or not the robot should speak during navigation
        :param reset_pose: Whether or not the robot is allowed to change its pose for navigation.
        """
        check_resolve_type(entity_designator, Entity)  # Check that the entity_designator resolves to an Entity
        pose_designator = AttrDesignator(entity_designator, 'pose', resolve_type=FrameStamped)

        constraint_list = [lambda: arms_reach_constraint(pose_designator=pose_designator,
                                                         look=True,
                                                         arm_designator=arm_designator)]
        if room:
            constraint_list.append(lambda: symbolic_constraint(robot, {room: "in"}))

        super(NavigateToGrasp, self).__init__(robot, lambda: combine_constraints(constraint_list), reset_head=reset_head, speak=speak, reset_pose=reset_pose)
