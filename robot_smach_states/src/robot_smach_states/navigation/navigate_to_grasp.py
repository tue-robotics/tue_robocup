from __future__ import absolute_import

# TU/e Robotics
from ed.entity import Entity
from pykdl_ros import FrameStamped
from .navigation import NavigateTo
from .constraint_functions import arms_reach_constraint, symbolic_constraint, combine_constraints
from ..util.designators import check_resolve_type, AttrDesignator


class NavigateToGrasp(NavigateTo):
    def __init__(self, robot, arm_designator, entity_designator, room=None):
        """"
        Navigate so that the arm can easily grasp the entity

        :param robot: robot object
        :param arm_designator: which arm to eventually grasp with?
        :param entity_designator: designator that resolves to an Ed Entity
        :param room: (Optional) Designator to the room you want to stay in
        """
        check_resolve_type(entity_designator, Entity)  # Check that the entity_designator resolves to an Entity
        pose_designator = AttrDesignator(entity_designator, 'pose', resolve_type=FrameStamped)
        if room:
            check_resolve_type(room, Entity)

        constraint_list = [lambda: arms_reach_constraint(pose_designator=pose_designator,
                                                         look=True,
                                                         arm_designator=arm_designator)]
        if room:
            constraint_list.append(lambda: symbolic_constraint(robot, {room: "in"}))

        super(NavigateToGrasp, self).__init__(robot, lambda: combine_constraints(constraint_list), reset_head=False)
