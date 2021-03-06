from __future__ import absolute_import

# TU/e Robotics
from robot_skills.util.entity import Entity
from robot_skills.util.kdl_conversions import FrameStamped
from .navigation import NavigateTo
from .constraint_functions import arms_reach_constraint
from ..util.designators import check_resolve_type, AttrDesignator


class NavigateToGrasp(NavigateTo):
    def __init__(self, robot, arm_designator, entity_designator):
        """"
        Navigate so that the arm can easily grasp the entity

        :param robot: robot object
        :param arm_designator: which arm to eventually grasp with?
        :param entity_designator: designator that resolves to an Ed Entity
        """
        check_resolve_type(entity_designator, Entity)  # Check that the entity_designator resolves to an Entity
        pose_designator = AttrDesignator(entity_designator, 'pose', resolve_type=FrameStamped)

        super(NavigateToGrasp, self).__init__(robot,
                                              lambda: arms_reach_constraint(pose_designator=pose_designator,
                                                                            look=True,
                                                                            arm_designator=arm_designator),
                                              reset_head=False)
