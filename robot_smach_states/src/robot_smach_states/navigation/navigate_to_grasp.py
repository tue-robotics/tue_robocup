from __future__ import absolute_import

# TU/e Robotics
from robot_skills.util.entity import Entity
from robot_skills.util.kdl_conversions import FrameStamped
from .navigation import NavigateTo
from .constraint_functions import arms_reach_constraint
from ..util.designators import check_resolve_type, AttrDesignator


class NavigateToGrasp(NavigateTo):
    """"
    Navigate so that the arm can easily grasp the entity

    :param robot: robot object
    :param entity_designator: designator that resolves to an Ed Entity
    """
    def __init__(self, robot, entity_designator):
        check_resolve_type(entity_designator, Entity)  # Check that the entity_designator resolves to an Entity
        pose_designator = AttrDesignator(entity_designator, 'pose', resolve_type=FrameStamped)

        super(NavigateToGrasp, self).__init__(robot,
                                              lambda userdata: arms_reach_constraint(pose_designator=pose_designator,
                                                                                     look=True,
                                                                                     arm=userdata.arm),
                                              reset_head=False,
                                              input_keys=["arm"],
                                              output_keys=["arm"])
