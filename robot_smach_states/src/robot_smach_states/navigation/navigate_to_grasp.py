from __future__ import absolute_import

# TU/e Robotics
from . import NavigateToDesignator
from ..util.designators.navigation import ArmsreachConstraintsDesignator


class NavigateToGrasp(NavigateToDesignator):
    """Navigate so that the arm can reach the grasp entity
    :param robot: robot object
    :param entity_designator: designator that resolves to an Ed Entity
    :param arm_designator: which arm to eventually grasp with?
    """
    def __init__(self, robot, entity_designator, arm_designator=None):
        constraint_designator = ArmsreachConstraintsDesignator(robot, entity_designator, arm_designator, look=True)
        super(NavigateToGrasp, self).__init__(robot, constraint_designator, reset_head=False)
