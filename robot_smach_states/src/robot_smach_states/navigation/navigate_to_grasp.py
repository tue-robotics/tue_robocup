from __future__ import absolute_import

# TU/e Robotics
from . import NavigateToDesignator
from ..util.designators.navigation import GraspConstraintsDesignator


class NavigateToGrasp(NavigateToDesignator):
    def __init__(self, robot, entity_designator, arm_designator=None):
        constraint_designator = GraspConstraintsDesignator(robot, entity_designator, arm_designator)
        super(NavigateToGrasp, self).__init__(robot, constraint_designator, reset_head=False)
