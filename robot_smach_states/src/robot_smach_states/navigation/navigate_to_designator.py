from __future__ import absolute_import

# TU/e Robotics
from .navigation import NavigateTo
from ..util.designators.navigation import PoseConstraint
from ..util.designators import check_resolve_type


class NavigateToDesignator(NavigateTo):
    def __init__(self, robot, constraint_designator, reset_head=True, speak=True):
        """
        @param constraint_designator a Designator that resolves to navigation constraints
        """
        super(NavigateToDesignator, self).__init__(robot, reset_head=reset_head, speak=speak)

        check_resolve_type(constraint_designator, PoseConstraint)
        self.constraint_designator = constraint_designator

    def generateConstraint(self):
        return self.constraint_designator.resolve()
