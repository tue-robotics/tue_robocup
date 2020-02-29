from __future__ import absolute_import

# ROS

# TU/e Robotics
from .navigation import NavigateTo
from ..util.designators import check_resolve_type


class NavigateToDesignator(NavigateTo):
    def __init__(self, robot, constraint_designator):
        """
        @param constraint_designator a Designator that resolves to navigation constraints
        """
        super(NavigateToDesignator, self).__init__(robot)

        check_resolve_type(constraint_designator, tuple)  # Check that the constraint designator resolves to a tuple
        self.constraint_designator = constraint_designator

    def generateConstraint(self):
        return self.constraint_designator.resolve()
