from __future__ import absolute_import

# TU/e Robotics
from . import NavigateToDesignator
from ..util.designators.navigation import CompoundConstraintsDesignator, RadiusConstraintsDesignator,\
    LookAtConstraintsDesignator


class NavigateToObserve(NavigateToDesignator):
    """ Navigates to a radius from an ed entity. If a convex hull is present, the distance
    to the convex hull is used. Otherwise, the distance to the pose of the entity is used

    """
    def __init__(self, robot, entity_designator, radius=0.7, margin=0.075):
        """
        :param robot: (Robot) object
        :param entity_designator: EdEntityDesignator for the object to observe
        :param radius: (float) desired distance to the pose of the entity [m]
        :param margin: (float) allowed margin w.r.t. specified radius on both sides [m]
        """
        constraint_designator = CompoundConstraintsDesignator()
        constraint_designator.add(RadiusConstraintsDesignator(entity_designator, radius, margin), 'area')
        constraint_designator.add(LookAtConstraintsDesignator(entity_designator), 'lookat')
        super(NavigateToObserve, self).__init__(robot, constraint_designator)
