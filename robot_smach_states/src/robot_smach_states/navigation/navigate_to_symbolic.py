from __future__ import absolute_import

# TU/e Robotics
from . import NavigateToDesignator
from ..util.designators.navigation import SymbolicConstraintsDesignator, RoomConstraintsDesignator


class NavigateToSymbolic(NavigateToDesignator):
    """ Navigation class to navigate to a semantically annotated goal, e.g., in front of the dinner table.
    """
    def __init__(self, robot, entity_designator_area_name_map, entity_lookat_designator, speak=True):
        """
        Constructor

        :param robot: robot object
        :param entity_designator_area_name_map: dictionary mapping EdEntityDesignators to a string or designator
            resolving to a string, representing the area, e.g., entity_designator_area_name_map[<EdEntity>] = 'in_front_of'.
        :param entity_lookat_designator: EdEntityDesignator defining the entity the robot should look at. This is used
            to compute the orientation constraint.
        :param speak: Optional parameter to (not) let the robot speak. default=True
        """
        constraint_designator = SymbolicConstraintsDesignator(robot, entity_designator_area_name_map, entity_lookat_designator)
        super(NavigateToSymbolic, self).__init__(robot, constraint_designator, speak=speak)


class NavigateToRoom(NavigateToSymbolic):
    """
    Navigation class to navigate to the 'in' area of the provided entity, typically a room.
    """
    def __init__(self, robot, entity_designator_room, entity_lookat_designator=None, speak=True):
        """
        :param robot: robot object
        :type robot: robot
        :param entity_designator_room: Designator to the room resolving to an ED entity
        :param entity_lookat_designator: (Optional) Designator defining the entity the robot should look at. This is
            used to compute the orientation constraint. If not provided, the entity_designator_room is used.
        :param speak: Optional parameter to (not) let the robot speak. default=True
        """
        constraint_designator = RoomConstraintsDesignator(robot, entity_designator_room, entity_lookat_designator)
        super(NavigateToRoom, self).__init__(robot, constraint_designator, speak=speak)
