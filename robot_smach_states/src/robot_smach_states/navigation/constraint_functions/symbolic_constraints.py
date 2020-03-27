from __future__ import absolute_import

# ROS
import rospy


def symbolic_constraint(robot, entity_designator_area_name_map):
    """
    Generate the constraint based on an entity and an area of that entity.
    :param robot: robot object
    :param entity_designator_area_name_map: dictionary mapping EdEntityDesignators to a string or designator
        resolving to a string, representing the area, e.g., entity_designator_area_name_map[<EdEntity>] = 'in_front_of'.
    :return: (tuple(PositionConstraint, OrientationConstraint)). If one of the entities does not resolve,
        None is returned.
    """
    entity_id_area_name_map = {}
    for desig, area_name in entity_designator_area_name_map.iteritems():
        entity = desig.resolve()
        try:
            area_name = area_name.resolve()
        except:
            pass
        if entity:
            entity_id_area_name_map[entity.id] = area_name
        else:
            rospy.logerr("Designator {0} in entity_designator_area_name_map resolved to {1}.".format(desig, entity))
            # Put a None item in the dict. We check on that and if there's a None, something failed.
            entity_id_area_name_map[entity] = area_name

    if None in entity_id_area_name_map:
        rospy.logerr("At least 1 designator in self.entity_designator_area_name_map failed")
        return None

    rospy.logdebug("Navigating to symbolic {}".format(entity_id_area_name_map))

    pc = robot.ed.navigation.get_position_constraint(entity_id_area_name_map)
    oc = None
    return pc, oc


def room_constraint(robot, room_designator):
    """
    :param robot: robot object
    :param room_designator: Designator to the room
    """
    symbolic_constraint(robot, {room_designator: "in"})
