# ROS
from geometry_msgs.msg import *
import rospy

# TU/e Robotics
from cb_planner_msgs_srvs.msg import *
from robot_skills.util.entity import Entity
from robot_skills.util.kdl_conversions import kdl_vector_to_point_msg
from robot_smach_states.navigation import NavigateTo
from robot_smach_states.util.designators import check_resolve_type


# ----------------------------------------------------------------------------------------------------


class NavigateToSymbolic(NavigateTo):
    """ Navigation class to navigate to a semantically annotated goal, e.g., in front of the dinner table.
    """
    def __init__(self, robot, entity_designator_area_name_map, entity_lookat_designator):
        """
        Constructor

        :param robot: robot object
        :param entity_designator_area_name_map: dictionary mapping EdEntityDesignators to a string or designator
            resolving to a string, representing the area, e.g., entity_designator_area_name_map[<EdEntity>] = 'in_front_of'.
        :param entity_lookat_designator: EdEntityDesignator defining the entity the robot should look at. This is used
            to compute the orientation constraint.
        """
        super(NavigateToSymbolic, self).__init__(robot)

        self.robot = robot

        # Check that the entity_designator_area_name_map's keys all resolve to EntityInfo's
        assert(all(entity_desig.resolve_type == Entity for entity_desig in entity_designator_area_name_map.keys()))
        self.entity_designator_area_name_map = entity_designator_area_name_map

        check_resolve_type(entity_lookat_designator, Entity)  # Check that the entity_designator resolves to an Entity
        self.entity_lookat_designator = entity_lookat_designator

    def generateConstraint(self):
        """
        Generates the position constraint
        """
        return self.generate_constraint(self.robot, self.entity_designator_area_name_map, self.entity_lookat_designator)

    @staticmethod
    def generate_constraint(robot, entity_designator_area_name_map, entity_lookat_designator):
        """
        Staticmethod generating the position and orientation constraint.
        By implementing this as a staticmethod, it can also be used for other purposes.

        :param robot: robot object
        :param entity_designator_area_name_map: dictionary mapping EdEntityDesignators to a string or designator
            resolving to a string, representing the area, e.g., entity_designator_area_name_map[<EdEntity>] = 'in_front_of'.
        :param entity_lookat_designator: EdEntityDesignator defining the entity the robot should look at. This is used
            to compute the orientation constraint.
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

        # Orientation constraint is the entity itself...
        entity_lookat = entity_lookat_designator.resolve()
        if not entity_lookat:
            rospy.logerr("Could not resolve entity_lookat_designator".format(entity_lookat_designator))
            return None

        look_at = kdl_vector_to_point_msg(entity_lookat.pose.extractVectorStamped().vector)
        oc = OrientationConstraint(look_at=look_at, frame=entity_lookat.pose.frame_id)

        return pc, oc


class NavigateToRoom(NavigateToSymbolic):
    """
    Navigation class to navigate to the 'in' area of the provided entity, typically a room.
    """
    def __init__(self, robot, entity_designator_room, entity_lookat_designator=None):
        """
        :param robot: robot object
        :type robot: robot
        :param entity_designator_room: Designator to the room
        :type entity_designator_room: Designator resolving to an ED entity
        :param entity_lookat_designator: (Optional) Designator defining the entity the robot should look at. This is
            used to compute the orientation constraint. If not provided, the entity_designator_room is used.
        :type entity_lookat_designator: Designator resolving to an ED entity
        """
        room_area = "in"
        if not entity_lookat_designator:
            entity_lookat_designator = entity_designator_room
        super(NavigateToRoom, self).__init__(robot, {entity_designator_room: room_area}, entity_lookat_designator)
