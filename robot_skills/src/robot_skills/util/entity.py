# ROS
import PyKDL as kdl

from robot_skills.src.robot_skills.util.area import areas_from_entity_info_data
from robot_skills.src.robot_skills.util.convex_hull import convex_hull_from_entity_info


class Entity(object):
    """ Holds all data concerning entities

    """
    def __init__(self, identifier, object_type, frame_id, pose, convex_hull, volumes, super_types):
        """ Constructor

        :param identifier: string with the id of this entity
        :param object_type: string with the type of this entity
        :param frame_id: frame id w.r.t. which the pose is defined
        :param pose: kdl frame with the pose of this entity
        :param convex_hull: ConvexHull
        :param volumes: dict mapping strings to Areas
        :param super_types: list with strings representing super types in an ontology of object types
        """
        self.id = identifier
        self.type = object_type
        self.frame_id = frame_id
        self.pose = pose
        self.convex_hull = convex_hull
        self._volumes = volumes
        self.volumes = volumes.keys()
        self.super_types = super_types


def from_entity_info(e):
    """ Converts ed.msg.EntityInfo to an Entity

    :param e: ed.msg.EntityInfo
    :return: Entity
    """
    identifier = e.id
    object_type = e.type
    frame_id = "/map"  # ED has all poses in map
    # ToDo: pose
    pose = kdl.Frame()
    convex_hull = convex_hull_from_entity_info(e)
    areas = areas_from_entity_info_data(e.data)
    super_types = e.types
    return Entity(identifier=identifier, object_type=object_type, frame_id=frame_id, pose=pose, convex_hull=convex_hull,
                  volumes=areas, super_types=super_types)
