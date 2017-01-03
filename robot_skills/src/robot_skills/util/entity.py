# System
import yaml

# ROS
import PyKDL as kdl

from robot_skills.util.kdl_conversions import pose_msg_to_kdl_frame
from robot_skills.util.volume import volumes_from_entity_info_data
from robot_skills.util.convex_hull import convex_hull_from_entity_info


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

    def distance_to(self, point):
        """
        Calculate the distance between this entity's pose and the given point.
        :param point: kdl.Vector, assumed to be in the same frame_id as the entity itself
        :return: the distance between the entity's pose and the point

        >>> e = Entity("dummy", None, None, kdl.Frame(kdl.Rotation.RPY(1, 0, 0), kdl.Vector(3, 3, 3)), None, {}, None)
        >>> point = kdl.Vector(1, 1, 1)
        >>> e.distance_to(point)
        3.4641016151377544
        """

        # The length of the difference vector between the pose's position and the point
        return (self.pose.p - point).Norm()


def from_entity_info(e):
    """ Converts ed.msg.EntityInfo to an Entity

    :param e: ed.msg.EntityInfo
    :return: Entity
    """
    identifier = e.id
    object_type = e.type
    frame_id = "/map"  # ED has all poses in map
    pose = pose_msg_to_kdl_frame(e.pose)
    convex_hull = convex_hull_from_entity_info(e)

    # The data is a string but can be parsed as yaml, which then represent is a much more usable data structure
    volumes = volumes_from_entity_info_data(yaml.load(e.data))

    super_types = e.types
    return Entity(identifier=identifier, object_type=object_type, frame_id=frame_id, pose=pose, convex_hull=convex_hull,
                  volumes=volumes, super_types=super_types)

if __name__ == "__main__":
    import doctest
    doctest.testmod()
