# System
import yaml

# ROS
import rospy
import PyKDL as kdl

from robot_skills.util.kdl_conversions import poseMsgToKdlFrame, kdlFrameToPoseMsg
from robot_skills.util.volume import volumes_from_entity_info_data
from robot_skills.util.shape import shape_from_entity_info


class Entity(object):
    """ Holds all data concerning entities

    """
    def __init__(self, identifier, object_type, frame_id, pose, shape, volumes, super_types, last_update_time):
        """ Constructor

        :param identifier: str with the id of this entity
        :param object_type: str with the type of this entity
        :param frame_id: str frame id w.r.t. which the pose is defined
        :param pose: kdl.Frame with the pose of this entity
        :param shape: Shape of this entity
        :param volumes: dict mapping strings to Areas
        :param super_types: list with strings representing super types in an ontology of object types
        """
        self.id = identifier
        self.type = object_type
        self.frame_id = frame_id
        self._pose = pose
        self.shape = shape
        self._volumes = volumes if volumes else {}
        self.super_types = super_types
        self._last_update_time = last_update_time

    @property
    def volumes(self):
        return self._volumes

    @property
    def last_update_time(self):
        return self._last_update_time

    def distance_to_2d(self, point):
        """
        Calculate the distance between this entity's pose and the given point.
        :param point: kdl.Vector, assumed to be in the same frame_id as the entity itself
        :return: the distance between the entity's pose and the point

        >>> e = Entity("dummy", None, None, kdl.Frame(kdl.Rotation.RPY(1, 0, 0), kdl.Vector(3, 3, 3)), None, {}, None, 0)
        >>> point = kdl.Vector(1, 1, 1)
        >>> e.distance_to_2d(point)
        2.8284271247461903
        """

        # The length of the difference vector between the pose's position and the point
        difference = self._pose.p - point
        difference.z(0)
        return difference.Norm()

    def distance_to_3d(self, point):
        """
        Calculate the distance between this entity's pose and the given point.
        :param point: kdl.Vector, assumed to be in the same frame_id as the entity itself
        :return: the distance between the entity's pose and the point

        >>> e = Entity("dummy", None, None, kdl.Frame(kdl.Rotation.RPY(1, 0, 0), kdl.Vector(3, 3, 3)), None, {}, None, 0)
        >>> point = kdl.Vector(1, 1, 1)
        >>> e.distance_to_3d(point)
        3.4641016151377544
        """
        return (self._pose.p - point).Norm()

    def is_a(self, super_type):
        """
        Check whether the entity is a (subclass of) some supertype
        :param super_type: str representing the name of the super_type
        :return: bool True if the entity is a (sub)type of the given super_type

        >>> e = Entity("dummy", "coffee_table", None, None, None, {}, ["coffee_table", "table", "furniture", "thing"], 0)
        >>> e.is_a("furniture")
        True
        >>> e.is_a("food")
        False
        """
        return super_type in self.super_types

    @property
    def pose(self):
        """ Returns the pose (at this point, as a geometry_msgs.Pose, in the future as a kdl frame """
        return kdlFrameToPoseMsg(self._pose)

    @pose.setter
    def pose(self, pose):
        """ Setter """
        self._pose = poseMsgToKdlFrame(pose)


def from_entity_info(e):
    """ Converts ed.msg.EntityInfo to an Entity

    :param e: ed.msg.EntityInfo
    :return: Entity
    """
    identifier = e.id
    object_type = e.type
    frame_id = "/map"  # ED has all poses in map
    pose = poseMsgToKdlFrame(e.pose)
    shape = shape_from_entity_info(e)

    last_update_time = e.last_update_time.to_sec()

    # The data is a string but can be parsed as yaml, which then represent is a much more usable data structure
    volumes = volumes_from_entity_info_data(yaml.load(e.data))
    rospy.logdebug("Entity(id={id}) has volumes {vols} ".format(id=identifier, vols=volumes.keys()))

    super_types = e.types

    # TODO: this must be part of the definition of the entity in ED.
    if e.has_shape and "amigo" not in e.id and "sergio" not in e.id and e.id is not "floor" and e.id is not "walls":
        super_types += ["furniture"]

    if 'possible_human' in e.flags:
        super_types += ["possible_human"]

    return Entity(identifier=identifier, object_type=object_type, frame_id=frame_id, pose=pose, shape=shape,
                  volumes=volumes, super_types=super_types, last_update_time=last_update_time)

if __name__ == "__main__":
    import doctest
    doctest.testmod()
