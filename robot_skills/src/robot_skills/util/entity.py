# System
import yaml

# ROS
import rospy
import PyKDL as kdl

# TU/e Robotics
from ed_msgs.msg import EntityInfo
from robot_skills.util.kdl_conversions import pose_msg_to_kdl_frame, FrameStamped
from robot_skills.util.shape import shape_from_entity_info
from robot_skills.util.volume import volumes_from_entity_volumes_msg


class Entity(object):
    """ Holds all data concerning entities

    """
    def __init__(self, identifier, object_type, frame_id, pose, shape, volumes, super_types, last_update_time,
                 person_properties=None):
        """ Constructor

        :param identifier: str with the id of this entity
        :param object_type: str with the type of this entity
        :param frame_id: str frame id w.r.t. which the pose is defined
        :param pose: kdl.Frame with the pose of this entity
        :param shape: Shape of this entity
        :param volumes: dict mapping strings to Volume
        :param super_types: list with strings representing super types in an ontology of object types
        """
        self.id = identifier
        self.type = object_type
        self.frame_id = frame_id
        self._pose = pose
        self.shape = shape
        self._volumes = volumes if volumes else {}
        self.super_types = super_types if super_types else []
        self._last_update_time = last_update_time

        self._person_properties = person_properties

    @property
    def volumes(self):
        return self._volumes

    def in_volume(self, point, volume_id):
        """ Checks if the point is in the volume identified by the volume id

        :param point: VectorStamped with the point to check
        :param volume_id: string with the volume
        :return: boolean indicating whether the point is in the designated volume. If an error occurs, False is returned
        """
        # Check if the volume exists
        if volume_id not in self._volumes:
            rospy.logdebug("{} not a volume of {}".format(volume_id, self.id))
            return False

        # Transform the point
        fid1 = point.frame_id if point.frame_id[0] != "/" else point.frame_id[1:]  # Remove slash for comparison
        fid2 = self.frame_id if self.frame_id[0] != "/" else self.frame_id[1:]  # Remove slash for comparison
        if fid1 != fid2:
            rospy.logerr("Cannot compute with volume and entity defined w.r.t. different frame: {} and {}".format(
                point.frame_id, self.frame_id
            ))
            return False
        vector = self._pose.Inverse() * point.vector

        # Check if the point is inside of the volume
        return self._volumes[volume_id].contains(vector)

    def entities_in_volume(self, entities, volume_id):
        """
        Filter the collection of entities down to only those in the given volume

        :param entities: collection/sequence of entities
        :type entities: List[Entity]
        :param volume_id: volume these entities need to be in
        :type volume_id: str
        :return: entities that are both in the given volume and in the list 'entities'
        :rtype: List[Entities]
        """

        entities = [e for e in entities if self.in_volume(e.pose.extractVectorStamped(), volume_id)]

        return entities

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
        """ Returns the pose of the Entity as a FrameStamped"""
        return FrameStamped(frame=self._pose, frame_id=self.frame_id)

    @pose.setter
    def pose(self, pose):
        """ Setter """
        self._pose = pose_msg_to_kdl_frame(pose)

    @property
    def person_properties(self):
        if self._person_properties:
            return self._person_properties
        else:
            rospy.logwarn("{} is not a person".format(self))
            return None

    @person_properties.setter
    def person_properties(self, value):
        self._person_properties = value

    def __repr__(self):
        return "Entity(id='{id}', type='{type}', frame={frame}, person_properties={pp})"\
            .format(id=self.id, type=self.type, frame=self.pose, pp=self._person_properties)


class PersonProperties(object):
    def __init__(self, name, age, emotion, gender, gender_confidence, pointing_pose, posture, reliability, shirt_colors,
                 tags, tagnames, velocity, parent_entity):
        #ToDo: "In the legacy message definition a field named tagnames was defined but never used. Only tags is used.
        # Once the message definition in people_recognition is cleaned up then tagnames should be removed completely."
        """
        Container for several properties related to a person

        :param name: the person's name. This is separate from the entity, which is unique while this doesn't have to be
        :param age: Estimated age of the person
        :param emotion: str indicating the emotion
        :param gender: Predicted gender of the person
        :param gender_confidence: Confidence of the classifier in the gender above.
        :param pointing_pose: In which direction is the person pointing
        :param posture: String with a value like 'sitting', 'laying', 'standing' etc.
        :param reliability:  ?
        :param shirt_colors: list of 3 shirt colors, sorted from most dominant to less dominant
        :param tags: Other tags
        :param tagnames: Other tagnames
        :param velocity: Velocity with which the person in moving
        :param parent_entity: The Entity that these properties belong to
        """
        self._name = name
        self.age = age
        self.emotion = emotion
        self.gender = gender
        self.gender_confidence = gender_confidence
        self.pointing_pose = pointing_pose
        self.posture = posture
        self.reliability = reliability
        self.shirt_colors = shirt_colors
        self.tags = tags
        self.tagnames = tagnames
        self.velocity = velocity

        self._parent_entity = parent_entity

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        rospy.loginfo("Changing {}'s name to {}".format(self._parent_entity.id, value))
        self._name = value

    def __repr__(self):
        return "PersonProperties(age='{age}', gender='{g}', gender_confidence={gc}, shirt_colors={sc})"\
            .format(age=self.age, g=self.gender, gc=self.gender_confidence, sc=self.shirt_colors)


def from_entity_info(e):
    """ Converts ed_msgs.msg.EntityInfo to an Entity

    :param e: ed_msgs.msg.EntityInfo
    :return: Entity
    """
    assert isinstance(e, EntityInfo)
    identifier = e.id
    object_type = e.type
    frame_id = "/map"  # ED has all poses in map
    pose = pose_msg_to_kdl_frame(e.pose)
    shape = shape_from_entity_info(e)

    last_update_time = e.last_update_time.to_sec()

    # The data is a string but can be parsed as yaml, which then represent is a much more usable data structure
    volumes = volumes_from_entity_volumes_msg(e.volumes)
    rospy.logdebug("Entity(id={id}) has volumes {vols} ".format(id=identifier, vols=volumes.keys()))

    super_types = e.types

    # TODO: this must be part of the definition of the entity in ED.
    if e.has_shape and not any([name in e.id for name in ["amigo", "sergio", "hero"]])\
        and e.id != "floor" and "wall" not in e.id:
        super_types += ["furniture"]

    if 'possible_human' in e.flags:
        super_types += ["possible_human"]

    entity = Entity(identifier=identifier, object_type=object_type, frame_id=frame_id, pose=pose, shape=shape,
                  volumes=volumes, super_types=super_types, last_update_time=last_update_time)

    if e.type == 'person':
        try:
            pp_dict = yaml.load(e.data)
            del pp_dict['position']
            del pp_dict['header']
            entity.person_properties = PersonProperties(parent_entity=entity, **pp_dict)
        except TypeError as te:
            rospy.logerr("Cannot instantiate PersonProperties from {}: {}".format(e.data, te))

    return entity


if __name__ == "__main__":
    import doctest
    doctest.testmod()
