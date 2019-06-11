# System
import enum
import math
from collections import namedtuple, OrderedDict

# ROS
import PyKDL as kdl
import rospy
import smach

# TU/e Robotics
from robot_skills.robot import Robot
from robot_skills.util.entity import Entity
from robot_skills.util.kdl_conversions import point_msg_to_kdl_vector, VectorStamped

# Robot Smach States
from robot_smach_states.navigation.navigate_to_symbolic import NavigateToSymbolic
from robot_smach_states.util.designators.ed_designators import Designator


class Side(enum.Enum):
    LEFT = "left"
    RIGHT = "right"


DirectionItem = namedtuple("DirectionItem", ["object_id", "room_id", "side"])


def _get_entity_pose_in_path(point, next_point, entity_pose):
    # type: (kdl.Vector, kdl.Vector, kdl.Frame) -> kdl.Frame
    """
    Computes the entity pose w.r.t. the virtual frame that is spanned by the two points
    :param point: First point in fixed frame
    :type point: kdl.Vector
    :param next_point: Next point in fixed frame
    :type next_point: kdl.Vector
    :param entity_pose: (kdl.Frame) Entity pose in fixed frame
    :type entity_pose: kdl.Frame
    :return: Entity pose in virtual 'path' frame
    :rtype: kdl.Frame
    """
    # Determine a 6D path pose
    path_pose = create_frame_from_points(point, next_point)

    # Transform entity pose into path frame
    entity_pose_path = path_pose.Inverse() * entity_pose

    return entity_pose_path


def get_directions(robot, entity_designator, x_threshold=0.75, y_threshold=1.5):
    # type: (Robot, Designator, float, float) -> list
    """
    Computes a list of named tuples of the furniture objects that are passed and in which rooms these are on which side
    of the operator when moving towards a certain entity

    :param robot: API object
    :type robot: Robot
    :param entity_designator: resolving to the entity the operator wants to go to
    :type entity_designator: Designator
    :param x_threshold: if the entity is closer than this distance in x-direction w.r.t. the path frame
    it is considered 'passed'
    :type x_threshold: float
    :param y_threshold: if the entity is closer than this distance in y-direction w.r.t. the path frame
    it is considered 'passed'
    :type y_threshold: float
    :return: (list(DirectionItem)) directions the operator should follow
    :raises: (AssertionError, RuntimeError)
    """
    # Get the constraints for the global planner
    nav_constraints = OrderedDict()
    target_entity = entity_designator.resolve()  # type: Entity
    rospy.loginfo("Resolved to Entity: {}".format(target_entity.id))
    if not target_entity:
        raise RuntimeError("I cannot give directions if I don't know where to go")

    possible_areas = ["in"] if target_entity.is_a("room") else ["in_front_of", "near"]
    for area in possible_areas:
        nav_constraints[area] = NavigateToSymbolic.generate_constraint(
            robot=robot,
            entity_designator_area_name_map={entity_designator: area},
            entity_lookat_designator=entity_designator)

    if not nav_constraints:
        raise RuntimeError("I cannot give directions if I don't know where to go")

    # Call the global planner for the shortest path to this entity
    path = None
    for name, nav_con in nav_constraints.iteritems():
        path = robot.base.global_planner.getPlan(position_constraint=nav_con[0])
        if path is not None:
            break

    if path is None:
        raise RuntimeError("I don't know how to get to the {}".format(target_entity.id))

    # Convert the path to a list of kdl Vectors
    assert (all([p.header.frame_id.endswith("map") for p in path])), "Not all path poses are defined w.r.t. 'map'"
    kdl_path = [point_msg_to_kdl_vector(p.pose.position) for p in path]

    # Get all entities
    entities = robot.ed.get_entities()
    furniture_entities = [entity for entity in entities if entity.is_a("furniture")]
    room_entities = [room for room in entities if room.type == "room"]

    # ToDo (future) separate parts above and below here to improve testability

    # Match the furniture entities to rooms
    furniture_entities_room = {room: [] for room in room_entities}  # maps room entities to furniture entities
    for item in furniture_entities:  # type: Entity

        try:
            room = get_room(room_entities, item._pose.p)
            furniture_entities_room[room].append(item)
            rospy.loginfo("{} ({}) is in the {}".format(item.id, item._pose.p, room.id))
        except RuntimeError:
            rospy.logwarn("{} ({}) not in any room".format(item.id, item._pose.p))
            # continue

    # Match the path to rooms
    passed_room_ids = []  # Will contain the ids of the rooms that are passed
    kdl_path_rooms = []  # Will contain tuples (kdl.Vector, Entity) with a waypoint and the room this waypoint is in
    for position in kdl_path:

        try:
            room = get_room(room_entities, position)
        except RuntimeError:
            continue
        kdl_path_rooms.append((position, room))
        if room.id not in passed_room_ids:
            passed_room_ids.append(room.id)

    # With this information: start creating the text for the robot
    # ToDo: kdl_path_rooms should not be empty
    result = []
    start_room_id = ""
    if kdl_path_rooms:
        start_room_id = kdl_path_rooms[0][1].id
        result.append(DirectionItem(None, start_room_id, None))

    # We need to remember the 'previous' room id so the robot can mention when the next room is entered
    prev_room_id = start_room_id

    # Keep track of the ids of the entities that are passed so that the robot doesn't mention any entity twice
    passed_ids = []
    for (position, room), (next_position, _) in zip(kdl_path_rooms[:-1],
                                                    kdl_path_rooms[1:]):

        # Check if the room has changed
        if prev_room_id != room.id:
            prev_room_id = room.id

        furniture_objects = furniture_entities_room[room]  # Furniture objects of the room in which this waypoint
        # is situated
        to_add = []  # will contain tuples (str, distance, str) with the entity id, the distance between this
        # waypoint and the center of this entity (in x-direction) and the side where the entity is (w.r.t. the path)
        reached_target = False
        for entity in furniture_objects:  # type: Entity
            rospy.logdebug("\tEntity: {}".format(room.id))

            # Check if in passed ids
            if entity.id in passed_ids:
                rospy.logdebug("Skipping {}, already in passed ids".format(room.id))
                continue

            # Compute the pose of the entity w.r.t. the 'path'
            entity_pose_path = _get_entity_pose_in_path(position, next_position, entity.pose.frame)

            # Check the distance
            if abs(entity_pose_path.p.x()) < x_threshold and abs(entity_pose_path.p.y()) < y_threshold:
                rospy.logdebug("Appending {}: ({}, {})".format(
                    entity.id, entity_pose_path.p.x(), entity_pose_path.p.y()))

                # side = "left" if entity_pose_path.p.y() >= 0.0 else "right"
                side = Side.LEFT if entity_pose_path.p.y() >= 0.0 else Side.RIGHT

                to_add.append((entity.id, entity_pose_path.p.y(), side))

        # Sort the list based on the distance
        to_add.sort(key=lambda id_distance_tuple: id_distance_tuple[1])

        # Add all items to the list
        for entity_id, _, side in to_add:
            if entity_id == target_entity.id:
                reached_target = True
                break
            else:
                result.append(DirectionItem(entity_id, room.id, side.value))

            passed_ids.append(entity_id)

        if reached_target:
            break

    result.append(DirectionItem(target_entity.id, None, None))
    rospy.loginfo("Result: {}".format(result))
    return result


def sentence_from_directions(directions):
    # type: (list) -> str
    """
    Creates a pronounceable string from a list of directions

    :param directions: (list(DirectionItems)). N.B.: starting room is in item with *no object_id* and *no side*. Final
    object_id is in item with *no side*
    :return: str with pronounceable string
    :raises: (AssertionError) if len < 0 or no object id in last item
    """
    assert len(directions) > 0 and directions[-1].object_id is not None,\
        "Directions should at least contain item with target id"

    result = ""
    previous_room_id = ""
    for idx, item in enumerate(directions[:-1]):  # type: int, DirectionItem

        # Check for start room
        if idx == 0 and item.object_id is None and item.side is None:
            result += "We are now in the {}\n".format(item.room_id)
            previous_room_id = item.room_id
            continue

        # Check for next room
        if item.room_id != previous_room_id:
            result += "You now enter the {}\n".format(item.room_id)

        # Add item id
        result += "You walk by the {} on your {}\n".format(item.object_id, item.side)

        previous_room_id = item.room_id

    # Add the target id that's stored in the final item
    result += "You have now reached the {}".format(directions[-1].object_id)

    return result


class GiveDirections(smach.State):
    """
    Robot tells the operator how to get to a certain entity
    """
    def __init__(self, robot, entity_designator, x_threshold=0.75, y_threshold=1.5):
        # type: (Robot, Designator, float, float) -> None
        """
        Init
        :param robot: API object
        :type robot: Robot
        :param entity_designator: resolving to the entity the operator wants to go to
        :type entity_designator: Designator
        :param x_threshold: if the entity is closer than this distance in x-direction w.r.t. the path frame
        it is considered 'passed'
        :type x_threshold: float
        :param y_threshold: if the entity is closer than this distance in y-direction w.r.t. the path frame
        it is considered 'passed'
        :type y_threshold: float
        """

        super(GiveDirections, self).__init__(outcomes=["succeeded", "failed"])

        self._robot = robot
        self._entity_designator = entity_designator
        self._x_threshold = x_threshold
        self._y_threshold = y_threshold

    def execute(self, userdata=None):

        t_start = rospy.Time.now()

        try:
            directions = get_directions(self._robot, self._entity_designator, self._x_threshold, self._y_threshold)
        except (AssertionError, RuntimeError) as e:
            rospy.logerr("Get directions failed: {}".format(e.message))
            self._robot.speech.speak("I'm sorry but {}".format(e.message))
            return "failed"

        rospy.logdebug("Getting directions took {} seconds".format((rospy.Time.now() - t_start).to_sec()))

        sentence = sentence_from_directions(directions)

        self._robot.speech.speak(sentence)

        return "succeeded"


def create_frame_from_points(p0, p1):
    # type: (kdl.Vector, kdl.Vector) -> kdl.Frame
    """
    Creates a frame from two points. The origin of the frame is the first point. The x-direction points into the
    direction of the next point
    :param p0: Origin of the frame
    :type p0: kdl.Vector
    :param p1: x-direction of the frame will point to this vector
    :type p1: kdl.Vector
    :return: Created frame
    :rtype: kdl.Frame
    """
    unit_x = p1 - p0  # difference
    unit_x.Normalize()  # normalize it so that Norm is 1.0
    unit_y = kdl.Rotation.RPY(0.0, 0.0, 0.5 * math.pi) * unit_x
    unit_z = unit_x * unit_y  # cross-product
    rotation = kdl.Rotation(unit_x, unit_y, unit_z)
    return kdl.Frame(rotation, p0)


def in_room(room, position):
    # type: (Entity, kdl.Vector) -> bool
    """
    Checks if the given position is in the given room
    :param room: Room entity
    :type room: Entity
    :param position: position to check. N.B.: it is assumed this is w.r.t. the same frame as the room
    entities
    :type position: kdl.Vector
    :return: whether or not the position is in the room
    :rtype: bool
    """
    if room.in_volume(VectorStamped(vector=position), "in"):
        return True
    return False


def get_room(rooms, position):
    # type: (list[Entity], kdl.Vector) -> Entity
    """
    Checks if the given position is in one of the provided rooms
    :param rooms: list(Entity) containing all room entities
    :type rooms: list[Entity]
    :param position: position to check. N.B.: it is assumed this is w.r.t. the same frame as the room entities
    :type position: kdl.Vector
    :return: room entity
    :rtype: Entity
    :raises: (RuntimeError)
    """
    for room in rooms:
        if in_room(room, position):
            return room
    raise RuntimeError("Position {} is not in any room".format(position))


if __name__ == "__main__":
    # The code in this __main__ are for testing purposes. Once the ToDos of the smach state are processed, this
    # can/should be moved to a different file (or CI test)

    import sys
    import robot_smach_states.util.designators as ds
    import numpy as np
    from robot_skills.get_robot import get_robot_from_argv

    def _test_rooms(robot):
        # type: (Robot) -> None
        """
        Tests the 'get room' method
        """
        entities = robot.ed.get_entities()
        room_entities = [e for e in entities if e.type == "room"]

        for x in np.arange(-4.0, 4.0, 0.1):
            for y in np.arange(-2.0, 6.0, 0.1):
                position = kdl.Vector(x, y, 0)
                try:
                    room = get_room(room_entities, position)
                    print("Position {} is in the {}".format(position, room.id))
                except RuntimeError as e:
                    print(e.message)

    rospy.init_node('give_directions')

    robot = get_robot_from_argv(index=1)

    if len(sys.argv) > 2:
        furniture_id = sys.argv[2]
    else:
        furniture_id = "bed"

    rospy.loginfo("Waiting for tf cache to be filled")
    rospy.sleep(2)  # wait for tf cache to be filled

    rospy.loginfo("Testing the 'get_room' method")
    _test_rooms(robot)

    rospy.loginfo("Starting giving directions to {}".format(furniture_id))
    state = GiveDirections(robot=robot,
                           entity_designator=ds.EntityByIdDesignator(robot=robot, id=furniture_id))
    state.execute()

# Outcome from init pose to bed
# [INFO][/give_directions][1560279947.767195]: 'We are now in the hallway.
# You walk by the hallway table on your left.
# You enter the workshop.
# You walk by the operator table on your right.
# You enter the bedroom.
# You walk by the black cabinet5 on your left.
# You have now reached the bed.
# '
