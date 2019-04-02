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


class GiveDirections(smach.State):
    """
    Robot tells the operator how to get to a certain entity
    """
    def __init__(self, robot, entity_designator, radius=1.5):
        # type: (Robot, any) -> any
        """
        Init

        :param robot: (Robot) API object
        :param entity_designator: (EdEntityDesignator) resolving to the entity the operator wants to go to
        :param radius: passing closer to an entity than this radius, the entity will be mentioned
        """

        super(GiveDirections, self).__init__(outcomes=["succeeded", "failed"])

        self._robot = robot
        self._entity_designator = entity_designator
        self._radius = radius

    def execute(self, ud):

        # Get the constraints for the global planner
        nav_contraints = NavigateToSymbolic.generate_constraint(
            robot=self._robot,
            entity_designator_area_name_map={self._entity_designator: "near"},
            entity_lookat_designator=self._entity_designator
        )
        if nav_contraints is None:
            rospy.logerr("Cannot give directions if I don't know where to go")
            self._robot.speech.speak("I'm sorry but I don't know where you want to go", mood="sad")
            return "failed"

        # Resolve the entity: we'll need it later
        target_entity = self._entity_designator.resolve()  # type: Entity

        # Call the global planner for the shortest path to this entity
        path = self._robot.base.global_planner.getPlan(position_constraint=nav_contraints[0])
        if path is None:

            rospy.logerr("No path to {}".format(target_entity.id))
            self._robot.speech.speak("I'm sorry but I don't know how to get to the {}".format(target_entity.id))
            return "failed"

        # Convert the path to a list of kdl Vectors
        assert(all([p.header.frame_id.endswith("map") for p in path])), "Not all path poses are defined w.r.t. 'map'"
        kdl_path = [point_msg_to_kdl_vector(p.pose.position) for p in path]

        # Get all entities
        entities = self._robot.ed.get_entities()
        furniture_entities = [room for room in entities if room.is_a("furniture")]
        room_entities = [room for room in entities if room.type == "room"]

        # Figure out which entities are passed along the way
        # N.B.: this is a first naive implementation: optimization might be desired
        t_start = rospy.Time.now()

        # Match the furniture entities to rooms
        # ToDo: move to separate method: may be useful in other cases
        furniture_entities_room = {room: [] for room in room_entities}  # maps room entities to furniture entities
        for item in furniture_entities:  # type: Entity

            # Set the pose to a minimum of 10 cm to avoid numerial issues
            item._pose.p.z(max(0.1, item._pose.p.z()))

            found = False
            for room in room_entities:  # type: Entity

                # Hack to handle 'flat' rooms (solution lays elsewhere)
                for volume in room.volumes.values():
                    volume.max_corner.z(2.0)

                # ToDo: check rooms in robotics testlabs, then see if this multiplication is required
                if room.in_volume(VectorStamped(vector=room._pose.Inverse() * item.pose.frame.p), "in"):
                    rospy.loginfo("The {} ({}) is in the {} ({})".format(
                        item.id, item.pose.frame.p, room.id, room.volumes.values()))
                    furniture_entities_room[room].append(item)
                    found = True
                    break

            if not found:
                rospy.logwarn("{} ({}) not in any room".format(item.id, item.pose.frame.p))

        # Match the path to rooms
        passed_room_ids = []
        kdl_path_rooms = []
        for position in kdl_path:

            try:
                room = get_room(room_entities, position)
            except RuntimeError:
                continue
            kdl_path_rooms.append((position, room))
            if room.id not in passed_room_ids:
                passed_room_ids.append(room.id)

            # for room in room_entities:  # type: Entity
            #
            #     # Hack to avoid numerical issues
            #     position.z(0.1)
            #
            #     # Check if it meets the requirement
            #     # ToDo: check rooms in robotics testlabs, then see if this multiplication is required
            #     if room.in_volume(VectorStamped(vector=room._pose.Inverse() * position), "in"):
            #         kdl_path_rooms.append((position, room))
            #
            #         # If the ID is already present: continue
            #         if room.id not in passed_room_ids:
            #             passed_room_ids.append(room.id)

        passed_ids = []
        for position in kdl_path:
            to_add = []
            reached_target = False
            for room in furniture_entities:  # type: Entity
                rospy.logdebug("\tEntity: {}".format(room.id))

                # If the id is already present: continue
                if room.id in passed_ids:
                    rospy.logdebug("Skipping {}, already in passed ids".format(room.id))
                    continue

                # Check the distance
                distance = room.distance_to_2d(position)

                if distance < self._radius:
                    rospy.logdebug("Appending {}: {}".format(room.id, distance))
                    to_add.append((room.id, distance))

            # Sort the list
            to_add.sort(key=lambda id_distance_tuple: id_distance_tuple[1])

            # Add all items to the list
            for item in to_add:
                if item[0] == target_entity.id:
                    reached_target = True
                    break

                passed_ids.append(item[0])

            if reached_target:
                break

        rospy.loginfo("Directions computation took {} seconds".format((rospy.Time.now() - t_start).to_sec()))

        self._robot.speech.speak("You have to walk through the {} to get to the {}".format(
            ", the ".join(passed_room_ids), target_entity.id
        ))
        self._robot.speech.speak("There you have to walk by the {} to get to the {}".format(
            ", the ".join(passed_ids), target_entity.id
        ))

        # ToDo's:
        # * Make acknowledged of rooms
        # * Identify left or right

        return "succeeded"


def in_room(room, position):
    # type: (Entity, kdl.Vector) -> bool
    """
    Checks if the given position is in the given room

    :param room: (Entity) Room entity
    :param position: (kdl.Vector) position to check. N.B.: it is assumed this is w.r.t. the same frame as the room
    entities
    :return: (bool) whether or not the position is in the room
    """
    # if room.in_volume(VectorStamped(vector=room._pose.Inverse() * position), "in"):
    if room.in_volume(VectorStamped(vector=position), "in"):
        return True
    return False


def get_room(rooms, position):
    # type: (list[Entity], kdl.Vector) -> Entity
    """
    Checks if the given position is in one of the provided rooms

    :param rooms: list(Entity) containing all room entities
    :param position: (kdl.Vector) position to check. N.B.: it is assumed this is w.r.t. the same frame as the room
    entities
    :return: (Entity) room entity
    :raises: (RuntimeError)
    """
    for room in rooms:
        if in_room(room, position):
            return room
    raise RuntimeError("Position {} is not in any room".format(position))


def _test_rooms(robot):
    """
    Tests the 'get room' method
    """
    import numpy as np

    entities = robot.ed.get_entities()
    room_entities = [e for e in entities if e.type == "room"]

    for x in np.arange(-4.0, 4.0, 0.1):
        for y in np.arange(-2.0, 6.0, 0.1):
    # for x in [-2.0, 2.0]:
    #     for y in [0.0, 5.0]:
            position = kdl.Vector(x, y, 0)
            try:
                room = get_room(room_entities, position)
                print("Position {} is in the {}".format(position, room.id))
            except RuntimeError as e:
                print e.message


if __name__ == "__main__":

    import sys
    import robot_smach_states.util.designators as ds

    rospy.init_node('give_directions')

    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    elif robot_name == 'hero':
        from robot_skills.hero import Hero as Robot
    else:
        print "unknown robot"
        sys.exit()

    robot = Robot()

    if len(sys.argv) > 2:
        furniture_id = sys.argv[2]
    else:
        furniture_id = "bed"

    rospy.loginfo("Waiting for tf cache to be filled")
    rospy.sleep(2)  # wait for tf cache to be filled

    rospy.loginfo("Starting giving directions to {}".format(furniture_id))

    # _test_rooms(robot)
    state = GiveDirections(robot=robot,
                           entity_designator=ds.EntityByIdDesignator(robot=robot, id=furniture_id),
                           )
    state.execute(None)
