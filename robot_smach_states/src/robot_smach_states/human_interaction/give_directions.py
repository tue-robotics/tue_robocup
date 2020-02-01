# System
import math
from collections import OrderedDict

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
        # Get the constraints for the global planner
        nav_constraints = OrderedDict()
        goal_entity = self._entity_designator.resolve()  # type: Entity
        rospy.loginfo("Resolved to Entity: {}".format(goal_entity.id))
        if not goal_entity:
            rospy.logerr("Cannot give directions if I don't know where to go")
            self._robot.speech.speak("I'm sorry but I don't know where you want to go", mood="sad")
            return "failed"
        if goal_entity.is_a("room"):
            possible_areas = ["in"]
        else:
            possible_areas = ["in_front_of", "near"]
        for area in possible_areas:
            nav_constraints[area] = NavigateToSymbolic.generate_constraint(
                robot=self._robot,
                entity_designator_area_name_map={self._entity_designator: area},
                entity_lookat_designator=self._entity_designator)

        if not nav_constraints:
            rospy.logerr("Cannot give directions if I don't know where to go")
            self._robot.speech.speak("I'm sorry but I don't know where you want to go", mood="sad")
            return "failed"

        # Call the global planner for the shortest path to this entity
        path = None
        for name, nav_con in nav_constraints.iteritems():
            path = self._robot.base.global_planner.getPlan(position_constraint=nav_con[0])
            if path:
                break

        if not path:
            rospy.logerr("No path to {}".format(goal_entity.id))
            self._robot.speech.speak("I'm sorry but I don't know how to get to the {}".format(goal_entity.id))
            return "failed"

        # Convert the path to a list of kdl Vectors
        assert(all([p.header.frame_id.endswith("map") for p in path])), "Not all path poses are defined w.r.t. 'map'"
        kdl_path = [point_msg_to_kdl_vector(p.pose.position) for p in path]

        # Get all entities
        entities = self._robot.ed.get_entities()
        room_entities = [room for room in entities if room.type == "room"]

        # Log the time we start iterating
        t_start = rospy.Time.now()

        # Match the path to rooms
        passed_room_ids = []  # Will contain the ids of the rooms that are passed
        for position, next_position in zip(kdl_path[:-1], kdl_path[1:]):

            try:
                room = get_room(room_entities, position)
            except RuntimeError:
                continue
            if room.id not in passed_room_ids:
                passed_room_ids.append(room.id)
                final_room_entry_pose = create_frame_from_points(position, next_position)

        # With this information: start creating the text for the robot
        sentence = ""
        if len(passed_room_ids) > 1:
            sentence += "We are now in the {}.\n".format(passed_room_ids[0])

            if len(passed_room_ids) > 2:
                sentence += "We will pass through "
                sentence += "the {}".format(passed_room_ids[1])
                if len(passed_room_ids) > 3:
                    for room in passed_room_ids[2:-2]:
                        sentence += ", the {}".format(room)
                    sentence += " and the {}".format(passed_room_ids[-2])
                sentence += "\n"
            sentence += "We will enter the {}\n".format(passed_room_ids[-1])

        # Check where the goal is upon entering the room
        entity_pose_path = final_room_entry_pose.Inverse() * goal_entity.pose.frame
        angle = math.atan2(entity_pose_path.p.y(), entity_pose_path.p.x())
        rospy.loginfo("angle = {} rad, {} degrees".format(angle, angle * 180 / math.pi))
        if abs(angle) < 0.25 * math.pi:
            sentence += "The {} will be in front of you.\n".format(goal_entity.id)
        else:
            side = "left" if angle > 0.0 else "right"
            sentence += "The {} is on your {}.\n".format(goal_entity.id, side)

        rospy.loginfo("Directions computation took {} seconds".format((rospy.Time.now() - t_start).to_sec()))

        self._robot.speech.speak(sentence)

        # ToDo's:
        # * Improve texts
        # * Break up this execute method into more (standalone/static) methods to improve readability and re-usability

        return "succeeded"

    @staticmethod
    def get_entity_pose_in_path(point, next_point, entity_pose):
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
                    rospy.loginfo("Position {} is in the {}".format(position, room.id))
                except RuntimeError as e:
                    rospy.logerr(e.message)

    rospy.init_node('give_directions')

    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    elif robot_name == 'hero':
        from robot_skills.hero import Hero as Robot
    else:
        rospy.logerr("unknown robot")
        sys.exit()

    robot = Robot()

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
