"""
Module contains states to guide an operator to a designated location.
"""
import math

# ROS
import rospy
import smach
import PyKDL as kdl

# Robot skills
from robot_skills.util.kdl_conversions import VectorStamped
from robot_smach_states import WaitTime
from robot_smach_states.util.designators import EdEntityDesignator

import navigation
from robot_smach_states.human_interaction import Say
from robot_smach_states.navigation.navigate_to_symbolic import NavigateToSymbolic


def _detect_operator_behind_robot(robot, distance=1.0, radius=0.5):
    # type: (Robot, float, float) -> bool
    """
    Checks if a person is within <radius> of the position <distance> behind the <robot>

    :param robot: (Robot) api object
    :param distance: (float) follow distance
    :param radius: (float) radius around the position at <distance> behind the robot
    :return: (bool) whether an operator was detected
    """
    image_data = robot.perception.get_rgb_depth_caminfo()
    success, found_people_ids = robot.ed.detect_people(*image_data)
    found_people = [robot.ed.get_entity(id_) for id_ in found_people_ids]

    rospy.loginfo("Found {} people: {}".format(len(found_people), found_people))
    found_people = [p for p in found_people if p]
    rospy.loginfo("{} people remaining after None-check".format(len(found_people)))

    # Assume the operator is a set distance behind the robot
    base_pose = robot.base.get_location()
    expected_person_pos = base_pose.frame * kdl.Vector(-distance, 0.0, 0.0)

    rospy.loginfo("\n\tBase pos, expected person pos: {}, {}".format(base_pose.frame.p, expected_person_pos))
    for person in found_people:
        rospy.loginfo("\n\tPerson at {}, distance: {} (radius = {})".format(
            person.pose.frame.p,
            (person.pose.frame.p - expected_person_pos).Norm(),
            radius
        ))
        if (person.pose.frame.p - expected_person_pos).Norm() < radius:
            return True
    return False


class TourGuide(object):
    def __init__(self, robot, x_threshold=0.75, y_threshold=1.5):
        # type: (Robot, float, float) -> ()
        """
        # Class to describe furniture near the robot.

        :param robot: (Robot) api object
        :param x_threshold: (float) threshold determining when a piece of furniture is close enough to be described [m]
        :param y_threshold: (float) threshold determining when a piece of furniture is close enough to be described [m]
        """
        self._robot = robot

        # Properties
        self._x_threshold = x_threshold
        self._y_threshold = y_threshold

        # State
        self._furniture_entities = []
        self._room_entities = []
        self._furniture_entities_room = {}  # map room entities to furniture entities
        self._passed_room_ids = []  # Will contain the ids of the rooms that are passed
        self._passed_furniture_ids = []  # Will contain the ids of the furniture that is passed

        self.initialize()

    def describe_near_objects(self):
        # type: () -> string
        """
        Describes near objects based on the robots current position
        :return: A description of the robots surroundings
        :rtype: string
        """
        position = self._robot.base.get_location()

        try:
            room = self.get_room(position.frame.p)
        except RuntimeError as e:
            rospy.logwarn(e)
            return ""

        if room.id not in self._passed_room_ids:
            self._passed_room_ids.append(room.id)
            rospy.logdebug("describe entering room: {}.\t passed rooms is now {}".format(room.id, self._passed_room_ids))
            return "We now enter the {}".format(room.id)

        # not entering a new room, checking furniture
        furniture_objects = self._furniture_entities_room[room]  # Furniture objects in our current room
        for entity in furniture_objects:  # type: Entity
            # Check if in passed ids
            if entity.id in self._passed_furniture_ids:
                continue

            # Compute the pose of the entity w.r.t. the 'path'
            entity_relative_pose = position.frame.Inverse() * entity.pose.frame

            # Check the distance
            if abs(entity_relative_pose.p.x()) < self._x_threshold and \
                    abs(entity_relative_pose.p.y()) < self._y_threshold:
                self._passed_furniture_ids.append(entity.id)
                side = "left" if entity_relative_pose.p.y() >= 0.0 else "right"
                rospy.logdebug(
                    "describe passing entity: {}.\t passed entities is now {}".format(entity.id,
                                                                                      self._passed_furniture_ids))
                return "On our {} you can see the {}".format(side, entity.id)

        # no furniture passed, nothing of interest
        return ""

    def initialize(self):
        entities = self._robot.ed.get_entities()
        self._furniture_entities = [entity for entity in entities if entity.is_a("furniture")]
        self._room_entities = [room for room in entities if room.type == "room"]

        # Match the furniture entities to rooms
        self._furniture_entities_room = {room: [] for room in self._room_entities}
        for item in self._furniture_entities:  # type: Entity
            try:
                room = self.get_room(item._pose.p)
            except RuntimeError:
                rospy.logwarn("{} ({}) not in any room".format(item.id, item._pose.p))
                continue
            self._furniture_entities_room[room].append(item)
            rospy.logdebug("{} ({}) is in the {}".format(item.id, item._pose.p, room.id))

        self._passed_room_ids = []  # Will contain the ids of the rooms that are passed
        self._passed_furniture_ids = []  # Will contain the ids of the furniture that is passed

        # get initial room
        position = self._robot.base.get_location()
        try:
            room = self.get_room(position.frame.p)
            self._passed_room_ids.append(room.id)
        except RuntimeError:
            rospy.logwarn("position ({}) not in any room".format(position.frame.p))

        # get initial entities
        r = math.hypot(self._x_threshold, self._y_threshold)
        close_entities = self._robot.ed.get_entities(center_point=position.extractVectorStamped(), radius=r)
        close_furniture_entities = [entity for entity in close_entities if entity.is_a("furniture")]
        for entity in close_furniture_entities:
            self._passed_furniture_ids.append(entity.id)
        rospy.loginfo("TourGuide: passed rooms: {}.\t passed entities {}"
                      .format(self._passed_room_ids, self._passed_furniture_ids))

    def reset(self):
        """
        Resets the passed room ids and the passed furniture ids
        """
        self._passed_room_ids = []  # Will contain the ids of the rooms that are passed
        self._passed_furniture_ids = []  # Will contain the ids of the furniture that is passed

    def get_room(self, position):
        # type: (kdl.Vector) -> Entity
        """
        Checks if the given position is in one of the provided rooms

        :param position: position to check. N.B.: it is assumed this is w.r.t. the same frame as the room entities
        :type position: kdl.Vector
        :return: room entity
        :rtype: Entity
        :raises: (RuntimeError)
        """
        for room in self._room_entities:
            if room.in_volume(VectorStamped(vector=position), "in"):
                return room
        raise RuntimeError("Position {} is not in any room".format(position))


class ExecutePlanGuidance(smach.State):
    """
    Similar to the "executePlan" smach state. The only difference is that after driving for x meters, "check for
    operator" is returned.
    """
    def __init__(self, robot, operator_distance=1.0, operator_radius=0.5):
        # type: (Robot, float, float) -> None
        """
        :param robot: (Robot) robot api object
        :param operator_distance: (float) check for the operator to be within this range of the robot
        :param operator_radius: (float) from the point behind the robot defined by `distance`, the person must be within
            this radius
        """
        smach.State.__init__(self, outcomes=["arrived", "blocked", "preempted", "lost_operator"])
        self.robot = robot
        self._distance_threshold = 1.0  # Only check if the operator is there once we've driven for this distance
        self._operator_distance = operator_distance  # Operator is expected to follow the robot around this distance
        self._operator_radius = operator_radius  # Operator is expected to be within this radius around the position
        # defined by the follow distance
        self._tourguide = TourGuide(robot)

    def execute(self, userdata=None):

        # Look backwards to have the operator in view
        self.robot.head.look_at_point(VectorStamped(-1.0, 0.0, 1.75, self.robot.base_link_frame))

        rate = rospy.Rate(10.0)  # Loop at 10 Hz
        distance = 0.0
        old_position = self._get_base_position()
        while not rospy.is_shutdown():

            if self.preempt_requested():
                self.robot.base.local_planner.cancelCurrentPlan()
                rospy.loginfo("execute: preempt_requested")
                return "preempted"

            status = self.robot.base.local_planner.getStatus()

            if status == "arrived":
                return "arrived"
            elif status == "blocked":
                return "blocked"

            new_position = self._get_base_position()
            distance += (new_position - old_position).Norm()
            old_position = new_position
            if distance > self._distance_threshold:
                rospy.logdebug(
                    "Distance {} exceeds threshold {}, check for operator".format(distance, self._distance_threshold))
                if not self._check_operator():
                    rospy.loginfo("Lost operator while guiding, cancelling plan")
                    self.robot.base.local_planner.cancelCurrentPlan()
                    return "lost_operator"

            sentence = self._tourguide.describe_near_objects()
            if sentence != "":
                self.robot.speech.speak(sentence)

            rate.sleep()

    def reset_tourguide(self):
        """
        Resets the internal state of the tourguide so that all operators get complete information if this guide state
        is used more than once.
        """
        self._tourguide.reset()

    def _check_operator(self):
        """
        Checks if the operator is still sufficiently close

        :return: (bool)
        """
        # ToDo: make robust (use time stamp?)
        return _detect_operator_behind_robot(self.robot, self._operator_distance, self._operator_radius)

    def _get_base_position(self):
        # type: () -> kdl.Vector
        """
        Gets the base position as a kdl Vector

        :return: (kdl Vector) with current base position
        """
        frame_stamped = self.robot.base.get_location()
        return frame_stamped.frame.p


class WaitForOperator(smach.State):
    def __init__(self, robot, timeout=10.0, distance=1.0, radius=0.5):
        # type: (Robot, float, float, float) -> None
        """
        Smach state to check if the operator is still following the robot.

        :param robot: (Robot) robot api object
        :param timeout: (float) if the operator has not been detected for this period, "is_lost" will be returned
        :param distance: (float) check for the operator to be within this range of the robot [m]
        :param radius: (float) from the point behind the robot defined by `distance`, the person must be within this
            radius [m]
        """
        smach.State.__init__(self, outcomes=["is_following", "is_lost", "preempted"])
        self._robot = robot
        self._timeout = timeout
        self._distance = distance
        self._radius = radius

    def execute(self, ud):

        self._robot.speech.speak("It seems that we have lost each other. Please stand one meter behind me.")

        rate = rospy.Rate(2.0)
        t_start = rospy.Time.now()
        while not rospy.is_shutdown():

            # Check if the operator is there
            if _detect_operator_behind_robot(self._robot, self._distance, self._radius):
                self._robot.speech.speak("There you are", block=False)
                return "is_following"

            # Check timeout
            if (rospy.Time.now() - t_start).to_sec() > self._timeout:
                rospy.loginfo("Guide - Wait for operator: timeout {} exceeded".format(self._timeout))
                return "is_lost"

            rate.sleep()

        return "preempted"


class Guide(smach.StateMachine):
    def __init__(self, robot, operator_distance=1.0, operator_radius=0.5):
        # type: (Robot, float, float) -> None
        """
        Base Smach state to guide an operator to a designated position

        :param robot: (Robot) robot api object
        :param operator_distance: (float) check for the operator to be within this range of the robot
        :param operator_radius: (float) from the point behind the robot defined by `distance`, the person must be within this radius
        """
        smach.StateMachine.__init__(
            self, outcomes=["arrived", "unreachable", "goal_not_defined", "lost_operator", "preempted"])
        self.robot = robot
        self.operator_distance = operator_distance
        self.operator_radius = operator_radius
        self.execute_plan = ExecutePlanGuidance(robot=self.robot,
                                                operator_distance=self.operator_distance,
                                                operator_radius=self.operator_radius)

        with self:
            @smach.cb_interface(outcomes=["done"])
            def _reset_mentioned_entities(userdata=None):
                """
                Resets the entities that have been mentioned so that the robot will mention all entities to all
                 operators
                 """
                self.execute_plan.reset_tourguide()
                return "done"

            smach.StateMachine.add("RESET_MENTIONED_ENTITIES",
                                   smach.CBState(_reset_mentioned_entities),
                                   transitions={"done": "SAY_BEHIND"})

            smach.StateMachine.add("SAY_BEHIND",
                                   Say(robot, "Please stand behind me and look at me", block=True),
                                   transitions={"spoken": "WAIT"})

            smach.StateMachine.add("WAIT",
                                   WaitTime(robot, waittime=3.0),
                                   transitions={"waited": "GET_PLAN",
                                                "preempted": "preempted"})

            smach.StateMachine.add("GET_PLAN", navigation.getPlan(self.robot, self.generate_constraint),
                                   transitions={"unreachable": "unreachable",
                                                "goal_not_defined": "goal_not_defined",
                                                "goal_ok": "EXECUTE_PLAN"})

            smach.StateMachine.add("EXECUTE_PLAN", self.execute_plan,
                                   transitions={"arrived": "arrived",
                                                "blocked": "PLAN_BLOCKED",
                                                "preempted": "preempted",
                                                "lost_operator": "WAIT_FOR_OPERATOR"})

            smach.StateMachine.add("WAIT_FOR_OPERATOR",
                                   WaitForOperator(robot=self.robot,
                                                   distance=self.operator_distance,
                                                   radius=self.operator_radius),
                                   transitions={"is_following": "GET_PLAN",
                                                "is_lost": "lost_operator"})

            smach.StateMachine.add("PLAN_BLOCKED", navigation.planBlocked(self.robot),
                                   transitions={"blocked": "GET_PLAN",
                                                "free": "EXECUTE_PLAN"})

    @staticmethod
    def generate_constraint():
        raise NotImplementedError("Inheriting Guide states must implement a generate constraint method, preferably"
                                  "by re-using it from a navigation state.")


class GuideToSymbolic(Guide):
    """ Guidance class to navigate to a semantically annotated goal, e.g., in front of the dinner table.
    """
    def __init__(self, robot, entity_designator_area_name_map, entity_lookat_designator, operator_distance=1.0,
                 operator_radius=0.5):
        # type: (Robot, dict, EdEntityDesignator, float, float) -> None
        """
        Constructor

        :param robot: robot object
        :param entity_designator_area_name_map: dictionary mapping EdEntityDesignators to a string or designator
            resolving to a string, representing the area, e.g., entity_designator_area_name_map[<EdEntity>] = 'in_front_of'.
        :param entity_lookat_designator: EdEntityDesignator defining the entity the robot should look at. This is used
            to compute the orientation constraint.
        :param operator_distance: (float) check for the operator to be within this range of the robot [m]
        :param operator_radius: (float) from the point behind the robot defined by `distance`, the person must be within
            this radius [m]
        """
        super(GuideToSymbolic, self).__init__(robot=robot,
                                              operator_distance=operator_distance,
                                              operator_radius=operator_radius)

        self._entity_designator_area_name_map = entity_designator_area_name_map
        self._entity_lookat_designator = entity_lookat_designator

    def generate_constraint(self):
        # type: () -> tuple
        """
        Generates the constraint using the generate constraint method of NavigateToSymbolic

        :return: (tuple(PositionConstraint, OrientationConstraint)). If one of the entities does not resolve,
        None is returned.
        """
        return NavigateToSymbolic.generate_constraint(
            self.robot, self._entity_designator_area_name_map, self._entity_lookat_designator)
