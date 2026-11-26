
# System
import math
from contextlib import redirect_stdout

import numpy as np

# ROS
import rospy
import smach

# TU/e Robotics
from ed.entity import Entity

import robot_smach_states.util.designators as ds
from pykdl_ros import VectorStamped
from robot_smach_states.designator_iterator import IterateDesignator
from robot_smach_states.navigation.navigate_to_symbolic import NavigateToRoom
from robot_smach_states.navigation.navigate_to_waypoint import NavigateToWaypoint
from robot_smach_states.util.entity_descriptions import person_matches
from robot_smach_states.world_model.world_model import UpdateDestEntityPoseWithSrcEntity


def _lower(value):
    return value.lower() if isinstance(value, str) else value


# WARN: vibe coded. Covered by test/human_interaction/test_find_people_filter.py.
def check_person_property(person, prop_name, prop_value, strict=True, reverse=False):
    """
    Whether a person has a property, as FindPeople filters on its 'properties'. Names and values are compared without
    regard to case, and a strict list check does not depend on the order of the values.

    - prop_value None: the person has some value for the property.
    - strict (and not reverse): a list property must contain all values ({'tags': ['LWaving', 'RWaving']}: waving with
      both arms); another property must equal the value.
    - otherwise: a list property must contain one of the values, another property must be one of them.
    - 'id' is the person's id or name ({'id': 'anna'}).

    :param person: person entity (ed_python Entity with person_properties)
    :return: True when the person has the property
    """
    if prop_name == "id":
        names = {_lower(person.uuid), _lower(getattr(person.person_properties, "name", None))}
        return _lower(prop_value) in names if prop_value else True
    if person.person_properties is None:
        return False
    person_attr_val = getattr(person.person_properties, prop_name)
    rospy.logdebug("For person {}: {} is {}".format(person.uuid, prop_name, person_attr_val))
    if not prop_value:
        return bool(person_attr_val)

    wanted = [_lower(v) for v in prop_value] if isinstance(prop_value, (list, tuple, set)) else _lower(prop_value)
    if isinstance(person_attr_val, list):
        have = {_lower(v) for v in person_attr_val}
        if strict and not reverse:
            return set(wanted if isinstance(wanted, list) else [wanted]) <= have
        return bool(have & set(wanted if isinstance(wanted, list) else [wanted]))
    have = _lower(person_attr_val)
    if strict and not reverse:
        return have == wanted
    return have in wanted if have is not None else False


# WARN: vibe coded. Covered by test/human_interaction/test_find_people_filter.py.
def filter_people(people, properties=None, strict=True, reverse=False, attributes=None):
    """
    The people that have the properties and fit the attributes, as FindPeople selects them.

    Example: filter_people(people, properties={'tags': ['LWaving', 'RWaving']}, strict=False) keeps the people waving
    with either arm (challenge restaurant); with reverse=True, the people who do not (challenge stickler_for_rules:
    the people not holding a drink). attributes={'gesture': 'waving', 'pose': 'sitting'} keeps the people that fit
    them (see robot_smach_states.util.entity_descriptions.person_matches).

    :param people: person entities
    :param properties: dict property -> value(s) of the person_properties, see check_person_property
    :param attributes: dict attribute -> value of a description, see PERSON_ATTRIBUTES
    :return: the remaining people
    """
    if properties:
        for k, v in properties.items():
            found_people_temp = [x for x in people if check_person_property(x, k, v, strict, reverse)]
            if reverse:
                people = list(set(people) - set(found_people_temp))
            else:
                people = found_people_temp
            rospy.loginfo("{} people remaining after {}={} check".format(len(people), k, v))
    if attributes:
        people = [p for p in people if person_matches(p.person_properties, attributes)[0]]
        rospy.loginfo("{} people remaining after the attributes {}".format(len(people), attributes))
    return people


class FindPeople(smach.State):
    """
    Smach state to find a person. The robot looks around and tries to find
    people in its view.

    >>> from robot_skills.mockbot import Mockbot
    >>> with redirect_stdout(None): robot = Mockbot()
    >>> # Designator to store the result
    >>> des = ds.VariableDesignator(resolve_type=[Entity])
    >>>
    >>> sm = FindPeople(robot=robot, properties={'id': 'NAME'}, found_people_designator=des.writeable)
    >>> # sm.execute()
    >>> # des.resolve()

    Entity(uuid=something, )

    """

    def __init__(self, robot, properties=None, query_entity_designator=None,
                 found_people_designator=None, look_distance=10.0, speak=False,
                 strict=True, reverse=False, nearest=False, attempts=1, search_timeout=60,
                 look_range=(-np.pi/2, np.pi/2), look_steps=8, attributes=None):
        """
        Initialization method

        :param robot: robot api object
        :param properties: (dict) (default: None) keyvalue pair of the properties a person must
            possess. None as a value for a property would search for all possible
            values of the property.
        :param query_entity_designator: (default: None) An entity designator to match all found
            people to
        :param found_people_designator: (default: None) A designator to write the search result to.
            The designator always has a list of found people written to it.
        :param look_distance: (float) (default: 10.0) The distance (radius) which the robot must look at
        :param speak: (bool) (default: False) If True, the robot will speak while trying to find
            a named person
        :param strict: (bool) (default: True)  Only used if properties is not None AND the {key:value} pair of a
            property has non None values. Disabled when setting `reverse` to True.
            If set to True then only people with all specified
            properties are returned, else all people with at least one true property.
            Example:
                properties = {'tags': ['LWaving', 'RWaving', 'LHolding', 'RHolding']}
                strict = True
                    This will return a list of people who have the tags:
                        'LWaving' AND 'RWaving' AND 'LHolding' AND 'RHolding'

                strict = False
                    This will return a list of people who have the tags:
                        'LWaving' OR 'RWaving' OR 'LHolding' OR 'RHolding'

        :param reverse: (bool) (default: False) Want all the people that don't match ANY property. So enabling `reverse`
            will make the property matching a `not (x1 OR x2 OR ...)`.
        :param nearest: (bool) (default: False) If True, selects the people nearest to the robot who match the
            requirements posed using the properties, query_entity_designator, look distance and strict arguments
        :param attempts: (int) (default: 1) Max number of search attempts
        :param search_timeout: (float) (default: 60) maximum time the robot is allowed to search
        :param look_range: (tuple of size 2) (default: (-np.pi/2, np.pi/2)) from what to what head angle should the
            robot search
        :param look_steps: (int) (default: 8) How many steps does it take in that range
        :param attributes: (dict) (default: None) attributes of a description the people must fit, e.g.
            {'gesture': 'waving'} (see robot_smach_states.util.entity_descriptions.person_matches)
        """
        smach.State.__init__(self, outcomes=['found', 'failed'])
        self._attributes = attributes

        self._robot = robot

        self._properties = properties
        self._look_distance = look_distance
        self._speak = speak
        self._strict = strict
        self._reverse = reverse
        self._nearest = nearest
        self._attempts = attempts

        self._search_timeout = search_timeout

        self._look_angles = np.flip(np.linspace(look_range[0], look_range[1], look_steps))

        if found_people_designator:
            ds.is_writeable(found_people_designator)
            ds.check_type(found_people_designator, [Entity])
        self._found_people_designator = found_people_designator

        if query_entity_designator:
            ds.check_type(query_entity_designator, Entity)
        self._query_entity_designator = query_entity_designator

    def execute(self, userdata=None):
        self._look_angles = np.flip(self._look_angles)
        person_label = None

        if self._properties:
            try:
                person_label = self._properties["id"]
                ds.check_type(person_label, "str")
                person_label = person_label.resolve() if hasattr(person_label, 'resolve') else person_label

                rospy.loginfo("Trying to find {}".format(person_label))
            except:
                # The try except is to check if a named person is queried for
                # in the properties. If there is none then exception is raised
                # and nothing is to be done with it
                pass

        if self._speak:
            self._robot.speech.speak(
                "{}please look at me while I am looking for you".format(
                    person_label+', ' if person_label else ''),
                block=False)

        start_time = rospy.Time.now()

        head_goals = [VectorStamped.from_xyz(x=self._look_distance * math.cos(angle),
                                             y=self._look_distance * math.sin(angle),
                                             z=1.7,
                                             stamp=start_time,
                                             frame_id=self._robot.base_link_frame)
                      for angle in self._look_angles]

        i = 0
        attempts = 0

        while not rospy.is_shutdown() and attempts < self._attempts and (rospy.Time.now() - start_time).to_sec() < self._search_timeout:
            if self.preempt_requested():
                return 'failed'

            self._robot.head.look_at_point(head_goals[i])
            i += 1

            if i == len(head_goals):
                i = 0
                attempts += 1

            self._robot.head.wait_for_motion_done()

            found_people_ids = []
            for _ in range(2):  # TODO: parametrize
                self._image_data = self._robot.perception.get_rgb_depth_caminfo()
                if self._image_data:
                    success, found_ids = self._robot.ed.detect_people(*self._image_data)
                else:
                    rospy.logwarn("Could not get_rgb_depth_caminfo")
                    success, found_ids = False, []
                found_people_ids += found_ids

            # Use only unique IDs in the odd case ED sees the same people twice
            found_people = [self._robot.ed.get_entity(eid) for eid in set(found_people_ids)]

            rospy.loginfo("Found {} people: {}".format(len(found_people), found_people))
            found_people = [p for p in found_people if p]
            rospy.loginfo("{} people remaining after None-check".format(len(found_people)))

            robot_pose = self._robot.base.get_location()
            found_people = [p for p in found_people if (p.pose.frame.p - robot_pose.frame.p).Norm() <= self._look_distance]

            rospy.loginfo("{} people remaining after distance < {}-check".format(len(found_people), self._look_distance))

            found_people = filter_people(found_people, self._properties, self._strict, self._reverse,
                                         self._attributes)

            result_people = []

            if self._query_entity_designator:
                query_entity = self._query_entity_designator.resolve()
                if query_entity:
                    result_people = [p for p in found_people if query_entity.in_volume(VectorStamped.from_framestamped(p.pose), 'in', padding=-0.15)]
                    rospy.loginfo("{} result_people remaining after 'in'-'{}' check".format(len(result_people), query_entity.uuid))

                    # If people not in query_entity then try if query_entity in people
                    if not result_people:
                        # This is for a future feature when object recognition
                        # becomes more advanced
                        try:
                            result_people = [x for x in found_people if x.in_volume(VectorStamped.from_framestamped(query_entity.pose), 'in', padding=-0.05)]
                            rospy.loginfo(
                                "{} result_people remaining after 'in'-'{}' check".format(len(result_people), query_entity.uuid))

                        except Exception:
                            pass
            else:
                result_people = found_people

            if result_people:
                if self._nearest:
                    result_people.sort(key=lambda e: (e.pose.frame.p - robot_pose.frame.p).Norm())
                if person_label and \
                    [x for x in result_people if self._check_person_property(x, "id", person_label)] \
                    and self._speak:
                    self._robot.speech.speak("I think I found {}.".format(person_label, block=False))
                self._robot.head.close()

                if self._found_people_designator:
                    self._found_people_designator.write(result_people)

                return 'found'
            else:
                rospy.logwarn("Could not find people meeting the requirements")
                # rate.sleep()

        rospy.logwarn("Exceeded trial or time limit")
        self._robot.head.close()
        rospy.sleep(2.0)
        return 'failed'

    def _check_person_property(self, person, prop_name, prop_value):
        return check_person_property(person, prop_name, prop_value, self._strict, self._reverse)


class FindFirstPerson(smach.StateMachine):
    """
    Wrapper around FindPeople to get the first person that matches the search criteria
    """
    def __init__(self,
                 robot,
                 found_person_designator,
                 properties=None,
                 query_entity_designator=None,
                 look_distance=10.0,
                 speak=False,
                 strict=True,
                 reverse=False,
                 nearest=False,
                 attempts=1,
                 search_timeout=60,
                 look_range=(-np.pi/2, np.pi/2),
                 look_steps=8,
                 attributes=None):
        """
        Initialization method

        :param robot: robot api object
        :param found_person_designator: A designator to write the search result to.
        :param properties: (dict) (default: None) keyvalue pair of the properties a person must
            possess. None as a value for a property would search for all possible
            values of the property.
        :param query_entity_designator: (default: None) An entity designator to match all found
            people to
        :param look_distance: (float) (default: 10.0) The distance (radius) which the robot must look at
        :param speak: (bool) (default: False) If True, the robot will speak while trying to find
            a named person
        :param strict: (bool) (default: True)  Only used if properties is not None AND the {key:value} pair of a
            property has non None values.
            If set to True then only people with all specified
            properties are returned, else all people with at least one true property.
            Example:
                properties = {'tags': ['LWaving', 'RWaving', 'LHolding', 'RHolding']}
                strict = True
                    This will return a list of people who have the tags:
                        'LWaving' AND 'RWaving' AND 'LHolding' AND 'RHolding'

                strict = False
                    This will return a list of people who have the tags:
                        'LWaving' OR 'RWaving' OR 'LHolding' OR 'RHolding'

        :param nearest: (bool) (default: False) If True, selects the people nearest to the robot who match the
            requirements posed using the properties, query_entity_designator, look distance and strict arguments
        :param attempts: (int) (default: 1) Max number of search attempts
        :param search_timeout: (float) (default: 60) maximum time the robot is allowed to search
        :param look_range: (tuple of size 2) (default: (-np.pi/2, np.pi/2)) from what to what head angle should the
            robot search
        :param look_steps: (int) (default: 8) How many steps does it take in that range
        :param attributes: (dict) (default: None) attributes of a description the person must fit, e.g.
            {'gesture': 'waving'}
        """
        super(FindFirstPerson, self).__init__(outcomes=["found", "failed"])
        ds.is_writeable(found_person_designator)
        ds.check_type(found_person_designator, Entity)

        found_people_designator = ds.VariableDesignator(resolve_type=[Entity], name='new_people')

        with self:
            self.add("FIND_PEOPLE",
                     FindPeople(
                         robot=robot,
                         properties=properties,
                         query_entity_designator=query_entity_designator,
                         found_people_designator=found_people_designator.writeable,
                         look_distance=look_distance,
                         speak=speak,
                         strict=strict,
                         reverse=reverse,
                         nearest=nearest,
                         attempts=attempts,
                         search_timeout=search_timeout,
                         look_range=look_range,
                         look_steps=look_steps,
                         attributes=attributes),
                     transitions={
                         'found': 'GET_FIRST_ITERATE',
                         'failed': 'failed'
                     })

            self.add("GET_FIRST_ITERATE",
                     IterateDesignator(found_people_designator,
                                       found_person_designator),
                     transitions={'next': 'found',
                                  'stop_iteration': 'failed'})


class SetPoseFirstFoundPersonToEntity(smach.StateMachine):
    """
    Wrapper around FirstFoundPerson and addition of UpdateDestEntityPoseWithSrcEntity of top of it
    """

    def __init__(self,
                 robot,
                 dst_entity_designator,
                 dst_entity_type="waypoint",
                 found_person_designator=None,
                 properties=None,
                 query_entity_designator=None,
                 look_distance=10.0,
                 speak=False,
                 strict=True,
                 reverse=False,
                 nearest=False,
                 attempts=1,
                 search_timeout=60,
                 look_range=(-np.pi/2, np.pi/2),
                 look_steps=8):
        """
        Initialization method

        :param robot: robot api object
        :param dst_entity_designator: A designator of an Entity whose pose must be updated.
        :param dst_entity_type: (str) (default: waypoint) Type of the destination entity
        :param found_person_designator: (default: None) A designator to write the search result to.
        :param properties: (dict) (default: None) keyvalue pair of the properties a person must
            possess. None as a value for a property would search for all possible
            values of the property.
        :param query_entity_designator: (default: None) An entity designator to match all found
            people to
        :param look_distance: (float) (default: 10.0) The distance (radius) which the robot must look at
        :param speak: (bool) (default: False) If True, the robot will speak while trying to find
            a named person
        :param strict: (bool) (default: True)  Only used if properties is not None AND the {key:value} pair of a
            property has non None values.
            If set to True then only people with all specified
            properties are returned, else all people with at least one true property.
            Example:
                properties = {'tags': ['LWaving', 'RWaving', 'LHolding', 'RHolding']}
                strict = True
                    This will return a list of people who have the tags:
                        'LWaving' AND 'RWaving' AND 'LHolding' AND 'RHolding'

                strict = False
                    This will return a list of people who have the tags:
                        'LWaving' OR 'RWaving' OR 'LHolding' OR 'RHolding'

        :param nearest: (bool) (default: False) If True, selects the people nearest to the robot who match the
            requirements posed using the properties, query_entity_designator, look distance and strict arguments
        :param attempts: (int) (default: 1) Max number of search attempts
        :param search_timeout: (float) (default: 60) maximum time the robot is allowed to search
        :param look_range: (tuple of size 2) (default: (-np.pi/2, np.pi/2)) from what to what head angle should the
            robot search
        :param look_steps: (int) (default: 8) How many steps does it take in that range
        """
        super(SetPoseFirstFoundPersonToEntity,
              self).__init__(outcomes=["done", "failed"])

        if not found_person_designator:
            found_person_designator = ds.VariableDesignator(resolve_type=Entity, name='new_person').writeable

        with self:
            self.add("FIND_FIRST_PERSON",
                     FindFirstPerson(
                         robot=robot,
                         found_person_designator=found_person_designator,
                         properties=properties,
                         query_entity_designator=query_entity_designator,
                         look_distance=look_distance,
                         speak=speak,
                         strict=strict,
                         reverse=reverse,
                         nearest=nearest,
                         attempts=attempts,
                         search_timeout=search_timeout,
                         look_range=look_range,
                         look_steps=look_steps),
                     transitions={
                         'found': 'UPDATE_POSE',
                         'failed': 'failed'
                     })

            self.add("UPDATE_POSE",
                     UpdateDestEntityPoseWithSrcEntity(
                         robot=robot,
                         src_entity_designator=found_person_designator,
                         dst_entity_designator=dst_entity_designator,
                         dst_entity_type=dst_entity_type),
                     transitions={
                         'done': 'done',
                         'failed': 'failed'
                     })


class _DecideNavigateState(smach.State):
    """ Helper state to decide whether to use a NavigateToWaypoint or a NavigateToRoom state
    """
    def __init__(self, robot, waypoint_designator, room_designator):
        """
        Initialize method

        :param robot: Robot API object
        :param waypoint_designator: EdEntityDesignator that should resolve to a waypoint
        :param room_designator: EdEntityDesignator that should resolve to the room in which the waypoint is located
        """
        smach.State.__init__(self, outcomes=["waypoint", "room", "none"])
        self._robot = robot
        self._waypoint_designator = waypoint_designator
        self._room_designator = room_designator

    def execute(self, ud):

        # First: see if the waypoint exists
        entity = self._waypoint_designator.resolve()
        if entity:
            return "waypoint"

        # If not: try the room
        entity = self._room_designator.resolve()
        if entity:
            return "room"

        return "none"


class FindPeopleInRoom(smach.StateMachine):
    """ Uses NavigateToWaypoint or NavigateToRoom and subsequently tries to find people in that room
    in that room.
    """

    def __init__(self, robot, room, found_people_designator,
                 look_range=(-np.pi/2, np.pi/2),
                 look_steps=8):
        """
        Constructor

        :param robot: robot object
        :param area: (str) if a waypoint "<area>_waypoint" is present in the world model, the robot will navigate
            to this waypoint. Else, it will navigate to the room called "<area>"
        :param name: (str) Name of the person to look for
        :param discard_other_labels: (bool) Whether or not to discard faces based on label
        :param found_person_designator: (Designator) A designator that will resolve to the found object
        """
        smach.StateMachine.__init__(self, outcomes=["found", "not_found"])

        waypoint_designator = ds.EntityByIdDesignator(robot=robot, uuid=room + "_waypoint")
        room_designator = ds.EntityByIdDesignator(robot=robot, uuid=room)
        ds.check_type(found_people_designator, [Entity])
        ds.is_writeable(found_people_designator)

        with self:
            smach.StateMachine.add("DECIDE_NAVIGATE_STATE",
                                   _DecideNavigateState(robot=robot, waypoint_designator=waypoint_designator,
                                                        room_designator=room_designator),
                                   transitions={"waypoint": "NAVIGATE_TO_WAYPOINT",
                                                "room": "NAVIGATE_TO_ROOM",
                                                "none": "not_found"})

            smach.StateMachine.add("NAVIGATE_TO_WAYPOINT",
                                   NavigateToWaypoint(robot=robot,
                                                      waypoint_designator=waypoint_designator, radius=0.15, speak=False),
                                   transitions={"arrived": "FIND_PEOPLE",
                                                "unreachable": "not_found",
                                                "goal_not_defined": "not_found"})

            smach.StateMachine.add("NAVIGATE_TO_ROOM",
                                   NavigateToRoom(robot=robot,
                                                  entity_designator_room=room_designator, speak=False),
                                   transitions={"arrived": "FIND_PEOPLE",
                                                "unreachable": "not_found",
                                                "goal_not_defined": "not_found"})

            # Wait for the operator to appear and detect what he's pointing at
            smach.StateMachine.add("FIND_PEOPLE",
                                   FindPeople(robot=robot,
                                              query_entity_designator=room_designator,
                                              found_people_designator=found_people_designator,
                                              speak=True,
                                              look_range=look_range,
                                              look_steps=look_steps),
                                   transitions={"found": "found",
                                                "failed": "not_found"})


if __name__ == "__main__":

    import doctest
    doctest.testmod()

    # from robot_skills import get_robot
    #
    # if len(sys.argv) > 1:
    #     robot_name = sys.argv[1]
    #     _area = sys.argv[2]
    #
    #     rospy.init_node('test_find_people_in_room')
    #     _robot = get_robot(robot_name)
    #
    #     people = ds.VariableDesignator(resolve_type=[Entity])
    #     sm = FindPeopleInRoom(_robot, _area, people.writeable)
    #     sm.execute()
    #
    #     rospy.loginfo("Found {}".format(people.resolve()))
    # else:
    #     print("Please provide robot name as argument.")
    #     exit(1)
