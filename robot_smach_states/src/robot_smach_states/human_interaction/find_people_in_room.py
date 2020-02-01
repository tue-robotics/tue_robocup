from __future__ import print_function

# System
import math
import sys
import numpy as np

# ROS
import rospy
import smach

# TU/e Robotics
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_skills.util import kdl_conversions
from robot_skills.util.entity import Entity

__all__ = ['FindPeople', 'FindFirstPerson', 'SetPoseFirstFoundPersonToEntity', 'FindPeopleInRoom']


class FindPeople(smach.State):
    """
    Smach state to find a person. The robot looks around and tries to find
    people in its view.

    >>> from robot_skills.mockbot import Mockbot
    >>> robot = Mockbot()
    >>> # Designator to store the result
    >>> des = ds.VariableDesignator(resolve_type=[Entity])
    >>>
    >>> sm = FindPeople(robot=robot, properties={'id': 'NAME'}, found_people_designator=des.writeable)
    >>> # sm.execute()
    >>> # des.resolve()

    Entity(id=something, )

    """

    def __init__(self, robot, properties=None, query_entity_designator=None,
                 found_people_designator=None, look_distance=10.0, speak=False,
                 strict=True, nearest=False, attempts=1, search_timeout=60,
                 look_range=(-np.pi/2, np.pi/2), look_steps=8):
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
            property has non None values.
            If set to True then only people with all specified
            properties are returned, else all people with at least one true property.
            Example:
                properties = {'tags': ['LWave', 'RWave', 'LHolding', 'RHolding']}
                strict = True
                    This will return a list of people who have the tags:
                        'LWave' AND 'RWave' AND 'LHolding' AND 'RHolding'

                strict = False
                    This will return a list of people who have the tags:
                        'LWave' OR 'RWave' OR 'LHolding' OR 'RHolding'

        :param nearest: (bool) (default: False) If True, selects the people nearest to the robot who match the
            requirements posed using the properties, query_entity_designator, look distance and strict arguments
        :param attempts: (int) (default: 1) Max number of search attempts
        :param search_timeout: (float) (default: 60) maximum time the robot is allowed to search
        :param look_range: (tuple of size 2) (default: (-np.pi/2, np.pi/2)) from what to what head angle should the
            robot search
        :param look_steps: (int) (default: 8) How many steps does it take in that range
        """
        smach.State.__init__(self, outcomes=['found', 'failed'])

        self._robot = robot

        self._properties = properties
        self._look_distance = look_distance
        self._speak = speak
        self._strict = strict
        self._nearest = nearest
        self._attempts = attempts

        self._search_timeout = search_timeout

        self._look_angles = np.linspace(look_range[0], look_range[1], look_steps)

        if found_people_designator:
            ds.is_writeable(found_people_designator)
            ds.check_type(found_people_designator, [Entity])
        self._found_people_designator = found_people_designator

        if query_entity_designator:
            ds.check_type(query_entity_designator, Entity)
        self._query_entity_designator = query_entity_designator

    def execute(self, userdata=None):
        look_angles = None
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

        head_goals = [kdl_conversions.VectorStamped(x=self._look_distance * math.cos(angle),
                                                    y=self._look_distance * math.sin(angle),
                                                    z=1.5,
                                                    frame_id="/%s/base_link" % self._robot.robot_name)
                      for angle in self._look_angles]

        i = 0
        attempts = 0

        rate = rospy.Rate(2)
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
            # TODO: Check probable bug here
            found_people = filter(lambda x: (x.pose.frame.p - robot_pose.frame.p).Norm() < self._look_distance,
                                  found_people)

            rospy.loginfo("{} people remaining after distance < {}-check".format(len(found_people), self._look_distance))

            if self._properties:
                for k, v in self._properties.items():
                    found_people = filter(lambda x:
                            self._check_person_property(x, k, v), found_people)
                    rospy.loginfo("{} people remaining after {}={} check".format(len(found_people), k, v))

            result_people = []

            if self._query_entity_designator:
                query_entity = self._query_entity_designator.resolve()
                if query_entity:
                    result_people = filter(lambda x: query_entity.in_volume(x.pose.extractVectorStamped(), 'in'),
                                           found_people)
                    rospy.loginfo("{} result_people remaining after 'in'-'{}' check".format(len(result_people), query_entity.id))

                    # If people not in query_entity then try if query_entity in
                    # people
                    if not result_people:
                        # This is for a future feature when object recognition
                        # becomes more advanced
                        try:
                            result_people = filter(lambda x: x.in_volume(query_entity.pose.extractVectorStamped(), 'in'),
                                                   found_people)
                            rospy.loginfo(
                                "{} result_people remaining after 'in'-'{}' check".format(len(result_people), query_entity.id))
                        except Exception:
                            pass
            else:
                result_people = found_people

            if result_people:
                if self._nearest:
                    result_people.sort(key=lambda e: (e.pose.frame.p - robot_pose.frame.p).Norm())
                if person_label and \
                    filter(lambda x: self._check_person_property(x, "id", person_label), result_people) \
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
        person_attr_val = getattr(person.person_properties, prop_name)
        rospy.loginfo("For person {}: {} is {}".format(person.id, prop_name, person_attr_val))
        if prop_value:
            if self._strict:
                # Making the conditon less strict to increase search domain
                rospy.loginfo("Executing strict=True")
                if isinstance(person_attr_val, list):
                    sub_list = filter(lambda x: x in prop_value, person_attr_val)
                    return sub_list == prop_value
                else:
                    return person_attr_val == prop_value
            else:
                if (isinstance(person_attr_val, list)
                        and filter(lambda x: x in prop_value, person_attr_val)
                    ) or person_attr_val in prop_value:
                    rospy.loginfo("Executing strict=False")
                    return True
        else:
            if person_attr_val:
                rospy.loginfo("By passing strict check")
                return True

        return False


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
                 nearest=False,
                 attempts=1,
                 search_timeout=60,
                 look_range=(-np.pi/2, np.pi/2),
                 look_steps=8):
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
                properties = {'tags': ['LWave', 'RWave', 'LHolding', 'RHolding']}
                strict = True
                    This will return a list of people who have the tags:
                        'LWave' AND 'RWave' AND 'LHolding' AND 'RHolding'

                strict = False
                    This will return a list of people who have the tags:
                        'LWave' OR 'RWave' OR 'LHolding' OR 'RHolding'

        :param nearest: (bool) (default: False) If True, selects the people nearest to the robot who match the
            requirements posed using the properties, query_entity_designator, look distance and strict arguments
        :param attempts: (int) (default: 1) Max number of search attempts
        :param search_timeout: (float) (default: 60) maximum time the robot is allowed to search
        :param look_range: (tuple of size 2) (default: (-np.pi/2, np.pi/2)) from what to what head angle should the
            robot search
        :param look_steps: (int) (default: 8) How many steps does it take in that range
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
                         nearest=nearest,
                         attempts=attempts,
                         search_timeout=search_timeout,
                         look_range=look_range,
                         look_steps=look_steps),
                     transitions={
                         'found': 'GET_FIRST_ITERATE',
                         'failed': 'failed'
                     })

            self.add("GET_FIRST_ITERATE",
                     states.IterateDesignator(found_people_designator,
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
                properties = {'tags': ['LWave', 'RWave', 'LHolding', 'RHolding']}
                strict = True
                    This will return a list of people who have the tags:
                        'LWave' AND 'RWave' AND 'LHolding' AND 'RHolding'

                strict = False
                    This will return a list of people who have the tags:
                        'LWave' OR 'RWave' OR 'LHolding' OR 'RHolding'

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
                     states.UpdateDestEntityPoseWithSrcEntity(
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

        waypoint_designator = ds.EntityByIdDesignator(robot=robot, id=room + "_waypoint")
        room_designator = ds.EntityByIdDesignator(robot=robot, id=room)
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
                                   states.NavigateToWaypoint(robot=robot,
                                                             waypoint_designator=waypoint_designator, radius=0.15),
                                   transitions={"arrived": "FIND_PEOPLE",
                                                "unreachable": "not_found",
                                                "goal_not_defined": "not_found"})

            smach.StateMachine.add("NAVIGATE_TO_ROOM", states.NavigateToRoom(robot=robot,
                                                                             entity_designator_room=room_designator),
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
