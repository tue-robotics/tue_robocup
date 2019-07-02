#! /usr/bin/env python

# System
import math
import sys

# ROS
import rospy
import smach

# TU/e Robotics
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_skills.util import kdl_conversions

__all__ = ['FindPerson', 'FindPersonInRoom']


class FindPerson(smach.State):
    """
    Smach state to find a person. The robot looks around and tries to recognize all faces in view.
    """

    def __init__(self, robot, properties=None, query_entity_designator=None,
                 result_designator=None, look_distance=1.0, speak=False,
                 strict=True, nearest=False, attempts=1, search_timeout=60):
        """ Initialization method
        :param robot: robot api object
        :param properties: (dict) keyvalue pair of the properties a person must
            possess. None as a value for a property would search for all possible
            values of the property.
        :param query_entity_designator: An entity designator to match all found
            people to
        :param result_designator: A designator to write the search result to.
            The designator always has a list of found people written to it.
        :param look_distance: (float) The distance (radius) which the robot must look at
        :param speak: (bool) If True, the robot will speak while trying to find
            a named person
        :param strict: (bool) If True then only people with all specified
            properties is returned else all people with at least one true property
        :param nearest: (bool) If True, selects the person nearest to the robot
        :param attempts: (int) Max number of search attempts
        :param search_timeout: (float) maximum time the robot is allowed to search
        """
        smach.State.__init__(self, outcomes=['found', 'failed'])

        self._robot = robot

        self._properties = properties
        self._look_distance = look_distance
        self._speak = speak
        self._strict = strict
        self._nearest = nearest

        self._search_timeout = search_timeout

        if result_designator:
            ds.is_writeable(result_designator)
        self._result_designator = result_designator

        if query_entity_designator:
            ds.check_type(query_entity_designator, ds.Entity)
        self._query_entity_designator = query_entity_designator

    def execute(self, userdata=None):
        look_angles = None
        person_label = None
        if not self._properties:
            look_angles = [0]
        else:
            look_angles = [f * math.pi / d if d != 0 else 0.0 for f in [-1, 1] for d in [0, 6, 4, 2.3]]  # Magic numbers
            try:
                person_label = self._properties["id"]
                ds.check_type(person_label, "str")
                person_label = person_label.resolve() if hasattr(person_label, 'resolve') else person_label

                rospy.loginfo("Trying to find {}".format(person_label))
                if self._speak:
                    self._robot.speech.speak(
                            "{}, please look at me while I am looking for you".format(
                            person_label),
                        block=False)
            except:
                # The try except is to check if a named person is queried for
                # in the properties. If there is none then exception is raised
                # and nothing is to be done with it
                pass

        start_time = rospy.Time.now()

        head_goals = [kdl_conversions.VectorStamped(x=self._look_distance * math.cos(angle),
                                                    y=self._look_distance * math.sin(angle),
                                                    z=1.5,
                                                    frame_id="/%s/base_link" % self._robot.robot_name)
                      for angle in look_angles]

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

            self._image_data = self._robot.perception.get_rgb_depth_caminfo()
            success, found_people_ids = self._robot.ed.detect_people(*self._image_data)
            found_people = [self._robot.ed.get_entity(eid) for eid in found_people_ids]

            robot_pose = self._robot.base.get_location()
            # TODO: Check probable bug here
            found_people = filter(lambda x: (x.pose.frame.p -
                robot_pose.frame.p).Norm() < self._look_distance, found_people)


            if self._properties:
                for k, v in self._properties.items():
                    found_people = filter(lambda x:
                            self._check_person_property(x, k, v), found_people)

            result_people = None

            if self._query_entity_designator:
                query_entity = self._query_entity_designator.resolve()
                result_people = filter(lambda x:
                        query_entity.in_volume(x.pose.extractVectorStamped(),
                        'in'), found_people)

                # If people not in query_entity then try if query_entity in
                # people
                if not result_people:
                    # This is for a future feature when object recognition
                    # becomes more advanced
                    try:
                        result_people = filter(lambda x:
                            x.in_volume(query_entity.pose.extractVectorStamped(),
                                'in'), found_people)
                    except:
                        pass
            else:
                result_people = found_people


            if result_people:
                if person_label and filter(lambda x:
                        self._check_person_property(x, "id", person_label),
                        result_people) and self._speak:
                    self._robot.speech.speak("I think I found {}.".format(person_label, block=False))
                self._robot.head.close()

                if self._result_designator:
                    self._result_designator.write(result_people)

                return 'found'
            else:
                rospy.logwarn("Could not find people meeting the requirements")
                rate.sleep()

        rospy.logwarn("Exceeded trail or time limit")
        self._robot.head.close()
        rospy.sleep(2.0)
        return 'failed'

    def _check_person_property(self, person, prop_name, prop_value):
        person_attr_val = getattr(person, prop_name)
        if prop_value:
            if self._strict:
                if person_attr_val == prop_value:
                    return True
            else:
                if person_attr_val in prop_value:
                    return True
        else:
            if person_attr_val:
                return True

        return False


class _DecideNavigateState(smach.State):
    """ Helper state to decide whether to use a NavigateToWaypoint or a NavigateToRoom state
    """
    def __init__(self, robot, waypoint_designator, room_designator):
        """ Initialize method
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


class FindPersonInRoom(smach.StateMachine):
    """ Uses NavigateToWaypoint or NavigateToRoom and subsequently tries to find a person
    in that room.
    """

    def __init__(self, robot, area, name, discard_other_labels=True, found_person_designator=None):
        """ Constructor
        :param robot: robot object
        :param area: (str) if a waypoint "<area>_waypoint" is present in the world model, the robot will navigate
        to this waypoint. Else, it will navigate to the room called "<area>"
        :param name: (str) Name of the person to look for
        :param discard_other_labels: (bool) Whether or not to discard faces based on label
        :param found_person_designator: (Designator) A designator that will resolve to the found object
        """
        smach.StateMachine.__init__(self, outcomes=["found", "not_found"])

        waypoint_designator = ds.EntityByIdDesignator(robot=robot, id=area + "_waypoint")
        room_designator = ds.EntityByIdDesignator(robot=robot, id=area)

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
                                   transitions={"arrived": "FIND_PERSON",
                                                "unreachable": "not_found",
                                                "goal_not_defined": "not_found"})

            smach.StateMachine.add("NAVIGATE_TO_ROOM", states.NavigateToRoom(robot=robot,
                                                                             entity_designator_room=room_designator),
                                   transitions={"arrived": "FIND_PERSON",
                                                "unreachable": "not_found",
                                                "goal_not_defined": "not_found"})

            # Wait for the operator to appear and detect what he's pointing at
            smach.StateMachine.add("FIND_PERSON",
                                   FindPerson(robot=robot,
                                              properties={'id': name},
                                              query_entity_designator=room_designator,
                                              result_designator=found_person_designator,
                                              speak=True),
                                   transitions={"found": "found",
                                                "failed": "not_found"})


if __name__ == "__main__":

    from robot_skills import get_robot

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
        _area = sys.argv[2]
        _name = sys.argv[3]

        rospy.init_node('test_follow_operator')
        _robot = get_robot(robot_name)
        sm = FindPersonInRoom(_robot, _area, _name)
        sm.execute()
    else:
        print "Please provide robot name as argument."
        exit(1)
