from __future__ import print_function

# System
import math
import sys
import numpy as np

# ROS
import PyKDL as kdl
import geometry_msgs
import rospy
import smach
from sensor_msgs.msg import Image, CameraInfo
import message_filters

# TU/e Robotics
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_smach_states.util.designators import check_type
from robot_skills.util import kdl_conversions

# class CheckIfPersonInRoom(smach.State):
#     def __init__(self, robot, room):
#         """
#
#         :param robot: robot api object
#         :param room: room where person should be found
#         """
#         smach.State.__init__(self, outcomes=['true', 'false'])
#         self._robot = robot
#         self._room = room
#
#     def execute(self, userdata=None):


class FindPerson(smach.State):
    """
    Smach state to find a person. The robot looks around and tries to recognize all faces in view.
    """

    def __init__(self, robot, person_label='operator', search_timeout=60, look_distance=3.0, probability_threshold=1.5,
                 discard_other_labels=True, found_entity_designator=None, room=None, speak_when_found=True,
                 look_range=(-np.pi/2, np.pi/2), look_steps=8):
        """
        Initialization method

        :param robot: robot api object
        :param person_label: (str) person label or a designator resolving to a str
        :param search_timeout: (float) maximum time the robot is allowed to search
        :param look_distance: (float) robot only considers laser entities within this radius
        :param discard_other_labels: (bool) whether or not to discard recognitions based on the label
        :param room: has to be the id of a room type in the knowledge (f.e. bedroom)
        :param speak_when_found: bool indicating whether or not the robot should speak upon finding a person
        :param look_range: from what to what head angle should the robot search (defaults to -90 to +90 deg)
        :param look_steps: How many steps does it take in that range (default = 8)
        """
        smach.State.__init__(self, outcomes=['found', 'failed'])

        self._robot = robot

        check_type(person_label, str)
        self._person_label = person_label

        self._search_timeout = search_timeout
        self._look_distance = look_distance
        self._face_pos_pub = rospy.Publisher(
            '/%s/find_person/person_detected_face' % robot.robot_name,
            geometry_msgs.msg.PointStamped, queue_size=10)
        self._probability_threshold = probability_threshold
        self._discard_other_labels = discard_other_labels

        if found_entity_designator:
            ds.is_writeable(found_entity_designator)
        self._found_entity_designator = found_entity_designator

        self._room = room

        self._look_angles = np.linspace(look_range[0], look_range[1], look_steps)

        self.speak_when_found = speak_when_found

    def execute(self, userdata=None):
        person_label = self._person_label.resolve() if hasattr(self._person_label, 'resolve') else self._person_label

        rospy.loginfo("Trying to find {}".format(person_label))
        self._robot.head.look_at_standing_person()
        self._robot.speech.speak("{}, please look at me while I am looking for you".format(person_label),
                                 block=False)
        start_time = rospy.Time.now()

        look_distance = 2.0  # magic number 4
        head_goals = [kdl_conversions.VectorStamped(x=look_distance * math.cos(angle),
                                                    y=look_distance * math.sin(angle), z=1.3,
                                                    frame_id="/%s/base_link" % self._robot.robot_name)
                      for angle in self._look_angles]

        i = 0

        rate = rospy.Rate(2)
        while (rospy.Time.now() - start_time).to_sec() < self._search_timeout and not rospy.is_shutdown():
            if self.preempt_requested():
                return 'failed'

            self._robot.head.look_at_point(head_goals[i])
            i += 1
            if i == len(head_goals):
                i = 0
            self._robot.head.wait_for_motion_done()

            self._image_data = self._robot.perception.get_rgb_depth_caminfo()
            if self._image_data:
                success, found_people_ids = self._robot.ed.detect_people(*self._image_data)
            else:
                rospy.logwarn("Could not get_rgb_depth_caminfo")
                success, found_people_ids = False, []
            found_people = [self._robot.ed.get_entity(id) for id in found_people_ids]

            rospy.loginfo("Found {} people: {}".format(len(found_people), found_people))
            found_people = [p for p in found_people if p]
            rospy.loginfo("{} people remaining after None-check".format(len(found_people)))

            found_names = {person.id: person for person in found_people}

            found_person = None

            if self._discard_other_labels:
                found_person = found_names.get(person_label, None)
            else:
                # find which of those is closest
                robot_pose = self._robot.base.get_location()
                found_person = min(found_people, key=lambda person: person.pose.frame.p - robot_pose.frame.p)

            if self._room:
                room_entity = self._robot.ed.get_entity(id=self._room)
                if not room_entity.in_volume(found_person.pose.extractVectorStamped(), 'in'):
                    # If the person is not in the room we are looking for, ignore the person
                    rospy.loginfo("We found a person '{}' but was not in desired room '{}' so ignoring that person"
                                  .format(found_person.id, room_entity.id))
                    found_person = None

            if found_person:
                rospy.loginfo("I found {} who I assume is {} at {}".format(found_person.id, person_label, found_person.pose.extractVectorStamped(), block=False))
                if self.speak_when_found:
                    self._robot.speech.speak("I think I found {}.".format(person_label, block=False))
                self._robot.head.close()

                if self._found_entity_designator:
                    self._found_entity_designator.write(found_person)

                return 'found'
            else:
                rospy.logwarn("Could not find {}".format(person_label))
                rate.sleep()

        self._robot.head.close()
        rospy.sleep(2.0)
        return 'failed'


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


class FindPersonInRoom(smach.StateMachine):
    """ Uses NavigateToWaypoint or NavigateToRoom and subsequently tries to find a person
    in that room.
    """

    def __init__(self, robot, area, name, discard_other_labels=True, found_entity_designator=None,
                 look_range=(-np.pi/2, np.pi/2), look_steps=8):
        """
        Constructor

        :param robot: robot object
        :param area: (str) if a waypoint "<area>_waypoint" is present in the world model, the robot will navigate
            to this waypoint. Else, it will navigate to the room called "<area>"
        :param name: (str) Name of the person to look for
        :param discard_other_labels: (bool) Whether or not to discard faces based on label
        :param found_entity_designator: (Designator) A designator that will resolve to the found object
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
            smach.StateMachine.add("FIND_PERSON", FindPerson(robot=robot, person_label=name,
                                                             discard_other_labels=discard_other_labels,
                                                             found_entity_designator=found_entity_designator,
                                                             look_range=look_range,
                                                             look_steps=look_steps),
                                   transitions={"found": "found",
                                                "failed": "not_found"})


if __name__ == "__main__":

    from robot_skills import get_robot

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
        _area = sys.argv[2]
        _name = sys.argv[3]

        rospy.init_node('test_find_person_in_room')
        _robot = get_robot(robot_name)
        sm = FindPersonInRoom(_robot, _area, _name)
        sm.execute()
    else:
        print("Please provide robot name as argument.")
        exit(1)


