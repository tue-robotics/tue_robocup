#! /usr/bin/env python

# System
import math
import sys

# ROS
import PyKDL as kdl
import geometry_msgs
import rospy
import smach

# TU/e Robotics
import robot_smach_states as states
import robot_smach_states.util.designators as ds
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
    """ Smach state to find a person. The robot looks around and tries to recognize all faces in view.

        """
    # ToDo: robot only mentions that it has found the person. Doesn't do anything else...

    def __init__(self, robot, person_label='operator', lost_timeout=60, look_distance=0.5, probability_threshold=1.5,
                 discard_other_labels=True, found_entity_designator=None, room=None):
        """ Initialization method

        :param robot: robot api object
        :param person_label: (str) person label
        :param lost_timeout: (float) maximum time the robot is allowed to search
        :param look_distance: (float) robot only considers laser entities within this radius
        :param discard_other_labels: (bool) whether or not to discard recognitions based on the label
        :param room: has to be the id of a room type in the knowledge (f.e. bedroom)
        """
        smach.State.__init__(self, outcomes=['found', 'failed'])

        self._robot = robot
        self._person_label = person_label
        self._lost_timeout = lost_timeout
        self._look_distance = look_distance
        self._face_pos_pub = rospy.Publisher(
            '/%s/find_person/person_detected_face' % robot.robot_name,
            geometry_msgs.msg.PointStamped, queue_size=10)
        self._probability_threshold = probability_threshold
        self._discard_other_labels = discard_other_labels
        self._found_entity_designator = found_entity_designator
        self._room = room

    def execute(self, userdata=None):
        rospy.loginfo("Trying to find {}".format(self._person_label))
        self._robot.head.look_at_standing_person()
        self._robot.speech.speak("{}, please look at me while I am looking for you".format(self._person_label),
                                 block=False)
        start_time = rospy.Time.now()

        look_distance = 2.0  # magic number 4
        look_angles = [f * math.pi / d if d != 0 else 0.0 for f in [-1, 1] for d in [0, 6, 4, 2.3]]  # Magic numbers
        head_goals = [kdl_conversions.VectorStamped(x=look_distance * math.cos(angle),
                                                    y=look_distance * math.sin(angle), z=1.7,
                                                    frame_id="/%s/base_link" % self._robot.robot_name)
                      for angle in look_angles]

        i = 0
        while (rospy.Time.now() - start_time).to_sec() < self._lost_timeout:
            if self.preempt_requested():
                return 'failed'

            self._robot.head.look_at_point(head_goals[i])
            i += 1
            if i == len(head_goals):
                i = 0
            self._robot.head.wait_for_motion_done()
            raw_detections = self._robot.perception.detect_faces()
            if self._discard_other_labels:
                best_detection = self._robot.perception.get_best_face_recognition(
                    raw_detections, self._person_label, probability_threshold=self._probability_threshold)
            else:
                if raw_detections:
                    # Take the biggest ROI
                    best_detection = max(raw_detections, key=lambda r: r.roi.height)
                else:
                    best_detection = None

            rospy.loginfo("best_detection = {}".format(best_detection))
            if not best_detection:
                continue
            roi = best_detection.roi
            try:
                person_pos_kdl = self._robot.perception.project_roi(roi=roi, frame_id="map")
            except Exception as e:
                rospy.logerr("head.project_roi failed: %s", e)
                return 'failed'
            person_pos_ros = kdl_conversions.kdl_vector_stamped_to_point_stamped(person_pos_kdl)
            self._face_pos_pub.publish(person_pos_ros)

            if self._room:
                room_entity = self._robot.ed.get_entity(id=self._room)
                if room_entity.in_volume(person_pos_kdl, 'in'):
                    found_person = self._robot.ed.get_closest_laser_entity(radius=self._look_distance,
                                                                           center_point=person_pos_kdl)
                else:
                    continue
            else:
                found_person = self._robot.ed.get_closest_laser_entity(radius=self._look_distance,
                                                                       center_point=person_pos_kdl)

            if found_person:
                self._robot.speech.speak("I found {}".format(self._person_label), block=False)
                self._robot.head.close()

                self._robot.ed.update_entity(
                    id=self._person_label,
                    frame_stamped=kdl_conversions.FrameStamped(kdl.Frame(person_pos_kdl.vector), "/map"),
                    type="waypoint")

                if self._found_entity_designator:
                    self._found_entity_designator.write(found_person)

                return 'found'
            else:
                rospy.logwarn("Could not find {}".format(self._person_label))

        self._robot.head.close()
        rospy.sleep(2.0)
        return 'failed'


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

    def __init__(self, robot, area, name, discard_other_labels=True, found_entity_designator=None):
        """ Constructor
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
                                                             found_entity_designator=found_entity_designator),
                                   transitions={"found": "found",
                                                "failed": "not_found"})


if __name__ == "__main__":
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
        _area = sys.argv[2]
        _name = sys.argv[3]
    else:
        print "Please provide robot name as argument."
        exit(1)

    if robot_name == "amigo":
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == "sergio":
        from robot_skills.sergio import Sergio as Robot

    rospy.init_node('test_follow_operator')
    _robot = Robot()
    sm = FindPersonInRoom(_robot, _area, _name)
    sm.execute()
