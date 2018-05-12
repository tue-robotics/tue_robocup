#! /usr/bin/env python

# System
import math
import sys

# ROS
import geometry_msgs
import PyKDL as kdl
import rospy
import smach

# TU/e Robotics
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_skills.util import kdl_conversions
from robocup_knowledge import load_knowledge


challenge_knowledge = load_knowledge('challenge_help_me_carry')


class FindPerson(smach.State):
    """ Smach state to find a person. The robot looks around and tries to recognize all faces in view.

    """
    # ToDo: robot only mentions that it has found the person. Doesn't do anything else...
    def __init__(self, robot, person_label="operator", _timeout=60.0, look_distance=2.0, probability_threshold=1.5):
        """ Initialization method

        :param robot: robot api object
        :param person_label: (str) person label
        :param _timeout: (float) maximum time the robot is allowed to search
        :param look_distance: (float) robot only considers laser entities within this radius
        """
        smach.State.__init__(self, outcomes=['found', 'failed'])

        self._robot = robot
        self._person_label = person_label
        self._timeout = _timeout
        self._look_distance = look_distance
        self._face_pos_pub = rospy.Publisher(
            '/%s/find_person/person_detected_face' % robot.robot_name,
            geometry_msgs.msg.PointStamped, queue_size=10)
        self._probability_threshold = probability_threshold

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
        while (rospy.Time.now() - start_time).to_sec() < self._timeout:
            if self.preempt_requested():
                return 'failed'

            self._robot.head.look_at_point(head_goals[i])
            i += 1
            if i == len(head_goals):
                i = 0
            self._robot.head.wait_for_motion_done()
            raw_detections = self._robot.perception.detect_faces()
            best_detection = self._robot.perception.get_best_face_recognition(
                raw_detections, self.person_label, probability_threshold=self._probability_threshold)

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

            found_person = self._robot.ed.get_closest_laser_entity(radius=self._look_distance,
                                                                   center_point=person_pos_kdl)
            if found_person:
                self._robot.speech.speak("I found {}".format(self.person_label), block=False, mood="excited")
                self._robot.head.close()

                self._robot.ed.update_entity(
                    id=self.person_label,
                    frame_stamped=kdl_conversions.FrameStamped(kdl.Frame(person_pos_kdl.vector), "/map"),
                    type="waypoint")

                return 'found'
            else:
                rospy.logwarn("Could not find {} in the {}".format(self._person_label, self.area))

        self._robot.head.close()
        rospy.sleep(2.0)
        return 'failed'


class FindPersoninRoom(smach.StateMachine):

    def __init__(self, robot, area, name):
        """ Constructor
        :param robot: robot object
        """
        smach.StateMachine.__init__(self, outcomes=["Found", "Not_found"])

        self.userdata.operator = None
        self.robot = robot

        with self:
            smach.StateMachine.add("NAVIGATE_TO_AREA", states.NavigateToWaypoint(
                robot=robot, waypoint_designator=ds.EntityByIdDesignator(robot=robot, id=area), radius=0.15),
                                   transitions={"arrived": "FIND_PERSON",
                                                "unreachable": "Not_found",
                                                "goal_not_defined": "Not_found"})

            # Wait for the operator to appear and detect what he's pointing at
            smach.StateMachine.add("FIND_PERSON", FindPerson(robot=robot, person_label=name),
                                   transitions={"found": "Found",
                                                "failed": "Not_found"})


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
    sm = FindPersoninRoom(_robot, _area, _name)
    sm.execute()
