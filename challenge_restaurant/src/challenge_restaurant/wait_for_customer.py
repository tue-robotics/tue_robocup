#!/usr/bin/python

# ROS
import rospy
import smach

# TU/e Robotics
from robot_skills.util.kdl_conversions import frame_stamped
from hmi import TimeoutException

class WaitForCustomer(smach.State):
    """ Wait for the waiving person """

    def __init__(self, robot, location_id):
        """ Constructor

        :param robot: robot object
        :param location_id: string with which the location of the caller will be designated
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'aborted', 'rejected'])
        self._robot = robot
        self._location_id = location_id

    def execute(self, userdata=None):
        """ Does the actual work

        :param userdata:
        :return:
        """

        self._robot.head.reset()
        rospy.sleep(1)

        while True:
            self._robot.speech.speak("I'm looking for waving persons")
            persons = self._robot.head.detect_waving_persons()
            if not persons:
                continue

            self._robot.speech.speak("I found a waving person")

            person = persons[0]

            try:
                point = self._robot.head.project_roi(person.roi, frame_id="map")
                break
            except ValueError as e:
                rospy.logerr('project failed: %s', e)

        pose = frame_stamped("map", point.vector.x(), point.vector.y(), 0.0)

        self._robot.speech.speak("I have seen a waving person, should I continue?")

        if self._confirm():
            self._robot.ed.update_entity(id="customer", frame_stamped=pose, type="waypoint")
            return 'succeeded'
        else:
            return 'rejected'

    def _confirm(self):
        cgrammar = """
        C[True] -> amigo take the order
        C[False] -> amigo wait
        """
        self._robot.head.look_at_standing_person()
        for i in range(3):
            try:
                speech_result = self._robot.hmi.query(description="Should I get the order?",
                                                      grammar=cgrammar, target="C")
                return speech_result.semantics
            except TimeoutException:
                pass
        return False
