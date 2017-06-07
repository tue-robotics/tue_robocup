#!/usr/bin/python

import rospy
import smach
import PyKDL as kdl
from hmi import TimeoutException
from robot_skills.util.kdl_conversions import FrameStamped


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

    def execute(self, userdata):
        """ Does the actual work

        :param userdata:
        :return:
        """

        self._robot.head.reset()
        rospy.sleep(1)

        while True:
            self._robot.speech.speak("I'm looking for waving persons")
            persons = self._robot.head.detect_waving_persons_3d()
            if not persons:
                continue

            self._robot.speech.speak("I found a waving person")

            person = persons[0]

            try:
                point = person.kdl_point
                pose = FrameStamped(frame=kdl.Frame(kdl.Rotation(), point.vector), frame_id=point.frame_id)
                break
            except ValueError as e:
                rospy.logerr('project failed: %s', e)


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
