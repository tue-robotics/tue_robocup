#!/usr/bin/python

# ROS
import smach

# TU/e Robotics
from robot_skills.util.kdl_conversions import frame_stamped


class WaitForCustomer(smach.State):
    """ Wait for the waiving person """

    def __init__(self, robot, location_id):
        """ Constructor

        :param robot: robot object
        :param location_id: string with which the location of the caller will be designated
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'aborted'])
        self._robot = robot
        self._location_id = location_id

    def execute(self, userdata):
        """ Does the actual work

        :param userdata:
        :return:
        """
        self._robot.speech.speak("I can't really detect waving persons, but I'll try anyway")
        # ToDo: fill in a pose

        dummy_pose = frame_stamped("map", 3.0, 0.0, 0.0)
        self._robot.ed.update_entity(id="customer", frame_stamped=dummy_pose, type="waypoint")

        # ToDo: add the speech: do we wan't to continue???

        return 'succeeded'
