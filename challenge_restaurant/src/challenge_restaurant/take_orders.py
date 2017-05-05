#!/usr/bin/python

# ROS
import smach


class TakeOrder(smach.State):
    """ Take an order """

    def __init__(self, robot):
        """ Constructor

        :param robot: robot object
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        self._robot = robot

    def execute(self, userdata):
        """ Does the actual work """
        # ToDo: create interaction to take an order
        self._robot.speech.speak("I'm sorry, but I'm not able to understand your order")

        return 'succeeded'
