#!/usr/bin/python

# ROS
import smach

# TU/e Robotics
import robot_smach_states as states


class Restaurant(smach.StateMachine):
    """ Main statemachine for the restaurant challenge """
    def __init__(self, robot):
        """ Constructor

        :param robot: robot object
        """
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted'])

        with self:
            smach.StateMachine.add('INITIALIZE',
                                   states.Initialize(robot),
                                   transitions={'initialized': 'AWAIT_START',
                                                'abort': 'Aborted'})
