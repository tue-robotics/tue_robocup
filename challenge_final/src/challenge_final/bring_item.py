#!/usr/bin/python

# ROS
import rospy
import smach

# TU/e Robotics
import robot_smach_states as states

# Challenge storing groceries
from config import *
from open_door import OpenDoorMachine
from storing_groceries import StoringGroceries
import robot_smach_states.util.designators as ds

class Final_complete_bring_command(smach.StateMachine):
    def __init__(self, robot, cabinet_id, cabinet_navigate_area, cabinet_inspect_area):
        self.cabinet = ds.EntityByIdDesignator(robot=robot, id=cabinet_id)
        self.empty_arm_designator = ds.UnoccupiedArmDesignator(robot.arms, robot.leftArm, name="empty_arm_designator")
        self.grab_designator = grab_designator

        """ Constructor

        :param robot: robot object
        """
        smach.StateMachine.__init__(self, outcomes=['succeeded'])

        with self:
            smach.StateMachine.add('OPEN_CABINET',
                                    OpenDoorMachine(robot, 'temp', 'in_front_of', 'shelf6'),
                                    transitions={'succeeded': 'OPEN_CABINET2',
                                                 'failed': 'OPEN_CABINET2'})

            smach.StateMachine.add('OPEN_CABINET',
                                    OpenDoorMachine(robot, 'temp', 'in_front_of', 'shelf6'),
                                    transitions={'succeeded': 'NAV_TO_START',
                                                 'failed': 'NAV_TO_START'})

            smach.StateMachine.add("NAVIGATE_TO_CABINET",
                                   states.NavigateToSymbolic(robot, {self.cabinet: cabinet_navigate_area}, self.cabinet),
                                   transitions={'arrived': 'UPDATE_CABINET_POSE',
                                                'unreachable': 'failed',
                                                'goal_not_defined': 'failed'})

            smach.StateMachine.add("INSPECT_SHELVES",
                                    InspectShelves(robot, cabinet),
                                    transitions={'succeeded': 'WRITE_PDF_SHELVES',
                                                 'nothing_found': 'WRITE_PDF_SHELVES',
                                                 'failed': 'WRITE_PDF_SHELVES'})

            smach.StateMachine.add("GRAB_ITEM",
                                   states.Grab(robot, self.grab_designator, self.empty_arm_designator),
                                   transitions={'done': 'succeeded',
                                                'failed': 'succeeded'})

