#!/usr/bin/python
import roslib;
import rospy
import smach
import sys
import time
import datetime
from hmi import TimeoutException

from cb_planner_msgs_srvs.msg import PositionConstraint

import robot_smach_states.util.designators as ds
from robot_smach_states.util.designators import check_type
from robot_skills.arms import Arm, GripperState

import robot_smach_states as states

from robocup_knowledge import load_knowledge
from robot_skills.util import kdl_conversions
from robot_skills.util.entity import Entity

challenge_knowledge = load_knowledge('challenge_help_me_carry')

def setup_challenge(self, robot):

    self.place_name = ds.EntityByIdDesignator(robot, id=challenge_knowledge.default_place, name="place_name")
    self.place_position = ds.LockingDesignator(ds.EmptySpotDesignator(robot, self.place_name, 
                                                                    name="placement",
                                                                    area=challenge_knowledge.default_area),
                                                                    name="place_position")

    self.empty_arm_designator = ds.UnoccupiedArmDesignator(robot.arms, robot.rightArm, name="empty_arm_designator")

    # With the empty_arm_designator locked, it will ALWAYS resolve to the same arm (first resolve is cached), unless it is unlocked
    # For this challenge, unlocking is not needed
    self.bag_arm_designator = self.empty_arm_designator.lockable()
    self.bag_arm_designator.lock()

    # We don't actually grab something, so there is no need for an actual thing to grab
    self.current_item = ds.VariableDesignator(Entity("dummy",
                                                    "dummy",
                                                    "/{}/base_link".format(robot.robot_name),
                                                    kdl_conversions.kdlFrameFromXYZRPY(0.6, 0, 0.5),
                                                    None,
                                                    {},
                                                    [],
                                                    datetime.datetime.now()),
                                                    name="current_item")

class WaitForOperatorCommand(smach.State):

    def __init__(self, robot, possible_commands, commands_as_outcomes=False, commands_as_userdata=False):

        self._robot = robot
        self._possible_commands = possible_commands
        self._commands_as_outcomes = commands_as_outcomes
        self._commands_as_userdata = commands_as_userdata

        if commands_as_outcomes:  # each possible command is a separate outcome
            _outcomes = possible_commands + ['abort']
        else:                     # outcome is success or abort, recognized command is returned using output_keys
            _outcomes = ['success', 'abort']

        if commands_as_userdata:  # pass the recognized command to the next state
            _output_keys = ['command_recognized']
        else:                     # do not pass data to the next state
            _output_keys = []

        smach.State.__init__(self, outcomes=_outcomes,  output_keys=_output_keys)

    def _listen_for_commands(self, tries=5, time_out=30):
        for i in range(0, tries):
            try:
                result = self._robot.hmi.query('What command?', 'T -> ' + ' | '.join(self._possible_commands), 'T', timeout=time_out)
                command_recognized = result.sentence
            except TimeoutException:
                command_recognized = None
            if command_recognized == "":
                self._robot.speech.speak("I am still waiting for a command and did not hear anything")

            elif command_recognized in self._possible_commands:
                self._robot.speech.speak(command_recognized + "OK")
                return "success", command_recognized
            else:
                self._robot.speech.speak("I don't understand, I expected a command like " + ", ".join(self._possible_commands))

        self._robot.speech.speak("I did not recognize a command and will stop now")
        return "abort", "abort"

    def execute(self, userdata):
        # Stop the base
        self._robot.base.local_planner.cancelCurrentPlan()
        self._robot.head.look_at_standing_person()

        outcome, command_recognized = self._listen_for_commands()

        if self._commands_as_userdata:
            userdata.command_recognized = command_recognized

        if self._commands_as_outcomes:
            return command_recognized
        else:
            return outcome

class StoreCarWaypoint(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['success', 'abort'])
        self._robot = robot
        robot.base.local_planner.cancelCurrentPlan()

    def execute(self, userdata):
        success = self._robot.ed.update_entity(id=challenge_knowledge.waypoint_car['id'],
                                               frame_stamped=self._robot.base.get_location(),
                                               type="waypoint")

        if success:
            return "success"
        else:
            return "abort"

class GrabItem(smach.State):
    def __init__(self, robot, empty_arm_designator, current_item):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed', 'timeout'],
                             input_keys=['target_room_in'],
                             output_keys=['target_room_out'])
        self._robot = robot
        self._empty_arm_designator = empty_arm_designator
        self._current_item = current_item
        robot.base.local_planner.cancelCurrentPlan()

    def execute(self, userdata):

        handOverHuman = states.HandoverFromHuman(self._robot,
                                                 self._empty_arm_designator,
                                                 "current_item",
                                                 self._current_item,
                                                 arm_configuration="carrying_bag_pose")

        userdata.target_room_out = userdata.target_room_in

        return handOverHuman.execute()


class NavigateToRoom(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,
                             outcomes=['unreachable','arrived','goal_not_defined'],
                             input_keys=['target_room'])
        self._robot = robot
        robot.base.local_planner.cancelCurrentPlan()

    def execute(self, userdata):

        target_waypoint = challenge_knowledge.waypoints[userdata.target_room]['id']
        target_radius = challenge_knowledge.waypoints[userdata.target_room]['radius']

        navigateToWaypoint = states.NavigateToWaypoint(self._robot, ds.EntityByIdDesignator(self._robot, id=target_waypoint), target_radius)

        return navigateToWaypoint.execute()


class DropBagOnGround(smach.StateMachine):
    """
    Put a bag in the robot's gripper on the ground
    """
    def __init__(self, robot, arm_designator):
        """
        :param robot: the robot with which to execute this state machine
        :param arm_designator: ArmDesignator resolving to Arm holding the bag to drop

        """
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        check_type(arm_designator, Arm)

        with self:
            smach.StateMachine.add('DROP_POSE', states.ArmToJointConfig(robot, arm_designator, "drop_bag_pose"),
                                    transitions={'succeeded':'OPEN_AFTER_DROP',
                                                'failed':'OPEN_AFTER_DROP'})

            smach.StateMachine.add('OPEN_AFTER_DROP', states.SetGripper(robot, arm_designator, gripperstate=GripperState.OPEN),
                                    transitions={'succeeded':'RESET_ARM_OK',
                                             'failed':'RESET_ARM_FAIL'})

            smach.StateMachine.add( 'RESET_ARM_OK', states.ResetArms(robot),
                                    transitions={'done':'succeeded'})

            smach.StateMachine.add( 'RESET_ARM_FAIL', states.ResetArms(robot),
                                    transitions={'done':'failed'})