#!/usr/bin/python

import math
import smach
import robot_smach_states as states

from robot_smach_states.util.designators import check_type
from robot_skills.arms import Arm, GripperState
from hmi import TimeoutException

from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_help_me_carry')


class WaitForOperatorCommand(smach.State):
    """
    The robot waits for command from the operator
    Possible commands are two types:
        - commands as outcomes: each possible command is possible outcome
        - commands as userdata: the command is passed to the next state

    """
    def __init__(self, robot, possible_commands, commands_as_outcomes=False, commands_as_userdata=False, target=None):

        self._robot = robot
        self._possible_commands = possible_commands
        self._commands_as_outcomes = commands_as_outcomes
        self._commands_as_userdata = commands_as_userdata
        self.target_destination = target

        if commands_as_outcomes:  # each possible command is a separate outcome
            _outcomes = possible_commands + ['abort']
        else:  # outcome is success or abort, recognized command is returned using output_keys
            _outcomes = ['success', 'abort']

        if commands_as_userdata:  # pass the recognized command to the next state
            _output_keys = ['command_recognized']
        else:  # do not pass data to the next state
            _output_keys = []

        smach.State.__init__(self, outcomes=_outcomes, output_keys=_output_keys)

    def _listen_for_commands(self, tries=5, time_out=30):
        for i in range(0, tries):
            try:
                result = self._robot.hmi.query('What command?', 'T -> ' + ' | '.join(self._possible_commands), 'T',
                                               timeout=time_out)
                command_recognized = result.sentence
            except TimeoutException:
                command_recognized = None
            if command_recognized == "":
                self._robot.speech.speak("I am still waiting for a command and did not hear anything")

            elif command_recognized in self._possible_commands:
                if command_recognized == "no":
                    self._robot.speech.speak("OK")
                else:
                    self._robot.speech.speak("OK, I will go to {}".format(command_recognized))
                return "success", command_recognized
            else:
                self._robot.speech.speak(
                    "I don't understand, I expected a command like " + ", ".join(self._possible_commands))

        self._robot.speech.speak("I did not recognize a command and will stop now")
        return "abort", "abort"

    def execute(self, userdata):
        # Stop the base
        self._robot.base.local_planner.cancelCurrentPlan()
        self._robot.head.look_at_standing_person()

        outcome, command_recognized = self._listen_for_commands()

        if self._commands_as_userdata:
            userdata.command_recognized = command_recognized
            self.target_destination.id_ = command_recognized

        if self._commands_as_outcomes:
            return command_recognized
        else:
            return outcome


class StoreCarWaypoint(smach.State):
    """
    The robot remembers the position of the car for future use

    """
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['success', 'abort'])
        self._robot = robot

    def execute(self, userdata=None):
        response = self._robot.ed.update_entity(id=challenge_knowledge.waypoint_car['id'],
                                               frame_stamped=self._robot.base.get_location(),
                                               type="waypoint")

        if not response.response:
            return "success"
        else:
            return "abort"

class TurnToReplan(smach.State):
    """
    Turn 180 degrees to attempt to find a new (reachable) plan
    """
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['success', 'abort'])
        self._robot = robot

    def execute(self, userdata=None):
        robot_frame = self._robot.base.get_location().frame
        robot_pose = robot_frame.p
        r, p, y = robot_frame.M.GetRPY()
        states.NavigateToPose(self._robot, robot_pose.position.x, robot_pose.position.y, y+math.pi)

class DropBagOnGround(smach.StateMachine):
    """
    Put the bag in the robot's gripper on the ground

    """
    def __init__(self, robot, arm_designator, drop_bag_pose):
        """
        :param robot: the robot with which to execute this state machine
        :param arm_designator: ArmDesignator resolving to Arm holding the bag to drop

        """
        smach.StateMachine.__init__(self, outcomes=['done'])

        check_type(arm_designator, Arm)

        with self:
            smach.StateMachine.add('DROP_POSE', states.ArmToJointConfig(robot, arm_designator, drop_bag_pose),
                                   transitions={'succeeded': 'OPEN_AFTER_DROP',
                                                'failed': 'OPEN_AFTER_DROP'})

            smach.StateMachine.add('OPEN_AFTER_DROP',
                                   states.SetGripper(robot, arm_designator, gripperstate=GripperState.OPEN),
                                   transitions={'succeeded': 'RESET_ARM',
                                                'failed': 'RESET_ARM'})

            smach.StateMachine.add('RESET_ARM', states.ResetArms(robot),
                                   transitions={'done': 'done'})
