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

print "=============================================="
print "==         CHALLENGE HELP ME CARRY          =="
print "=============================================="


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
                self._robot.speech.speak(command_recognized + " OK")
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

        navigateToWaypoint = states.NavigateToWaypoint(self._robot,
                                                       ds.EntityByIdDesignator(self._robot,
                                                                               id=target_waypoint),
                                                       target_radius)

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
                                   transitions={'succeeded': 'OPEN_AFTER_DROP',
                                                'failed': 'OPEN_AFTER_DROP'})

            smach.StateMachine.add('OPEN_AFTER_DROP', states.SetGripper(robot, arm_designator, gripperstate=GripperState.OPEN),
                                transitions={'succeeded'    :   'RESET_ARM_OK',
                                             'failed'       :   'RESET_ARM_FAIL'})

            smach.StateMachine.add( 'RESET_ARM_OK', states.ResetArms(robot),
                                transitions={'done'         :   'succeeded'})

            smach.StateMachine.add( 'RESET_ARM_FAIL', states.ResetArms(robot),
                                transitions={'done'         :   'failed'})

def setup_statemachine(robot):

    place_name = ds.EntityByIdDesignator(robot, id=challenge_knowledge.default_place, name="place_name")
    place_position = ds.LockingDesignator(ds.EmptySpotDesignator(robot,
                                                                 place_name,
                                                                 name="placement",
                                                                 area=challenge_knowledge.default_area),
                                                                 name="place_position")
    empty_arm_designator = ds.UnoccupiedArmDesignator(robot.arms,
                                                      robot.rightArm,
                                                      name="empty_arm_designator")


    # With the empty_arm_designator locked, it will ALWAYS resolve to the same arm (first resolve is cached), unless it is unlocked
    # For this challenge, unlocking is not needed
    bag_arm_designator = empty_arm_designator.lockable()
    bag_arm_designator.lock()

    # We don't actually grab something, so there is no need for an actual thing to grab
    current_item = ds.VariableDesignator(Entity("dummy",
                                                "dummy",
                                                "/{}/base_link".format(robot.robot_name),
                                                kdl_conversions.kdlFrameFromXYZRPY(0.6, 0, 0.5),
                                                None,
                                                {},
                                                [],
                                                datetime.datetime.now()),
                                         name="current_item")

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    with sm:
        smach.StateMachine.add('INITIALIZE',
                               states.Initialize(robot),
                               transitions={'initialized':    'SET_INITIAL_POSE',
                                            'abort':          'Aborted'})

        smach.StateMachine.add('SET_INITIAL_POSE',
                               states.SetInitialPose(robot, challenge_knowledge.starting_point),
                               transitions={'done': 'FOLLOW_OPERATOR',
                                            "preempted": 'Aborted',
                                            'error': 'FOLLOW_OPERATOR'})

        # TODO: learn operator state needs to be added before follow
        # smach.StateMachine.add('WAIT_TO_FOLLOW',
        #                        WaitForOperatorCommand(robot, possible_commands=['follow', 'follow me']),
        #                        transitions={'success':        'FOLLOW_OPERATOR',
        #                                     'abort':          'Aborted'})

        smach.StateMachine.add('ASK_FOLLOW_OR_REMEMBER',
                                states.Say(robot, ["Are we at the car or should I follow you?"], block=True),
                                transitions={'spoken':  'WAIT_TO_FOLLOW_OR_REMEMBER'})

        smach.StateMachine.add('WAIT_TO_FOLLOW_OR_REMEMBER',
                               WaitForOperatorCommand(robot,
                                                      possible_commands=[
                                                          "follow",
                                                          'follow me',
                                                          "here is the car",
                                                          "stop following",
                                                          "stop following me",
                                                      ],
                                                      commands_as_outcomes=True),
                               transitions={'follow':            'FOLLOW_OPERATOR',
                                            'follow me':         'FOLLOW_OPERATOR',
                                            'here is the car':   'REMEMBER_CAR_LOCATION',
                                            'stop following':    'REMEMBER_CAR_LOCATION',
                                            'stop following me': 'REMEMBER_CAR_LOCATION',
                                            'abort':             'Aborted'})
        # Follow the operator until (s)he states that you have arrived at the "car".
        smach.StateMachine.add('FOLLOW_OPERATOR',
                               states.FollowOperator(robot,
                                                     operator_timeout=30,
                                                     ask_follow=True,
                                                     learn_face=True,
                                                     replan=True),
                               transitions={'stopped':        'ASK_FOLLOW_OR_REMEMBER',
                                            'lost_operator':  'ASK_FOLLOW_OR_REMEMBER',
                                            'no_operator':    'ASK_FOLLOW_OR_REMEMBER'})

        smach.StateMachine.add('REMEMBER_CAR_LOCATION',
                               StoreCarWaypoint(robot),
                               transitions={'success':        'ASK_DESTINATION',
                                            'abort':          'Aborted'})

        smach.StateMachine.add('ASK_DESTINATION',
                               states.Say(robot, ["Where should I bring the groceries?"], block=True),
                               transitions={'spoken':  'WAIT_FOR_DESTINATION'})

        smach.StateMachine.add('WAIT_FOR_DESTINATION',
                               WaitForOperatorCommand(robot,
                                                      possible_commands=challenge_knowledge.waypoints.keys(),
                                                      commands_as_userdata=True),
                               transitions={'success':        'GRAB_ITEM',
                                            'abort':          'Aborted'})

        # Grab the item (bag) the operator hands to the robot, when they are at the "car".
        smach.StateMachine.add('GRAB_ITEM',
                               GrabItem(robot, bag_arm_designator, current_item),
                               transitions={'succeeded':        'ARM_DRIVING_POSE',
                                            'timeout':          'BACKUP_CLOSE_GRIPPER', # For now in simulation timeout is considered a succes.
                                            'failed':           'BACKUP_CLOSE_GRIPPER'},
                               remapping={'target_room_in':   'command_recognized',
                                          'target_room_out':  'target_room'})

        smach.StateMachine.add('BACKUP_CLOSE_GRIPPER',
                               states.SetGripper(robot, bag_arm_designator, gripperstate=GripperState.CLOSE),
                               transitions={'succeeded': 'ARM_DRIVING_POSE',
                                            'failed': 'ARM_DRIVING_POSE'})

        smach.StateMachine.add('ARM_DRIVING_POSE',
                               states.ArmToJointConfig(robot, bag_arm_designator, 'driving_bag_pose'),
                               transitions={'succeeded': 'SAY_GOING_TO_ROOM',
                                            'failed': 'SAY_GOING_TO_ROOM'})

        smach.StateMachine.add('SAY_GOING_TO_ROOM',
                               states.Say(robot, ["Let me bring in your groceries",
                                                  "Helping you carry stuff",
                                                  "I'm going back inside"],
                                          block=True),
                               transitions={'spoken':  'GOTO_DESTINATION'})

        smach.StateMachine.add('GOTO_DESTINATION',
                               NavigateToRoom(robot),
                               transitions={'arrived':          'PUTDOWN_ITEM',
                                            'unreachable':      'PUTDOWN_ITEM',  # implement avoid obstacle behaviour later
                                            'goal_not_defined': 'Aborted'})

        # Put the item (bag) down when the robot has arrived at the "drop-off" location (house).
        smach.StateMachine.add('PUTDOWN_ITEM',
                               DropBagOnGround(robot, bag_arm_designator),
                               transitions={'succeeded':        'ASKING_FOR_HELP',
                                            'failed':           'ASKING_FOR_HELP'})

        smach.StateMachine.add('ASKING_FOR_HELP',
                               #TODO: look and then face new operator
                               states.Say(robot, "Please follow me and help me carry groceries into the house"),
                               transitions={'spoken': 'GOTO_CAR'})
                               #transitions={'success':        'GOTO_CAR',
                               #             'abort':          'Aborted'})

        smach.StateMachine.add('GOTO_CAR',
                               states.NavigateToWaypoint(robot,
                                                         ds.EntityByIdDesignator(robot,
                                                         id=challenge_knowledge.waypoint_car['id']),
                                                         challenge_knowledge.waypoint_car['radius']),

                               # TODO: detect closed door
                               transitions={'unreachable':      'OPEN_DOOR',
                                            'arrived':          'AT_END',
                                            'goal_not_defined': 'Aborted'})

        smach.StateMachine.add('OPEN_DOOR',
                               #TODO: implement functionality
                               states.Say(robot, "Please open the door for me"),
                               transitions={'spoken': 'GOTO_CAR'})
                               #transitions={'success':        'GOTO_CAR',
                               #             'abort':          'Aborted'})

        smach.StateMachine.add('AT_END',
                               states.Say(robot, ["We arrived at the car, goodbye",
                                                  "You have reached your destination, goodbye",
                                                  "The car is right here, see you later!"],
                                          block=True),
                               transitions={'spoken':  'Done'})

    ds.analyse_designators(sm, "help_me_carry")
    return sm


############################## initializing program ##############################
if __name__ == '__main__':
    rospy.init_node('help_me_carry_exec')

    states.util.startup(setup_statemachine, challenge_name="help_me_carry")

# In order to start in a different initial state, start the challenge as following:
# rosrun challenge_help_me_carry challenge_help_me_carry.py amigo --initial=<state name>
