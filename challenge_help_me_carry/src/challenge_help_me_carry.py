#!/usr/bin/python
import roslib;
import rospy
import smach
import sys
import time

from cb_planner_msgs_srvs.msg import PositionConstraint

from robot_smach_states.util.designators import VariableDesignator, EdEntityDesignator, EntityByIdDesignator, analyse_designators
import robot_smach_states as states

from robocup_knowledge import load_knowledge
from robot_skills.util import kdl_conversions

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

    def _listen_for_commands(self, tries=3, time_out = rospy.Duration(30)):
        for i in range(0, tries):
            result = self._robot.ears.recognize("<option>", {"option" : self._possible_commands}, time_out)
            command_recognized = result.result

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
                                               frame_stamped=kdl_conversions.FrameStamped(self._robot.base.get_location(), "/map"),
                                               type="waypoint")

        if success:
            return "success"
        else:
            return "abort"

class GrabItem(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,
                             outcomes=['success', 'abort'],
                             input_keys=['target_room_in'],
                             output_keys=['target_room_out'])
        self._robot = robot
        robot.base.local_planner.cancelCurrentPlan()

    def execute(self, userdata):
        # TODO: reuse grab action to grab bag from operator
        # states.ArmToJointConfig(robot, arm_designator, "carrying_pose"),
        states.Say(self._robot, "Grabbing item")
        userdata.target_room_out = userdata.target_room_in

        return "success"

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

        navigateToWaypoint = states.NavigateToWaypoint(self._robot, EntityByIdDesignator(self._robot, id=target_waypoint), target_radius)

        return navigateToWaypoint.execute()


def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    with sm:
        # Tim
        smach.StateMachine.add('INITIALIZE',
                               states.Initialize(robot),
                               transitions={'initialized':    'WAIT_TO_FOLLOW',
                                            'abort':          'Aborted'})

        # Tim
        smach.StateMachine.add('WAIT_TO_FOLLOW',
                               WaitForOperatorCommand(robot, possible_commands=challenge_knowledge.commands['follow']),
                               transitions={'success':        'FOLLOW_OPERATOR',
                                            'abort':          'Aborted'})

        # Tim
        smach.StateMachine.add('WAIT_TO_FOLLOW_OR_REMEMBER',
                               #TODO: add that robot should memorize operator
                               WaitForOperatorCommand(robot,
                                                      possible_commands=challenge_knowledge.commands['follow_or_remember'],
                                                      commands_as_outcomes=True),
                               transitions={'follow':         'FOLLOW_OPERATOR',
                                            'remember':       'REMEMBER_CAR_LOCATION',
                                            'stop':           'REMEMBER_CAR_LOCATION',
                                            'car':            'REMEMBER_CAR_LOCATION',
                                            'abort':          'Aborted'})
        # Kwin
        # Follow the operator until (s)he states that you have arrived at the "car".
        smach.StateMachine.add('FOLLOW_OPERATOR',
                               #states.FollowOperator(robot, operator_timeout=30, ask_follow=False, learn_face=False, replan=True),
                               states.Say(robot, "Following operator"),
                               transitions={'spoken': 'WAIT_TO_FOLLOW_OR_REMEMBER'})
                               #transitions={'stopped':        'WAIT_TO_FOLLOW_OR_REMEMBER',
                               #             'lost_operator':  'WAIT_TO_FOLLOW_OR_REMEMBER',
                               #             'no_operator':    'WAIT_TO_FOLLOW_OR_REMEMBER'})


        # Tim
        smach.StateMachine.add('REMEMBER_CAR_LOCATION',
                               StoreCarWaypoint(robot),
                               transitions={'success':        'WAIT_FOR_DESTINATION',
                                            'abort':          'Aborted'})


        # Tim
        smach.StateMachine.add('WAIT_FOR_DESTINATION',
                               WaitForOperatorCommand(robot, possible_commands=challenge_knowledge.waypoints.keys(), commands_as_userdata=True),
                               transitions={'success':        'GRAB_ITEM',
                                            'abort':          'Aborted'})

        # Kwin
        # Grab the item (bag) the operator hands to the robot, when they are at the "car".
        smach.StateMachine.add('GRAB_ITEM',
                               GrabItem(robot),
                               transitions={'success':        'GOTO_DESTINATION',
                                            'abort':          'Aborted'},
                               remapping = {'target_room_in': 'command_recognized',
                                            'target_room_out': 'target_room'})


        # Tim
        smach.StateMachine.add('GOTO_DESTINATION',
                               NavigateToRoom(robot),
                               transitions={'arrived':          'PUTDOWN_ITEM',
                                            'unreachable':      'PUTDOWN_ITEM',  #implement avoid obstacle behaviour later
                                            'goal_not_defined': 'Aborted'})

        # Kwin
        # Put the item (bag) down when the robot has arrived at the "drop-off" location (house).
        smach.StateMachine.add('PUTDOWN_ITEM',
                               #states.Place(robot, self.current_item, self.place_position, self.arm_with_item_designator),
                               states.Say(robot, "Putting down item"),
                               transitions={'spoken': 'ASKING_FOR_HELP'})
                               #transitions={'success':        'ASKING_FOR_HELP',
                               #             'abort':          'Aborted'})

        smach.StateMachine.add('ASKING_FOR_HELP',
                               #TODO: look and then face new operator
                               #TODO: add that robot should memorize new operator
                               states.Say(robot, "Please help me carry groceries into the house"),
                               transitions={'spoken': 'GOTO_CAR'})
                               #transitions={'success':        'GOTO_CAR',
                               #             'abort':          'Aborted'})

        smach.StateMachine.add('GOTO_CAR',
                               states.NavigateToWaypoint(robot,
                                                         EntityByIdDesignator(robot,
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
                                states.Say(robot, "Goodbye"),
                                transitions={'spoken':  'Done'})

    analyse_designators(sm, "help_me_carry")
    return sm


############################## initializing program ##############################
if __name__ == '__main__':
    rospy.init_node('help_me_carry_exec')

    states.util.startup(setup_statemachine, challenge_name="help_me_carry")

# In order to start in a different initial state, start the challenge as following:
# rosrun challenge_help_me_carry challenge_help_me_carry.py amigo --initial=<state name>