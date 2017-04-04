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

challenge_knowledge = load_knowledge('challenge_help_me_carry')

print "=============================================="
print "==         CHALLENGE HELP ME CARRY          =="
print "=============================================="

class WaitForOperatorCommand(smach.State):
    def __init__(self, robot, command_options, commands_as_outcomes=False):

        if commands_as_outcomes:
            # each possible command is a separate outcome
            smach.State.__init__(self, outcomes=command_options + ['abort'])
        else:
            # outcome is success or abort, recognized command is returned using output_keys
            smach.State.__init__(self, outcomes=['success', 'abort'],  output_keys=['command_recognized'])

        self._robot = robot
        self._command_options = command_options

    def _listen_for_commands(self, tries=3, time_out = rospy.Duration(30)):
        for i in range(0, tries):
            result = self._robot.ears.recognize("<option>", {"option" : self._command_options}, time_out)
            command_recognized = result.result

            if command_recognized == "":
                self._robot.speech.speak("I am still waiting for a command and did not hear anything")

            elif command_recognized in self._command_options:
                self._robot.speech.speak(command_recognized + " OK")
                return "success", command_recognized

            else:
                self._robot.speech.speak("I don't understand, I expected a command like " + self._command_options)

        self._robot.speech.speak("I did not recognize a command and will stop now")
        return "abort", ""

    def execute(self, userdata):
        # Stop the base
        self._robot.base.local_planner.cancelCurrentPlan()
        self._robot.head.look_at_standing_person()

        outcome, userdata.command_recognized = self._listen_for_commands()

        return outcome

class StoreCarWaypoint(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['success', 'abort'])
        self._robot = robot
        robot.base.local_planner.cancelCurrentPlan()

    def execute(self, userdata):
        success = self._robot.ed.update_entity(id=challenge_knowledge.waypoints['car']['id'], posestamped=self._robot.base.get_location(), type="waypoint")

        if success:
            return "success"
        else:
            return "abort"

class NavigateToRoom(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,
                             outcomes=['success', 'abort'],
                             input_keys=['target_room'])
        self._robot = robot
        robot.base.local_planner.cancelCurrentPlan()

    def execute(self, userdata):

        target_waypoint = challenge_knowledge.waypoint[userdata.target_room]['id']
        target_radius = challenge_knowledge.waypoint[userdata.target_room]['radius']

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
                               WaitForOperatorCommand(robot, command_options=challenge_knowledge.commands['follow']),
                               transitions={'success':        'FOLLOW_OPERATOR',
                                            'abort':          'Aborted'})

        # Tim
        smach.StateMachine.add('WAIT_TO_FOLLOW_OR_REMEMBER',
                               #TODO: add that robot should memorize operator
                               WaitForOperatorCommand(robot,
                                                      command_options=challenge_knowledge.commands['follow_or_remember'],
                                                      commands_as_outcomes=True),
                               transitions={'follow':         'FOLLOW_OPERATOR',
                                            'remember':       'REMEMBER_CAR_LOCATION',
                                            'stop':           'REMEMBER_CAR_LOCATION',
                                            'car':            'REMEMBER_CAR_LOCATION',
                                            'abort':          'Aborted'})
        # Kwin
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
                               WaitForOperatorCommand(robot, command_options=challenge_knowledge.waypoints.keys()),
                               transitions={'success':        'GRAB_ITEM',
                                            'abort':          'Aborted'})

        # Kwin
        smach.StateMachine.add('GRAB_ITEM',
                               #TODO: reuse grab action to grab bag from operator
                               #states.ArmToJointConfig(robot, arm_designator, "carrying_pose"),
                               states.Say(robot, "Grabbing item"),
                               transitions={'spoken': 'GOTO_DESTINATION'})
                               #transitions={'success':        'GOTO_DESTINATION',
                               #             'abort':          'Aborted'})


        # Tim
        smach.StateMachine.add('GOTO_DESTINATION',
                               NavigateToRoom(robot),
                               transitions={'success':        'PUTDOWN_ITEM',
                                            'abort':          'Aborted'},
                               remapping={  'target_room':    'command_recognized'})

        # Kwin
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
                                                         id=challenge_knowledge.waypoints['car']['id']),
                                                         challenge_knowledge.waypoints['car']['radius']),
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
