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
#challenge_knowledge = load_knowledge('challenge_help_me_carry')

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
        success = self._robot.ed.update_entity(id=challenge_knowledge.waypoints.car, posestamped=self._robot.base.get_location(), type="waypoint")

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

        target_waypoint, target_radius = challenge_knowledge.commands.waypoint_for_room[userdata.target_room]

        navigateToWaypoint = states.NavigateToWaypoint(self._robot, EntityByIdDesignator(self._robot, id=target_waypoint), target_radius)

        return navigateToWaypoint.execute()


def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    with sm:
        smach.StateMachine.add('INITIALIZE',
                               states.Initialize(robot),
                               transitions={'initialized':    'WAIT_TO_FOLLOW',
                                            'abort':          'aborted'})

        smach.StateMachine.add('WAIT_TO_FOLLOW',
                               WaitForOperatorCommand(robot, command_options=challenge_knowledge.commands.follow),
                               transitions={'success':        'FOLLOW_OPERATOR',
                                            'abort':          'aborted'})

        smach.StateMachine.add('WAIT_TO_FOLLOW_OR_REMEMBER',
                               WaitForOperatorCommand(robot,
                                                      command_options=challenge_knowledge.commands.follow_or_remember,
                                                      commands_as_outcomes=True),
                               transitions={'follow':         'FOLLOW_OPERATOR',
                                            'remember':       'REMEMBER_CAR_LOCATION',
                                            'abort':          'aborted'})

        smach.StateMachine.add('FOLLOW_OPERATOR',
                               states.FollowOperator(robot, operator_timeout=30, ask_follow=False, learn_face=False, replan=True),
                               transitions={'stopped':        'WAIT_TO_FOLLOW_OR_REMEMBER',
                                            'lost_operator':  'WAIT_TO_FOLLOW_OR_REMEMBER',
                                            'no_operator':    'WAIT_TO_FOLLOW_OR_REMEMBER'})

        smach.StateMachine.add('REMEMBER_CAR_LOCATION',
                               StoreCarWaypoint(robot),
                               transitions={'success':        'WAIT_TO_CARRY',
                                            'abort':          'aborted'})

        smach.StateMachine.add('WAIT_TO_GRAB',
                               WaitForOperatorCommand(robot, command_options=challenge_knowledge.commands.carry),
                               transitions={'success':        'GRAB_ITEM',
                                            'abort':          'aborted'})

        smach.StateMachine.add('GRAB_ITEM',

                               transitions={'success':        'WAIT_FOR_DESTINATION',
                                            'abort':          'aborted'})

        smach.StateMachine.add('WAIT_FOR_DESTINATION',
                               WaitForOperatorCommand(robot, command_options=challenge_knowledge.commands.waypoint_for_room.keys()),
                               transitions={'success':        'GOTO_DESTINATION',
                                            'abort':          'aborted'})

        smach.StateMachine.add('GOTO_DESTINATION',
                               NavigateToRoom(robot),
                               transitions={'success':        'PUTDOWN_ITEM',
                                            'abort':          'aborted'},
                               remapping={  'target_room':    'command_recognized'})

        smach.StateMachine.add('PUTDOWN_ITEM',
                               states.Place(robot, self.current_item, self.place_position, self.arm_with_item_designator),
                               transitions={'success':        'GOTO_CAR',
                                            'abort':          'aborted'})

        smach.StateMachine.add('GOTO_CAR',
                               states.NavigateToWaypoint(robot,
                                                         EntityByIdDesignator(robot,
                                                         id=challenge_knowledge.waypoints.car.id),
                                                         challenge_knowledge.waypoints.car.radius),
                               transitions={'success':        'WAIT_TO_CARRY',
                                            'abort':          'aborted'})

        smach.StateMachine.add('AT_END',
                                states.Say(robot, "Goodbye"),
                                transitions={   'spoken'            :   'Done'})

    analyse_designators(sm, "help_me_carry")
    return sm


############################## initializing program ##############################
if __name__ == '__main__':
    rospy.init_node('help_me_carry_exec')

    #states.util.startup(setup_statemachine, challenge_name="help_me_carry")
