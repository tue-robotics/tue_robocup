#!/usr/bin/python
import math
import rospy
import smach
import datetime
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_smach_states.manipulation.place_designator import EmptySpotDesignator

from robot_skills.arms import GripperState
from robocup_knowledge import load_knowledge
from challenge_hmc_functions import hmc_states
from robot_skills.util import kdl_conversions
from robot_skills.util.entity import Entity

challenge_knowledge = load_knowledge('challenge_help_me_carry')

print "=============================================="
print "==         CHALLENGE HELP ME CARRY          =="
print "=============================================="


class ChallengeHelpMeCarry(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted'])

        self.target_destination = ds.EntityByIdDesignator(robot, id=challenge_knowledge.default_place)

        self.car_waypoint = ds.EntityByIdDesignator(robot, id=challenge_knowledge.waypoint_car['id'])

        self.empty_arm_designator = ds.UnoccupiedArmDesignator(robot, {}, name="empty_arm_designator")

        # With the empty_arm_designator locked, it will ALWAYS resolve to the same arm, unless it is unlocked.
        # For this challenge, unlocking is not needed.

        self.bag_arm_designator = self.empty_arm_designator.lockable()
        self.bag_arm_designator.lock()

        self.place_position = ds.LockingDesignator(EmptySpotDesignator(robot, self.target_destination,
                                                                       arm_designator=self.bag_arm_designator,
                                                                       name="placement",
                                                                       area=challenge_knowledge.default_area),
                                                   name="place_position")
        # We don't actually grab something, so there is no need for an actual thing to grab

        self.current_item = ds.VariableDesignator(Entity("dummy", "dummy", "/{}/base_link".format(robot.robot_name),
                                                         kdl_conversions.kdl_frame_from_XYZRPY(0.6, 0, 0.5), None, {}, [],
                                                         datetime.datetime.now()), name="current_item")

        with self:
            smach.StateMachine.add('INITIALIZE',
                                   states.Initialize(robot),
                                   transitions={'initialized': 'SET_INITIAL_POSE',
                                                'abort': 'Aborted'})

            smach.StateMachine.add('SET_INITIAL_POSE',
                                   states.SetInitialPose(robot, challenge_knowledge.starting_point),
                                   transitions={'done': 'FOLLOW_OPERATOR',
                                                'preempted': 'Aborted',
                                                'error': 'FOLLOW_OPERATOR'})

            # Follow the operator until (s)he states that you have arrived at the "car".
            # smach.StateMachine.add('FOLLOW_OPERATOR',
            #                        states.FollowOperator(robot, operator_timeout=30, ask_follow=True, learn_face=True, replan=True),
            #                        transitions={'stopped': 'ASK_FOR_TASK',
            #                                     'lost_operator': 'ASK_FOR_TASK',
            #                                     'no_operator': 'FOLLOW_OPERATOR'})

            # Use NEW:
            smach.StateMachine.add('FOLLOW_OPERATOR',
                                   states.FollowOperator2(robot),
                                   transitions={'Done': 'ASK_FOR_TASK',
                                                'Failed': 'ASK_FOR_TASK',
                                                'Aborted': 'FOLLOW_OPERATOR'})

            smach.StateMachine.add('ASK_FOR_TASK',
                                   states.Say(robot, ["Are we at the car already?"],
                                              block=True,
                                              look_at_standing_person=True),
                                   transitions={'spoken': 'WAIT_FOR_TASK'})

            smach.StateMachine.add('WAIT_FOR_TASK',
                                   states.HearOptions(robot, ['yes', 'no']),
                                   transitions={'no': 'FOLLOW_OPERATOR',
                                                'yes': 'CONFIRM_CAR_LOCATION',
                                                'no_result': 'ASK_FOR_TASK'})

            smach.StateMachine.add('CONFIRM_CAR_LOCATION',
                                   states.Say(robot, [
                                       "OK, I will remember this location as the car location."],
                                              block=True,
                                              look_at_standing_person=True),
                                   transitions={'spoken': 'REMEMBER_CAR_LOCATION'})

            smach.StateMachine.add('REMEMBER_CAR_LOCATION',
                                   hmc_states.StoreCarWaypoint(robot),
                                   transitions={'success': 'ASK_FOR_DESTINATION',
                                                'abort': 'Aborted'})

            smach.StateMachine.add('ASK_FOR_DESTINATION',
                                   states.Say(robot, ["Where should I bring the groceries?"],
                                              block=True,
                                              look_at_standing_person=True),
                                   transitions={'spoken': 'RECEIVE_DESTINATION'})

            smach.StateMachine.add('RECEIVE_DESTINATION',
                                   hmc_states.WaitForOperatorCommand(robot,
                                                                     possible_commands=challenge_knowledge.destinations,
                                                                     commands_as_userdata=True,
                                                                     target=self.target_destination),
                                   transitions={'success': 'GRAB_ITEM',
                                                'abort': 'Aborted'})
            #
            # smach.StateMachine.add('CONFIRM_DESTINATION',
            #                        states.Say(robot, [
            #                            "I will deliver the groceries to the %s" % ds.EntityByIdDesignator(self.target_destination)],
            #                                   block=True,
            #                                   look_at_standing_person=True),
            #                        transitions={'spoken': 'GRAB_ITEM'})

            # Grab the item (bag) the operator hands to the robot, when they are at the "car".
            smach.StateMachine.add('GRAB_ITEM',
                                   # states.HandoverFromHuman(robot, self.bag_arm_designator, "current_item",
                                   #                          self.current_item,
                                   #                          arm_configuration=challenge_knowledge.carrying_bag_pose),

                                   # transitions={'succeeded': 'ARM_DRIVING_POSE',
                                   #              'timeout': 'BACKUP_CLOSE_GRIPPER',
                                   #              # For now in simulation timeout is considered a success.
                                   #              'failed': 'BACKUP_CLOSE_GRIPPER'})
                                   states.Say(robot, ["I can't pick up the groceries since I don't have arms. Please place them in my basket."],
                                              block=True,
                                              look_at_standing_person=True),
                                   transitions={'spoken': 'WAIT_FOR_GRAB_ITEM'})

            smach.StateMachine.add('WAIT_FOR_GRAB_ITEM',
                                   states.WaitTime(robot),
                                   transitions={'waited': 'SAY_GOING_TO_ROOM',
                                                'preempted': 'Aborted'})

            # smach.StateMachine.add('BACKUP_CLOSE_GRIPPER',
            #                        states.SetGripper(robot, self.bag_arm_designator, gripperstate=GripperState.CLOSE),
            #                        transitions={'succeeded': 'ARM_DRIVING_POSE',
            #                                     'failed': 'ARM_DRIVING_POSE'})
            #
            # smach.StateMachine.add('ARM_DRIVING_POSE',
            #                        states.ArmToJointConfig(robot, self.bag_arm_designator,
            #                                                challenge_knowledge.driving_bag_pose),
            #                        transitions={'succeeded': 'SAY_GOING_TO_ROOM',
            #                                     'failed': 'SAY_GOING_TO_ROOM'})

            smach.StateMachine.add('SAY_GOING_TO_ROOM',
                                   states.Say(robot, ["Let me bring in your groceries",
                                                      "Helping you carry stuff",
                                                      "I'm going back inside"],
                                              block=True,
                                              look_at_standing_person=True),
                                   transitions={'spoken': 'GOTO_DESTINATION'})

            smach.StateMachine.add('GOTO_DESTINATION',
                                   states.NavigateToSymbolic(robot,
                                                             {self.target_destination: "in_front_of"},
                                                             self.target_destination),
                                   transitions={'arrived': 'PUTDOWN_ITEM',
                                                'unreachable': 'TURN_180_TO_REPLAN',
                                                # implement avoid obstacle behaviour later
                                                'goal_not_defined': 'Aborted'})

            smach.StateMachine.add('TURN_180_TO_REPLAN',
                                   hmc_states.TurnToReplan(robot),
                                   transitions={'success': 'GOTO_DESTINATION_BACKUP',
                                                'abort': 'GOTO_DESTINATION_BACKUP',
                                                # implement avoid obstacle behaviour later
                                                #'goal_not_defined': 'Aborted'})
                                                })

            smach.StateMachine.add('GOTO_DESTINATION_BACKUP',
                                   states.NavigateToSymbolic(robot,
                                                             {self.target_destination: "in_front_of"},
                                                             self.target_destination),
                                   transitions={'arrived': 'PUTDOWN_ITEM',
                                                'unreachable': 'PUTDOWN_ITEM',
                                                # implement avoid obstacle behaviour later
                                                'goal_not_defined': 'Aborted'})

            # Put the item (bag) down when the robot has arrived at the "drop-off" location (house).
            smach.StateMachine.add('PUTDOWN_ITEM',
                                   # hmc_states.DropBagOnGround(robot, self.bag_arm_designator,
                                   #                            challenge_knowledge.drop_bag_pose),
                                   states.Say(robot, ["I can't put the groceries down since I have no arms. Please take them from my basket and put it down."],
                                              block=True,
                                              look_at_standing_person=True),
                                   transitions={'spoken': 'WAIT_FOR_PUTDOWN_ITEM'})

            smach.StateMachine.add('WAIT_FOR_PUTDOWN_ITEM',
                                   states.WaitTime(robot),
                                   transitions={'waited': 'ASKING_FOR_HELP',
                                                'preempted': 'Aborted'})

            smach.StateMachine.add('ASKING_FOR_HELP',
                                   # TODO: look and then face new operator
                                   states.Say(robot, "Please follow me and help me carry groceries into the house",
                                              block=True,
                                              look_at_standing_person=True),
                                   transitions={'spoken': 'GOTO_CAR'}) #'LEARN_OPERATOR'})

            # smach.StateMachine.add('LEARN_OPERATOR',
            #                        hmc_states.LearnOperator(robot),
            #                        transitions={'learned': 'GOTO_CAR',
            #                                     'failed': 'GOTO_CAR'})

            smach.StateMachine.add('GOTO_CAR',
                                   states.NavigateToWaypoint(robot, self.car_waypoint,
                                                             challenge_knowledge.waypoint_car['radius']),

                                   # TODO: detect closed door
                                   transitions={'unreachable': 'OPEN_DOOR',
                                                'arrived': 'AT_END',
                                                'goal_not_defined': 'Aborted'})

            smach.StateMachine.add('OPEN_DOOR',
                                   # TODO: implement functionality
                                   states.Say(robot, "Please open the door for me"),
                                   transitions={'spoken': 'GOTO_CAR'})

            smach.StateMachine.add('AT_END',
                                   states.Say(robot, ["We arrived at the car, goodbye",
                                                      "You have reached your destination, goodbye",
                                                      "The car is right here, see you later!"],
                                              block=True,
                                              look_at_standing_person=True),
                                   transitions={'spoken': 'Done'})

            ds.analyse_designators(self, "help_me_carry")

############################## initializing program ##############################


if __name__ == '__main__':
    rospy.init_node('help_me_carry_exec')

    states.util.startup(ChallengeHelpMeCarry, challenge_name="help_me_carry")

    # In order to start in a different initial state, start the challenge as following:
    # rosrun challenge_help_me_carry challenge_help_me_carry.py amigo --initial=<state name>
