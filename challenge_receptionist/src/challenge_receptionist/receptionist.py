import smach

from ed.entity import Entity

from robot_smach_states.utility import SetInitialPose
from robot_smach_states.reset import ResetArmsTorsoHead
from robot_smach_states.human_interaction import Say
from robot_smach_states.navigation import NavigateToWaypoint
import robot_smach_states.util.designators as ds

from hmi import HMIResult
from robocup_knowledge import load_knowledge
from challenge_receptionist.find_empty_seat import FindEmptySeat
from challenge_receptionist.learn_guest import LearnGuest
from challenge_receptionist.introduce_guest import IntroduceGuest

challenge_knowledge = load_knowledge('challenge_receptionist')


class HandleSingleGuest(smach.StateMachine):
    def __init__(self, robot, assume_john, default_name, default_drink):
        """

        :param robot:
        :param assume_john: bool indicating that John (the homeowner) is already there.
        """
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted'])

        door_waypoint = ds.EntityByIdDesignator(robot, uuid=challenge_knowledge.waypoint_door['id'])
        livingroom_waypoint = ds.EntityByIdDesignator(robot, uuid=challenge_knowledge.waypoint_livingroom['id'])

        guest_entity_des = ds.VariableDesignator(resolve_type=Entity, name='guest_entity')
        guest_name_des = ds.VariableDesignator('guest 1', name='guest_name')
        guest_drink_des = ds.VariableDesignator(resolve_type=HMIResult, name='guest_drink')
        guest_drinkname_des = ds.FieldOfHMIResult(guest_drink_des, semantics_path='drink', name='guest_drinkname')

        with self:
            smach.StateMachine.add('LEARN_GUEST',
                                   LearnGuest(robot,
                                              door_waypoint,
                                              guest_entity_des,
                                              guest_name_des,
                                              guest_drink_des, default_name, default_drink),
                                   transitions={'succeeded': 'SAY_GOTO_OPERATOR',
                                                'aborted': 'SAY_GOTO_OPERATOR',
                                                'failed': 'SAY_GOTO_OPERATOR'})

            smach.StateMachine.add('SAY_GOTO_OPERATOR',
                                   Say(robot, ["Okidoki, you are {name} and you like {drink}, lets go inside. "
                                               "Please follow me closely until i'll point you a seat"],
                                                       name=guest_name_des, drink=guest_drinkname_des,
                                                       block=False,
                                                       look_at_standing_person=True),
                                   transitions={'spoken': 'GOTO_LIVINGROOM'})

            smach.StateMachine.add('GOTO_LIVINGROOM',
                                   NavigateToWaypoint(robot,
                                                      livingroom_waypoint,
                                                      challenge_knowledge.waypoint_livingroom['radius']),
                                   transitions={'arrived': 'INTRODUCE_GUEST',
                                                'unreachable': 'INTRODUCE_GUEST',
                                                'goal_not_defined': 'aborted'})

            smach.StateMachine.add('INTRODUCE_GUEST',
                                   IntroduceGuest(robot,
                                                  guest_entity_des,
                                                  guest_name_des,
                                                  guest_drinkname_des,
                                                  assume_john=assume_john),
                                   transitions={'succeeded': 'FIND_SEAT_FOR_GUEST',
                                                'abort': 'FIND_SEAT_FOR_GUEST'})

            smach.StateMachine.add('FIND_SEAT_FOR_GUEST',
                                   FindEmptySeat(robot,
                                                 seats_to_inspect=challenge_knowledge.seats,
                                                 room=ds.EntityByIdDesignator(robot, challenge_knowledge.sitting_room),
                                                 seat_is_for=guest_name_des),
                                   transitions={'succeeded': 'succeeded',
                                                'failed': 'aborted'})


class ChallengeReceptionist(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted'])

        runs = ds.Designator([0, 1])
        run = ds.VariableDesignator(resolve_type=int)

        with self:
            smach.StateMachine.add('RESET',
                                   ResetArmsTorsoHead(robot),
                                   transitions={'done': 'SET_INITIAL_POSE'})

            smach.StateMachine.add('SET_INITIAL_POSE',
                                   SetInitialPose(robot, challenge_knowledge.starting_point),
                                   transitions={'done': 'HANDLE_GUEST_1',
                                                "preempted": 'aborted',
                                                'error': 'HANDLE_GUEST_1'})

            smach.StateMachine.add('HANDLE_GUEST_1',
                                   HandleSingleGuest(robot, assume_john=True, default_name='Ava',
                                                     default_drink=HMIResult('coke', {'drink': 'coke'})),
                                   transitions={'succeeded': 'HANDLE_GUEST_2',
                                                'aborted': 'HANDLE_GUEST_2'})

            smach.StateMachine.add('HANDLE_GUEST_2',
                                   HandleSingleGuest(robot, assume_john=False, default_name='Laura',
                                                     default_drink=HMIResult('water', {'drink': 'water'})),
                                   transitions={'succeeded': 'SAY_DONE',
                                                'aborted': 'SAY_DONE'})

            smach.StateMachine.add('SAY_DONE',
                                   Say(robot, ["That's all folks, my job is done, bye bye!"],
                                              block=False),
                                   transitions={'spoken': 'GO_BACK'})

            smach.StateMachine.add('GO_BACK',
                                   NavigateToWaypoint(robot,
                                                             ds.EntityByIdDesignator(robot,
                                                                                     challenge_knowledge.waypoint_door['id']),
                                                             challenge_knowledge.waypoint_door['radius']),
                                   transitions={'arrived': 'succeeded',
                                                'unreachable': 'succeeded',
                                                'goal_not_defined': 'succeeded'})
