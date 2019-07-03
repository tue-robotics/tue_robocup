import rospy
import robot_smach_states as states
import robot_smach_states.util.designators as ds
import smach
from hmi import HMIResult
from robocup_knowledge import load_knowledge
from robot_skills.util.entity import Entity
from challenge_receptionist.find_empty_seat import FindEmptySeat
from challenge_receptionist.learn_guest import LearnGuest
from challenge_receptionist.introduce_guest import IntroduceGuest

challenge_knowledge = load_knowledge('challenge_receptionist')


class HandleSingleGuest(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted'])

        door_waypoint = ds.EntityByIdDesignator(robot, id=challenge_knowledge.waypoint_door['id'])
        livingroom_waypoint = ds.EntityByIdDesignator(robot, id=challenge_knowledge.waypoint_livingroom['id'])

        guest_entity_des = ds.VariableDesignator(resolve_type=Entity, name='guest_entity')
        guest_name_des = ds.VariableDesignator('guest 1', name='guest_name')
        guest_drink_des = ds.VariableDesignator(resolve_type=HMIResult, name='guest_drink')
        guest_drinkname_des = ds.FieldOfHMIResult(guest_drink_des, semantics_field='drink', name='guest_drinkname')

        with self:
            smach.StateMachine.add('LEARN_GUEST',
                                   LearnGuest(robot,
                                              door_waypoint,
                                              guest_entity_des,
                                              guest_name_des,
                                              guest_drink_des),
                                   transitions={'succeeded': 'SAY_GOTO_OPERATOR',
                                                'aborted': 'SAY_GOTO_OPERATOR',
                                                'failed': 'SAY_GOTO_OPERATOR'})

            smach.StateMachine.add('SAY_GOTO_OPERATOR',
                                   states.Say(robot, ["Okidoki, lets go inside. Please follow me"],
                                              block=True,
                                              look_at_standing_person=True),
                                   transitions={'spoken': 'GOTO_LIVINGROOM'})

            smach.StateMachine.add('GOTO_LIVINGROOM',
                                   states.NavigateToWaypoint(robot,
                                                             livingroom_waypoint,
                                                             challenge_knowledge.waypoint_livingroom['radius']),
                                   transitions={'arrived': 'INTRODUCE_GUEST',
                                                'unreachable': 'INTRODUCE_GUEST',
                                                'goal_not_defined': 'aborted'})

            smach.StateMachine.add('INTRODUCE_GUEST',
                                   IntroduceGuest(robot,
                                                            guest_entity_des,
                                                            guest_name_des,
                                                            guest_drinkname_des),
                                   transitions={'succeeded': 'FIND_SEAT_FOR_GUEST',
                                                'abort': 'FIND_SEAT_FOR_GUEST'})

            smach.StateMachine.add('FIND_SEAT_FOR_GUEST',
                                   FindEmptySeat(robot,
                                                 seats_to_inspect=challenge_knowledge.seats,
                                                 room=ds.EntityByIdDesignator(robot, challenge_knowledge.sitting_room)),
                                   transitions={'succeeded': 'succeeded',
                                                'failed': 'aborted'})


class ChallengeReceptionist(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted'])

        runs = ds.Designator([0, 1])
        run = ds.VariableDesignator(resolve_type=int)

        with self:
            smach.StateMachine.add('INITIALIZE',
                                   states.Initialize(robot),
                                   transitions={'initialized': 'SET_INITIAL_POSE',
                                                'abort': 'aborted'})

            smach.StateMachine.add('SET_INITIAL_POSE',
                                   states.SetInitialPose(robot, challenge_knowledge.starting_point),
                                   transitions={'done': 'ITERATE_NUM_GUESTS',
                                                "preempted": 'aborted',
                                                'error': 'ITERATE_NUM_GUESTS'})

            smach.StateMachine.add('ITERATE_NUM_GUESTS',
                                   states.IterateDesignator(runs, run.writeable),
                                   transitions={'next': 'HANDLE_SINGLE_GUEST',
                                                'stop_iteration': 'SAY_DONE'})

            smach.StateMachine.add('HANDLE_SINGLE_GUEST',
                                   HandleSingleGuest(robot),
                                   transitions={'succeeded': 'ITERATE_NUM_GUESTS',
                                                'aborted': 'ITERATE_NUM_GUESTS',})

            smach.StateMachine.add('SAY_DONE',
                                   states.Say(robot, ["That's all folks, my job is done, bye bye!"],
                                              block=False),
                                   transitions={'spoken': 'succeeded'})

            # - [x] Wait at the door, say you're waiting
            # - [x] Wait until person can come in
            # - [x] Ask their name
            # - [x] Ask their favourite drink
            # - [x] Ask for favourite drink <drink1>
            # - [x] GOTO living room
            # - [x] Locate John (not sure how that should work, maybe just FindPersonInRoom)
            # - [x] GOTO John
            # - [x] Locate guest1:
            # - [x]   rotate head until <guest1> is detected
            # - [x] Point at guest1
            # - [x] Say: This is <guest1> and (s)he likes to drink <drink1>
            # - [x] Iterate to guest 2
            # - [x] Point at empty chair for guest to sit in
