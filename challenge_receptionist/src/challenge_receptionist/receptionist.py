import rospy
import smach
import datetime
import robot_smach_states as states
import robot_smach_states.util.designators as ds

from robocup_knowledge import load_knowledge
from robot_skills.util import kdl_conversions
from robot_skills.util.entity import Entity
from hmi_msgs.msg import QueryResult

challenge_knowledge = load_knowledge('challenge_receptionist')


class ChallengeReceptionist(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted'])

        self.door_waypoint = ds.EntityByIdDesignator(robot, id=challenge_knowledge.waypoint_door['id'])
        self.livingroom_waypoint = ds.EntityByIdDesignator(robot, id=challenge_knowledge.waypoint_lingroom['id'])

        self.operator_designator = ds.VariableDesignator(resolve_type=Entity)

        self.drink_spec_des = ds.Designator(challenge_knowledge.drink_spec, name='drink_spec')
        self.guest1_name_des = ds.VariableDesignator('guest 1', name='guest1_name')
        self.guest1_drink_des = ds.VariableDesignator(resolve_type=QueryResult, name='guest1_drink')

        with self:
            smach.StateMachine.add('INITIALIZE',
                                   states.Initialize(robot),
                                   transitions={'initialized': 'SET_INITIAL_POSE',
                                                'abort': 'Aborted'})

            smach.StateMachine.add('SET_INITIAL_POSE',
                                   states.SetInitialPose(robot, challenge_knowledge.starting_point),
                                   transitions={'done': 'GOTO_DOOR',
                                                "preempted": 'Aborted',
                                                'error': 'GOTO_DOOR'})

            smach.StateMachine.add('GOTO_DOOR',
                                   states.NavigateToWaypoint(robot,
                                                             self.door_waypoint,
                                                             challenge_knowledge.waypoint_door['radius']),
                                   transitions={'arrived': 'SAY_PLEASE_COME_IN',
                                                'unreachable': 'SAY_PLEASE_COME_IN',
                                                'goal_not_defined': 'Aborted'})

            smach.StateMachine.add('SAY_PLEASE_COME_IN',
                                   states.Say(robot, ["Please come in, I'm waiting"],
                                              block=True,
                                              look_at_standing_person=True),
                                   transitions={'spoken': 'WAIT_FOR_GUEST'})

            smach.StateMachine.add("WAIT_FOR_GUEST",
                                   states.WaitForPersonInFront(robot, attempts=30, sleep_interval=1),
                                   transitions={'success': 'SAY_HELLO',
                                                'failed': 'SAY_PLEASE_COME_IN'})

            smach.StateMachine.add('SAY_HELLO',
                                   states.Say(robot, ["Hi there, I'll learn your face now"],
                                              block=True,
                                              look_at_standing_person=True),
                                   transitions={'spoken': 'ASK_GUEST_NAME'})

            smach.StateMachine.add('ASK_GUEST_NAME',
                                   states.AskPersonName(robot, challenge_knowledge.common.names, self.guest1_name_des.writeable),
                                   transitions={'succeeded': 'LEARN_PERSON',
                                                'failed': 'SAY_HELLO'})

            smach.StateMachine.add('LEARN_PERSON',
                                   states.LearnPerson(robot, name_designator=self.guest1_name_des),
                                   transitions={'succeeded': 'SAY_GUEST_LEARNED',
                                                'failed': 'SAY_FAILED_LEARNING'})

            smach.StateMachine.add('SAY_FAILED_LEARNING',
                                   states.Say(robot, ["Oops, I'm confused, let's try again"],
                                              block=False),
                                   transitions={'spoken': 'LEARN_PERSON'})

            smach.StateMachine.add('SAY_GUEST_LEARNED',
                                   states.Say(robot, ["Okidoki, now I know what you look like"], block=True),
                                   transitions={'spoken': 'SAY_DRINK_QUESTION'})

            smach.StateMachine.add('SAY_DRINK_QUESTION',
                                   states.Say(robot, ["What's your favorite drink?"], block=True),
                                   transitions={'spoken': 'HEAR_DRINK_ANSWER'})

            smach.StateMachine.add('HEAR_DRINK_ANSWER',
                                   states.HearOptionsExtra(robot,
                                                      self.drink_spec_des,
                                                      self.guest1_drink_des.writeable),
                                   transitions={'heard': 'GOTO_LIVINGROOM',
                                                'no_result': 'SAY_DRINK_QUESTION'})

            smach.StateMachine.add('GOTO_LIVINGROOM',
                                   states.NavigateToWaypoint(robot,
                                                             self.livingroom_waypoint,
                                                             challenge_knowledge.waypoint_livingroom['radius']),
                                   transitions={'arrived': 'FIND_OPERATOR',
                                                'unreachable': 'FIND_OPERATOR',
                                                'goal_not_defined': 'Aborted'})

            smach.StateMachine.add('FIND_OPERATOR',
                                   states.FindPersonInRoom(robot,
                                                           challenge_knowledge.waypoint_livingroom['id'],
                                                           challenge_knowledge.operator_name,
                                                           self.operator_designator.writeable),
                                   transitions={'found': 'GOTO_OPERATOR',
                                                'not_found': 'GOTO_OPERATOR'})

            smach.StateMachine.add('GOTO_OPERATOR',
                                   states.NavigateToObserve(robot,
                                                            self.operator_designator),
                                   transitions={'arrived': 'SAY_LOOKING_FOR_GUEST',
                                                'unreachable': 'SAY_LOOKING_FOR_GUEST',
                                                'goal_not_defined': 'Aborted'})

            smach.StateMachine.add('SAY_LOOKING_FOR_GUEST',
                                   states.Say(robot, ["Now I should be looking at the guest and pointing at him or her"], block=True),
                                   transitions={'spoken': 'INTRODUCE_GUEST'})

            smach.StateMachine.add('INTRODUCE_GUEST',
                                   states.Say(robot, ["This is person X and he likes drink Y"], block=True),
                                   transitions={'spoken': 'Done'})  # TODO: Iterate to guest 2


            # - [x] Wait at the door, say you're waiting
            # - [x] Wait until person can come in
            # - [x] Ask their name
            # - [x] Ask their favourite drink
            # - [x] Ask for favourite drink <drink1>
            # - [x] GOTO living room
            # - [x] Locate John (not sure how that should work, maybe just FindPersonInRoom)
            # - [x] GOTO John
            # - [.] Locate guest1:
            # - [ ]   rotate head until <guest1> is detected
            # - [.] Point at guest1
            # - [.] Say: This is <guest1> and (s)he likes to drink <drink1>


