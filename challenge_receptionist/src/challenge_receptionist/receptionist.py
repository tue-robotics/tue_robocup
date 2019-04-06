import rospy
import smach
import datetime
import robot_smach_states as states
import robot_smach_states.util.designators as ds

from robocup_knowledge import load_knowledge
from robot_skills.util import kdl_conversions
from robot_skills.util.entity import Entity

challenge_knowledge = load_knowledge('challenge_receptionist')


class ChallengeReceptionist(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted'])

        self.door_waypoint = ds.EntityByIdDesignator(robot, id=challenge_knowledge.waypoint_door['id'])

        self.guest1_name_des = ds.VariableDesignator('guest 1')
        self.guest1_drink_des = ds.VariableDesignator('coke')

        with self:
            smach.StateMachine.add('INITIALIZE',
                                   states.Initialize(robot),
                                   transitions={'initialized': 'SET_INITIAL_POSE',
                                                'abort': 'Aborted'})

            smach.StateMachine.add('SET_INITIAL_POSE',
                                   states.SetInitialPose(robot, challenge_knowledge.starting_point),
                                   transitions={'done': 'GOTO_DOOR',
                                                "preempted": 'Aborted',
                                                'error': 'GOTO:DOOR'})

            smach.StateMachine.add('GOTO_DOOR',
                                   states.NavigateToWaypoint(robot, self.door_waypoint,
                                                             challenge_knowledge.default_target_radius),
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
                                   states.AskPersonName(robot, self.guest1_name_des.writeable),
                                   transitions={'succeeded': 'SAY_IS_YOUR_NAME',
                                                'failed': 'SAY_HEAR_FAILED'})

            smach.StateMachine.add('LEARN_PERSON',
                                   states.LearnPerson(robot, name_designator=self.guest1_name_des),
                                   transitions={'succeeded_learning': 'SAY_OPERATOR_LEARNED',
                                                'failed_learning': 'SAY_FAILED_LEARNING',
                                                'timeout_learning': 'SAY_FAILED_LEARNING'})

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
                                   states.HearOptions(self.robot,
                                                      challenge_knowledge.common.drink_names,
                                                      self.guest1_drink_des.writeable),
                                   transitions={'heard': 'Done',
                                                'no_result': 'Done'})
            # Then:
            # - [x] Ask for favourite drink <drink1>
            # - [ ] GOTO living room
            # - [ ] Locate John (not sure how that should work, maybe just FindPersonInRoom)
            # - [ ] GOTO John
            # - [ ] Locate guest1:
            # - [ ]   rotate head until <guest1> is detected
            # - [ ] Point at guest1
            # - [ ] Say: This is <guest1> and (s)he likes to drink <drink1>


