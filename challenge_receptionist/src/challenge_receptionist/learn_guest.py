#! /usr/bin/env python
import rospy
import robot_smach_states as states
import robot_smach_states.util.designators as ds
import smach
from robocup_knowledge import load_knowledge
from robot_skills.util.entity import Entity
from hmi import HMIResult

challenge_knowledge = load_knowledge('challenge_receptionist')


class LearnGuest(smach.StateMachine):
    def __init__(self, robot,
                 door_waypoint, guest_ent_des,
                 guest_name_des, guest_drink_des):
        """
        Learn what a guest looks like and what his/her favourite drink is

        :param robot: Robot that should execute this state
        :param door_waypoint: Entity-designator resolving to a waypoint Where are guests expected to come in
        :param guest_ent_des: Entity of the guest
        :param guest_name_des: designator that the name (str) of the guest is written to
        :param guest_drink_des: designator that the drink type (str) of the drink the guest wants
        """
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed', 'aborted'])

        self.drink_spec_des = ds.Designator(challenge_knowledge.common.drink_spec, name='drink_spec')

        with self:
            smach.StateMachine.add('GOTO_DOOR',
                                   states.NavigateToWaypoint(robot,
                                                             door_waypoint,
                                                             challenge_knowledge.waypoint_door['radius']),
                                   transitions={'arrived': 'SAY_OPEN_DOOR',
                                                'unreachable': 'SAY_OPEN_DOOR',
                                                'goal_not_defined': 'aborted'})

            smach.StateMachine.add('SAY_OPEN_DOOR',
                                   states.Say(robot, ["Someone please open the door, I'm expecting guests"],
                                              block=True,
                                              look_at_standing_person=True),
                                   transitions={'spoken': 'SAY_PLEASE_COME_IN'})

            smach.StateMachine.add('SAY_PLEASE_COME_IN',
                                   states.Say(robot, ["Please come in, I'm waiting for someone to step in front of me"],
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
                                   states.AskPersonName(robot, guest_name_des.writeable, challenge_knowledge.common.names),
                                   transitions={'succeeded': 'LEARN_PERSON',
                                                'failed': 'SAY_HELLO',
                                                'timeout': 'SAY_HELLO'})

            smach.StateMachine.add('LEARN_PERSON',
                                   states.LearnPerson(robot, name_designator=guest_name_des),
                                   transitions={'succeeded': 'SAY_GUEST_LEARNED',
                                                'failed': 'SAY_FAILED_LEARNING'})

            smach.StateMachine.add('SAY_FAILED_LEARNING',
                                   states.Say(robot, ["Not sure if I remember you, but I'll do my best"],
                                              block=False),
                                   transitions={'spoken': 'SAY_DRINK_QUESTION'})

            smach.StateMachine.add('SAY_GUEST_LEARNED',
                                   states.Say(robot, ["Okidoki, now I know what you look like"], block=True),
                                   transitions={'spoken': 'SAY_DRINK_QUESTION'})

            smach.StateMachine.add('SAY_DRINK_QUESTION',
                                   states.Say(robot, ["What's your favorite drink?"], block=True),
                                   transitions={'spoken': 'HEAR_DRINK_ANSWER'})

            smach.StateMachine.add('HEAR_DRINK_ANSWER',
                                   states.HearOptionsExtra(robot,
                                                           self.drink_spec_des,
                                                           guest_drink_des.writeable),
                                   transitions={'heard': 'RESET_1',
                                                'no_result': 'SAY_DRINK_QUESTION'})

            smach.StateMachine.add('RESET_1',
                                   states.ResetArms(robot),
                                   transitions={'done': 'succeeded'})


if __name__ == "__main__":
    rospy.init_node("learn_guest")
    from robot_smach_states.util.startup import startup

    class Test(smach.StateMachine):
        def __init__(self, robot):
            smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed', 'aborted'])

            self.door_waypoint = ds.EntityByIdDesignator(robot, id=challenge_knowledge.waypoint_door['id'])
            self.livingroom_waypoint = ds.EntityByIdDesignator(robot, id=challenge_knowledge.waypoint_livingroom['id'])

            self.operator_designator = ds.VariableDesignator(resolve_type=Entity)

            self.guest1_entity_des = ds.VariableDesignator(resolve_type=Entity, name='guest1_entity')
            self.guest1_name_des = ds.VariableDesignator('guest 1', name='guest1_name')
            self.guest1_drink_des = ds.VariableDesignator(resolve_type=HMIResult, name='guest1_drink')
            self.guest1_drinkname_des = ds.FieldOfHMIResult(self.guest1_drink_des, semantics_field='drink',
                                                         name='guest1_drinkname')

            with self:
                smach.StateMachine.add('LEARN_GUEST',
                                       LearnGuest(robot, self.door_waypoint, self.guest1_entity_des,
                                                  self.guest1_name_des,
                                                  self.guest1_drink_des),
                                       transitions={'succeeded': 'succeeded',
                                                    'failed': 'failed',
                                                    'aborted': 'aborted'})


    startup(Test)
