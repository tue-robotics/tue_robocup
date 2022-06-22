#! /usr/bin/env python
import rospy
from ed.entity import Entity
from robot_smach_states.navigation.navigate_to_waypoint import NavigateToWaypoint
from robot_smach_states.human_interaction import Say
from robot_smach_states.human_interaction.human_interaction import WaitForPersonInFront, AskPersonName, LearnPerson, HearOptionsExtra, AskYesNo
from robot_smach_states.reset import ResetArms

import robot_smach_states.util.designators as ds
import smach
from smach import cb_interface, CBState
from robocup_knowledge import load_knowledge
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
        guest_drink_hmi_des = ds.VariableDesignator(resolve_type=HMIResult, name='guest_drink')
        guest_drink_name_des = ds.FieldOfHMIResult(guest_drink_des, semantics_path='drink', name='guest_drinkname')

        with self:
            smach.StateMachine.add('GOTO_DOOR',
                                   NavigateToWaypoint(robot,
                                                      door_waypoint,
                                                      challenge_knowledge.waypoint_door['radius']),
                                   transitions={'arrived': 'SAY_OPEN_DOOR',
                                                'unreachable': 'SAY_OPEN_DOOR',
                                                'goal_not_defined': 'aborted'})

            smach.StateMachine.add('SAY_OPEN_DOOR',
                                   Say(robot, ["Someone please open the door, I'm expecting guests"],
                                              block=True,
                                              look_at_standing_person=True),
                                   transitions={'spoken': 'SAY_PLEASE_COME_IN'})

            smach.StateMachine.add('SAY_PLEASE_COME_IN',
                                   Say(robot, ["Please come in, I'm waiting for someone to step in front of me"],
                                              block=True,
                                              look_at_standing_person=True),
                                   transitions={'spoken': 'WAIT_FOR_GUEST'})

            smach.StateMachine.add("WAIT_FOR_GUEST",
                                   WaitForPersonInFront(robot, attempts=30, sleep_interval=1),
                                   transitions={'success': 'SAY_HELLO',
                                                'failed': 'SAY_PLEASE_COME_IN'})

            smach.StateMachine.add('SAY_HELLO',
                                   Say(robot, ["Hi there, I'll learn your face now"],
                                              block=True,
                                              look_at_standing_person=True),
                                   transitions={'spoken': 'ASK_GUEST_NAME'})

            smach.StateMachine.add('ASK_GUEST_NAME',
                                   AskPersonName(robot, guest_name_des.writeable, challenge_knowledge.common.names),
                                   transitions={'succeeded': 'SAY_HEARD_CORRECT_NAME',
                                                'failed': 'SAY_HELLO',
                                                'timeout': 'SAY_HELLO'})

            smach.StateMachine.add('SAY_HEARD_CORRECT_NAME',
                                   Say(robot, "I heard your name is {name}, is this correct?",
                                       name=guest_name_des),
                                   transitions={'spoken': 'HEAR_NAME_CORRECT'})

            smach.StateMachine.add("HEAR_NAME_CORRECT", AskYesNo(robot),
                                   transitions={"yes": "LEARN_PERSON",
                                                "no": "ASK_GUEST_NAME",
                                                "no_result": "ASK_GUEST_NAME"})

            smach.StateMachine.add('LEARN_PERSON',
                                   LearnPerson(robot, name_designator=guest_name_des),
                                   transitions={'succeeded': 'SAY_GUEST_LEARNED',
                                                'failed': 'SAY_FAILED_LEARNING'})

            smach.StateMachine.add('SAY_FAILED_LEARNING',
                                   Say(robot, ["Not sure if I remember you, but I'll do my best"],
                                              block=False),
                                   transitions={'spoken': 'SAY_DRINK_QUESTION'})

            smach.StateMachine.add('SAY_GUEST_LEARNED',
                                   Say(robot, ["Okidoki, now I know what you look like"], block=True),
                                   transitions={'spoken': 'SAY_DRINK_QUESTION'})

            smach.StateMachine.add('SAY_DRINK_QUESTION',
                                   Say(robot, ["What's your favorite drink?"], block=True),
                                   transitions={'spoken': 'HEAR_DRINK_ANSWER'})

            smach.StateMachine.add('HEAR_DRINK_ANSWER',
                                   HearOptionsExtra(robot,
                                                    self.drink_spec_des,
                                                    guest_drink_hmi_des.writeable),
                                   transitions={'heard': 'WRITE_DRINK_NAME',
                                                'no_result': 'SAY_DRINK_QUESTION'})

            @cb_interface(outcomes=['done'])
            def _write_drink_name(answer, drink_name_designator):
                name = answer.resolve()
                drink_name_designator.write(name)
                return 'done'

            smach.StateMachine.add("WRITE_DRINK_NAME",
                                   CBState(_write_drink_name,
                                           cb_args=[guest_drink_name_des, guest_drink_des.writeable]),
                                   transitions={'done': 'SAY_DRINK_CORRECT_NAME'})

            smach.StateMachine.add('SAY_DRINK_CORRECT_NAME',
                                   Say(robot, "I heard your favorite drink is {drink}, is this correct?",
                                       drink=guest_drink_des),
                                   transitions={'spoken': 'HEAR_DRINK_CORRECT'})

            smach.StateMachine.add("HEAR_DRINK_CORRECT", AskYesNo(robot),
                                   transitions={"yes": "RESET_1",
                                                "no": "SAY_DRINK_QUESTION",
                                                "no_result": "SAY_DRINK_QUESTION"})

            smach.StateMachine.add('RESET_1',
                                   ResetArms(robot),
                                   transitions={'done': 'succeeded'})


if __name__ == "__main__":
    rospy.init_node("learn_guest")
    from robot_smach_states.util.startup import startup

    class Test(smach.StateMachine):
        def __init__(self, robot):
            smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed', 'aborted'])

            self.door_waypoint = ds.EntityByIdDesignator(robot, uuid=challenge_knowledge.waypoint_door['id'])
            self.livingroom_waypoint = ds.EntityByIdDesignator(robot, uuid=challenge_knowledge.waypoint_livingroom['id'])

            self.operator_designator = ds.VariableDesignator(resolve_type=Entity)

            self.guest1_entity_des = ds.VariableDesignator(resolve_type=Entity, name='guest1_entity')
            self.guest1_name_des = ds.VariableDesignator('guest 1', name='guest1_name')
            self.guest1_drink_des = ds.VariableDesignator(resolve_type=HMIResult, name='guest1_drink')
            self.guest1_drinkname_des = ds.FieldOfHMIResult(self.guest1_drink_des, semantics_path='drink',
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
