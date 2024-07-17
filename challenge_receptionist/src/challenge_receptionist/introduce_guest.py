import rospy

import numpy as np

from ed.entity import Entity
from robot_smach_states.human_interaction import Say
from robot_smach_states.human_interaction.find_people_in_room import FindPeople
from robot_smach_states.designator_iterator import IterateDesignator
from robot_smach_states.navigation.navigate_to_observe import NavigateToObserve
from robot_smach_states.navigation.navigation import ForceDrive
from robot_smach_states.human_interaction.find_person_in_room import FindPerson
from robot_smach_states.manipulation.point_at import PointAt
from robot_smach_states.reset import ResetArms

import robot_smach_states.util.designators as ds
import smach
from smach import cb_interface, CBState
from robocup_knowledge import load_knowledge

challenge_knowledge = load_knowledge('challenge_receptionist')


class SayForIntroduceGuest(smach.State):
    # TODO: test
    # TODO: make sure that has person_properties
    def __init__(self, robot_name, entity_des, guest_name_des, guest_drinkname_des, assume_john, previous_guest_name_des, previous_guest_drink_des):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot_name
        self.entity = entity_des
        self.assume_john = assume_john
        self.guest_name_des = guest_name_des
        self.guest_drinkname_des = guest_drinkname_des
        self._number_of_executions = 0
        ds.is_writeable(previous_guest_name_des)
        ds.is_writeable(previous_guest_drink_des)
        self.previous_guest_name_des = previous_guest_name_des
        self.previous_guest_drink_des = previous_guest_drink_des

    def execute(self, userdata=None):
        if self.assume_john:
            self.previous_guest_drink_des.write(ds.value_or_resolve(self.guest_drinkname_des))
            self.previous_guest_name_des.write(ds.value_or_resolve(self.guest_name_des))
            self.robot.speech.speak("This is {name} who likes {drink}".format(name=challenge_knowledge.operator_name,
                                                                              drink=challenge_knowledge.operator_drink))
        else:
            self.robot.speech.speak(
                "The other guest is {name} who likes {drink} and the operator is {name_operator} who liks {drink_operator}".format(name=ds.value_or_resolve(self.previous_guest_name_des),
                                                          drink=ds.value_or_resolve(self.previous_guest_drink_des),
                                                          name_operator = challenge_knowledge.operator_name,
                                                          drink_operator = challenge_knowledge.operator_drink))
            # if self._number_of_executions == 0:
            #     self.robot.speech.speak("This is {name} who likes {drink}".format(name=ds.value_or_resolve(self.previous_guest_name_des),
            #                                                                       drink=ds.value_or_resolve(self.previous_guest_drink_des)))
            #     self._number_of_executions += 1
            # else:
            #     self.robot.speech.speak("This is {name} who likes {drink}".format(name=challenge_knowledge.operator_name,
            #                                                                       drink=challenge_knowledge.operator_drink))

            # else:
            #     self.robot.speech.speak("Since I could not recognize the person who is already inside,"
            #                             " I can not introduce you. Sorry ")
        return "done"


class GuestDescriptionStrDesignator(ds.Designator):
    def __init__(self, guest_name_des, drinkname, name=None):
        super(GuestDescriptionStrDesignator, self).__init__(resolve_type=str, name=name)

        ds.check_type(guest_name_des, str)
        ds.check_type(drinkname, str)

        self.guest_name_des = guest_name_des
        self.drinkname = drinkname

    def _resolve(self):
        name = self.guest_name_des.resolve()
        drinkname = self.drinkname.resolve()
        return "This is {name} whose favourite drink is {drink}".format(name=name, drink=drinkname)


class IntroduceGuest(smach.StateMachine):
    def __init__(self, robot, guest_ent_des, guest_name_des, guest_drinkname_des, previous_guest_name_des, previous_guest_drink_des, assume_john=False):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'abort'])

        self.num_tries = 0

        ds.check_type(guest_name_des, str)
        ds.check_type(guest_drinkname_des, str)
        ds.check_type(previous_guest_name_des, str)
        ds.check_type(previous_guest_drink_des, str)
        ds.check_type(guest_ent_des, Entity)

        all_old_guests = ds.VariableDesignator(resolve_type=[Entity], name='all_old_guests')
        current_old_guest = ds.VariableDesignator(resolve_type=Entity, name='current_old_guest')

        room_designator = ds.EntityByIdDesignator(robot=robot, uuid=challenge_knowledge.sitting_room)


        # For each person:
        #   0. Go to the person (old guest)
        #   1. Look at the person and point at the guest
        #   2. Say 'Hi <person name>, this is <guest name>

        with self:
            # smach.StateMachine.add('SAY_INTRO',
            #                        Say(robot,
            #                                 ["Hi {name}, let me introduce you our new guest {guest_name}. I'll show you in a bit"],
            #                                 name=ds.Designator(challenge_knowledge.operator_name) if assume_john else ds.Designator("folks"),
            #                                 guest_name=guest_name_des,
            #                                 block=True),
            #                        transitions={'spoken': 'FIND_OLD_GUESTS'})

            smach.StateMachine.add("FIND_OLD_GUESTS",
                                   FindPeople(robot=robot,
                                              query_entity_designator=room_designator,
                                              found_people_designator=all_old_guests.writeable,
                                              look_distance=3.0,
                                              speak=True),
                                   transitions={"found": "CHECK_NUM_PEOPLE",
                                                "failed": "CHECK_NUM_PEOPLE"})


            @cb_interface(outcomes=["incorrect", "correct", "continue"])
            def check_num_people(ud=None):
                check_correct_num_people = all_old_guests.resolve()
                self.num_tries += 1
                if self.num_tries > 2:
                    return "continue"

                if check_correct_num_people:
                    if len(check_correct_num_people) == 1 and assume_john:
                        self.num_tries = 0
                        return "correct"

                    if len(check_correct_num_people) == 2 and not assume_john:
                        return "correct"

                return "incorrect"

            smach.StateMachine.add("CHECK_NUM_PEOPLE", CBState(check_num_people),
                                   transitions={"correct": "ITERATE_OLD_GUESTS",
                                                "incorrect": "FIND_OLD_GUESTS",
                                                "continue": "ITERATE_OLD_GUESTS"})

            smach.StateMachine.add('ITERATE_OLD_GUESTS',
                                   IterateDesignator(all_old_guests,
                                                     current_old_guest.writeable),
                                   transitions={'next': 'SAY_STAY_BEHIND',
                                                'stop_iteration': 'succeeded'})

            smach.StateMachine.add('SAY_STAY_BEHIND',
                                   Say(robot,
                                       ["{name}, please follow me and stand behind me"],
                                       name=guest_name_des,
                                       block=True),
                                   transitions={'spoken': 'GOTO_OPERATOR'})

            smach.StateMachine.add('GOTO_OPERATOR',
                                   NavigateToObserve(robot,
                                                     current_old_guest,
                                                            radius=1.0,
                                                            margin=1.0, # Makes the robot go within 2m of current_old_guest
                                                            speak=False),
                                   transitions={'arrived': 'SAY_LOOK_AT_GUEST',
                                                'unreachable': 'SAY_LOOK_AT_GUEST',
                                                'goal_not_defined': 'SAY_LOOK_AT_GUEST'})

            smach.StateMachine.add('SAY_LOOK_AT_GUEST',
                                   Say(robot,
                                       ["Hi {name}, let me show you our guest"],
                                        name=ds.Designator(challenge_knowledge.operator_name) if assume_john else ds.Designator(""),
                                        block=True),
                                   transitions={'spoken': 'INTRODUCE_GUEST_WITHOUT_POINTING'})

            # smach.StateMachine.add('TURN_TO_GUEST',
            #                        ForceDrive(robot, 0, 0, 1.05, 3.0),
            #                        transitions={"done": "FIND_GUEST"})
            #
            # smach.StateMachine.add('FIND_GUEST',
            #                        FindPerson(robot=robot,
            #                                          person_label=guest_name_des,
            #                                          search_timeout=30,
            #                                          found_entity_designator=guest_ent_des.writeable,
            #                                          speak_when_found=False),
            #                        transitions={"found": "POINT_AT_GUEST",
            #                                     "failed": "INTRODUCE_GUEST_WITHOUT_POINTING"})
            #
            # smach.StateMachine.add('POINT_AT_GUEST',
            #                        PointAt(robot=robot,
            #                                       arm_designator=ds.UnoccupiedArmDesignator(robot,{'required_goals':['point_at']}),
            #                                       point_at_designator=guest_ent_des,
            #                                       look_at_designator=current_old_guest),
            #                        transitions={"succeeded": "INTRODUCE_GUEST_BY_POINTING",
            #                                     "failed": "INTRODUCE_GUEST_WITHOUT_POINTING"})
            #
            #
            #
            # smach.StateMachine.add('INTRODUCE_GUEST_BY_POINTING',
            #                        Say(robot, GuestDescriptionStrDesignator(guest_name_des, guest_drinkname_des),
            #                            block=False,
            #                            look_at_standing_person=True),
            #                        transitions={'spoken': "POINT_AT_OLD_GUEST"})
            #
            # smach.StateMachine.add('POINT_AT_OLD_GUEST',
            #                        PointAt(robot=robot,
            #                                       arm_designator=ds.UnoccupiedArmDesignator(robot,{'required_goals':['point_at']}),
            #                                       point_at_designator=current_old_guest,
            #                                       look_at_designator=guest_ent_des),
            #                        transitions={"succeeded": "SAY_FOR_INTRODUCE_GUEST",
            #                                     "failed": "SAY_FOR_INTRODUCE_GUEST"})

            smach.StateMachine.add('INTRODUCE_GUEST_WITHOUT_POINTING',
                                   Say(robot,
                                            ["Our new guest is {name} who likes {drink}"],
                                            name=guest_name_des, drink=guest_drinkname_des,
                                            block=True,
                                            look_at_standing_person=True),
                                   transitions={'spoken': 'TURN_TO_GUEST'})

            # TODO: Test the todo above (it is implemented)
            smach.StateMachine.add('TURN_TO_GUEST',
                                    ForceDrive(robot, 0, 0, 1.05, 3.0),
                                    transitions={"done": "SAY_FOR_INTRODUCE_GUEST"})

            smach.StateMachine.add('SAY_FOR_INTRODUCE_GUEST',
                                   SayForIntroduceGuest(robot, current_old_guest, guest_name_des, guest_drinkname_des, assume_john, previous_guest_name_des.writeable,
                                                        previous_guest_drink_des.writeable),
                                   transitions={'done': 'RESET_ARM'})

            smach.StateMachine.add('RESET_ARM',
                                   ResetArms(robot),
                                   transitions={'done': 'succeeded' if assume_john else 'ITERATE_OLD_GUESTS'})

if __name__ == "__main__":
    import sys
    from robot_skills import get_robot

    if len(sys.argv) < 3:
        print("Please provide robot_name, room and seats_to_inspect as arguments. Eg. 'hero livingroom dinner_table bar dinnertable")
        sys.exit(1)

    robot_name = sys.argv[1]
    room = sys.argv[2]
    seats_to_inspect = sys.argv[3:]

    rospy.init_node('test_find_emtpy_seat')
    robot = get_robot(robot_name)

    guest_entity_des = ds.VariableDesignator(resolve_type=Entity, name='guest_entity')
    guest_name_des = ds.VariableDesignator('dummy_guest', name='guest_name')
    guest_drinkname_des = ds.VariableDesignator('dummy_drink', name='guest_drinkname')

    sm = IntroduceGuest(robot,
                        guest_entity_des,
                        guest_name_des,
                        guest_drinkname_des,
                        assume_john=True)

    sm.execute()

    rospy.loginfo("Guest is {}".format(guest_entity_des.resolve()))
