from __future__ import print_function


import rospy
from ed.entity import Entity
from hmi import HMIResult
from robot_smach_states.human_interaction import Say
from robot_smach_states.human_interaction.find_people_in_room import FindPeopleInRoom
from robot_smach_states.designator_iterator import IterateDesignator
from robot_smach_states.navigation.navigate_to_observe import NavigateToObserve
from robot_smach_states.navigation.navigation import ForceDrive
from robot_smach_states.human_interaction.find_person_in_room import FindPerson
from robot_smach_states.manipulation.point_at import PointAt
from robot_smach_states.reset import ResetArms

import robot_smach_states.util.designators as ds
import smach
from robocup_knowledge import load_knowledge

challenge_knowledge = load_knowledge('challenge_receptionist')


class SayForIntroduceGuest(smach.State):
    #todo test
    # todo add fourth person thingy
    def __init__(self, robot_name, entity_des, guest_drinkname_des, assume_john, previous_guest_drink_des):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot_name
        self.entity = entity_des.resolve()
        self.assume_john = assume_john
        self.guest_drinkname_des = guest_drinkname_des
        self.previous_guest_drink_des = previous_guest_drink_des

    def execute(self, userdata=None):
        if self.assume_john:
            self.previous_guest_drink_des.write(self.guest_drinkname_des.resolve())
            self.robot.speech.speak("This is {name} who likes {drink}".format(name=challenge_knowledge.operator_name,
                                                                              drink=challenge_knowledge.operator_drink))
        else:
            if hasattr(self.entity, 'person_properties'):
                name = self.entity.person_properties.name
                if name == "John":
                    self.robot.speech.speak("This is {name} who likes {drink}".format(name=name,
                                                                                      drink=challenge_knowledge.operator_drink))
                else:
                    if self.entity.person_properties.gender == 1.0:
                        gender = 'female'
                    else:
                        gender = 'male'
                    age = self.entity.person_properties.age
                    shirt_color = self.entity.person_properties.shirt_colors
                    shirt_color = shirt_color[0]
                    drink = self.previous_guest_drink_des.resolve()
                    pose = self.entity.person_properties.tags
                    pose = pose[0][1:]
                    self.robot.speech.speak("This is {name}. Who is {gender}, likes {drink}, is {age} years old, is {pose} and"
                                            " wears a {shirt_color} shirt.".format(name=name, gender=gender, drink=drink,
                                                                                   age=age, pose=pose, shirt_color=shirt_color))
            else:
                self.robot.speech.speak("Since I could not recognize the person who is already inside,"
                                        " I can not introduce you. Sorry ")
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
    def __init__(self, robot, guest_ent_des, guest_name_des, guest_drinkname_des, assume_john=False):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'abort'])

        ds.check_type(guest_name_des, str)
        ds.check_type(guest_drinkname_des, str)
        ds.check_type(guest_ent_des, Entity)

        all_old_guests = ds.VariableDesignator(resolve_type=[Entity], name='all_old_guests')
        current_old_guest = ds.VariableDesignator(resolve_type=Entity, name='current_old_guest')
        previous_guest_drink_des = ds.VariableDesignator(resolve_type=str, name='previous_guest_drink')

        # For each person:
        #   0. Go to the person (old guest)
        #   1. Look at the person and point at the guest
        #   2. Say 'Hi <person name>, this is <guest name>

        with self:
            smach.StateMachine.add('SAY_INTRO',
                                   Say(robot,
                                            ["Hi {name}, let me introduce you our new guest {guest_name}. I'll show you in a bit"],
                                            name=ds.Designator(challenge_knowledge.operator_name) if assume_john else ds.Designator("folks"),
                                            guest_name=guest_name_des,
                                            block=False),
                                   transitions={'spoken': 'FIND_OLD_GUESTS'})

            smach.StateMachine.add('FIND_OLD_GUESTS',
                                   FindPeopleInRoom(robot,
                                                    room=challenge_knowledge.waypoint_livingroom['id'],
                                                    found_people_designator=all_old_guests.writeable),
                                   transitions = {'found': 'ITERATE_OLD_GUESTS',
                                                   'not_found': 'ITERATE_OLD_GUESTS'})

            smach.StateMachine.add('ITERATE_OLD_GUESTS',
                                   IterateDesignator(all_old_guests,
                                                     current_old_guest.writeable),
                                   transitions={'next': 'GOTO_OPERATOR',
                                                'stop_iteration': 'succeeded'})

            smach.StateMachine.add('GOTO_OPERATOR',
                                   NavigateToObserve(robot,
                                                     current_old_guest,
                                                            radius=1.0,
                                                            margin=1.0),  # Makes the robot go within 2m of current_old_guest
                                   transitions={'arrived': 'SAY_LOOK_AT_GUEST',
                                                'unreachable': 'SAY_LOOK_AT_GUEST',
                                                'goal_not_defined': 'SAY_LOOK_AT_GUEST'})

            smach.StateMachine.add('SAY_LOOK_AT_GUEST',
                                   Say(robot,
                                       ["Hi {name}, let me show you our guest"],
                                        name=ds.Designator(challenge_knowledge.operator_name) if assume_john else ds.AttrDesignator(current_old_guest, "person_properties.name", resolve_type=str),
                                        block=True),
                                   transitions={'spoken': 'TURN_TO_GUEST'})

            smach.StateMachine.add('TURN_TO_GUEST',
                                   ForceDrive(robot, 0, 0, 1.05, 3.0),
                                   transitions={"done": "FIND_GUEST"})

            smach.StateMachine.add('FIND_GUEST',
                                   FindPerson(robot=robot,
                                                     person_label=guest_name_des,
                                                     search_timeout=30,
                                                     found_entity_designator=guest_ent_des.writeable,
                                                     speak_when_found=False),
                                   transitions={"found": "POINT_AT_GUEST",
                                                "failed": "INTRODUCE_GUEST_WITHOUT_POINTING"})

            smach.StateMachine.add('POINT_AT_GUEST',
                                   PointAt(robot=robot,
                                                  arm_designator=ds.UnoccupiedArmDesignator(robot,{'required_goals':['point_at']}),
                                                  point_at_designator=guest_ent_des,
                                                  look_at_designator=current_old_guest),
                                   transitions={"succeeded": "INTRODUCE_GUEST_BY_POINTING",
                                                "failed": "INTRODUCE_GUEST_WITHOUT_POINTING"})

            # TODO: check if this works lookat john/current_old_guest

            smach.StateMachine.add('INTRODUCE_GUEST_BY_POINTING',
                                   Say(robot, GuestDescriptionStrDesignator(guest_name_des, guest_drinkname_des),
                                       block=True,
                                       look_at_standing_person=True),        # todo: look if this should be false
                                   transitions={'spoken': "POINT_AT_OLD_GUEST"})

            smach.StateMachine.add('POINT_AT_OLD_GUEST',
                                   PointAt(robot=robot,
                                                  arm_designator=ds.UnoccupiedArmDesignator(robot,{'required_goals':['point_at']}),
                                                  point_at_designator=current_old_guest,
                                                  look_at_designator=guest_ent_des),
                                   transitions={"succeeded": "SAY_FOR_INTRODUCE_GUEST",
                                                "failed": "SAY_FOR_INTRODUCE_GUEST"})

            smach.StateMachine.add('INTRODUCE_GUEST_WITHOUT_POINTING',
                                   Say(robot,
                                            ["Our new guest is {name} who likes {drink}"],
                                            name=guest_name_des, drink=guest_drinkname_des,
                                            block=True,
                                            look_at_standing_person=True),
                                   transitions={'spoken': 'SAY_FOR_INTRODUCE_GUEST'})

            # TODO: still need to add 1 characteristics (pose) of guest 1 to guest 2
            # TODO: Test this

            smach.StateMachine.add('SAY_FOR_INTRODUCE_GUEST',
                                   SayForIntroduceGuest(robot, current_old_guest, guest_drinkname_des, assume_john,
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
                        guest_drinkname_des)

    sm.execute()

    rospy.loginfo("Guest is {}".format(guest_entity_des.resolve()))
