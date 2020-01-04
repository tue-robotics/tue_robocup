import rospy
import robot_smach_states as states
import robot_smach_states.util.designators as ds
import smach
from robocup_knowledge import load_knowledge
from robot_skills.util.entity import Entity
from math import radians

challenge_knowledge = load_knowledge('challenge_receptionist')


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

        # For each person:
        #   0. Go to the person (old guest)
        #   1. Look at the person and point at the guest
        #   2. Say 'Hi <person name>, this is <guest name>

        with self:
            smach.StateMachine.add('SAY_INTRO',
                                   states.SayFormatted(robot,
                                                       ["Hi {name}, let me introduce you our new guest {guest_name}. I'll show you in a bit"],
                                                       name=ds.Designator(challenge_knowledge.operator_name) if assume_john else ds.Designator("folks"),
                                                       guest_name=guest_name_des,
                                                       block=False),
                                   transitions={'spoken': 'FIND_OLD_GUESTS'})

            smach.StateMachine.add('FIND_OLD_GUESTS',
                                   states.FindPeopleInRoom(robot,
                                                           room=challenge_knowledge.waypoint_livingroom['id'],
                                                           found_people_designator=all_old_guests.writeable),
                                   transitions = {'found': 'ITERATE_OLD_GUESTS',
                                                   'not_found': 'ITERATE_OLD_GUESTS'})

            smach.StateMachine.add('ITERATE_OLD_GUESTS',
                                   states.IterateDesignator(all_old_guests,
                                                            current_old_guest.writeable),
                                   transitions={'next': 'GOTO_OPERATOR',
                                                'stop_iteration': 'succeeded'})

            smach.StateMachine.add('GOTO_OPERATOR',
                                   states.NavigateToObserve(robot,
                                                            current_old_guest,
                                                            radius=1.0,
                                                            margin=1.0),  # Makes the robot go within 2m of current_old_guest
                                   transitions={'arrived': 'SAY_LOOK_AT_GUEST',
                                                'unreachable': 'SAY_LOOK_AT_GUEST',
                                                'goal_not_defined': 'SAY_LOOK_AT_GUEST'})

            smach.StateMachine.add('SAY_LOOK_AT_GUEST',
                                   states.SayFormatted(robot,
                                                       ["Hi {name}, let me show you our guest"],
                                                       name=ds.Designator(challenge_knowledge.operator_name) if assume_john else ds.AttrDesignator(current_old_guest, "person_properties.name", resolve_type=str),
                                                       block=True),
                                   transitions={'spoken': 'TURN_TO_GUEST'})

            smach.StateMachine.add('TURN_TO_GUEST',
                                   states.Turn(robot=robot,
                                               radians=radians(180)),
                                   transitions={"turned": "FIND_GUEST"})

            smach.StateMachine.add('FIND_GUEST',
                                   states.FindPerson(robot=robot,
                                                     person_label=guest_name_des,
                                                     search_timeout=30,
                                                     found_entity_designator=guest_ent_des.writeable,
                                                     speak_when_found=False),
                                   transitions={"found": "POINT_AT_GUEST",
                                                "failed": "INTRODUCE_GUEST_WITHOUT_POINTING"})

            smach.StateMachine.add('POINT_AT_GUEST',
                                   states.PointAt(robot=robot,
                                                  arm_designator=ds.UnoccupiedArmDesignator(robot,{'required_goals':['point_at']}),
                                                  point_at_designator=guest_ent_des,
                                                  look_at_designator=current_old_guest),
                                   transitions={"succeeded": "INTRODUCE_GUEST_BY_POINTING",
                                                "failed": "INTRODUCE_GUEST_WITHOUT_POINTING"})

            smach.StateMachine.add('INTRODUCE_GUEST_BY_POINTING',
                                   states.Say(robot, GuestDescriptionStrDesignator(guest_name_des, guest_drinkname_des),
                                              block=True,
                                              look_at_standing_person=True),
                                   transitions={'spoken': 'RESET_ARM'})

            smach.StateMachine.add('INTRODUCE_GUEST_WITHOUT_POINTING',
                                   states.SayFormatted(robot,
                                                       "Our new guest is {name} who likes {drink}",
                                                       name=guest_name_des, drink=guest_drinkname_des,
                                                       block=True,
                                                       look_at_standing_person=True),
                                   transitions={'spoken': 'RESET_ARM'})

            smach.StateMachine.add('RESET_ARM',
                                   states.ResetArms(robot),
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
