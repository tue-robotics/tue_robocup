#!/usr/bin/env python
import rospy
import sys
import robot_smach_states as states
import smach
from robot_skills.util.entity import Entity
import robot_smach_states.util.designators as ds
from robocup_knowledge import load_knowledge

challenge_knowledge = load_knowledge('challenge_find_my_mates')

STARTING_POINT = challenge_knowledge.starting_point
ROOM_ID = challenge_knowledge.room
SEARCH_POINT = challenge_knowledge.search_point


class IdentifyPeople(smach.State):
    """
    Navigate to all found people, determine their attributes and name
    The name is assigned to the Entity.person_properties.name
    """

    def __init__(self, robot, found_people_designator, identified_people_designator):
        """
        :param robot:
        :param found_people_designator: People to identify
        :type found_people_designator: ds.Designator
        :param identified_people_designator:
        :type identified_people_designator: ds.VariableWriter
        """
        smach.State.__init__(self, outcomes=['done', 'aborted', 'failed'])
        self._robot = robot
        ds.check_type(found_people_designator, [Entity])
        ds.check_type(identified_people_designator, [Entity])
        ds.is_writeable(identified_people_designator)

        current_guest = ds.VariableDesignator(resolve_type=Entity, name='current_guest')
        current_guest_name = ds.VariableDesignator(resolve_type=str, name='current_guest_name')

        with self:
            smach.StateMachine.add('ITERATE_PEOPLE',
                                   states.IterateDesignator(found_people_designator,
                                                            current_guest.writeable),
                                   transitions={'next': 'GOTO_GUEST',
                                                'stop_iteration': 'done'})

            smach.StateMachine.add('GOTO_GUEST',
                                   states.NavigateToObserve(robot,
                                                            current_guest,
                                                            radius=0.5,
                                                            margin=0.5),  # Makes the robot go within 1m of guest
                                   transitions={'arrived': 'SAY_LOOK_AT_GUEST',
                                                'unreachable': 'SAY_LOOK_AT_GUEST',
                                                'goal_not_defined': 'SAY_LOOK_AT_GUEST'})

            smach.StateMachine.add('ASK_GUEST_NAME',
                                   states.AskPersonName(robot, current_guest_name.writeable, challenge_knowledge.common.names),
                                   transitions={'succeeded': 'STORE_GUEST_NAME',
                                                'failed': 'ITERATE_PEOPLE',
                                                'timeout': 'ITERATE_PEOPLE'})

            @smach.cb_interface(outcomes=['succeeded', 'failed'])
            def store_person_name(userdata):
                guest = current_guest.resolve()  # type: Entity
                guest_name = current_guest_name.resolve()  # type; str
                if guest is not None and guest_name is not None:
                    if guest.person_properties:
                        guest.person_properties.name = guest_name

                        identified_people = identified_people_designator.resolve()
                        if not identified_people:
                            identified_people = []
                        identified_people += [guest]
                        identified_people_designator.write(identified_people)
                    else:
                        rospy.logwarn("Guest doesn't have person_properties so will remain name-less")
                        return 'failed'
                else:
                    rospy.logwarn("Did not get both guest and guest_name, so nothing to assign")
                    return 'failed'
            smach.StateMachine.add('STORE_GUEST_NAME',
                                   smach.CBState(store_person_name),
                                   transitions={'succeeded': 'ITERATE_PEOPLE',
                                                'failed': 'ITERATE_PEOPLE'})

    # def execute(self, userdata=None):
    #     entities = self._robot.ed.get_entities()
    #     person_entities = [entity for entity in entities if (entity.is_a("waypoint") and entity.id.startswith("person"))]
    #     for person in person_entities:
    #         states.NavigateToWaypoint(self._robot, ds.EntityByIdDesignator(self._robot, id=person.id), radius=0.7)
    #         self._robot.head.look_at_standing_person()
    #         self._robot.speech.speak("Please tell me your name.", block=True)
    #
    #         try:
    #             result = self._robot.hmi.query('What names?', 'T -> ' + ' | '.join(challenge_knowledge.names), 'T',
    #                                            timeout=self._time_out)
    #             command_recognized = result.sentence
    #         except TimeoutException:
    #             command_recognized = None
    #         if command_recognized == "":
    #             self._robot.speech.speak("I could not hear your name, please tell me your name.")
    #         elif command_recognized in challenge_knowledge.names:
    #             self._robot.speech.speak(
    #                 "OK, I understand your name is {}, but I will think of a nice nickname.".format(command_recognized))
    #         else:
    #             self._robot.speech.speak(
    #                 "I don't understand, I expected one of the following names: " + ", ".join(challenge_knowledge.names))
    #         try:
    #             rospy.loginfo("Detecting this person")
    #             # detect person -> update person entity with certain attributes
    #         except:
    #             rospy.loginfo("Detecting failed.")
    #             continue
    #             # catch exceptions
    #     return 'done'


if __name__ == "__main__":
    from robot_skills import get_robot

    if len(sys.argv) > 3:
        robot_name = sys.argv[1]

        rospy.init_node('test_find_emtpy_seat')
        robot = get_robot(robot_name)

        image_data = robot.perception.get_rgb_depth_caminfo()
        success, found_people_ids = robot.ed.detect_people(*image_data)

        rospy.loginfo("ED reports these people found: {}".format(found_people_ids))
        entities = [robot.ed.get_entity(i) for i in found_people_ids]

        found_people = ds.Designator([e for e in entities if e])
        identified_ppl = ds.VariableDesignator(resolve_type=[Entity])

        sm = IdentifyPeople(robot, found_people, identified_ppl.writeable)
        sm.execute(None)

        for person in identified_ppl.resolve():
            rospy.loginfo("{} = {}".format(person.id, person.person_prooperties.name))
    else:
        print "Please provide robot_name, room and seats_to_inspect as arguments. Eg. 'hero livingroom dinner_table bar dinnertable",
        exit(1)

