#!/usr/bin/env python
import rospy
import sys
import robot_smach_states as states
import smach
from robot_skills.util.entity import Entity
import robot_smach_states.util.designators as ds
from hmi import TimeoutException
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
        smach.State.__init__(self, outcomes=['done', 'aborted', 'failed'])
        self._robot = robot
        ds.check_type(found_people_designator, [Entity])
        ds.check_type(identified_people_designator, [Entity])
        ds.is_writeable(identified_people_designator)

    def execute(self, userdata=None):
        entities = self._robot.ed.get_entities()
        person_entities = [entity for entity in entities if (entity.is_a("waypoint") and entity.id.startswith("person"))]
        for person in person_entities:
            states.NavigateToWaypoint(self._robot, ds.EntityByIdDesignator(self._robot, id=person.id), radius=0.7)
            self._robot.head.look_at_standing_person()
            self._robot.speech.speak("Please tell me your name.", block=True)

            try:
                result = self._robot.hmi.query('What names?', 'T -> ' + ' | '.join(challenge_knowledge.names), 'T',
                                               timeout=self._time_out)
                command_recognized = result.sentence
            except TimeoutException:
                command_recognized = None
            if command_recognized == "":
                self._robot.speech.speak("I could not hear your name, please tell me your name.")
            elif command_recognized in challenge_knowledge.names:
                self._robot.speech.speak(
                    "OK, I understand your name is {}, but I will think of a nice nickname.".format(command_recognized))
            else:
                self._robot.speech.speak(
                    "I don't understand, I expected one of the following names: " + ", ".join(challenge_knowledge.names))
            try:
                rospy.loginfo("Detecting this person")
                # detect person -> update person entity with certain attributes
            except:
                rospy.loginfo("Detecting failed.")
                continue
                # catch exceptions
        return 'done'


if __name__ == "__main__":
    from robot_skills import get_robot

    if len(sys.argv) > 3:
        robot_name = sys.argv[1]

        rospy.init_node('test_find_emtpy_seat')
        robot = get_robot(robot_name)

        sm = IdentifyPeople(robot)
        sm.execute()
    else:
        print "Please provide robot_name, room and seats_to_inspect as arguments. Eg. 'hero livingroom dinner_table bar dinnertable",
        exit(1)

