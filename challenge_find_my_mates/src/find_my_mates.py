#!/usr/bin/env python
import rospy
from robot_smach_states.util import startup
from robot_skills.util import kdl_conversions
from hmi import TimeoutException
from collections import Counter
from robocup_knowledge import load_knowledge

import robot_smach_states as states
import smach
from robot_skills.util.entity import Entity
import robot_smach_states.util.designators as ds
from challenge_find_my_mates.identify_people import IdentifyPeople

challenge_knowledge = load_knowledge('challenge_find_my_mates')

STARTING_POINT = challenge_knowledge.starting_point
ROOM_ID = challenge_knowledge.room
SEARCH_POINT = challenge_knowledge.search_point


class ReportPeople(smach.State):
    """
    Form the sentences for all three people and say that sentence
    """

    def __init__(self, robot, identified_people_designator):
        smach.State.__init__(self, outcomes=['done', 'aborted', 'failed'])
        self._robot = robot

        ds.check_type(identified_people_designator, [Entity])
        self._identified_people_designator = identified_people_designator

    def execute(self, userdata=None):
        self._room_entity = self._robot.ed.get_entity(id=ROOM_ID)
        entities = self._identified_people_designator.resolve()
        person_entities = [entity for entity in entities if entity.is_a("person")]

        furniture_entities = [entity for entity in entities if entity.is_a("furniture")]

        # Match the furniture entities to rooms
        furniture_entities_room = []
        for item in furniture_entities:
            if self._room_entity.in_volume(kdl_conversions.VectorStamped(vector=item._pose.p), "in"):
                furniture_entities_room.append(item)
        closest_entity = []

        if len(furniture_entities_room)>0:
            for person in person_entities:
                distance_closest = furniture_entities_room[0].distance_to_2d(person.position)
                temp_entity_near = furniture_entities_room[0]
                for entity in furniture_entities_room:
                    distance = entity.distance_to_2d(person.position)
                    if distance < distance_closest:
                        temp_entity_near = entity
                        distance_closest = distance
                closest_entity.append(temp_entity_near)

        self._robot.speech.speak('I found many people but none were as pretty as you.', block=True)
        people_descriptions = {}
        for i, person in enumerate(person_entities):
            sentence = "I found {} near the {}.\n".format(getattr(person, 'name'), closest_entity[i].id)

            unique_property = None
            properties = [prop for prop in dir(person) if not prop.startswith('__')]
            attributes = dict()
            # ToDo: should be updated to contain the new person attributes
            for prop in properties:
                temp_prop = [getattr(person, prop) for person in person_entities]
                c = Counter(temp_prop)
                attributes[prop] = c[getattr(person, prop)]
                if prop == 'gender_confidence':
                    if any(j <= 68 for j in temp_prop):
                        del attributes['gender']
                    del attributes['gender_confidence']
                del attributes['id']
                for attr in attributes:
                    if attributes[attr] == 1:
                        unique_property = attr

            # Edit these sentences based on available attributes
            if unique_property == 'gender':
                sentence += "and the person was a {}.".format(getattr(person, unique_property))
            elif unique_property == 'tall':
                sentence += "and the person was tall. "
            elif unique_property == 'shirt_colour':
                sentence += "and the person was wearing a mostly {} shirt.".format(getattr(person, unique_property))

            people_descriptions[getattr(person, 'name')] = sentence

        n = 0
        while n < 3:
            self._robot.speech.speak('Which mate would you like me to describe?', block=True)
            try:
                result = self._robot.hmi.query('Name', 'T -> ' + ' | '.join(challenge_knowledge.common.names), 'T')
                command_recognized = result.sentence
            except TimeoutException:
                command_recognized = None
            if command_recognized == "":
                self._robot.speech.speak("I am still waiting for a name and did not hear anything")
            elif command_recognized in challenge_knowledge.common.names:
                    self._robot.speech.speak("OK, I will describe {}".format(command_recognized), block=True)
                    self._robot.speech.speak(people_descriptions[command_recognized], block=True)
                    n += 1
            else:
                self._robot.speech.speak(
                    "I don't understand, I expected a name like " + ", ".join(challenge_knowledge.names))
                continue

        return 'done'


def setup_statemachine(robot):
    sm = smach.StateMachine(outcomes=['done', 'failed', 'aborted'])

    found_people_designator = ds.VariableDesignator(resolve_type=[Entity], name='found_people_designator')
    identified_people_designator = ds.VariableDesignator(resolve_type=[Entity], name='identified_people_designator')

    with sm:

        # smach.StateMachine.add('INITIALIZE',
        #                  Initialize(robot),
        #                  transitions={'initialized': 'INIT_POSE',
        #                               'abort': 'aborted'})

        smach.StateMachine.add('INIT_POSE', states.SetInitialPose(robot, STARTING_POINT), transitions={'done': 'GO_TO_SEARCH_POSE',
                                                                                          'preempted': 'aborted',
                                                                                          'error': 'GO_TO_SEARCH_POSE'})

        # smach.StateMachine.add('START_CHALLENGE',
        #                  StartChallengeRobust(robot, initial_pose=STARTING_POINT, door=False),
        #                  transitions={'Done': 'GO_TO_SEARCH_POSE',
        #                               'Aborted': 'aborted',
        #                               'Failed': 'WAIT_TIME'})

        smach.StateMachine.add('GO_TO_SEARCH_POSE',
                         states.NavigateToWaypoint(robot, ds.EntityByIdDesignator(robot, id=SEARCH_POINT), radius=0.4),
                         transitions={'arrived': 'LOCATE_PEOPLE',
                                      'goal_not_defined': 'LOCATE_PEOPLE',
                                      'unreachable': 'LOCATE_PEOPLE'})

        # locate three (or all four) people
        smach.StateMachine.add('LOCATE_PEOPLE',
                               states.FindPeople(robot,
                                                 found_people_designator=found_people_designator.writeable,
                                                 query_entity_designator=ds.EntityByIdDesignator(robot, id=ROOM_ID),
                                                 attempts=2),
                               transitions={'done': 'IDENTIFY_PEOPLE',
                                            'aborted': 'done',
                                            'failed': 'LOCATE_PEOPLE'})

        # drive past all thee people and fill their description
        smach.StateMachine.add('IDENTIFY_PEOPLE',
                               IdentifyPeople(robot,
                                              found_people_designator=found_people_designator,
                                              identified_people_designator=identified_people_designator.writeable),
                               transitions={'done': 'GO_BACK_TO_OPERATOR',
                                            'aborted': 'done',
                                            'failed': 'GO_BACK_TO_OPERATOR'})

        # drive back to the operator to describe the mates
        smach.StateMachine.add('GO_BACK_TO_OPERATOR',
                         states.NavigateToWaypoint(robot, ds.EntityByIdDesignator(robot, id=STARTING_POINT), radius=0.7),
                         transitions={'arrived': 'REPORT_PEOPLE',
                                      'goal_not_defined': 'REPORT_PEOPLE',
                                      'unreachable': 'GO_TO_OPERATOR_MORE_ROOM'})

        smach.StateMachine.add('GO_TO_OPERATOR_MORE_ROOM',
                         states.NavigateToWaypoint(robot, ds.EntityByIdDesignator(robot, id=STARTING_POINT), radius=1.2),
                         transitions={'arrived': 'REPORT_PEOPLE',
                                      'goal_not_defined': 'REPORT_PEOPLE',
                                      'unreachable': 'GO_BACK_TO_OPERATOR'})

        # check how to uniquely define them
        smach.StateMachine.add('REPORT_PEOPLE', ReportPeople(robot, identified_people_designator),
                               transitions={'done': 'done',
                                                                            'aborted': 'done',
                                                                            'failed': 'failed'})

    return sm


if __name__ == '__main__':
    rospy.init_node('find_my_mates_exec')
    startup(setup_statemachine, challenge_name="find_my_mates")
