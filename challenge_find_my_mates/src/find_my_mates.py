#!/usr/bin/env python
import rospy
import math
import PyKDL as kdl
from robot_smach_states.util import startup
from smach import StateMachine, State
from robot_smach_states import NavigateToWaypoint, SetInitialPose, Initialize, WaitTime, StartChallengeRobust, Say
from robot_smach_states.util.designators import EntityByIdDesignator
from robot_skills.util import kdl_conversions
from hmi import TimeoutException
from collections import Counter
from robocup_knowledge import load_knowledge

challenge_knowledge = load_knowledge('challenge_find_my_mates')

STARTING_POINT = challenge_knowledge.starting_point
ROOM_ID = challenge_knowledge.room
SEARCH_POINT = challenge_knowledge.search_point


class FindPeople(State):
    """
    Locate three people in the provided room
    """

    def __init__(self, robot, search_timeout=60, look_distance=2.0, min_dist=0.5):
        """ Initialization method
        :param robot: robot api object
        :param search_timeout: (float) maximum time the robot is allowed to search
        :param look_distance: (float) robot only considers laser entities within this radius
        """
        State.__init__(self, outcomes=['Done', 'Aborted', 'Failed'])

        self._robot = robot
        self._search_timeout = search_timeout
        self._look_distance = look_distance
        self._min_dist = min_dist
        self._people = []

    def execute(self, userdata=None):
        rospy.loginfo("Trying to find my mates")
        self._robot.head.look_at_standing_person()
        self._robot.speech.speak("Please look at me while I am looking for you",
                                 block=False)

        start_time = rospy.Time.now()

        look_distance = 2.0  # magic number 4
        look_angles = [f * math.pi / d if d != 0 else 0.0 for f in [-1, 1] for d in [0, 6, 4, 2.3]]  # Magic numbers
        head_goals = [kdl_conversions.VectorStamped(x=look_distance * math.cos(angle),
                                                    y=look_distance * math.sin(angle), z=1.7,
                                                    frame_id="/%s/base_link" % self._robot.robot_name)
                      for angle in look_angles]
        head_goals.extend([kdl_conversions.VectorStamped(x=2.0 * look_distance * math.cos(angle),
                                                         y=2.0 * look_distance * math.sin(angle), z=1.7,
                                                         frame_id="/%s/base_link" % self._robot.robot_name)
                           for angle in look_angles])
        n = 0
        i = 0

        while (rospy.Time.now() - start_time).to_sec() < self._search_timeout:
            if self.preempt_requested():
                return 'Aborted'

            self._robot.head.look_at_point(head_goals[i])
            i += 1
            if i == len(head_goals):
                i = 0
            self._robot.head.wait_for_motion_done()
            raw_detections = self._robot.perception.detect_faces()

            # if not raw_detections:
            #     continue
            if raw_detections:
                for detection in raw_detections:
                    roi = detection.roi
                    try:
                        person_pos_kdl = self._robot.perception.project_roi(roi=roi, frame_id="map")
                    except Exception as e:
                        rospy.logerr("head.project_roi failed: %s", e)
                        continue  # move onto next detection

                    detected_person = self._robot.ed.get_closest_laser_entity(radius=self._look_distance,
                                                                              center_point=person_pos_kdl)

                    room_entity = self._robot.ed.get_entity(id=ROOM_ID)
                    if not room_entity.in_volume(detected_person.pose.extractVectorStamped(), 'in'):
                        detected_person = None

                    for person in self._people:
                        if person.pose.extractVectorStamped() - detected_person.pose.extractVectorStamped() > self._min_dist\
                            and EntityByIdDesignator(self._robot, id=STARTING_POINT).pose.extractVectorStamped()\
                                - detected_person.pose.extractVectorStamped() > self._min_dist:
                            self._people[n] = detected_person

                    if len(self._people) == (n+1):
                        rospy.loginfo(
                            "I found someone at {}".format(self._people[n].pose.extractVectorStamped(),
                                                           block=False))

                        self._robot.ed.update_entity(
                            id="person" + str(n),
                            frame_stamped=kdl_conversions.FrameStamped(kdl.Frame(person_pos_kdl.vector), "map"),
                            type="waypoint")

                        n += 1
                    if len(self._people) > 3:
                        self._robot.head.close()
                        return 'Done'

        self._robot.head.close()
        # rospy.sleep(2.0)
        return 'Done'


class IdentifyPeople(State):
    """
    Navigate to all three people and determine their attributes
    """

    def __init__(self, robot):
        State.__init__(self, outcomes=['Done', 'Aborted', 'Failed'])
        self._robot = robot

    def execute(self, userdata=None):
        entities = self._robot.ed.get_entities()
        person_entities = [entity for entity in entities if entity.is_a("person")]
        for person in person_entities:
            NavigateToWaypoint(self._robot, EntityByIdDesignator(self._robot, id=person.id), radius=0.7)
            self._robot.head.look_at_standing_person()
            try:
                print("Detecting this person")
                # detect person -> update person entity with certain attributes
            except:
                print("Detecting failed.")
                continue
                # catch exceptions
        return 'Done'


class ReportPeople(State):
    """
    Form the sentences for all three people and say that sentence
    """

    def __init__(self, robot):
        State.__init__(self, outcomes=['Done', 'Aborted', 'Failed'])
        self._robot = robot
        self._room_entity = self._robot.ed.get_entity(id=ROOM_ID)

    def execute(self, userdata=None):
        entities = self._robot.ed.get_entities()
        person_entities = [entity for entity in entities if entity.is_a("person")]

        furniture_entities = [entity for entity in entities if entity.is_a("furniture")]

        # Match the furniture entities to rooms
        furniture_entities_room = []
        for item in furniture_entities:
            if self._room_entity.in_volume(kdl_conversions.VectorStamped(vector=item._pose.p), "in"):
                furniture_entities_room.append(item)
        closest_entity = []
        for person in person_entities:
            distance_closest = 10.0
            temp_entity_near = furniture_entities_room[0]
            for entity in furniture_entities_room:
                distance = entity.distance_to_2d(person.position)
                if distance < distance_closest:
                    temp_entity_near = entity
                    distance_closest = distance
            closest_entity.append(temp_entity_near)

        sentence = "I found many people but none were as pretty as you."
        for i, person in enumerate(person_entities):
            sentence += "I found someone near the {}.\n".format(closest_entity[i].id)

            unique_property = None
            properties = [prop for prop in dir(person) if not prop.startswith('__')]
            attributes = dict()
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

            if unique_property == 'gender':
                sentence += "and the person was a {}.".format(getattr(person, unique_property))
            elif unique_property == 'tall':
                sentence += "and the person was tall. "
            elif unique_property == 'shirt_colour':
                sentence += "and the person was wearing a mostly {} shirt.".format(getattr(person, unique_property))

            if i > 2:
                break

        self._robot.speech.speak(sentence, block=True)
        return 'Done'


class AskNames(State):
    """
    Ask for which three people to describe
    """

    def __init__(self, robot):
        State.__init__(self, outcomes=['Done', 'Aborted', 'Failed'])
        self._robot = robot
        self._tries = 5
        self._time_out = 30

    def execute(self, userdata=None):
        self._robot.head.look_at_standing_person()
        self._robot.speech.speak("Which mates would you like me to find?", block=True)

        try:
            result = self._robot.hmi.query('What names?', 'T -> ' + ' | '.join(challenge_knowledge.names), 'T',
                                           timeout=self._time_out)
            command_recognized = result.sentence
        except TimeoutException:
            command_recognized = None
        if command_recognized == "":
            self._robot.speech.speak("I am still waiting for a command and did not hear anything")
        elif command_recognized in challenge_knowledge.names:
            if command_recognized == "no":
                self._robot.speech.speak("OK")
            else:
                self._robot.speech.speak("OK, I find {}".format(command_recognized))
            return 'Done'
        else:
            self._robot.speech.speak(
                "I don't understand, I expected a command like " + ", ".join(challenge_knowledge.names))
            return 'Failed'


def setup_statemachine(robot):
    sm = StateMachine(outcomes=['done', 'failed', 'aborted'])

    with sm:

        # StateMachine.add('INITIALIZE',
        #                  Initialize(robot),
        #                  transitions={'initialized': 'INIT_POSE',
        #                               'abort': 'aborted'})
        #
        StateMachine.add('INIT_POSE',
                         SetInitialPose(robot, STARTING_POINT),
                         transitions={'done': 'WAIT_TIME',
                                      'preempted': 'aborted',
                                      # This transition will never happen at the moment.
                                      #  It should never go to aborted.
                                      'error': 'WAIT_TIME'})
        # StateMachine.add('START_CHALLENGE',
        #                  StartChallengeRobust(robot, initial_pose=STARTING_POINT, door=False),
        #                  transitions={'Done': 'GO_TO_SEARCH_POSE',
        #                               'Aborted': 'aborted',
        #                               'Failed': 'WAIT_TIME'})
        #
        StateMachine.add('WAIT_TIME',
                         WaitTime(robot, waittime=2.0),
                         transitions={'waited': 'GO_TO_SEARCH_POSE',
                                      'preempted': 'aborted'})

        # StateMachine.add('ASK_FOR_NAMES',
        #                  AskNames(robot),
        #                  transitions={'Done': 'GO_TO_SEARCH_POSE',
        #                               'Failed': 'GO_TO_SEARCH_POSE',
        #                               'Aborted': 'aborted'})

        StateMachine.add('GO_TO_SEARCH_POSE',
                         NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=SEARCH_POINT), radius=0.4),
                         transitions={'arrived': 'LOCATE_PEOPLE',
                                      'goal_not_defined': 'LOCATE_PEOPLE',
                                      'unreachable': 'LOCATE_PEOPLE'})

        # locate three (or all four) people
        StateMachine.add('LOCATE_PEOPLE',
                         FindPeople(robot),
                         transitions={'Done': 'IDENTIFY_PEOPLE',
                                      'Aborted': 'done',
                                      'Failed': 'LOCATE_PEOPLE'})

        # drive past all thee people and fill their description
        StateMachine.add('IDENTIFY_PEOPLE',
                         IdentifyPeople(robot),
                         transitions={'Done': 'GO_BACK_TO_OPERATOR',
                                      'Aborted': 'done',
                                      'Failed': 'GO_BACK_TO_OPERATOR'})

        # drive back to the operator to describe the mates
        StateMachine.add('GO_BACK_TO_OPERATOR',
                         NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=STARTING_POINT), radius=0.7),
                         transitions={'arrived': 'REPORT_PEOPLE',
                                      'goal_not_defined': 'REPORT_PEOPLE',
                                      'unreachable': 'GO_TO_OPERATOR_MORE_ROOM'})

        StateMachine.add('GO_TO_OPERATOR_MORE_ROOM',
                         NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=STARTING_POINT), radius=1.2),
                         transitions={'arrived': 'REPORT_PEOPLE',
                                      'goal_not_defined': 'REPORT_PEOPLE',
                                      'unreachable': 'GO_BACK_TO_OPERATOR'})

        # check how to uniquely define them
        StateMachine.add('REPORT_PEOPLE',
                         ReportPeople(robot),
                         transitions={'Done': 'done',
                                      'Aborted': 'done',
                                      'Failed': 'failed'})

    return sm


if __name__ == '__main__':
    rospy.init_node('find_my_mates_exec')
    startup(setup_statemachine, challenge_name="find_my_mates")


# Ask operator for names
# Ask people for names
# Provide descriptions of desired people
