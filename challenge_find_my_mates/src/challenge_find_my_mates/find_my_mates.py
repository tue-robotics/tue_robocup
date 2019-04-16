#!/usr/bin/env python
import rospy
import math
import PyKDL as kdl
from robot_smach_states.util import startup
from smach import StateMachine, State
from robot_smach_states import StartChallengeRobust, NavigateToWaypoint, Say
from robot_smach_states.util.designators import EntityByIdDesignator
from robot_skills.util import kdl_conversions

STARTING_POINT = "initial_pose"

room_id = "living_room"
# source_entity = 'dining_table'
# dishwasher_id = 'dishwasher'
# dishwasher_navigate_area = 'to_the_side_of'


class FindPeople(State):
    """
    Locate three (or all four) people
    """
    def __init__(self, robot, search_timeout=60, look_distance=2.0):
        """ Initialization method
        :param robot: robot api object
        :param search_timeout: (float) maximum time the robot is allowed to search
        :param look_distance: (float) robot only considers laser entities within this radius
        """
        State.__init__(self, outcomes=['Done', 'Aborted', 'Failed'])

        self._robot = robot
        self._search_timeout = search_timeout
        self._look_distance = look_distance

    def execute(self, userdata=None):
        rospy.loginfo("Trying to find my mates")
        self._robot.head.look_at_standing_person()
        self._robot.speech.speak("Please look at me while I am looking for you",
                                 block=False)
        self._people[3]
        start_time = rospy.Time.now()

        look_distance = 2.0  # magic number 4
        look_angles = [f * math.pi / d if d != 0 else 0.0 for f in [-1, 1] for d in [0, 6, 4, 2.3]]  # Magic numbers
        head_goals = [kdl_conversions.VectorStamped(x=look_distance * math.cos(angle),
                                                    y=look_distance * math.sin(angle), z=1.7,
                                                    frame_id="/%s/base_link" % self._robot.robot_name)
                      for angle in look_angles]
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

            if raw_detections:
                # Take the biggest ROI, for a start this is fine,
                # but this might be a problem since we want to detect multiple people
                best_detection = max(raw_detections, key=lambda r: r.roi.height)
            else:
                best_detection = None

            rospy.loginfo("best_detection = {}".format(best_detection))
            if not best_detection:
                continue
            roi = best_detection.roi
            try:
                person_pos_kdl = self._robot.perception.project_roi(roi=roi, frame_id="map")
            except Exception as e:
                rospy.logerr("head.project_roi failed: %s", e)
                return 'Failed'

            detected_person = self._robot.ed.get_closest_laser_entity(radius=self._look_distance,
                                                                      center_point=person_pos_kdl)

            room_entity = self._robot.ed.get_entity(id=room_id)
            if not room_entity.in_volume(detected_person.pose.extractVectorStamped(), 'in'):
                detected_person = None

            for person in self._people:
                if person.pose.extractVectorStamped() - detected_person.pose.extractVectorStamped() > min_dist:
                    self._people[n] = detected_person

            if self._people[n]:
                rospy.loginfo(
                    "I found someone at {}".format(self._people[n].pose.extractVectorStamped(),
                                                   block=False))

                self._robot.ed.update_entity(
                    id="person" + str(n),
                    frame_stamped=kdl_conversions.FrameStamped(kdl.Frame(person_pos_kdl.vector), "map"),
                    type="waypoint")

                n += 1
                # if self._found_entity_designator:
                #     self._found_entity_designator.write(found_person)

                if len(self._people) > 2:
                    self._robot.head.close()
                    return 'Done'

            self._robot.head.close()
            # rospy.sleep(2.0)
            return 'Failed'


class IdentifyPeople(State):
    """
    Locate three (or all four) people
    """

    def __init__(self, robot):
        State.__init__(self, outcomes=['Done', 'Aborted', 'Failed'])
        self._robot = robot

    def execute(self, userdata=None):

        if True:
            return 'Done'
        else:
            return 'Failed'


class ReportPeople(State):
    """
    Locate three (or all four) people
    """

    def __init__(self, robot):
        State.__init__(self, outcomes=['Done', 'Aborted', 'Failed'])
        self._robot = robot

    def execute(self, userdata=None):

        if True:
            return 'Done'
        else:
            return 'Failed'

def setup_statemachine(robot):
    sm = StateMachine(outcomes=['done'])

    with sm:
        # Start challenge via StartChallengeRobust
        StateMachine.add('START_CHALLENGE_ROBUST',
                         StartChallengeRobust(robot, STARTING_POINT, use_entry_points=True),
                         transitions={'Done': 'OPEN_DISHWASHER',
                                      'Aborted': 'done',
                                      'Failed': 'OPEN_DISHWASHER'})

        # locate three (or all four) people
        StateMachine.add('LOCATE_PEOPLE',
                         FindPeople(robot),
                         transitions={'Done': 'IDENTIFY_PEOPLE',
                                      'Aborted': 'done',
                                      'Failed': 'DUNNO'})

        # drive past all thee people and fill their description
        StateMachine.add('IDENTIFY_PEOPLE',
                         IdentifyPeople(robot),
                         transitions={'Done': 'GO_BACK_TO_OPERATOR',
                                      'Aborted': 'done',
                                      'Failed': 'DUNNO'})

        # drive back to the operator to describe the mates
        StateMachine.add('GO_BACK_TO_OPERATOR',
                         NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=STARTING_POINT), radius=0.7),
                         transitions={'arrived': 'REPORT_PEOPLE',
                                      'unreachable': 'GO_TO_OPERATOR_MORE_ROOM'})

        StateMachine.add('GO_TO_OPERATOR_MORE_ROOM',
                         NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=STARTING_POINT), radius=1.2),
                         transitions={'arrived': 'REPORT_PEOPLE',
                                      'unreachable': 'GO_BACK_TO_OPERATOR'})

        # check how to uniquely define them
        StateMachine.add('REPORT_PEOPLE',
                         ReportPeople(robot),
                         transitions={'Done': 'IDENTIFY_PEOPLE',
                                      'Aborted': 'done',
                                      'Failed': 'DUNNO'})

    return sm


if __name__ == '__main__':
    rospy.init_node('find_my_mates_exec')
    startup(setup_statemachine, challenge_name="find_my_mates")
