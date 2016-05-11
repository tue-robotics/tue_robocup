import robot_skills.util.msg_constructors as msgs
import time
import smach
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from ed_perception.msg import PersonDetection
import rospy
from robocup_knowledge import load_knowledge

challenge_knowledge = load_knowledge("challenge_speech_recognition")


class RecognizePersons(smach.State):
    def __init__(self, robot, operator_name_designator, operator_person_detection_designator):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        self.robot = robot

        ds.check_resolve_type(operator_person_detection_designator, PersonDetection)
        ds.check_resolve_type(operator_name_designator, str)

        ds.is_writeable(operator_person_detection_designator)

        self.operator_name_designator = operator_name_designator
        self.operator_person_detection_designator = operator_person_detection_designator

    def execute(self, userdata=None):

        operator_name = self.operator_name_designator.resolve()
        self.robot.speech.speak("I am looking for you %s" % operator_name)

        self.robot.head.look_at_point(point_stamped=msgs.PointStamped(100.0, 0.0, 1.0, self.robot.robot_name +
                                                                      "/base_link"), end_time=0, timeout=8)

        import time
        time.sleep(2)

        detections = self.robot.ed.detect_persons()
        if not detections:
            return 'failed'

        operator_candidates = [candidate for candidate in detections if candidate.name == operator_name]

        rospy.loginfo(operator_candidates)

        if operator_candidates:
            operator = max(operator_candidates, key=lambda candidate: candidate.name_score)

            if operator.gender == 1:
                gender = "male"
            else:
                gender = "female"

            z = operator.pose.pose.position.z
            pose_str = "standing"
            if z < 1.4:
                pose_str = "sitting"
            if z < 0.6:
                pose_str = "lying"

            self.robot.speech.speak("The operator is a %s" % gender)
            self.robot.speech.speak("The operator is %s" % pose_str)

            self.operator_person_detection_designator.write(operator)
            return 'succeeded'
        else:
            return "failed"


class FindAndDescribeCrowd(smach.StateMachine):
    def __init__(self, robot, operator_person_detection_designator):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        with self:

            @smach.cb_interface(outcomes=['done'])
            def wait_a_sec(userdata):
                self.robot.speech.speak("I will wait for 10 seconds for you to join the crowd", block=False)
                time.sleep(10)
                return 'done'

            smach.StateMachine.add('LOOK_AT_OPERATOR',
                                   smach.CBState(wait_a_sec),
                                   transitions={'done': 'GOTO_WAYPOINT'})

            smach.StateMachine.add('GOTO_WAYPOINT',
                                   states.NavigateToWaypoint(robot, ds.EntityByIdDesignator(robot, id=challenge_knowledge.waypoint)),
                                   transitions={'arrived': 'SAY_SEARCHING_CROWD',
                                                'unreachable': 'GOTO_WAYPOINT_BACKUP',
                                                'goal_not_defined': 'GOTO_WAYPOINT_BACKUP'})

            smach.StateMachine.add('GOTO_WAYPOINT_BACKUP',
                                   states.NavigateToWaypoint(robot, ds.EntityByIdDesignator(robot, id=challenge_knowledge.waypoint_backup)),
                                   transitions={'arrived': 'SAY_SEARCHING_CROWD',
                                                'unreachable': 'GOTO_WAYPOINT',
                                                'goal_not_defined': 'GOTO_WAYPOINT'})

            smach.StateMachine.add('SAY_SEARCHING_CROWD',
                                    states.Say(robot, "I'm looking for the crowd.", block=False),
                                    transitions={'spoken': 'FIND_CROWD'})

            smach.StateMachine.add('FIND_CROWD', RecognizePersons(robot, operator_person_detection_designator),
                                   transitions={   'succeeded' : 'succeeded', 'failed' : 'GOTO_WAYPOINT_BACKUP'})
