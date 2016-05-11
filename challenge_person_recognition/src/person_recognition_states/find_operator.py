import robot_skills.util.msg_constructors as msgs
import time
import smach
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from ed_perception.msg import PersonDetection
import rospy
from robocup_knowledge import load_knowledge
from ed.msg import EntityInfo
from robot_skills.util import transformations


challenge_knowledge = load_knowledge("challenge_speech_recognition")


class EntityOfPersonDetection(ds.Designator):
    def __init__(self, robot, persondetection_designator, name=None):
        """
        Resolve to an entity based on a PersonDetection.
        This is needed because most tates only understand Entities rather than PersonDetections
        :param persondetection_designator: a designator resolving to an ed_perception.msg.PersonDetection, which will be converted to an entity
        """
        super(EntityOfPersonDetection, self).__init__(resolve_type=EntityInfo, name=name)

        self.robot = robot
        self.persondetectionDes = persondetection_designator

    def _resolve(self):
        person_detection = self.persondetectionDes.resolve()

        entity = EntityInfo()
        entity.id = "PersonDetection"+str(id(person_detection))
        point_in_map = transformations.tf_transform(person_detection.pose.pose.position, person_detection.pose.header.frame_id, "/map", self.robot.tf_listener)
        entity.pose.position = point_in_map
        return entity


class PointAtOperator(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.robot = robot

    def execute(self, robot):
        rospy.loginfo("PointAtOperator")

        # Get information about the operator and point at the location
        self.robot.rightArm.send_goal(0.5, -0.2, 0.9, 0, 0, 0, 60)

        self.robot.head.look_at_ground_in_front_of_robot(distance=100)

        return 'succeeded'


class IndicateWhichPersonIsOperator(smach.StateMachine):
    def __init__(self, robot, operator_person_detection_designator):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        ds.check_resolve_type(operator_person_detection_designator, PersonDetection)
        operator_entity = EntityOfPersonDetection(robot, operator_person_detection_designator, name="operator_entity")

        with self:
            smach.StateMachine.add('GOTO_OPERATOR',
                                   states.NavigateToObserve(robot, operator_entity, radius = 1.0),
                                   transitions={'arrived':   'SAY_FOUND_OPERATOR',
                                                'unreachable':   'SAY_CANT_REACH',
                                                'goal_not_defined':   'SAY_CANT_REACH'})

            smach.StateMachine.add( 'SAY_FOUND_OPERATOR',
                                    states.Say(robot,"This is my operator!", block=True),
                                    transitions={'spoken': 'POINT_AT_OPERATOR'})

            smach.StateMachine.add( 'SAY_CANT_REACH',
                                    states.Say(robot, "I could not reach my operator but I will point anyway.", block=True),
                                    transitions={'spoken': 'POINT_AT_OPERATOR'})

            smach.StateMachine.add( 'POINT_AT_OPERATOR',
                                    PointAtOperator(robot),
                                    transitions={'succeeded': 'GREET_OPERATOR',
                                                 'failed': 'SAY_CANT_POINT'})

            smach.StateMachine.add( 'SAY_CANT_POINT',
                                    states.Say(robot, "Sorry but i can't point at my operator!", block=True),
                                    transitions={'spoken': 'failed'})

            @smach.cb_interface(outcomes=['spoken'])
            def greetOperatorCB(userdata):
                robot.speech.speak( "I have found you, you are right there!", block=True)
                return 'spoken'

            smach.StateMachine.add( 'GREET_OPERATOR',
                                    smach.CBState(greetOperatorCB),
                                    transitions={'spoken': 'RESET_ARMS'})

            smach.StateMachine.add( 'RESET_ARMS',
                                    states.ResetArms(robot, timeout=5.0),
                                    transitions={'done': 'succeeded'})


class FindOperator(smach.StateMachine):
    def __init__(self, robot, operator_person_detection_designator):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        self.robot = robot
        ds.check_resolve_type(operator_person_detection_designator, PersonDetection)

        with self:
            smach.StateMachine.add( 'SAY_LOOKING_OPERATOR',
                                    states.Say(robot, "I'm looking for my operator.", block=False),
                                    transitions={'spoken':'GREET_OPERATOR'})

            smach.StateMachine.add( 'GREET_OPERATOR',
                                    IndicateWhichPersonIsOperator(robot, operator_person_detection_designator),
                                    transitions={'succeeded': 'succeeded',
                                                 'failed': 'failed'})
