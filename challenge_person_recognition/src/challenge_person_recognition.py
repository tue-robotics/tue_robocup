#! /usr/bin/env python

import rospy
import smach
import robot_smach_states as states

from robocup_knowledge import load_knowledge
from robot_smach_states.util.startup import startup

import robot_smach_states.util.designators as ds
from ed_perception.msg import PersonDetection

from person_recognition_states import LearnOperatorFace, LearnOperatorName, FindOperator, FindAndDescribeCrowd

challenge_knowledge = load_knowledge("challenge_person_recognition")


class ChallengePersonRecognition(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        # ------------------------ INITIALIZATIONS ------------------------

        operator_name_designator = ds.VariableDesignator("person X", name="operator_name")
        operator_person_detection_designator = ds.VariableDesignator(name="operator_person_detection", resolve_type=PersonDetection)

        #  -----------------------------------------------------------------

        with self:
            smach.StateMachine.add( 'INITIALIZE',
                                    states.Initialize(robot),
                                    transitions={'succeeded': 'LEARN_OPERATOR_NAME',
                                                 'failed': 'Aborted'})

            smach.StateMachine.add( 'LEARN_OPERATOR_NAME',
                                    LearnOperatorName(robot, operator_name_designator.writeable),
                                    transitions={'succeeded': 'LEARN_OPERATOR_FACE',
                                                 'failed': 'LEARN_OPERATOR_FACE'})

            smach.StateMachine.add( 'LEARN_OPERATOR_FACE',
                                    LearnOperatorFace(robot, operator_name_designator),
                                    transitions={'succeeded': 'FIND_AND_DESCRIBE_CROWD',
                                                 'failed': 'FIND_AND_DESCRIBE_CROWD'})

            smach.StateMachine.add( 'FIND_AND_DESCRIBE_CROWD',
                                    FindAndDescribeCrowd(robot, operator_person_detection_designator.writeable),
                                    transitions={'succeeded': 'FIND_OPERATOR',
                                                 'failed': 'FIND_OPERATOR'})

            smach.StateMachine.add( 'FIND_OPERATOR',
                                    FindOperator(robot, operator_person_detection_designator),
                                    transitions={'succeeded': 'END_CHALLENGE',
                                                 'failed': 'END_CHALLENGE'})

            smach.StateMachine.add('END_CHALLENGE',
                                   states.Say(robot, "My work here is done, goodbye!"),
                                   transitions={'spoken': 'Done'})

            ds.analyse_designators(self, "person_recognition")

if __name__ == "__main__":
    rospy.init_node('person_recognition_exec')

    startup(ChallengePersonRecognition, challenge_name="person_recognition")
