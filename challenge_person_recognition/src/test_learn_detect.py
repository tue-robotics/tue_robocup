#! /usr/bin/env python


import smach, rospy
from robot_smach_states.util.startup import startup
from robocup_knowledge import load_knowledge
import person_recognition_states as PersonRecStates
from person_recognition import LearnOperatorName, LearnOperatorFace
from ed_perception.msg import PersonDetection
import robot_smach_states.util.designators as ds
import robot_smach_states as states

challenge_knowledge = load_knowledge("challenge_person_recognition")


class ChallengePersonRecognition(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        # ------------------------ INITIALIZATIONS ------------------------

        operatorNameDes = ds.VariableDesignator("person X", name="operatorNameDes")
        detectedPersonsListDes = ds.VariableDesignator([], name="detectedPersonsListDes", resolve_type=[PersonDetection])
        operator_person_des = ds.VariableDesignator(name="operator_person_des", resolve_type=PersonDetection)

        #  -----------------------------------------------------------------

        with self:
            smach.StateMachine.add( 'LEARN_OPERATOR_NAME',
                                    LearnOperatorName(robot, operatorNameDes.writeable),
                                    transitions={    'succeeded' :'LEARN_OPERATOR_FACE',
                                                    'failed'    :'LEARN_OPERATOR_FACE'})

            smach.StateMachine.add( 'LEARN_OPERATOR_FACE',
                                    LearnOperatorFace(robot, operatorNameDes),
                                    transitions={    'succeeded' :'SAY_FIND',
                                                     'failed'    :'SAY_FIND'})

            smach.StateMachine.add("SAY_FIND", states.Say(robot,["Please let the crowd stand in front of me!"], block=True), transitions={'spoken':'FIND_CROWD'})

            smach.StateMachine.add( 'FIND_CROWD',
                                    PersonRecStates.RecognizePersons(robot, detectedPersonsListDes.writeable),
                                    transitions={   'succeeded' : 'DESCRIBE_PEOPLE',
                                                    'failed' : 'DESCRIBE_PEOPLE'})

            smach.StateMachine.add( 'DESCRIBE_PEOPLE',
                                    PersonRecStates.DescribePeople(robot, detectedPersonsListDes, operator_person_des),
                                    transitions={   'succeeded' :'Done',
                                                    'failed'    :'Done'})

# ----------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    rospy.init_node('person_recognition_exec')
    
    startup(ChallengePersonRecognition, challenge_name="person_recognition")
