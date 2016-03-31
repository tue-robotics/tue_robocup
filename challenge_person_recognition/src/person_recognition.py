#! /usr/bin/env python

import roslib;
import rospy
import sys
import smach
import smach_ros
import person_recognition_states as PersonRecStates
import robot_smach_states as states
import geometry_msgs.msg as gm
import robot_skills.util.msg_constructors as msgs
from robot_skills.util import transformations

from robocup_knowledge import load_knowledge
from robot_smach_states.util.startup import startup

import robot_smach_states.util.designators as ds
from ed_perception.msg import PersonDetection
from ed.msg import EntityInfo

challenge_knowledge = load_knowledge("challenge_person_recognition")

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

def printOk(sentence):
    challenge_knowledge.printOk(sentence)

def printError(sentence):
    challenge_knowledge.printError(sentence)

def printWarning(sentence):
    challenge_knowledge.printWarning(sentence)

# ---------------------------------------------------------------------------------------------------
class Start(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        with self:
            smach.StateMachine.add( 'INITIALIZE',
                                    states.Initialize(robot),
                                    transitions={   'initialized':'INIT_WM',
                                                    'abort':'failed'})

            smach.StateMachine.add( "INIT_WM",
                                    states.InitializeWorldModel(robot),
                                    transitions={    'done':'succeeded'})


class LearnOperatorName(smach.StateMachine):
    def __init__(self, robot, operatorNameDes):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        ds.check_resolve_type(operatorNameDes, str)
        ds.is_writeable(operatorNameDes)
        self.operatorNameDes = operatorNameDes

        @smach.cb_interface(outcomes=['spoken'])
        def sayCouldNotLearnNameCB(userdata):
            robot.head.look_at_standing_person()
            robot.speech.speak( "Sorry but I could not understand your name. I will just call you " + operatorNameDes.resolve(), block=False)
            return 'spoken'

        @smach.cb_interface(outcomes=['spoken'])
        def sayIsYourName(userdata):
            robot.speech.speak( "I heard " + operatorNameDes.resolve() + ". Is this correct?", block=True)
            return 'spoken'

        @smach.cb_interface(outcomes=['spoken'])
        def sayHelloCB(userdata):
            robot.speech.speak( "Hello " + operatorNameDes.resolve() + "!", block=False)
            return 'spoken'

        with self:
                smach.StateMachine.add("SAY_WAITING_OPERATOR", states.Say(robot,[  "I'm waiting for the operator to stand in front of me.","I need an operator, please stand in front of me."], block=False), transitions={   'spoken':'LOOK_AT_OPERATOR'})

                smach.StateMachine.add('LOOK_AT_OPERATOR',
                                        states.LookAtPersonInFront(robot, lookDown=False),
                                        transitions={   'succeeded':'LEARN_NAME_ITERATOR',
                                                        'failed':'LOOK_AT_OPERATOR'})

                learnNameIterator = smach.Iterator( outcomes=['container_success', 'container_failed'],
                                                    it = lambda:range(0, 3),
                                                    it_label='counter',
                                                    input_keys=[],
                                                    output_keys=['personName_userData'],
                                                    exhausted_outcome = 'container_failed')

                with learnNameIterator:

                    learnNameContainer = smach.StateMachine(output_keys=['personName_userData'],
                                                            outcomes = ['container_success', 'container_failed'])
                    with learnNameContainer:

                        # initialize personName_userData
                        learnNameContainer.userdata.personName_userData = ""

                        smach.StateMachine.add( 'ASK_PERSON_NAME',
                                                PersonRecStates.AskPersonName(robot, operatorNameDes),
                                                remapping={     'personName_out':'personName_userData'},
                                                transitions={   'succeeded':'SAY_IS_YOUR_NAME',
                                                                'failed':'SAY_HEAR_FAILED'})

                        smach.StateMachine.add( 'SAY_IS_YOUR_NAME',
                                                smach.CBState(sayIsYourName),
                                                transitions={    'spoken':'HEAR_YES_NO_1'})

                        smach.StateMachine.add( 'HEAR_YES_NO_1',
                                                states.HearYesNo(robot),
                                                transitions={   'heard_yes' : 'container_success',
                                                                'heard_no' : 'SAY_NAME_WRONG',
                                                                'heard_failed' : 'SAY_HEAR_FAILED'})

                        smach.StateMachine.add( 'SAY_NAME_WRONG',
                                                states.Say(robot, [  "Sorry, I understood wrong.",
                                                                    "Oh I'm sorry."], block=False),
                                                transitions={    'spoken':'container_failed'})

                        smach.StateMachine.add( 'SAY_HEAR_FAILED',
                                                states.Say(robot, [  "I did not understand your name.",
                                                                    "I didn't listen correctly."], block=False),
                                                transitions={    'spoken':'container_failed'})

                    smach.Iterator.set_contained_state( 'LEARN_NAME_CONTAINER',
                                                        learnNameContainer,
                                                        # loop_outcomes=['container_failed'],
                                                        break_outcomes=['container_success'])

                smach.StateMachine.add( 'LEARN_NAME_ITERATOR',
                                        learnNameIterator,
                                        transitions = { 'container_failed':'SAY_COULD_NOT_LEARN_NAME',
                                                        'container_success':'SAY_HELLO'})

                # ----------------------------------------

                smach.StateMachine.add( 'SAY_COULD_NOT_LEARN_NAME',
                                        smach.CBState(sayCouldNotLearnNameCB),
                                        transitions={    'spoken':'failed'})

                smach.StateMachine.add( 'SAY_HELLO',
                                        smach.CBState(sayHelloCB),
                                        transitions={    'spoken':'succeeded'})


class LearnOperatorFace(smach.StateMachine):
    def __init__(self, robot, operatorNameDes):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        self.robot = robot

        ds.check_resolve_type(operatorNameDes, str)
        self.operatorNameDes = operatorNameDes

        with self:
            smach.StateMachine.add( 'LOOK_AT_OPERATOR',
                                    states.LookAtPersonInFront(robot, lookDown=False),
                                    transitions={   'succeeded':'SAY_LOOK_AT_ME',
                                                    'failed':'SAY_LOOK_AT_ME'})

            smach.StateMachine.add( 'SAY_LOOK_AT_ME',
                                    states.Say(robot,"Please stand one meter in front of me and look at me while I learn to recognize your face.", block=True),
                                    transitions={    'spoken':'LEARN_PERSON'})

            smach.StateMachine.add('LEARN_PERSON',
                                    states.LearnPerson(robot, name_designator=operatorNameDes),
                                    transitions={   'succeeded_learning':'SAY_OPERATOR_LEARNED',
                                                    'failed_learning':'SAY_FAILED_LEARNING',
                                                    'timeout_learning':'SAY_FAILED_LEARNING'})

            # ------- FAILED -----

            smach.StateMachine.add( 'TOGGLE_PERCEPTION_OFF_FAILED',
                                    PersonRecStates.TogglePerceptionMode(robot, toggle_mode=False),
                                    transitions={   'done':'SAY_LEARN_FACE_FAILED'})

            smach.StateMachine.add( 'SAY_FAILED_LEARNING',
                                    states.Say(robot,"I could not learn my operator's face. Let me try again.", block=True),
                                    transitions={    'spoken':'failed'})

            smach.StateMachine.add('SAY_LEARN_FACE_FAILED',
                                   states.Say(robot,"I could not learn your face for some reason. Let's try again.", block=True),
                                   transitions={    'spoken':'LOOK_AT_OPERATOR'})

            # ------- SUCCESS -----
            smach.StateMachine.add( 'TOGGLE_PERCEPTION_OFF_SUCCESS',
                                    PersonRecStates.TogglePerceptionMode(robot, toggle_mode=False),
                                    transitions={   'done':'SAY_OPERATOR_LEARNED'})

            smach.StateMachine.add('SAY_OPERATOR_LEARNED',
                                   states.Say(robot,"Now i know what you look like. Please go mix with the crowd."),
                                   transitions={'spoken':   'succeeded'})


class WaitForStart(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        with self:
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                             WAIT_CONTINUE_ITERATOR
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            waitContinueIterator = smach.Iterator(  outcomes=['succeeded', 'failed'],
                                                    it = lambda:range(0, 3),
                                                    it_label='waitCounter',
                                                    input_keys=[],
                                                    output_keys=[],
                                                    exhausted_outcome = 'failed')
            with waitContinueIterator:

                waitContinueContainer = smach.StateMachine( outcomes = ['succeeded', 'heard_nothing'])

                with waitContinueContainer:

                    smach.StateMachine.add("ASK_CONTINUE",
                                            states.AskContinue(robot),
                                            transitions={   'continue':'succeeded',
                                                            'no_response':'heard_nothing'})

                smach.Iterator.set_contained_state( 'WAIT_CONTINUE_CONTAINER',
                                                     waitContinueContainer,
                                                     # loop_outcomes=['heard_nothing'],
                                                     break_outcomes=['succeeded'])

            smach.StateMachine.add( 'WAIT',
                        waitContinueIterator,
                        transitions={    'succeeded' :'succeeded',
                                         'failed'    :'failed'})


class FindCrowdByDrivingAround(smach.StateMachine):
    def __init__(self, robot, detectedPersonsListDes):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        with self:
            smach.StateMachine.add( 'RESET_ED_1',
                                        PersonRecStates.ResetEd(robot),
                                        transitions={   'done':  'GOTO_LIVING_ROOM_1'})

            smach.StateMachine.add( 'GOTO_LIVING_ROOM_1',
                                    states.NavigateToWaypoint(robot, ds.EntityByIdDesignator(robot, id=challenge_knowledge.waypoint_living_room_1)),
                                    transitions={   'arrived' : 'SAY_SEARCHING_CROWD',
                                                    'unreachable' : 'SAY_SEARCHING_CROWD',
                                                    'goal_not_defined' : 'SAY_SEARCHING_CROWD'})

            smach.StateMachine.add( 'SAY_SEARCHING_CROWD',
                                    states.Say(robot, "I'm searching for the crowd.", block=False),
                                    transitions={   'spoken':'FIND_CROWD'})

            smach.StateMachine.add( 'FIND_CROWD',
                                    PersonRecStates.RecognizePersons(robot, detectedPersonsListDes),
                                    transitions={   'succeeded' : 'succeeded',
                                                    'failed' : 'failed'})


class IndicateWhichPersonIsOperator(smach.StateMachine):
    def __init__(self, robot, operatorNameDes, operatorPersonDes):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        ds.check_resolve_type(operatorNameDes, str)
        ds.check_resolve_type(operatorPersonDes, PersonDetection)
        operator_entity = EntityOfPersonDetection(robot, operatorPersonDes, name="operator_entity")

        with self:
            smach.StateMachine.add( 'GOTO_OPERATOR',
                                    states.NavigateToObserve(robot, operator_entity, radius = 1.0),
                                    transitions={   'arrived'           :   'SAY_FOUND_OPERATOR',
                                                    'unreachable'       :   'SAY_CANT_REACH',
                                                    'goal_not_defined'  :   'SAY_CANT_REACH'})

            smach.StateMachine.add( 'SAY_FOUND_OPERATOR',
                                    states.Say(robot,"This is my operator!", block=True),
                                    transitions={   'spoken':'POINT_AT_OPERATOR'})

            smach.StateMachine.add( 'SAY_CANT_REACH',
                                    states.Say(robot,"I could not reach my operator but I will point anyway.", block=True),
                                    transitions={   'spoken':'POINT_AT_OPERATOR'})

            smach.StateMachine.add( 'POINT_AT_OPERATOR',
                                    PersonRecStates.PointAtOperator(robot),
                                    transitions={   'succeeded':'GREET_OPERATOR',
                                                    'failed':'SAY_CANT_POINT'})

            smach.StateMachine.add( 'SAY_CANT_POINT',
                                    states.Say(robot,"Sorry but i can't point at my operator!", block=True),
                                    transitions={   'spoken':'failed'})

            @smach.cb_interface(outcomes=['spoken'])
            def greetOperatorCB(userdata):
                robot.speech.speak( "I have found you {0}, you are right there!".format(operatorNameDes.resolve()), block=True)
                return 'spoken'
            smach.StateMachine.add( 'GREET_OPERATOR',
                                    smach.CBState(greetOperatorCB),
                                    transitions={   'spoken':'RESET_ARMS'})

            smach.StateMachine.add( 'RESET_ARMS',
                                    states.ResetArms(robot, timeout=5.0),
                                    transitions={   'done':'succeeded'})




class FindOperatorByInspectingAll(smach.StateMachine):
    def __init__(self, robot, operatorNameDes, detectedPersonsDes):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        self.robot = robot
        ds.check_resolve_type(detectedPersonsDes, [PersonDetection])

        remaining_persons = ds.VariableDesignator([None], resolve_type=[PersonDetection], name="remaining_persons")
        person_to_inspect = ds.VariableDesignator(None, resolve_type=PersonDetection, name="person_to_inspect")
        entity_to_inspect = EntityOfPersonDetection(person_to_inspect, name="entity_to_inspect")

        with self:
            smach.StateMachine.add( 'SAY_LOOKING_OPERATOR',
                                        states.Say(robot,"I'm looking for my operator.", block=False),
                                        transitions={   'spoken':'GET_NEXT_LOCATION'})

            smach.StateMachine.add( 'GET_NEXT_LOCATION',
                                    PersonRecStates.GetNextPerson(robot, detectedPersonsDes, person_to_inspect.writeable, remaining_persons.writeable),
                                    transitions={   'next_selected':'GOTO_LOCATION',
                                                    'visited_all':'succeeded'})

            smach.StateMachine.add( 'GOTO_LOCATION',
                                    states.NavigateToObserve(robot, entity_designator=entity_to_inspect, radius=1.8),
                                    transitions={   'arrived'           :   'SAY_LOOK_AT_ME',
                                                    'unreachable'       :   'SAY_FAILED_GOTO',
                                                    'goal_not_defined'  :   'SAY_FAILED_GOTO'})

            smach.StateMachine.add( 'SAY_LOOK_AT_ME',
                                    states.Say(robot,[  "Please look at me.",
                                                        "Would you look into my camera?",
                                                        "Let me see who is here, please face me."], block=True),
                                    transitions={   'spoken':'ANALYSE_ITERATOR'})

            analyzeIterator = smach.Iterator(   outcomes=['container_success', 'container_failed'],
                                                it = lambda:range(0, 5),
                                                it_label='counter',
                                                input_keys=[],
                                                output_keys=[],
                                                exhausted_outcome = 'container_failed')
            with analyzeIterator:

                analyzeIterator = smach.StateMachine( outcomes = ['container_success', 'container_failed'])

                with analyzeIterator:

                    smach.StateMachine.add("LOOK_AT_PERSON",
                                    states.LookAtPersonInFront(robot, lookDown=False),
                                    transitions={   'succeeded':'ANALYZE_PERSON',
                                                    'failed':'ANALYZE_PERSON'})

                    smach.StateMachine.add( 'ANALYZE_PERSON',
                                            PersonRecStates.AnalysePerson(robot),
                                            transitions={   'succeeded':'CANCEL_HEAD_GOALS',
                                                            'failed':'container_failed'})

                    smach.StateMachine.add( 'CANCEL_HEAD_GOALS',
                                PersonRecStates.CancelHeadGoals(robot),
                                transitions={    'done':'container_success'})


                smach.Iterator.set_contained_state( 'ANALYZE_CONTAINER',
                                                     analyzeIterator,
                                                     # loop_outcomes=['container_failed'],
                                                     break_outcomes=['container_success'])

            # add the lookoutIterator to the main state machine
            smach.StateMachine.add( 'ANALYSE_ITERATOR',
                                    analyzeIterator,
                                    {   'container_failed':'CANCEL_HEAD_GOALS_3',
                                        'container_success':'GET_NEXT_LOCATION'})

            smach.StateMachine.add( 'CANCEL_HEAD_GOALS_3',
                                PersonRecStates.CancelHeadGoals(robot),
                                transitions={    'done':'SAY_FAILED_ANALYSIS'})

            smach.StateMachine.add( 'SAY_FAILED_ANALYSIS',
                                    states.Say(robot,[  "I could not find a person here",
                                                        "I don't see any faces here",
                                                        "I guess there is no one here"], block=True),
                                    transitions={   'spoken':'GET_NEXT_LOCATION'})

            smach.StateMachine.add( 'SAY_FAILED_GOTO',
                                    states.Say(robot,[  "I could not go to the chosen location",
                                                        "I can't reach that location",
                                                        "I can't get there"], block=True),
                                    transitions={   'spoken':'GET_NEXT_LOCATION'})

            smach.StateMachine.add( 'GET_OPERATOR_LOCATION',
                                        PersonRecStates.ChooseOperator(robot, facesAnalyzedDes, operatorNameDes, operatorLocationDes),
                                        remapping={     'operatorIdx_out':'operatorIdx_userData'},
                                        transitions={   'succeeded': 'GOTO_OPERATOR',
                                                        'failed':'SAY_CANT_CHOOSE_OPERATOR'})

            smach.StateMachine.add( 'GOTO_OPERATOR',
                                    IndicateWhichPersonIsOperator(robot, operatorNameDes, person_to_inspect),
                                    transitions={       'succeeded':'succeeded',
                                                        'failed'   :'failed'})


class FindOperatorFromADistance(smach.StateMachine):
    def __init__(self, robot, operator_name_des, detected_persons_des, operator_person_des):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        self.robot = robot
        ds.check_resolve_type(detected_persons_des, [PersonDetection])
        ds.check_resolve_type(operator_name_des, str)
        ds.check_resolve_type(operator_person_des, PersonDetection)
        ds.is_writeable(operator_person_des)

        with self:
            smach.StateMachine.add( 'SAY_LOOKING_OPERATOR',
                                    states.Say(robot,"I'm looking for my operator.", block=False),
                                    transitions={   'spoken':'SELECT_OPERATOR_FROM_CROWD'})

            smach.StateMachine.add( 'SELECT_OPERATOR_FROM_CROWD',
                                    PersonRecStates.SelectOperator(robot, operator_name_des, detected_persons_des, operator_person_des),
                                    transitions={   'succeeded' :'GREET_OPERATOR',
                                                    'failed'    :'SAY_OPERATOR_NOT_FOUND'})

            smach.StateMachine.add( 'SAY_OPERATOR_NOT_FOUND',
                                    states.Say(robot, ["I did not detect my operator", "I couldn't see my operator", "My operator does not seem to be in the crowd"], block=True),
                                    transitions={   'spoken':'failed'})

            smach.StateMachine.add( 'GREET_OPERATOR',
                                    IndicateWhichPersonIsOperator(robot, operator_name_des, operator_person_des),
                                    transitions={   'succeeded' :'succeeded',
                                                    'failed'    :'failed'})


class ChallengePersonRecognition(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        # ------------------------ INITIALIZATIONS ------------------------

        operatorNameDes = ds.VariableDesignator("person X", name="operatorNameDes")
        detectedPersonsListDes = ds.VariableDesignator([], name="detectedPersonsListDes", resolve_type=[PersonDetection])
        operator_person_des = ds.VariableDesignator(name="operator_person_des", resolve_type=PersonDetection)

        #  -----------------------------------------------------------------

        with self:
            smach.StateMachine.add( 'START',
                                    Start(robot),
                                    transitions={    'succeeded' :'LEARN_OPERATOR_NAME',
                                                     'failed'    :'Aborted'})

            smach.StateMachine.add( 'LEARN_OPERATOR_NAME',
                                    LearnOperatorName(robot, operatorNameDes.writeable),
                                    transitions={    'succeeded' :'LEARN_OPERATOR_FACE',
                                                    'failed'    :'LEARN_OPERATOR_FACE'})

            smach.StateMachine.add( 'LEARN_OPERATOR_FACE',
                                    LearnOperatorFace(robot, operatorNameDes),
                                    transitions={    'succeeded' :'WAIT_CONTINUE_ITERATOR',
                                                     'failed'    :'WAIT_CONTINUE_ITERATOR'})

            # add the lookoutIterator to the main state machine
            smach.StateMachine.add( 'WAIT_CONTINUE_ITERATOR',
                                    WaitForStart(robot),
                                    {   'failed':'SAY_NO_CONTINUE',
                                        'succeeded':'FIND_CROWD_ITERATOR'})

            smach.StateMachine.add( 'SAY_NO_CONTINUE',
                                    states.Say(robot, "Ready or not, here I come!", block=True),
                                    transitions={   'spoken':'FIND_CROWD_ITERATOR'})


            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                                   FIND_CROWD_CONTAINER
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            find_crowd_iterator = smach.Iterator(   outcomes=['succeeded', 'failed'],
                                                it = lambda:range(0, 5),
                                                it_label='counter',
                                                input_keys=[],
                                                output_keys=[],
                                                exhausted_outcome = 'failed')
            with find_crowd_iterator:
                find_crowd = smach.StateMachine( outcomes = ['succeeded', 'failed'])
                with find_crowd:
                    smach.StateMachine.add( 'FIND_CROWD_BY_DRIVING_AROUND',
                                            FindCrowdByDrivingAround(robot, detectedPersonsListDes.writeable),
                                            transitions={   'succeeded':'succeeded',
                                                            'failed': 'failed'}) # TODO: Limit number of iterations
                smach.Iterator.set_contained_state( 'FIND_CROWD_CONTAINER',
                                                     find_crowd,
                                                     loop_outcomes=['failed'],
                                                     break_outcomes=['succeeded'])

            smach.StateMachine.add( 'FIND_CROWD_ITERATOR',
                                    find_crowd_iterator,
                                    {   'failed':'SAY_FAILED_FIND_CROWD',
                                        'succeeded':'SAY_FOUND_CROWD'})

            smach.StateMachine.add( 'SAY_FOUND_CROWD',
                                    states.Say(robot,[  "I think I found some people.",
                                                        "I think I saw several people over there"], block=True),
                                    transitions={   'spoken':'RESET_ED_2'})

            smach.StateMachine.add('SAY_FAILED_FIND_CROWD',
                                   states.Say(robot,"Could not find the crowd, sorry", block=True),
                                   transitions={    'spoken':'RESET_ED_2'})

            smach.StateMachine.add( 'RESET_ED_2',
                        PersonRecStates.ResetEd(robot),
                        transitions={   'done':  'DESCRIBE_PEOPLE'})

            smach.StateMachine.add( 'DESCRIBE_PEOPLE',
                                    PersonRecStates.DescribePeople(robot, detectedPersonsListDes, operator_person_des),
                                    transitions={   'succeeded' :'FIND_OPERATOR',
                                                    'failed'    :'FIND_OPERATOR'})
            #add container to the main state machine
            smach.StateMachine.add( 'FIND_OPERATOR',
                                    FindOperatorFromADistance(robot, operatorNameDes, detectedPersonsListDes, operator_person_des.writeable),
                                    transitions={   'succeeded':'END_CHALLENGE',
                                                    'failed':'END_CHALLENGE'})

            smach.StateMachine.add('END_CHALLENGE',
                                   states.Say(robot,"My work here is done, goodbye!"),
                                   transitions={'spoken':'Done'})

            ds.analyse_designators(self, "person_recognition")

# ----------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    rospy.init_node('person_recognition_exec')

    startup(ChallengePersonRecognition, challenge_name="person_recognition")
