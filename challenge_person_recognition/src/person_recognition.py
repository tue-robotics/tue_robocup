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

# from actionlib import *
# from actionlib.msg import *

from robot_skills.amigo import Amigo
from robot_skills.sergio import Sergio
from robot_skills.mockbot import Mockbot
# from smach_ros import SimpleActionState

# from ed_perception.srv import LearnPerson, LearnPersonRequest
from robot_smach_states.util.designators import EdEntityDesignator, VariableDesignator, DeferToRuntime



# ----------------------------------------------------------------------------------------------------

class ChallengePersonRecognition(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        # ------------------------ INITIALIZATIONS ------------------------

        waypoint_learning = EdEntityDesignator(robot, id="person_rec_learning")
        waypoint_living_room_1 = EdEntityDesignator(robot, id="person_rec_living_room_1")
        waypoint_living_room_2 = EdEntityDesignator(robot, id="person_rec_living_room_2")
        waypoint_living_room_3 = EdEntityDesignator(robot, id="person_rec_living_room_3")

        operatorNameDes = VariableDesignator(resolve_type=str)
        operatorNameDes.current = ""

        nextLocationDes = VariableDesignator(resolve_type=PersonRecStates.PointDesignator)
        nextLocationDes.current = PersonRecStates.PointDesignator()

        locationsToVisitDes = VariableDesignator(resolve_type=list)
        locationsToVisitDes.current = []

        facesAnalyzedDes = VariableDesignator(resolve_type=list)
        facesAnalyzedDes.current = []

        operatorLocationDes = VariableDesignator(resolve_type=PersonRecStates.PointDesignator) # @LBFereirra: a designator that resolves to another designator is a bit weird
        operatorLocationDes.current = PersonRecStates.PointDesignator()

        def helloOperator(): return "Hello " + operatorNameDes.resolve()
        helloOperatorDes = DeferToRuntime(helloOperator, resolve_type=str)

        def defaultNameOperator(): return "I did not understand your name, so I will call you " + operatorNameDes.resolve()
        defaultNameOperatorDes = DeferToRuntime(defaultNameOperator, resolve_type=str)

        # ------------------ SIMULATION ------------------------------------

        # print PersonRecStates.OUT_PREFIX + PersonRecStates.bcolors.WARNING + "Adding simulated knowledge!" + PersonRecStates.bcolors.ENDC
        # locationsToVisitDes.current += [PersonRecStates.Location(   point_stamped = msgs.PointStamped(x=0.386, y=0.261, z= 1.272, frame_id="/map"),
        #                                                             visited = False, 
        #                                                             attempts = 0)]

        # locationsToVisitDes.current += [PersonRecStates.Location(   point_stamped = msgs.PointStamped(x=0.452, y=0.363, z=1.248, frame_id="/map"),
        #                                                             visited = False, 
        #                                                             attempts = 0)]

        # locationsToVisitDes.current += [PersonRecStates.Location(   point_stamped = msgs.PointStamped(x=0.234, y=0.912, z=1.248, frame_id="/map"),
        #                                                             visited = False, 
        #                                                             attempts = 0)]

        # facesAnalyzedDes.current += [PersonRecStates.FaceAnalysed(  point_stamped = msgs.PointStamped(x=0.439086, y=0.786736, z=1.6135, frame_id="/map"),
        #                                                             name = "Mr. Operator",
        #                                                             score = 0.410413)]
        # facesAnalyzedDes.current += [PersonRecStates.FaceAnalysed(  point_stamped = msgs.PointStamped(x=1.71268, y=-0.49675, z=1.40221, frame_id="/map"),
        #                                                             name = "Mr. Operator",
        #                                                             score = 0.131082)]
        # facesAnalyzedDes.current += [PersonRecStates.FaceAnalysed(  point_stamped = msgs.PointStamped(x=0.594543, y=0.292026, z=1.61433, frame_id="/map"),
        #                                                             name = "Mr. Operator",
        #                                                             score = 0.140992)]
        # facesAnalyzedDes.current += [PersonRecStates.FaceAnalysed(  point_stamped = msgs.PointStamped(x=0.601418, y=0.140883, z=1.63293, frame_id="/map"),
        #                                                             name = "max",
        #                                                             score = 0.257553)]
        # facesAnalyzedDes.current += [PersonRecStates.FaceAnalysed(  point_stamped = msgs.PointStamped(x=0.433752, y=0.83502, z=1.62124, frame_id="/map"),
        #                                                             name = "Mr. Operator",
        #                                                             score = 0.455918)]
        # facesAnalyzedDes.current += [PersonRecStates.FaceAnalysed(  point_stamped = msgs.PointStamped(x=0.601418, y=0.140883, z=1.63293, frame_id="/map"),
        #                                                             name = "max",
        #                                                             score = 0.257553)]

        #  -----------------------------------------------------------------

        @smach.cb_interface(outcomes=['done'])
        def removeLocation(userdatam):
            # import ipdb; ipdb.set_trace()
            locationsList = locationsToVisitDes.resolve()
            locToRemove = nextLocationDes.resolve().resolve()
            updated = False

            print PersonRecStates.OUT_PREFIX + "removeLocationCB"

            print PersonRecStates.OUT_PREFIX + "Locations available: " + str(locationsList)

            #  iterate through all locations on the list and update the correct one
            for loc in locationsList:
                if locToRemove.pose.position == loc.point_stamped.point and loc.visited == False:
                    print PersonRecStates.OUT_PREFIX + "Updading this location to visited:\n" + str(loc.point_stamped)
                    loc.visited = True
                    updated = True
                    break

            if not updated:
                print PersonRecStates.OUT_PREFIX + PersonRecStates.bcolors.FAIL + "Location not found in the list!"

            return 'done'

        #  -----------------------------------------------------------------

        with self:

            smach.StateMachine.add( 'START_CHALLENGE',
                                    states.StartChallengeRobust(robot, 'initial_pose'),
                                    transitions={   'Done':'GOTO_ENTRY',
                                                    'Aborted':'Aborted',
                                                    'Failed':'GOTO_ENTRY'})

            smach.StateMachine.add('GOTO_ENTRY',
                                    states.NavigateToWaypoint(robot, waypoint_learning),
                                    transitions={   'arrived':'LEARN_OPERATOR_CONTAINER',
                                                    'unreachable':'LEARN_OPERATOR_CONTAINER',
                                                    'goal_not_defined':'LEARN_OPERATOR_CONTAINER'})


            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                                 LEARN_OPERATOR_CONTAINER
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            # set default name in case learning name fails

            # container for this stage
            learnOperatorContainer = smach.StateMachine(outcomes = ['container_success', 'container_failed'],
                                                        output_keys = ['personName_userData'])

            # learnOperatorContainer.userdata.personName_userData = "Mister Operator"

            with learnOperatorContainer:


                smach.StateMachine.add("SAY_WAITING_OPERATOR",
                                        states.Say(robot,[  "I'm waiting for the operator to stand in front of me.",
                                                            "Would the operator please come forward.",
                                                            "I need an operator, please stand in front of me."], block=False),
                                        transitions={   'spoken':'LOOK_AT_OPERATOR'})

                smach.StateMachine.add('LOOK_AT_OPERATOR',
                                        PersonRecStates.LookAtPersonInFront(robot, lookDown=False),
                                        transitions={   'succeded':'WAIT_FOR_OPERATOR',
                                                        'failed':'WAIT_FOR_OPERATOR'})

                smach.StateMachine.add("WAIT_FOR_OPERATOR",
                                        states.WaitForPersonInFront(robot, attempts=8, sleep_interval=1),
                                        transitions={   'success':'LEARN_NAME_ITERATOR',
                                                        'failed':'SAY_WAITING_OPERATOR'})

                # ----------------------------------------

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
                                                transitions={   'succeded':'container_success',
                                                                'failed':'SAY_LEARN_NAME_FAILED'})

                        smach.StateMachine.add('SAY_LEARN_NAME_FAILED',
                                           states.Say(robot, [  "I did not understand your name, could you repeat after the beep?",
                                                                "Could you repeat your name after the beep?"]),
                                           transitions={    'spoken':'container_failed'})

                    smach.Iterator.set_contained_state( 'LEARN_NAME_CONTAINER', 
                                                        learnNameContainer,
                                                        # loop_outcomes=['container_failed'],
                                                        break_outcomes=['container_success'])

                # add the learnNameIterator to the main state machine
                smach.StateMachine.add( 'LEARN_NAME_ITERATOR',
                                        learnNameIterator,
                                        transitions = { 'container_failed':'SAY_COULD_NOT_LEARN_NAME',
                                                        'container_success':'SAY_HELLO'})

                # ----------------------------------------

                smach.StateMachine.add( 'SAY_COULD_NOT_LEARN_NAME',
                                        states.Say(robot, defaultNameOperatorDes.resolve(), block=False),
                                        transitions={    'spoken':'SAY_LOOK_AT_ME'})

                smach.StateMachine.add( 'SAY_HELLO',
                                        states.Say(robot, helloOperatorDes.resolve(), block=False),
                                        transitions={    'spoken':'SAY_LOOK_AT_ME'})

                smach.StateMachine.add( 'SAY_LOOK_AT_ME',
                                        states.Say(robot,"Please look at me while I learn your face.", block=False),
                                        transitions={    'spoken':'LOOK_AT_OPERATOR_2'})

                smach.StateMachine.add( 'LOOK_AT_OPERATOR_2',
                                        PersonRecStates.LookAtPersonInFront(robot, lookDown=False),
                                        transitions={   'succeded':'LEARN_PERSON',
                                                        'failed':'LEARN_PERSON'})

                smach.StateMachine.add('LEARN_PERSON',
                                        states.LearnPerson(robot),
                                        remapping={     'personName_in':'personName_userData'},
                                        transitions={   'succeded_learning':'SAY_OPERATOR_LEARNED',
                                                        'failed_learning':'SAY_LEARN_FACE_FAILED'})

                smach.StateMachine.add('SAY_LEARN_FACE_FAILED',
                                       states.Say(robot,"I could not learn your face for some reason. Let's try again.", block=False),
                                       transitions={    'spoken':'LOOK_AT_OPERATOR'})

                smach.StateMachine.add('SAY_OPERATOR_LEARNED',
                                       states.Say(robot,"Now i know what you look like. Please go mix with the crowd."),
                                       transitions={'spoken':   'container_success'})

            smach.StateMachine.add( 'LEARN_OPERATOR_CONTAINER',
                                    learnOperatorContainer,
                                    transitions={   'container_success':'WAIT_CONTINUE_ITERATOR',
                                                    'container_failed': 'CANCEL_HEAD_GOALS_1'})

            smach.StateMachine.add( 'CANCEL_HEAD_GOALS_1',
                                    PersonRecStates.CancelHeadGoals(robot),
                                    transitions={    'done':'SAY_FAILED_LEARNING'})

            smach.StateMachine.add( 'SAY_FAILED_LEARNING',
                                    states.Say(robot,"I could not learn my operator's face. Let me try again.", block=True),
                                    transitions={    'spoken':'LEARN_OPERATOR_CONTAINER'})


            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                             WAIT_CONTINUE_ITERATOR
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            waitContinueIterator = smach.Iterator(  outcomes=['container_success', 'container_failed'], 
                                                    it = lambda:range(0, 3),
                                                    it_label='waitCounter',
                                                    input_keys=[],
                                                    output_keys=[],
                                                    exhausted_outcome = 'container_failed')
            with waitContinueIterator:

                waitContinueContainer = smach.StateMachine( outcomes = ['container_success', 'heard_nothing'])

                with waitContinueContainer:

                    smach.StateMachine.add("ASK_CONTINUE",
                                            states.AskContinue(robot),
                                            transitions={   'continue':'container_success',
                                                            'no_response':'heard_nothing'})

                smach.Iterator.set_contained_state( 'WAIT_CONTINUE_CONTAINER', 
                                                     waitContinueContainer, 
                                                     # loop_outcomes=['heard_nothing'],
                                                     break_outcomes=['container_success'])

            # add the lookoutIterator to the main state machine
            smach.StateMachine.add( 'WAIT_CONTINUE_ITERATOR',
                                    waitContinueIterator,
                                    {   'container_failed':'SAY_NO_CONTINUE',
                                        'container_success':'FIND_CROWD_CONTAINER'})

            smach.StateMachine.add( 'SAY_NO_CONTINUE',
                                    states.Say(robot, "I didn't hear continue, but I will move on.", block=False),
                                    transitions={   'spoken':'FIND_CROWD_CONTAINER'})


            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                                   FIND_CROWD_CONTAINER
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            # container for this stage
            findCrowndContainer = smach.StateMachine(   outcomes = ['container_success', 'container_failed'])
            with findCrowndContainer:

                smach.StateMachine.add( 'GOTO_LIVING_ROOM_1',
                                        states.NavigateToWaypoint(robot, waypoint_living_room_1),
                                        transitions={   'arrived':          'SAY_SEARCHING_CROWD',
                                                        'unreachable':      'SAY_SEARCHING_CROWD',
                                                        'goal_not_defined': 'SAY_SEARCHING_CROWD'})

                smach.StateMachine.add( 'SAY_SEARCHING_CROWD',
                                        states.Say(robot, "I'm searching for the crowd.", block=False),
                                        transitions={   'spoken':'FIND_CROWD'})

                smach.StateMachine.add( 'FIND_CROWD',
                                        PersonRecStates.FindCrowd(robot, locationsToVisitDes),
                                        transitions={   'succeded':  'CANCEL_HEAD_GOALS_2',
                                                        'failed':   'CANCEL_HEAD_GOALS_2'})

                smach.StateMachine.add( 'CANCEL_HEAD_GOALS_2',
                                        PersonRecStates.CancelHeadGoals(robot),
                                        transitions={    'done':'GOTO_LIVING_ROOM_2'})

                smach.StateMachine.add( 'GOTO_LIVING_ROOM_2',
                                        states.NavigateToWaypoint(robot, waypoint_living_room_2),
                                        transitions={   'arrived':          'FIND_CROWD_2',
                                                        'unreachable':      'FIND_CROWD_2',
                                                        'goal_not_defined': 'FIND_CROWD_2'})

                smach.StateMachine.add( 'FIND_CROWD_2',
                                        PersonRecStates.FindCrowd(robot, locationsToVisitDes),
                                        transitions={   'succeded':  'CANCEL_HEAD_GOALS_3',
                                                        'failed':   'GOTO_LIVING_ROOM_3'})

                smach.StateMachine.add( 'CANCEL_HEAD_GOALS_3',
                                        PersonRecStates.CancelHeadGoals(robot),
                                        transitions={    'done':'container_success'})

                smach.StateMachine.add( 'GOTO_LIVING_ROOM_3',
                                        states.NavigateToWaypoint(robot, waypoint_living_room_3),
                                        transitions={   'arrived':          'FIND_CROWD_3',
                                                        'unreachable':      'FIND_CROWD_3',
                                                        'goal_not_defined': 'FIND_CROWD_3'})

                smach.StateMachine.add( 'FIND_CROWD_3',
                                        PersonRecStates.FindCrowd(robot, locationsToVisitDes),
                                        transitions={   'succeded':  'CANCEL_HEAD_GOALS_4',
                                                        'failed':   'CANCEL_HEAD_GOALS_5'})

                smach.StateMachine.add( 'CANCEL_HEAD_GOALS_4',
                                        PersonRecStates.CancelHeadGoals(robot),
                                        transitions={    'done':'container_success'})

                smach.StateMachine.add( 'CANCEL_HEAD_GOALS_5',
                                        PersonRecStates.CancelHeadGoals(robot),
                                        transitions={    'done':'container_failed'})                

            #add container to the main state machine
            smach.StateMachine.add( 'FIND_CROWD_CONTAINER',
                                    findCrowndContainer,
                                    transitions={   'container_success':'SAY_FOUND_CROWD',
                                                    'container_failed': 'SAY_FAILED_FIND_CROWD'})
                
            smach.StateMachine.add( 'SAY_FOUND_CROWD',
                                    states.Say(robot,[  "I think I found some people.",
                                                        "I think I saw several people over there"], block=False),
                                    transitions={   'spoken':'FIND_OPERATOR_CONTAINER'})

            smach.StateMachine.add('SAY_FAILED_FIND_CROWD',
                                   states.Say(robot,"Still Searching for the crowd", block=False),
                                   transitions={    'spoken':'FIND_CROWD_CONTAINER'})



            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                               FIND_OPERATOR_CONTAINER
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            # container for this stage
            findOperatorContainer = smach.StateMachine( outcomes=['container_success', 'container_failed'])
            with findOperatorContainer:

                smach.StateMachine.add( 'SAY_LOOKING_OPERATOR',
                                        states.Say(robot,"I'm looking for my operator.", block=False),
                                        transitions={   'spoken':'GET_NEXT_LOCATION'})
                
                smach.StateMachine.add( 'GET_NEXT_LOCATION',
                                        PersonRecStates.GetNextLocation(robot, locationsToVisitDes, nextLocationDes),
                                        transitions={   'done':'GOTO_LOCATION',
                                                        'visited_all':'container_success'})

                smach.StateMachine.add( 'GOTO_LOCATION',
                                        states.NavigateToObserve(robot, entity_designator = nextLocationDes.current, radius=1.0),
                                        transitions={   'arrived'           :   'REMOVE_LOCATION',
                                                        'unreachable'       :   'SAY_FAILED_GOTO',
                                                        'goal_not_defined'  :   'SAY_FAILED_GOTO'})

                smach.StateMachine.add('REMOVE_LOCATION',
                                        smach.CBState(removeLocation),
                                        transitions={   'done':'SAY_LOOK_AT_ME'})

                smach.StateMachine.add( 'SAY_LOOK_AT_ME',
                                        states.Say(robot,[  "Please look at me.",
                                                            "Would you look into my camera?",
                                                            "Let me see who is here, please face me."], block=False),
                                        transitions={   'spoken':'ANALYSE_ITERATOR'})

                analyzeIterator = smach.Iterator(  outcomes=['container_success', 'container_failed'], 
                                        it = lambda:range(0, 5),
                                        it_label='counter',
                                        input_keys=[],
                                        output_keys=[],
                                        exhausted_outcome = 'container_failed')
                with analyzeIterator:

                    analyzeIterator = smach.StateMachine( outcomes = ['container_success', 'container_failed'])

                    with analyzeIterator:

                        smach.StateMachine.add("LOOK_AT_PERSON",
                                        PersonRecStates.LookAtPersonInFront(robot, lookDown=True),
                                        transitions={   'succeded':'ANALYZE_PERSON',
                                                        'failed':'ANALYZE_PERSON'})

                        smach.StateMachine.add( 'ANALYZE_PERSON',
                                                PersonRecStates.AnalysePerson(robot, facesAnalyzedDes),
                                                transitions={   'succeded':'CANCEL_HEAD_GOALS',
                                                                'failed':'container_failed'})

                        smach.StateMachine.add( 'CANCEL_HEAD_GOALS',
                                    PersonRecStates.CancelHeadGoals(robot),
                                    transitions={    'done':'container_success'})


                    smach.Iterator.set_contained_state( 'ANALYZE_CONTAINER',
                                                         analyzeIterator,
                                                         loop_outcomes=['container_failed'],
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
                                                            "I guess there is no one here"], block=False),
                                        transitions={   'spoken':'GET_NEXT_LOCATION'})

                smach.StateMachine.add( 'SAY_FAILED_GOTO',
                                        states.Say(robot,[  "I could not go to the chosen location",
                                                            "I can't reach that location",
                                                            "I can't get there"], block=False),
                                        transitions={   'spoken':'GET_NEXT_LOCATION'})

            #add container to the main state machine
            smach.StateMachine.add( 'FIND_OPERATOR_CONTAINER',
                                    findOperatorContainer,
                                    transitions={   'container_success':'DESCRIBE_CROWD_CONTAINER',
                                                    'container_failed':'DESCRIBE_CROWD_CONTAINER'})


            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                             DESCRIBE CROWD AND OPERATOR
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            # container for this stage
            describeCrowdContainer = smach.StateMachine(  outcomes = ['container_success', 'container_failed'])
            with describeCrowdContainer:

                smach.StateMachine.add( 'GET_OPERATOR_LOCATION',
                                        PersonRecStates.GetOperatorLocation(robot, facesAnalyzedDes, operatorNameDes, operatorLocationDes),
                                        remapping={     'operatorIdx_out':'operatorIdx_userData'},
                                        transitions={   'succeeded': 'GOTO_OPERATOR',
                                                        'failed':'SAY_CANT_CHOOSE_OPERATOR'})

                smach.StateMachine.add( 'GOTO_OPERATOR',
                                        states.NavigateToObserve(robot, entity_designator = operatorLocationDes.current, radius = 0.7),
                                        transitions={   'arrived'           :   'SAY_FOUND_OPERATOR',
                                                        'unreachable'       :   'SAY_CANT_REACH',
                                                        'goal_not_defined'  :   'SAY_CANT_REACH'})

                smach.StateMachine.add( 'SAY_FOUND_OPERATOR',
                                        states.Say(robot,"This is my operator!", block=False),
                                        transitions={   'spoken':'POINT_AT_OPERATOR'})

                smach.StateMachine.add( 'SAY_CANT_CHOOSE_OPERATOR',
                                        states.Say(robot,"I'm sorry but I don't know who my operator is.", block=False),
                                        transitions={   'spoken':'DESCRIBE_PEOPLE'})

                smach.StateMachine.add( 'SAY_CANT_REACH',
                                        states.Say(robot,"I could not reach my operator but i will point at anyway.", block=False),
                                        transitions={   'spoken':'POINT_AT_OPERATOR'})

                smach.StateMachine.add( 'POINT_AT_OPERATOR',
                                        PersonRecStates.PointAtOperator(robot),
                                        transitions={   'succeeded':'GREET_OPERATOR',
                                                        'failed':'SAY_CANT_POINT'})

                smach.StateMachine.add( 'SAY_CANT_POINT',
                                        states.Say(robot,"Sorry but i can't point at my operator!", block=False),
                                        transitions={   'spoken':'container_success'})

                

                smach.StateMachine.add( 'GREET_OPERATOR',
                                        states.Say(robot, PersonRecStates.DummyDesig(operatorNameDes), block=False),
                                        transitions={   'spoken':'DESCRIBE_PEOPLE'})

                smach.StateMachine.add( 'DESCRIBE_PEOPLE',
                                        PersonRecStates.DescribePeople(robot, facesAnalyzedDes),
                                        remapping={     'operatorIdx_in':'operatorIdx_userData'},
                                        transitions={   'done':'container_success'})

                # smach.StateMachine.add('SAY_DESCRIBE_CROWD',
                #                         states.Say(robot,"I see several humans in this crowd!", block=False),
                #                         transitions={'spoken':'DESCRIBE_CROWD'})

                # smach.StateMachine.add( 'DESCRIBE_CROWD',
                #                         PersonRecStates.DescribeCrowd(robot, EdEntityDesignator(robot, type='crowd')),
                #                         transitions={   'success':'container_success',
                #                                         'failed':'container_failed'})

            #add container to the main state machine
            smach.StateMachine.add( 'DESCRIBE_CROWD_CONTAINER',
                                    describeCrowdContainer,
                                    transitions={   'container_success':'END_CHALLENGE',
                                                    'container_failed':'END_CHALLENGE'})

            


            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                             END CHALLENGE
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            smach.StateMachine.add('END_CHALLENGE',
                                   states.Say(robot,"My work here is done, goodbye!"),
                                   transitions={'spoken':'Done'})



# ----------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    rospy.init_node('person_recognition_exec')

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "[CHALLENGE PERSON RECOGNITION] Please provide robot name as argument."
        exit(1)

    if robot_name == 'amigo':
        robot = Amigo(wait_services=True)
    elif robot_name == 'sergio':
        robot = Sergio(wait_services=True)
    elif robot_name == 'mockbot':
        robot = Mockbot(wait_services=True)
    else:
        print "[CHALLENGE PERSON RECOGNITION] Don't know robot name " + robot_name

    ''' If necessary: set initial state '''
    rospy.loginfo("Sys.argv = {0}, Length = {1}".format(sys.argv,len(sys.argv)))

    ''' Setup state machine'''
    machine = ChallengePersonRecognition(robot)

    if  len(sys.argv) > 2:
        print PersonRecStates.OUT_PREFIX + PersonRecStates.bcolors.WARNING + "Overriding initial_state to '" + sys.argv[2] +  "'" + PersonRecStates.bcolors.ENDC

        initial_state = [sys.argv[2]]
        machine.set_initial_state(initial_state)


    # for using smach viewer
    introserver = smach_ros.IntrospectionServer('server_name', machine, '/SM_ROOT_PRIMARY')
    introserver.start()

    try:
        machine.execute()
    except Exception, e:
        print "Exception occurred on state machine execution"

    introserver.stop()
