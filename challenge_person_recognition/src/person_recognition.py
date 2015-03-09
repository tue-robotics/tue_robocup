#! /usr/bin/env python

import roslib; 
import rospy
import sys
import smach
import smach_ros
import person_recognition_states as PersonRecStates
import robot_smach_states as states
import ed_perception.msg

from actionlib import *
from actionlib.msg import *

from robot_skills.amigo import Amigo
from robot_skills.sergio import Sergio
from smach_ros import SimpleActionState

from ed_perception.srv import LearnPerson, LearnPersonRequest
from robot_smach_states.util.designators import EdEntityDesignator


# ----------------------------------------------------------------------------------------------------

class ChallengePersonRecognition(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        waypoint_learning = EdEntityDesignator(robot, id="person_rec_learning")
        waypoint_living_room_1 = EdEntityDesignator(robot, id="person_rec_living_room_1")
        waypoint_living_room_2 = EdEntityDesignator(robot, id="person_rec_living_room_2")
        waypoint_living_room_3 = EdEntityDesignator(robot, id="person_rec_living_room_3")

        learning_goal = ed_perception.msg.FaceLearningGoal("Mister Operator")
        learning_result = ed_perception.msg.FaceLearningResult()

        with self:

            smach.StateMachine.add( 'START_CHALLENGE',
                                    states.StartChallengeRobust(robot, 'initial_pose'),
                                    transitions={   'Done':'GOTO_ENTRY',
                                                    'Aborted':'Aborted',
                                                    'Failed':'GOTO_ENTRY'})

            # smach.StateMachine.add('SAY_GOTO_ENTRY',
            #                         Say(robot, [    "Going to the entrance.",
            #                                         "Entering the room.",
            #                                         "Here we go."], block=False),
            #                         transitions={   'spoken':'GOTO_ENTRY'})

            smach.StateMachine.add('GOTO_ENTRY',
                                    states.NavigateToWaypoint(robot, waypoint_learning),
                                    transitions={   'arrived':'LEARN_OPERATOR_CONTAINER',
                                                    'unreachable':'LEARN_OPERATOR_CONTAINER',
                                                    'goal_not_defined':'LEARN_OPERATOR_CONTAINER'})


            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                                 LEARN_OPERATOR_CONTAINER
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            # container for this stage
            learnOperatorContainer = smach.StateMachine(outcomes = ['container_success', 'container_failed'])
            with learnOperatorContainer:

                smach.StateMachine.add("SAY_WAITING_OPERATOR",
                                        states.Say(robot,[  "I'm waiting for the operator to stand in front of me.",
                                                            "Would the operator please come forward.",
                                                            "I need an operator, please stand in front of me."], block=False),
                                        transitions={   'spoken':'LOOK_AT_OPERATOR'})

                smach.StateMachine.add('LOOK_AT_OPERATOR',
                                        PersonRecStates.LookAtPersonInFront(robot),
                                        transitions={   'succeded':'WAIT_FOR_OPERATOR',
                                                        'failed':'WAIT_FOR_OPERATOR'})

                smach.StateMachine.add("WAIT_FOR_OPERATOR",
                                        states.WaitForPersonInFront(robot, attempts=8, sleep_interval=1),
                                        transitions={   'success':'ASK_PERSON_NAME',
                                                        'failed':'SAY_WAITING_OPERATOR'})

                smach.StateMachine.add('ASK_PERSON_NAME',
                                        PersonRecStates.AskPersonName(robot),
                                        remapping={     'personName_out':'personName_userData'},
                                        transitions={   'succeded':'LOOK_AT_OPERATOR_2',
                                                        'failed':'SAY_LEARN_NAME_FAILED'})

                smach.StateMachine.add('LOOK_AT_OPERATOR_2',
                                        PersonRecStates.LookAtPersonInFront(robot),
                                        transitions={   'succeded':'LEARN_PERSON',
                                                        'failed':'LEARN_PERSON'})

                smach.StateMachine.add('LEARN_PERSON',
                                        PersonRecStates.LearnPerson(robot),
                                        remapping={     'personName_in':'personName_userData'},
                                        transitions={   'succeded_learning':'SAY_OPERATOR_LEARNED',
                                                        'failed_learning':'SAY_LEARN_FACE_FAILED'})

                smach.StateMachine.add('SAY_LEARN_NAME_FAILED',
                                       states.Say(robot, [  "I did not understand your name, could you repeat it?",
                                                            "Could you repeat your name?"]),
                                       transitions={    'spoken':'ASK_PERSON_NAME'})

                smach.StateMachine.add('SAY_LEARN_FACE_FAILED',
                                       states.Say(robot,"I could not learn your face for some reason. Let's try again.", block=False),
                                       transitions={    'spoken':'LOOK_AT_OPERATOR'})

                smach.StateMachine.add('SAY_OPERATOR_LEARNED',
                                       states.Say(robot,"Now i know how you look like. Please go mix with the crowd."),
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
                                                     loop_outcomes=['heard_nothing'],
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
            findCrowndContainer = smach.StateMachine(  outcomes = ['container_success', 'container_failed'])
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
                                        PersonRecStates.FindCrowd(robot),
                                        transitions={   'success':  'SAY_FOUND_CROWD',
                                                        'failed':   'GOTO_LIVING_ROOM_2'})

                smach.StateMachine.add( 'GOTO_LIVING_ROOM_2',
                                        states.NavigateToWaypoint(robot, waypoint_living_room_2),
                                        transitions={   'arrived':          'FIND_CROWD_2',
                                                        'unreachable':      'FIND_CROWD_2',
                                                        'goal_not_defined': 'FIND_CROWD_2'})

                smach.StateMachine.add( 'FIND_CROWD_2',
                                        PersonRecStates.FindCrowd(robot),
                                        transitions={   'success':  'SAY_FOUND_CROWD',
                                                        'failed':   'GOTO_LIVING_ROOM_3'})

                smach.StateMachine.add( 'GOTO_LIVING_ROOM_3',
                                        states.NavigateToWaypoint(robot, waypoint_living_room_2),
                                        transitions={   'arrived':          'FIND_CROWD_3',
                                                        'unreachable':      'FIND_CROWD_3',
                                                        'goal_not_defined': 'FIND_CROWD_3'})

                smach.StateMachine.add( 'FIND_CROWD_3',
                                        PersonRecStates.FindCrowd(robot),
                                        transitions={   'success':  'SAY_FOUND_CROWD',
                                                        'failed':   'container_failed'})

                smach.StateMachine.add( 'SAY_FOUND_CROWD',
                                        states.Say(robot,[  "I think I found it.",
                                                            "I think I saw several people over there"], block=False),
                                        transitions={   'spoken':'GOTO_CROWD'})


                smach.StateMachine.add('GOTO_CROWD',
                                        states.NavigateToObserve(robot, EdEntityDesignator(robot, type="crowd"), radius=1.5),
                                        transitions={   'arrived':          'container_success',
                                                        'unreachable':      'container_success',
                                                        'goal_not_defined': 'container_success'})
            #add container to the main state machine
            smach.StateMachine.add( 'FIND_CROWD_CONTAINER',
                                    findCrowndContainer,
                                    transitions={   'container_success':'FIND_OPERATOR_CONTAINER',
                                                    'container_failed': 'SAY_FAILED_FIND_CROWD'})

            smach.StateMachine.add('SAY_FAILED_FIND_CROWD',
                                   states.Say(robot,"Still Searching for the crowd", block=False),
                                   transitions={    'spoken':'FIND_CROWD_CONTAINER'})



            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                             FIND OPERATOR
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            # container for this stage
            findOperatorContainer = smach.StateMachine(  outcomes = ['container_success', 'container_failed'])
            with findOperatorContainer:

                smach.StateMachine.add( 'SAY_LOOKING_OPERATOR',
                                        states.Say(robot,"I'm looking for my operator.", block=False),
                                        transitions={   'spoken':'SAY_FOUND_OPERATOR'})


                smach.StateMachine.add( 'SAY_FOUND_OPERATOR',
                                        states.Say(robot,"Hey, I found my operator!", block=False),
                                        transitions={   'spoken':'POINT_AT_OPERATOR'})

                smach.StateMachine.add( 'POINT_AT_OPERATOR',
                                        PersonRecStates.PointAtOperator(robot),
                                        transitions={   'success':'container_success',
                                                        'failed':'SAY_CANT_POINT'})

                smach.StateMachine.add( 'SAY_CANT_POINT',
                                        states.Say(robot,"Sorry but i can't point at my operator!", block=False),
                                        transitions={   'spoken':'container_success'})

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

                smach.StateMachine.add( 'GREET_OPERATOR',
                                        states.Say(robot,"Hello Mister Operator", block=False),
                                        transitions={   'spoken':'DESCRIBE_OPERATOR'})

                smach.StateMachine.add( 'DESCRIBE_OPERATOR',
                                        PersonRecStates.DescribeOperator(robot),
                                        transitions={   'success':'SAY_DESCRIBE_CROWD',
                                                        'failed':'container_failed'})

                smach.StateMachine.add('SAY_DESCRIBE_CROWD',
                                        states.Say(robot,"I see several humans in this crowd!", block=False),
                                        transitions={'spoken':'DESCRIBE_CROWD'})

                smach.StateMachine.add( 'DESCRIBE_CROWD',
                                        PersonRecStates.DescribeCrowd(robot, EdEntityDesignator(robot, type='crowd')),
                                        transitions={   'success':'container_success',
                                                        'failed':'container_failed'})

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
