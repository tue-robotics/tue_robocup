#! /usr/bin/env python
import rospy
import sys

import person_recognition_states as PersonRecStates

from robot_smach_states.highlevel import StartChallengeRobust
from robot_smach_states.navigation import NavigateToObserve
from robot_smach_states.navigation import NavigateToWaypoint
from robot_smach_states.human_interaction import Say

from robot_skills.amigo import Amigo
from robot_skills.sergio import Sergio

from robot_smach_states.util.designators import Designator, VariableDesignator

import smach
import smach_ros




# ----------------------------------------------------------------------------------------------------

class ChallengePersonRecognition(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        with self:

            smach.StateMachine.add( 'START_CHALLENGE',
                                    StartChallengeRobust(robot, 'initial_pose'),
                                    transitions={   'Done':'SAY_GO_TO_ENTRY',
                                                    'Aborted':'Aborted',
                                                    'Failed':'SAY_GO_TO_ENTRY'})

            smach.StateMachine.add('SAY_GO_TO_ENTRY',
                                    Say(robot, [    "Going to the entrance.",
                                                    "Entering the room.",
                                                    "Here we go."], block=False),
                                    transitions={   'spoken':'GO_TO_ENTRY'})

            smach.StateMachine.add('GO_TO_ENTRY',
                                    NavigateToWaypoint(robot, Designator('person_rec_learning')),
                                    transitions={   'arrived':'LEARN_OPERATOR_CONTAINER',
                                                    'unreachable':'LEARN_OPERATOR_CONTAINER',
                                                    'goal_not_defined':'LEARN_OPERATOR_CONTAINER'})



            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                             LEARN OPERATOR
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            # container for this stage
            learnOperatorContainer = smach.StateMachine(  outcomes = ['success_container', 'failed_container'])
            with learnOperatorContainer:

                smach.StateMachine.add("SAY_WAITING_OPERATOR",
                                       Say(robot,"I will wait here for my operator.", block=False),
                                       transitions={'spoken':'WAIT_FOR_OPERATOR'})

                smach.StateMachine.add('WAIT_FOR_OPERATOR',
                                       PersonRecStates.WaitForOperator(robot),
                                       transitions={'success':'LEARN_PERSON',
                                                    'failed':'failed_container'})

                smach.StateMachine.add('LEARN_PERSON',
                                       PersonRecStates.LearnPerson(robot, name="Mister Operator"),
                                       transitions={'success':'SAY_OPERATOR_LEARNED',
                                                    'failed':'failed_container'})

                smach.StateMachine.add('SAY_OPERATOR_LEARNED',
                                       Say(robot,"Now i know how you look like. Please go mix with the crowd."),
                                       transitions={'spoken':'success_container'})


            #add container to the main state machine
            smach.StateMachine.add( 'LEARN_OPERATOR_CONTAINER',
                                    learnOperatorContainer,
                                    transitions={   'success_container':'FIND_CROWD_CONTAINER',
                                                    'failed_container':'SAY_FAILED_LEARNING'})

            smach.StateMachine.add('SAY_FAILED_LEARNING',
                                   Say(robot,"Could not learn my operator's face. Let's try that again.", block=False),
                                   transitions={    'spoken':'LEARN_OPERATOR_CONTAINER'})




            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                             FIND CROWD
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            # container for this stage
            findCrowndContainer = smach.StateMachine(  outcomes = ['success_container', 'failed_container'])
            with findCrowndContainer:

                smach.StateMachine.add( 'GO_TO_LIVING_ROOM_1',
                                        NavigateToWaypoint(robot, Designator('person_rec_living_room_1')),
                                        transitions={   'arrived':'SAY_SEARCHING_CROWD',
                                                        'unreachable':'SAY_SEARCHING_CROWD',
                                                        'goal_not_defined':'SAY_SEARCHING_CROWD'})

                smach.StateMachine.add( 'SAY_SEARCHING_CROWD',
                                        Say(robot,"I'm searching for the crowd.", block=False),
                                        transitions={   'spoken':'FIND_CROWD'})

                smach.StateMachine.add( 'FIND_CROWD',
                                        PersonRecStates.FindCrowd(robot, Designator('crowd')),
                                        transitions={   'success':'SAY_FOUND_CROWD',
                                                        'failed':'GO_TO_LIVING_ROOM_2'})

                smach.StateMachine.add( 'GO_TO_LIVING_ROOM_2',
                                        NavigateToWaypoint(robot, Designator('person_rec_living_room_2')),
                                        transitions={   'arrived':'FIND_CROWD_2',
                                                        'unreachable':'FIND_CROWD_2',
                                                        'goal_not_defined':'FIND_CROWD_2'})

                smach.StateMachine.add( 'FIND_CROWD_2',
                                        PersonRecStates.FindCrowd(robot, Designator('crowd')),
                                        transitions={   'success':'SAY_FOUND_CROWD',
                                                        'failed':'failed_container'})

                smach.StateMachine.add( 'SAY_FOUND_CROWD',
                                        Say(robot,"I think I found it.", block=False),
                                        transitions={   'spoken':'success_container'})


            #add container to the main state machine
            smach.StateMachine.add( 'FIND_CROWD_CONTAINER',
                                    findCrowndContainer,
                                    transitions={   'success_container':'FIND_OPERATOR_CONTAINER',
                                                    'failed_container':'SAY_FAILED_FIND_CROWD'})

            smach.StateMachine.add('SAY_FAILED_FIND_CROWD',
                                   Say(robot,"Still Searching for the crowd", block=False),
                                   transitions={    'spoken':'FIND_CROWD_CONTAINER'})



            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                             FIND OPERATOR
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            # container for this stage
            findOperatorContainer = smach.StateMachine(  outcomes = ['success_container', 'failed_container'])
            with findOperatorContainer:

                smach.StateMachine.add('SAY_LOOKING_OPERATOR',
                                        Say(robot,"I'm looking for my operator.", block=False),
                                        transitions={'spoken':'SAY_FOUND_OPERATOR'})


                smach.StateMachine.add('SAY_FOUND_OPERATOR',
                                        Say(robot,"I found my operator!", block=False),
                                        transitions={'spoken':'success_container'})


            #add container to the main state machine
            smach.StateMachine.add( 'FIND_OPERATOR_CONTAINER',
                                    findOperatorContainer,
                                    transitions={   'success_container':'DESCRIBE_CROWD_CONTAINER',
                                                    'failed_container':'DESCRIBE_CROWD_CONTAINER'})



            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                             DESCRIBE CROWD AND OPERATOR
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            # container for this stage
            describeCrowdContainer = smach.StateMachine(  outcomes = ['success_container', 'failed_container'])


            with describeCrowdContainer:

                smach.StateMachine.add( 'GREET_OPERATOR',
                                        Say(robot,"Hello Mister Operator", block=False),
                                        transitions={   'spoken':'DESCRIBE_OPERATOR'})

                smach.StateMachine.add( 'DESCRIBE_OPERATOR',
                                        PersonRecStates.DescribeOperator(robot, Designator('operator')),
                                        transitions={   'success':'SAY_DESCRIBE_CROWD',
                                                        'failed':'failed_container'})

                smach.StateMachine.add('SAY_DESCRIBE_CROWD',
                                        Say(robot,"I see several humans in this crowd!", block=False),
                                        transitions={'spoken':'DESCRIBE_CROWD'})

                smach.StateMachine.add( 'DESCRIBE_CROWD',
                                        PersonRecStates.DescribeCrowd(robot, Designator('crowd')),
                                        transitions={   'success':'success_container',
                                                        'failed':'failed_container'})

            #add container to the main state machine
            smach.StateMachine.add( 'DESCRIBE_CROWD_CONTAINER',
                                    describeCrowdContainer,
                                    transitions={   'success_container':'END_CHALLENGE',
                                                    'failed_container':'END_CHALLENGE'})



            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                             END CHALLENGE
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            smach.StateMachine.add('END_CHALLENGE',
                                   Say(robot,"My work here is done, goodbye!"),
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
    # if  len(sys.argv) > 2:
    #     if int(sys.argv[2]) == 1:
    #         initial_state = ["INIT_PICK_AND_PLACE"]
    #     elif int(sys.argv[2]) == 2:
    #         initial_state = ["INIT_AVOID_THAT"]
    #     elif int(sys.argv[2]) == 3:
    #         initial_state = ["INIT_WHAT_DID_YOU_SAY"]

    ''' Setup state machine'''
    machine = ChallengePersonRecognition(robot)
    if  len(sys.argv) > 2:
        #initial_state = [str(sys.argv[1])]
        rospy.logwarn("Setting initial state to {0}, please make sure the reasoner is reset and the robot is localized correctly".format(initial_state))
        machine.set_initial_state(initial_state)

    # for using smach viewer
    introserver = smach_ros.IntrospectionServer('server_name', machine, '/SM_ROOT_PRIMARY')
    introserver.start()

    try:
        machine.execute()
    except Exception, e:
        amigo.speech.speak(e)

    introserver.stop()
