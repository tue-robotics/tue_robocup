#! /usr/bin/env python

################################################
# Creator: Luis Ferreira (luisfferreira@outlook.com)
# Date: October 2015
################################################

import roslib; 
import rospy
import sys
import smach
import smach_ros
import robot_smach_states as states
import geometry_msgs.msg as gm
import robot_skills.util.msg_constructors as msgs
from robot_skills.amigo import Amigo
from robot_skills.sergio import Sergio
from robot_skills.mockbot import Mockbot
from robocup_knowledge import load_knowledge

# import designators
# from robot_smach_states.util.designators import EdEntityDesignator, VariableDesignator, DeferToRuntime, analyse_designators

# import states from another file
# import test_states as test_states


############################## INITIALIZATIONS ##############################

# load knowledge
common_knowledge = load_knowledge("common")
# challenge_knowledge = load_knowledge("challenge_test")

# define print shortcuts from common knowledge
printOk, printError, printWarning = common_knowledge.make_prints("[Challenge Test] ")


############################## MAIN STATE MACHINE ##############################

class ChallengeTest(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        with self:

            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                             TEST CONTAINER
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            # container for this stage
            testContainer = smach.StateMachine(outcomes = ['container_success', 'container_failed'])
            with testContainer:

                smach.StateMachine.add('TEST_SPEACH',
                                       states.Say(robot,"Testing speach"),
                                       transitions={'spoken':'container_success'})

            # add container to state machine
            smach.StateMachine.add( 'TEST_CONTAINER',
                                    testContainer,
                                    transitions={   'container_success':'END_CHALLENGE',
                                                    'container_failed': 'END_CHALLENGE'})

            smach.StateMachine.add('END_CHALLENGE',
                                   states.Say(robot,"My work here is done, goodbye!"),
                                   transitions={'spoken':'Done'})


############################## PYTHON ENTRY POINT ##############################

if __name__ == "__main__":
    rospy.init_node('person_recognition_exec')

    # make sure there is an argument with the robot name
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        printError("Please provide robot name as argument.")
        exit(1)

    # load robot name
    if robot_name == 'amigo':
        robot = Amigo(wait_services=True)
    elif robot_name == 'sergio':
        robot = Sergio(wait_services=True)
    elif robot_name == 'mockbot':
        robot = Mockbot(wait_services=True)
    else:
        printError("Unknown robot name: " + robot_name)

    rospy.loginfo("Sys.argv = {0}, Length = {1}".format(sys.argv,len(sys.argv)))

    # initialize state machine
    machine = ChallengeTest(robot)

    # override initial state if there is a second argument with the state name
    if  len(sys.argv) > 2:
        printWarning("Overriding initial_state to '" + sys.argv[2] +  "'")

        initial_state = [sys.argv[2]]
        machine.set_initial_state(initial_state)

    # introserver for using smach viewer
    introserver = smach_ros.IntrospectionServer('server_name', machine, '/SM_ROOT_PRIMARY')
    introserver.start()

    # execute state machine
    try:
        machine.execute()
        printOk("Executive finished")       
    except Exception, e:
        print "Exception occurred on state machine execution"

    introserver.stop()