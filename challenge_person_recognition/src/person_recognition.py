#! /usr/bin/env python
import rospy
import sys

import person_recognition_states

from robot_smach_states.highlevel import StartChallengeRobust
from robot_smach_states.navigation import NavigateToObserve
from robot_smach_states.navigation import NavigateToWaypoint
from robot_smach_states.human_interaction import Say

from robot_skills.amigo import Amigo
from robot_skills.sergio import Sergio

from robot_smach_states.designators.designator import Designator, VariableDesignator

import smach
import smach_ros


# ----------------------------------------------------------------------------------------------------

class ChallengePersonRecognition(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        with self:

            smach.StateMachine.add( "START_CHALLENGE",
                                    StartChallengeRobust(robot, "initial_pose"),
                                    transitions={   "Done":"SAY_GO_TO_EXIT",
                                                    "Aborted":"Aborted",
                                                    "Failed":"SAY_GO_TO_EXIT"})

            smach.StateMachine.add("SAY_GO_TO_EXIT",
                                    Say(robot, [ "I will now go to the exit"]),
                                    transitions={   'spoken':'GO_TO_EXIT'})

            smach.StateMachine.add('GO_TO_EXIT',
                                    NavigateToWaypoint(robot, Designator("exit")),
                                    transitions={   "arrived":"END_CHALLENGE",
                                                    "unreachable":'Aborted',
                                                    "goal_not_defined":'Aborted'})

            smach.StateMachine.add("END_CHALLENGE",
                                   Say(robot,"I finished this challenge, goodbye!"),
                                   transitions={'spoken':'Done'})

            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


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
