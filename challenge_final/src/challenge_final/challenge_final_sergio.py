#!/usr/bin/python

import rospy
import smach
import sys
import random
import math
import smach
import smach_ros
from robot_smach_states.util.designators import *
import robot_smach_states as states
from robot_smach_states.util.startup import startup
from robot_skills.amigo import Amigo
from robot_skills.sergio import Sergio
from robot_skills.mockbot import Mockbot
from robocup_knowledge import load_knowledge


class FinalChallengeSergio(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])


        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        #                                   VARIABLES
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        
        challenge_knowledge = load_knowledge('challenge_final')
        initial_pose = challenge_knowledge.initial_pose_sergio
        waypoint_list = challenge_knowledge.explore_locations_part_1

        # next_loc_des = VariableDesignator(resolve_type=EdEntityDesignator)
        # next_loc_des.current = EdEntityDesignator(robot)
        next_loc_des  = VariableDesignator(resolve_type=PointDesignator) #Here next_loc_des is a VaribleDesigntor or
        next_loc_des.current = PointDesignator() # To which you assign another designator??
    

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        #                                   CALLBACKS
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        @smach.cb_interface(outcomes=['successed', 'failed'])
        def getNextWaypoint(userdata):
            # import ipdb; ipdb.set_trace()
            print "removeWaypoint callback"

            if waypoint_list:
                print "Waypoints available: " + str(waypoint_list)

                next_loc_des = EdEntityDesignator(robot, id=waypoint_list.pop(0)) 


                return 'successed'
            else:
                print "No more waypoints to visit!"
                return 'failed'



        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        #                                   MAIN STATE MACHINE
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        with self:

            smach.StateMachine.add( 'START_CHALLENGE',
                                    states.StartChallengeRobust(robot, initial_pose),
                                    transitions={   'Done':'EXPLORE_CONTAINER',
                                                    'Aborted':'Aborted',
                                                    'Failed':'EXPLORE_CONTAINER'})

            # smach.StateMachine.add('INITIALIZE',
            #                     states.Initialize(robot),
            #                     transitions={   'initialized':'LEARN_OPERATOR_CONTAINER',
            #                                     'abort':'Aborted'})


            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                                 EXPLORE_CONTAINER
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


            # container for this stage
            wakeupContainer = smach.StateMachine(outcomes = ['container_successed', 'container_failed'])

            with wakeupContainer:

                smach.StateMachine.add('GET_NEXT_WAYPOINT',
                                        smach.CBState(getNextWaypoint),
                                        transitions={   'successed':'GOTO_LOCATION',
                                                        'failed':'container_successed'})

                smach.StateMachine.add( 'GOTO_LOCATION',
                                        states.NavigateToWaypoint(robot, next_loc_des.resolve()),
                                        transitions={   'arrived':          'GET_NEXT_WAYPOINT',
                                                        'unreachable':      'GET_NEXT_WAYPOINT',
                                                        'goal_not_defined': 'GET_NEXT_WAYPOINT'})

            smach.StateMachine.add( 'EXPLORE_CONTAINER',
                                    wakeupContainer,
                                    transitions={   'container_successed':'END_CHALLENGE',
                                                    'container_failed': 'END_CHALLENGE'})


            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                             END CHALLENGE
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            smach.StateMachine.add('END_CHALLENGE',
                                   states.Say(robot,"My work here is done, goodbye!"),
                                   transitions={'spoken':'Done'})


############################## initializing program ######################

if __name__ == '__main__':
    rospy.init_node('wakemeup_exec')


    # if len(sys.argv) > 1:
    #     robot_name = sys.argv[1]
    # else:
    #     print "[CHALLENGE MANIPULATION] Please provide robot name as argument."
    #     exit(1)

    # Force robot name
    robot_name = 'sergio'

    if robot_name == 'amigo':
        robot = Amigo(wait_services=True)
    elif robot_name == 'sergio':
        robot = Sergio(wait_services=True)
    elif robot_name == 'mockbot':
        robot = Mockbot(wait_services=True)
    else:
        print "Don't recognize that robot name: " + robot_name

    ''' If necessary: set initial state '''
    rospy.loginfo("Sys.argv = {0}, Length = {1}".format(sys.argv,len(sys.argv)))

    ''' Setup state machine'''
    machine = FinalChallengeSergio(robot)

    # Force initial state
    if  len(sys.argv) > 1:
        print "Overriding initial_state to '" + sys.argv[1] +  "'" 
        initial_state = [sys.argv[1]]
        machine.set_initial_state(initial_state)

    # for using smach viewer
    introserver = smach_ros.IntrospectionServer('server_name', machine, '/SM_ROOT_PRIMARY')
    introserver.start()

    try:
        machine.execute()
    except Exception, e:
        print "Exception occurred on state machine execution"

    introserver.stop()