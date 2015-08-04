#!/usr/bin/python

import sys
import rospy
import smach

import robot_smach_states as states
import timer_states as timer

from robot_skills.amigo import Amigo
from robot_skills.sergio import Sergio
from robot_skills.mockbot import Mockbot
import smach_ros

class TestPresentationTimer(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        with self:
            smach.StateMachine.add( 'SAY_TIME',
                                    timer.SayRemainingTime(robot,block=True),
                                    transitions={   'done':'START_TIMER',
                                                    'failed':'START_TIMER'})

            smach.StateMachine.add( 'START_TIMER',
                                    timer.StartPresentationTimer(robot,mins=10),
                                    transitions={   'done':'WAIT_A_WHILE',
                                                    'failed':'WAIT_A_WHILE'})

            smach.StateMachine.add( 'WAIT_A_WHILE',
                                    states.WaitTime(robot,10.0),
                                    transitions={   'waited':'SAY_TIME_LEFT',
                                                    'preempted':'SAY_TIME_LEFT'})

            smach.StateMachine.add( 'SAY_TIME_LEFT',
                                    timer.SayRemainingTime(robot,block=True),
                                    transitions={   'done':'Done',
                                                    'failed':'Done'})

if __name__ == '__main__':
    rospy.init_node('test_presentation_timer_exec')

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "[TEST PRESENTATION TIMER] Please provide robot name as argument."
        exit(1)

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
    machine = TestPresentationTimer(robot)

    # for using smach viewer
    introserver = smach_ros.IntrospectionServer('server_name', machine, '/SM_ROOT_PRIMARY')
    introserver.start()

    try:
        machine.execute()
    except Exception, e:
        print "Exception occurred on state machine execution"

    introserver.stop()