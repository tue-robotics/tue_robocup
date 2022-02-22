#!/usr/bin/python
import rospy
import smach

from robot_smach_states.util.startup import startup
from robot_smach_states.utility import Initialize


class Hear(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'preempted'],
                             output_keys=['heard'])
        self.robot = robot

    def execute(self, userdata):
        answer = self.robot.hmi.query(description='can i have a coke?',
                                      grammar="T -> DET coke \n DET -> a",
                                      target='T',
                                      timeout=60)
        if answer:
            rospy.loginfo('I heard:' + answer.sentence)
        userdata.heard = answer.sentence
        return 'succeeded'


class GetEntitysByType(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['type'],
                             output_keys=['entity'])
        self.robot = robot

    def execute(self, userdata):
        type = userdata.type

        rospy.loginfo('I\'m going to search for {}'.format(type))
        # call robot.ed.get_closest_entity()
        entity = self.robot.ed.get_closest_entity(etype=userdata.heard)
        #entity = {'id': 'coke-1'}

        userdata.entity = entsity
        return 'succeeded'


def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['succeeded', 'failed', 'preempted'])

    with sm:
        smach.StateMachine.add('INITIALIZE',
                               Initialize(robot),
                               transitions={'initialized': 'HEAR',
                                            'abort'      : 'preempted'})

        smach.StateMachine.add('HEAR',
                               Hear(robot),
                               transitions={'succeeded': 'SEARCH',
                                            'failed'   : 'failed',
                                            'preempted': 'preempted'},
                                remapping={ 'heared'   : 'entity_type'})

        smach.StateMachine.add('SEARCH',
                               GetEntitysByType(robot),
                               transitions={'succeeded': 'succeeded',
                                            'failed'   : 'failed'},
                               remapping={  'type'     : 'entity_type'})

    return sm

if __name__ == "__main__":
    rospy.init_node('amigo_final_rwc2015_exec')
    startup(setup_statemachine)
