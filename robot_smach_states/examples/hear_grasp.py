#!/usr/bin/python
import rospy
import smach

from robot_smach_states.util.startup import startup
from robot_smach_states import Initialize


class Hear(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'preempted'],
                             output_keys=['heared'])
        self.robot = robot

    def execute(self, userdata):
        result = self.robot.ears.recognize(spec='Bring me a <drink>',
                                           choices={'drink': ['coke', 'fanta']},
                                           time_out=rospy.Duration(60))
        choices = result.choices['drink'] if result else None
        if choices:
            rospy.loginfo('I heared:' + repr(choices))
        userdata.heared = choices
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
        entity = {'id': 'coke-1'}

        userdata.entity = entity
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
