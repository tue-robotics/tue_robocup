#! /usr/bin/env python
import sys
import rospy
import smach

from robot_smach_states.state import State
from robot_smach_states.util.designators import check_type, EdEntityDesignator
from ed.msg import EntityInfo


class LookAtEntity(State):
    def __init__(self, robot, entity):
        check_type(entity, EntityInfo)

        State.__init__(self, locals(), outcomes=['succeeded'])

    def run(self, robot, entity):
        return "succeeded"


# Testing

def setup_statemachine(robot):
    entity = EdEntityDesignator(robot, id='coke-1')

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])
    with sm:
        smach.StateMachine.add('TEST',
                               LookAtEntity(robot, entity),
                               transitions={'succeeded': 'Done'})
    return sm

if __name__ == "__main__":
    import doctest
    doctest.testmod()

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "Please provide robot name as argument."
        exit(1)

    rospy.init_node('manipulation_exec')
    from robot_smach_states.util.startup import startup
    startup(setup_statemachine, robot_name=robot_name)
