#! /usr/bin/env python
import sys
import rospy
import smach

from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped
from ed.msg import EntityInfo

from robot_smach_states.state import State
from robot_smach_states.util.designators import check_type, EdEntityDesignator


class LookAtEntity(State):
    def __init__(self, robot, entity, keep_following=False):
        check_type(entity, EntityInfo)

        State.__init__(self, locals(), outcomes=['succeeded'])

    def run(self, robot, entity, keep_following=False):
        if keep_following:
            center_point = Point()
            frame_id = '/'+entity
        else:
            center_point = entity.center_point
            frame_id = "/map"

        rospy.loginfo('Look at %s in frame %s' % (repr(center_point).replace('\n', ' '), frame_id))
        point_stamped = PointStamped(point=center_point,
                                     header=Header(frame_id=frame_id))
        robot.head.look_at_point(point_stamped)
        return "succeeded"


# Testing

def setup_statemachine(robot):
    entity = EdEntityDesignator(robot, id='hallway_couch')

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
