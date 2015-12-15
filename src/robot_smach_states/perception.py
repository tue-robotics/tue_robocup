#! /usr/bin/env python
import sys
import rospy
import smach

from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped
from ed.msg import EntityInfo

from robot_smach_states.state import State
import robot_smach_states.util.designators as ds


class LookAtEntity(State):
    def __init__(self, robot, entity, keep_following=False, waittime=0.0):
        ds.check_type(entity, EntityInfo)

        State.__init__(self, locals(), outcomes=['succeeded', 'failed'])

    def run(self, robot, entity, keep_following, waittime):
        if keep_following:
            rospy.logerr("Look at stuff: keep_following is obsolete")

        if not entity:
            return 'failed'

        #Entities define their own frame, so there is no need to transform the pose to /map.
        #That would be equivalent to defining coordinates 0,0,0 in its own frame, so that is what we do here.
        #The added benefit is that the entity's frame actually moves because the entity is tracked.
        #This makes the head track the entity
        center_point = Point()
        frame_id = "/"+entity.id

        rospy.loginfo('Look at %s in frame %s' % (repr(center_point).replace('\n', ' '), frame_id))
        point_stamped = PointStamped(point=center_point,
                                     header=Header(frame_id=frame_id))
        robot.head.look_at_point(point_stamped)
        rospy.sleep(rospy.Duration(waittime))
        return "succeeded"

class LookOnTopOfEntity(State):
    def __init__(self, robot, entity, keep_following=False, waittime=0.0):
        ds.check_type(entity, EntityInfo)

        State.__init__(self, locals(), outcomes=['succeeded', 'failed'])

    def run(self, robot, entity, keep_following, waittime):
        if keep_following:
            rospy.logerr("Look at stuff: keep_following is obsolete")

        if not entity:
            return 'failed'

        #Entities define their own frame, so there is no need to transform the pose to /map.
        #That would be equivalent to defining coordinates 0,0,0 in its own frame, so that is what we do here.
        #The added benefit is that the entity's frame actually moves because the entity is tracked.
        #This makes the head track the entity
        center_point = Point()
        frame_id = "/"+entity.id

        center_point.z = entity.z_max

        rospy.loginfo('Look at %s in frame %s' % (repr(center_point).replace('\n', ' '), frame_id))
        point_stamped = PointStamped(point=center_point,
                                     header=Header(frame_id=frame_id))
        robot.head.look_at_point(point_stamped)
        rospy.sleep(rospy.Duration(waittime))
        return "succeeded"

# Testing

def setup_statemachine(robot):
    entity = ds.EntityByIdDesignator(robot, id='hallway_couch')

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
