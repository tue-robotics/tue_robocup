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

class LookAtArea(State):
    """ Class to look at the center point of a specific area of an entity
    """
    def __init__(self, robot, entity, area, waittime=2.0):
        """ Constructor
        :param robot: robot object
        :param entity: EdEntityDesignator with the area to look at
        :param area: string with the area to look at
        :param waittime: (optional) waittime (in seconds) between giving a head target and returning
        :return:
        """
        ds.check_type(entity, EntityInfo)

        State.__init__(self, locals(), outcomes=['succeeded', 'failed'])

    def run(self, robot, entity, area, waittime):

        if not entity:
            return 'failed'

        #Entities define their own frame, so there is no need to transform the pose to /map.
        #That would be equivalent to defining coordinates 0,0,0 in its own frame, so that is what we do here.
        #The added benefit is that the entity's frame actually moves because the entity is tracked.
        #This makes the head track the entity
        center_point = Point()
        frame_id = "/"+entity.id

        for testarea in entity.data['areas']:
            ''' See if the area is in the list of inspection areas '''
            if testarea['name'] == area:
                ''' Check if we have a shape '''
                if 'shape' not in testarea:
                    rospy.logwarn("No shape in area {0}".format(area['name']))
                    continue
                ''' Check if length of shape equals one '''
                if not len(testarea['shape']) == 1:
                    rospy.logwarn("Shape of area {0} contains multiple entries, don't know what to do".format(area['name']))
                    continue
                ''' Check if the first entry is a box '''
                if 'box' not in testarea['shape'][0]:
                    rospy.logwarn("No box in {0}".format(area['name']))
                    continue
                box = testarea['shape'][0]['box']
                if 'min' not in box or 'max' not in box:
                    rospy.logwarn("Box in {0} either does not contain min or max".format(area['name']))
                    continue

                center_point.x = 0.5 * (box['min']['x'] + box['max']['x'])
                center_point.y = 0.5 * (box['min']['y'] + box['max']['y'])
                center_point.z = 0.5 * (box['min']['z'] + box['max']['z'])

                rospy.loginfo('Look at %s in frame %s' % (repr(center_point).replace('\n', ' '), frame_id))
                point_stamped = PointStamped(point=center_point,
                                             header=Header(frame_id=frame_id))
                robot.head.look_at_point(point_stamped)
                rospy.sleep(rospy.Duration(waittime))
                return "succeeded"

        rospy.logwarn("Cannot find {0} in {1}".format(area, entity.id))
        return "failed"

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
