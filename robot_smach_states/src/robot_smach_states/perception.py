from __future__ import absolute_import, print_function

# System
import sys

# ROS
from pykdl_ros import VectorStamped
import rospy
import smach

# TU/e Robotics
from ed.entity import Entity
from .state import State
from .util import designators as ds


class LookAtEntity(smach.State):
    def __init__(self, robot, entity, waittime=0.0, height=None):
        """
        :param robot: The robot
        :param entity: The entity to look at
        :param waittime: Amount of time the robot look at the entity
        :param height: Look higher up in the entity
        """
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self._robot = robot
        self._entity = entity
        self._waittime = waittime
        self._height = height if height is not None else 0

        ds.check_type(entity, Entity)

    def execute(self, userdata=None):

        entity = self._entity.resolve() if hasattr(self._entity, 'resolve') else self._entity
        height = self._height.resolve() if hasattr(self._height, 'resolve') else self._height

        if not entity:
            return 'failed'

        # Entities define their own frame, so there is no need to transform the pose to map.
        # That would be equivalent to defining coordinates 0,0,0 in its own frame, so that is what we do here.
        # The added benefit is that the entity's frame actually moves because the entity is tracked.
        # This makes the head track the entity
        vs = VectorStamped.from_xyz(0, 0, height, entity.last_update_time, frame_id=entity.uuid)
        rospy.loginfo(f'Look at {vs}')
        self._robot.head.look_at_point(vs)
        rospy.sleep(rospy.Duration(self._waittime))
        return "succeeded"


class LookAtArea(State):
    """
    Class to look at the center point of a specific area of an entity
    """
    def __init__(self, robot, entity, area, waittime=2.0):
        """
        Constructor

        :param robot: robot object
        :param entity: EdEntityDesignator with the area to look at
        :param area: string with the area to look at
        :param waittime: (optional) waittime (in seconds) between giving a head target and returning
        """
        ds.check_type(entity, Entity)

        State.__init__(self, locals(), outcomes=['succeeded', 'failed'])

    def run(self, robot, entity, area, waittime):

        if not entity:
            return 'failed'

        # Entities define their own frame, so there is no need to transform the pose to map.
        # That would be equivalent to defining coordinates 0,0,0 in its own frame, so that is what we do here.
        # The added benefit is that the entity's frame actually moves because the entity is tracked.
        # This makes the head track the entity
        frame_id = entity.uuid

        if area in entity.volumes:
            cp = entity.volumes[area].center_point
            vs = VectorStamped.from_xyz(cp.x(), cp.y(), cp.z(), entity.last_update_time, frame_id)

            rospy.loginfo('Look at %s' % (repr(vs)))

            # This is awefully hardcoded for AMIGO!!! (TODO)
            height = min(0.4, max(0.1, vs.vector.z()-0.55))
            robot.torso._send_goal([height], timeout=0)

            robot.head.look_at_point(vs, timeout=0)

            robot.head.wait_for_motion_done(timeout=5)
            robot.torso.wait_for_motion_done(timeout=5)

            rospy.sleep(rospy.Duration(waittime))
            return "succeeded"

        rospy.logwarn("Cannot find {0} in {1}".format(area, entity.uuid))
        return "failed"


# Testing
def setup_statemachine(robot):
    entity = ds.EntityByIdDesignator(robot, uuid='hallway_couch')

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
        print("Please provide robot name as argument.")
        exit(1)

    rospy.init_node('manipulation_exec')
    from robot_smach_states.util.startup import startup
    startup(setup_statemachine, robot_name=robot_name)
