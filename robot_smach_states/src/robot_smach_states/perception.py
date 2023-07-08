from __future__ import absolute_import, print_function

# System
import sys
import numpy as np

# ROS
from pykdl_ros import VectorStamped
import rospy
import smach

# TU/e Robotics
from ed.entity import Entity
from .state import State
from .util import designators as ds


class LookAtEntity(smach.State):
    def __init__(self, robot, entity, waittime=0.0, height=0.0):
        """
        :param robot: The robot
        :param entity: The entity to look at
        :param waittime: Amount of time the robot look at the entity
        :param height: Look higher up in the entity
        """
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self._robot = robot
        self._entity = entity
        self._waittime = rospy.Duration(waittime)
        self._height = height

        ds.check_type(entity, Entity)
        ds.check_type(height, float, int)

    def execute(self, userdata=None):

        entity = ds.value_or_resolve(self._entity)
        height = ds.value_or_resolve(self._height)

        if not entity:
            rospy.logerr(
                f"[LookAtEntity] Entity {self._entity} could not be resolved. Resolved to {entity}")
            return 'failed'

        if height is None:
            rospy.logerr(
                f"[LookAtEntity] Height {self._height} could not be resolved. Resolved to {height}")
            return 'failed'

        # Entities define their own frame, so there is no need to transform the pose to map.
        # That would be equivalent to defining coordinates 0,0,0 in its own frame, so that is what we do here.
        # The added benefit is that the entity's frame actually moves because the entity is tracked.
        # This makes the head track the entity
        vs = VectorStamped.from_xyz(0, 0, height, entity.last_update_time, frame_id=entity.uuid)
        rospy.loginfo(f'Look at {vs}')
        self._robot.head.look_at_point(vs)
        rospy.sleep(self._waittime)
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


class RotateToEntity(smach.State):
    def __init__(self, robot, entity, max_vel=np.pi / 10.0):
        """
        Rotate the robot so it is orientated towards entity of :param: entity

        :param robot: The robot
        :param entity: The entity to rotate to
        :param max_vel: Max rotational velocity
        """
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self._robot = robot
        self._entity = entity
        self._max_vel = max_vel

        ds.check_type(entity, Entity)

    def execute(self, userdata=None):

        entity = self._entity.resolve() if hasattr(self._entity, 'resolve') else self._entity

        if not entity:
            return 'failed'

        # Entities define their own frame, so there is no need to transform the pose to map.
        # That would be equivalent to defining coordinates 0,0,0 in its own frame, so that is what we do here.
        # The added benefit is that the entity's frame actually moves because the entity is tracked.
        # This makes the head track the entity
        vs = VectorStamped.from_xyz(0, 0, 0, entity.last_update_time, frame_id=entity.uuid)
        rospy.loginfo(f'Rotate to "{entity.uuid}": {vs}')

        vector_in_bs = self._robot.tf_buffer.transform(vs, self._robot.base_link_frame)
        # tan(angle) = dy / dx
        # angle = arctan(dy / dx)
        # Arm to position in a safe way
        rotate_base = np.arctan2(vector_in_bs.vector.y(), vector_in_bs.vector.x())  # Radians
        # For 1 second, rotate the base with vth == rotate_base.
        # vth is in radians/sec but we rotate for 1 s to that should equal $rotate_base in the end.

        duration = abs(rotate_base / self._max_vel)  # duration of rotation, in s
        vel = self._max_vel * np.sign(rotate_base)
        rospy.loginfo("Rotate base by {:.3f}deg. At {:.3f}deg/s this takes {}s".format(np.degrees(rotate_base),
                                                                                       np.degrees(vel),
                                                                                       duration))
        self._robot.base.force_drive(vx=0, vy=0, vth=vel, timeout=duration)
        return "succeeded"


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
