# ROS
import smach

import PyKDL as kdl

from robot_skills.util.kdl_conversions import VectorStamped


class RiseForHMI(smach.State):
    """
    State to pose the robot for conversations

    :var robot: Robot to execute state with
    """
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self._robot = robot

    def execute(self, userdata=None):
        # Get position to look at. Transform the position to map frame since taking the hmi pose may move base link
        goal = VectorStamped(1.0, 0.0, 1.6, frame_id="/" + self._robot.robot_name + "/base_link")
        tf_goal = goal.projectToFrame('/map', self._robot.tf_listener)

        self._robot.move_to_hmi_pose()

        self._robot.head.look_at_point(tf_goal)
        self._robot.head.wait_for_motion_done()
        return "succeeded"


class RiseForInspect(smach.State):
    """
    State to ensure that the robot is in a proper position to inspect. This means its head is at the correct height
    and his vision is unobstructed. (no arm in front of its face)

    :var _robot: Robot to execute state with
    :var _entity: entity which is to be inspected
    :var _volume: volume of the entity which is to be inspected or Designator to it.
    """
    def __init__(self, robot, entity, volume=None):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self._robot = robot
        self._entity = entity
        self._volume = volume

    def execute(self, userdata=None):
        # Determine the height of the head target
        # Start with a default
        entity = self._entity.resolve()
        if entity is None:
            return "failed"
        volume = self._volume.resolve() if hasattr(self._volume, "resolve") else self._volume

        # Check if we have areas: use these
        if volume in entity.volumes:
            search_volume = entity.volumes[volume]
            x_obj = 0.5 * (search_volume.min_corner.x() + search_volume.max_corner.x())
            y_obj = 0.5 * (search_volume.min_corner.y() + search_volume.max_corner.y())
            z_obj = search_volume.min_corner.z()
            pos = entity.pose.frame * kdl.Vector(x_obj, y_obj, z_obj)
        else:
            # Look at the top of the entity to inspect
            pos = entity.pose.frame.p

        if self._robot.move_to_inspect_pose(pos):
            return "succeeded"
        else:
            return "failed"
