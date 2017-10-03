#! /usr/bin/env python

import PyKDL as kdl
import rospy
import smach
from robot_skills.util.kdl_conversions import VectorStamped

from robot_smach_states.navigation import NavigateToObserve, NavigateToSymbolic
import robot_smach_states.util.designators as ds
from robot_skills.util.entity import Entity
from robot_skills.classification_result import ClassificationResult

import time


def _color_info(string):
    rospy.loginfo('\033[92m' + string + '\033[0m')


def look_at_segmentation_area(robot, entity, volume=None):
    """ Has a robot look at a certain object and possibly a volume

    :param robot: robot object
    :param entity: entity to look at
    :param volume: string indicating the specific volume to look at (e.g., 'on_top_on' or 'shelf3')
    """

    # Determine the height of the head target
    # Start with a default

    # Check if we have areas: use these
    if volume in entity.volumes:
        search_volume = entity.volumes[volume]
        x_obj = 0.5 * (search_volume.min_corner.x() + search_volume.max_corner.x())
        y_obj = 0.5 * (search_volume.min_corner.y() + search_volume.max_corner.y())
        z_obj = search_volume.min_corner.z()
        lookat_pos_map = entity.pose.frame * kdl.Vector(x_obj, y_obj, z_obj)
        x = lookat_pos_map.x()
        y = lookat_pos_map.y()
        z = lookat_pos_map.z()
    else:
        # Look at the top of the entity to inspect
        pos = entity.pose.frame.p
        x = pos.x()
        y = pos.y()
        z = pos.z() + entity.shape.z_max

    # Point the head at the right direction
    robot.head.look_at_point(VectorStamped(x, y, z, "/map"), timeout=0)

    # Make sure the spindle is at the appropriate height if we are AMIGO
    if robot.robot_name == "amigo":
        # Send spindle goal to bring head to a suitable location
        # Correction for standard height: with a table heigt of 0.76 a spindle position
        # of 0.35 is desired, hence offset = 0.76-0.35 = 0.41
        # Minimum: 0.15 (to avoid crushing the arms), maximum 0.4
        spindle_target = max(0.15, min(z - 0.41, robot.torso.upper_limit[0]))

        robot.torso._send_goal([spindle_target], timeout=0)
        robot.torso.wait_for_motion_done()

    robot.head.wait_for_motion_done()


class UpdateEntityPose(smach.State):
    """ Look at an entity and updates its pose. This assumes the robot is already in front of the object """
    def __init__(self, robot, entity_designator):
        """ Constructor

        :param robot: robot object
        :param entity_designator: EdEntityDesignator indicating the object for which the pose should be updated
        """
        smach.State.__init__(self, outcomes=["done"])
        self._robot = robot
        self._entity_designator = entity_designator

    def execute(self, userdata=None):
        """ Looks at the entity and updates its pose using the update kinect service """
        # Start by looking at the entity
        entity_to_inspect = self._entity_designator.resolve()
        look_at_segmentation_area(self._robot, entity_to_inspect)

        # This is needed because the head is not entirely still when the look_at_point function finishes
        time.sleep(0.5)

        # Inspect 'on top of' the entity
        res = self._robot.ed.update_kinect("{}".format(entity_to_inspect.id))

        # Return
        return "done"


class SegmentObjects(smach.State):
    """ Look at an entiy and segment objects within the area desired.
    """
    def __init__(self, robot, segmented_entity_ids_designator, entity_to_inspect_designator,
                 segmentation_area="on_top_of"):
        """ Constructor

        :param robot: robot object
        :param segmented_entity_ids_designator: designator that is used to store the segmented objects
        :param entity_to_inspect_designator: EdEntityDesignator indicating the (furniture) object to inspect
        :param segmentation_area: string defining where the objects are w.r.t. the entity, default = on_top_of
        """
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot

        ds.check_resolve_type(entity_to_inspect_designator, Entity)
        self.entity_to_inspect_designator = entity_to_inspect_designator

        if isinstance(segmentation_area, str):
            self.segmentation_area_des = ds.VariableDesignator(segmentation_area)
        else:
            # ds.check_resolve_type(segmentation_area, "str")
            self.segmentation_area_des = segmentation_area

        ds.check_resolve_type(segmented_entity_ids_designator, [ClassificationResult])
        ds.is_writeable(segmented_entity_ids_designator)
        self.segmented_entity_ids_designator = segmented_entity_ids_designator

    def execute(self, userdata=None):
        entity_to_inspect = self.entity_to_inspect_designator.resolve()
        segmentation_area = self.segmentation_area_des.resolve()

        if not entity_to_inspect.is_a('room'):
            look_at_segmentation_area(self.robot, entity_to_inspect, segmentation_area)

            # This is needed because the head is not entirely still when the look_at_point function finishes
            time.sleep(0.5)

        # Inspect 'on top of' the entity
        res = self.robot.ed.update_kinect("{} {}".format(segmentation_area, entity_to_inspect.id))
        segmented_object_ids = res.new_ids + res.updated_ids

        if segmented_object_ids:
            _color_info(">> Segmented %d objects!" % len(segmented_object_ids))

            # Classify and update IDs
            object_classifications = self.robot.ed.classify(ids=segmented_object_ids)

            if object_classifications:
                for idx, obj in enumerate(object_classifications):
                    _color_info("   - Object {} is a '{}' (ID: {})".format(idx, obj.type, obj.id))

                self.segmented_entity_ids_designator.write(object_classifications)
            else:
                rospy.logerr("    Classification failed, this should not happen!")
        else:
            rospy.logwarn(">> Tried to segment but no objects found")

        # Cancel the head goal
        self.robot.head.cancel_goal()

        return 'done'

# ----------------------------------------------------------------------------------------------------


class Inspect(smach.StateMachine):
    """ Class to navigate to a(n) (furniture) object and segment the objects on top of it.

    """
    def __init__(self, robot, entityDes, objectIDsDes=None, searchArea="on_top_of", navigation_area=""):
        """ Constructor

        :param robot: robot object
        :param entityDes: EdEntityDesignator indicating the (furniture) object to inspect
        :param objectIDsDes: designator that is used to store the segmented objects
        :param searchArea: string defining where the objects are w.r.t. the entity, default = on_top_of
        :param navigatoin_area: string identifying the inspection area. If provided, NavigateToSymbolic is used.
        If left empty, NavigateToObserve is used.
        """
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        if not objectIDsDes:
            objectIDsDes = ds.VariableDesignator([], resolve_type=[ClassificationResult])

        with self:
            if navigation_area:
                smach.StateMachine.add('NAVIGATE_TO_INSPECT', NavigateToSymbolic(robot, {entityDes: navigation_area},
                                                                                 entityDes),
                                       transitions={'unreachable': 'failed',
                                                    'goal_not_defined': 'failed',
                                                    'arrived': 'SEGMENT'})
            else:
                smach.StateMachine.add('NAVIGATE_TO_INSPECT', NavigateToObserve(robot, entityDes, radius=1.0),
                                       transitions={'unreachable': 'failed',
                                                    'goal_not_defined': 'failed',
                                                    'arrived': 'SEGMENT'})

            smach.StateMachine.add('SEGMENT', SegmentObjects(robot, objectIDsDes.writeable, entityDes, searchArea),
                                   transitions={'done': 'done'})


if __name__ == "__main__":

    from robot_skills.amigo import Amigo
    from robot_smach_states.util.designators import EdEntityDesignator

    rospy.init_node('state_machine')

    robot = Amigo()

    sm = Inspect(robot=robot, entityDes=EdEntityDesignator(robot=robot, id="closet"))
    print sm.execute()
