#! /usr/bin/env python

import rospy
import smach
from robot_skills.util.kdl_conversions import VectorStamped

from robot_smach_states.navigation import NavigateToObserve
import robot_smach_states.util.designators as ds
from robot_skills.util.entity import Entity
from robot_skills.classification_result import ClassificationResult

import time


def _color_info(string):
    rospy.loginfo('\033[92m' + string + '\033[0m')


'''
    Look at an entiy and segment objects whithin the area desired
     - entity: entity that holds the objects, for example on top
     - searchArea: where are the objects wrt the entity, default = on_top_of
'''
class SegmentObjects(smach.State):
    def __init__(self, robot, segmented_entity_ids_designator, entity_to_inspect_designator,
                 segmentation_area="on_top_of"):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot

        ds.check_resolve_type(entity_to_inspect_designator, Entity)
        self.entity_to_inspect_designator = entity_to_inspect_designator
        self.segmentation_area = segmentation_area

        ds.check_resolve_type(segmented_entity_ids_designator, [ClassificationResult])
        ds.is_writeable(segmented_entity_ids_designator)
        self.segmented_entity_ids_designator = segmented_entity_ids_designator

    def _look_at_segmentation_area(self, entity):
        look_at_point_z = 0.7   # Where does this come from?

        # Make sure the head looks at the entity
        pos = entity.pose.position
        self.robot.head.look_at_point(VectorStamped(pos.x, pos.y, look_at_point_z, "/map"), timeout=0)

        # Check if we have areas
        if self.segmentation_area in entity.volumes:
            search_volume = entity.volumes[self.segmentation_area]
            try:
                look_at_point_z = search_volume.min_corner.z()
            except:
                pass

        # Make sure the spindle is at the appropriate height if we are AMIGO
        if self.robot.robot_name == "amigo":
            # Send spindle goal to bring head to a suitable location
            # Correction for standard height: with a table heigt of 0.76 a spindle position
            # of 0.35 is desired, hence offset = 0.76-0.35 = 0.41
            # Minimum: 0.15 (to avoid crushing the arms), maximum 0.4
            spindle_target = max(0.15, min(look_at_point_z - 0.41, self.robot.torso.upper_limit[0]))

            self.robot.torso._send_goal([spindle_target], timeout=0)
            self.robot.torso.wait_for_motion_done()

        self.robot.head.wait_for_motion_done()

    def execute(self, userdata=None):
        entity_to_inspect = self.entity_to_inspect_designator.resolve()

        self._look_at_segmentation_area(entity_to_inspect)

        # This is needed because the head is not entirely still when the look_at_point function finishes
        time.sleep(0.5)

        # Inspect 'on top of' the entity
        res = self.robot.ed.update_kinect("{} {}".format(self.segmentation_area, entity_to_inspect.id))
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
    def __init__(self, robot, entityDes, objectIDsDes=None, searchArea="on_top_of"):
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        if not objectIDsDes:
            objectIDsDes = ds.VariableDesignator([], resolve_type=[ClassificationResult])

        with self:
            smach.StateMachine.add('NAVIGATE_TO_INSPECT', NavigateToObserve(robot, entityDes, radius=1.0),
                                   transitions={'unreachable':      'failed',
                                                'goal_not_defined': 'failed',
                                                'arrived':          'SEGMENT'})

            smach.StateMachine.add('SEGMENT', SegmentObjects(robot, objectIDsDes.writeable, entityDes, searchArea),
                                   transitions={'done':      'done'})
