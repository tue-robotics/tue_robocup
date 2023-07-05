# System
import time

# ROS
import PyKDL as kdl
from pykdl_ros import VectorStamped
import rospy
import smach

# TU/e Robotics
from ed.entity import Entity
from robot_skills.classification_result import ClassificationResult
from robot_smach_states.navigation import NavigateToObserve, NavigateToSymbolic
from robot_smach_states.util import designators as ds
from robot_smach_states.rise import RiseForInspect


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
    robot.head.look_at_point(VectorStamped(kdl.Vector(x, y, z), entity.last_update_time, "map"), timeout=0)

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
        res = self._robot.ed.update_kinect("{}".format(entity_to_inspect.uuid))

        # Return
        return "done"


class UpdateDestEntityPoseWithSrcEntity(smach.State):
    """ Update the pose of an entity from another entity"""

    def __init__(self, robot, src_entity_designator, dst_entity_designator,
                 dst_entity_type="waypoint"):
        """ Constructor

        :param robot: robot object
        :param src_entity_designator: (EdEntityDesignator) indicating the object from which the pose should be selected
        :param dst_entity_designator: (EdEntityDesignator) indicating the object of which the pose must be updated
        :param dst_entity_type: (str) Destination entity type
        """
        super(UpdateDestEntityPoseWithSrcEntity, self).__init__(outcomes=["done", "failed"])
        self._robot = robot
        ds.check_type(src_entity_designator, Entity)
        ds.check_type(dst_entity_designator, Entity, str)
        self._src_entity_designator = src_entity_designator
        self._dst_entity_designator = dst_entity_designator
        self._dst_entity_type = dst_entity_type

    def execute(self, userdata=None):
        """ Looks at the entity and updates its pose using the update kinect service """
        src_entity = self._src_entity_designator.resolve() if hasattr(self._src_entity_designator, 'resolve') else \
            self._src_entity_designator

        dst_entity = self._dst_entity_designator.resolve() if hasattr(self._dst_entity_designator, 'resolve') else \
            self._dst_entity_designator

        if (not src_entity) or (not self._robot.ed.get_entity(src_entity.uuid)) or (not dst_entity):
            return "failed"

        if isinstance(dst_entity, Entity):
            dst_id = dst_entity.uuid
        else:
            dst_id = dst_entity

        self._robot.ed.update_entity(uuid=dst_id,
                                     frame_stamped=src_entity.pose)

        return "done"


class SegmentObjects(smach.State):
    """
    Look at an entity and segment objects within the area desired.
    """
    def __init__(self, robot, segmented_entity_ids_designator, entity_to_inspect_designator,
                 segmentation_area="on_top_of", unknown_threshold=0.0, filter_threshold=0.0,
                 fit_supporting_entity=True):
        """ Constructor

        :param robot: robot object
        :param segmented_entity_ids_designator: designator that is used to store the segmented objects
        :param entity_to_inspect_designator: EdEntityDesignator indicating the (furniture) object to inspect
        :param segmentation_area: string defining where the objects are w.r.t. the entity, default = on_top_of
        :param unknown_threshold: Entities whose classification score is lower than this float are not marked with a type
        :param filter_threshold: Entities whose classification score is lower than this float are ignored
            (i.e. are not added to the segmented_entity_ids_designator)
        :param fit_supporting_entity: Fit or not fit the supporting entity
        """
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot

        self.unknown_threshold = unknown_threshold
        self.filter_threshold = filter_threshold
        self.fit_supporting_entity = fit_supporting_entity

        ds.check_resolve_type(entity_to_inspect_designator, Entity)
        self.entity_to_inspect_designator = entity_to_inspect_designator

        ds.check_type(segmentation_area, str)
        if isinstance(segmentation_area, str):
            self.segmentation_area_des = ds.VariableDesignator(segmentation_area)
        elif isinstance(segmentation_area, ds.Designator):
            self.segmentation_area_des = segmentation_area
        else:
            raise RuntimeError("This shouldn't happen. Wrong types should have raised an exception earlier")

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
        res = self.robot.ed.update_kinect(area_description=f"{segmentation_area} {entity_to_inspect.uuid}",
                                          fit_supporting_entity=self.fit_supporting_entity)

        segmented_object_ids = res.new_ids + res.updated_ids

        if segmented_object_ids:
            _color_info(">> Segmented %d objects!" % len(segmented_object_ids))
            # Classify and update IDs
            object_classifications = self.robot.ed.classify(uuids=segmented_object_ids, unknown_threshold=self.unknown_threshold)

            if object_classifications:
                for idx, obj in enumerate(object_classifications):
                    _color_info("   - Object {} is a '{}' (ID: {})".format(idx, obj.etype, obj.uuid))

                if self.filter_threshold:
                    over_threshold = [obj for obj in object_classifications if
                                      obj.probability >= self.filter_threshold]

                    dropped = {obj.uuid: obj.probability for obj in object_classifications if
                               obj.probability < self.filter_threshold}
                    rospy.loginfo("Dropping {ln} entities due to low class. score (< {th}): {dropped}"
                                  .format(th=self.filter_threshold, dropped=dropped, ln=len(dropped)))

                    object_classifications = over_threshold

                self.segmented_entity_ids_designator.write(object_classifications)
            else:
                rospy.logerr("    Classification failed, this should not happen!")
                objects = []
                for obj_id in segmented_object_ids:
                    objects.append(ClassificationResult(uuid=obj_id, etype="unknown", probability=0.0))
                self.segmented_entity_ids_designator.write(objects)
        else:
            rospy.logwarn(">> Tried to segment but no objects found")
            self.segmented_entity_ids_designator.write([])

        # Cancel the head goal
        self.robot.head.cancel_goal()

        return 'done'


class Inspect(smach.StateMachine):
    """
    Class to navigate to a(n) (furniture) object and segment the objects on top of it.
    Note that when inspecting a high entity the robot will end the Inspect in a high position.
    """
    def __init__(self, robot, entityDes, objectIDsDes=None, searchArea="on_top_of", navigation_area="",
                 unknown_threshold=0.0, filter_threshold=0.0, fit_supporting_entity=True, room=None):
        """
        Constructor

        :param robot: robot object
        :param entityDes: EdEntityDesignator indicating the (furniture) object to inspect
        :param objectIDsDes: designator that is used to store the segmented objects
        :param searchArea: string defining where the objects are w.r.t. the entity, default = on_top_of
        :param navigation_area: string identifying the inspection area. If provided, NavigateToSymbolic is used.
            If left empty, NavigateToObserve is used.
        :param unknown_threshold: Entities whose classification score is lower than this float are not marked with a type
        :param filter_threshold: Entities whose classification score is lower than this float are ignored
            (i.e. are not added to the segmented_entity_ids_designator)
        :param fit_supporting_entity: Fit or not fit the supporting entity
        :param room: EdEntityDesignator indicating the room in which the robot has to stay in
        """
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        if not objectIDsDes:
            objectIDsDes = ds.VariableDesignator([], resolve_type=[ClassificationResult])

        with self:
            if navigation_area:
                smach.StateMachine.add('NAVIGATE_TO_INSPECT', NavigateToSymbolic(robot, {entityDes: navigation_area},
                                                                                 entityDes, room=room),
                                       transitions={'unreachable': 'failed',
                                                    'goal_not_defined': 'failed',
                                                    'arrived': 'RISE'})
            else:
                smach.StateMachine.add('NAVIGATE_TO_INSPECT', NavigateToObserve(robot, entityDes,
                                                                                radius=1.0, room=room),
                                       transitions={'unreachable': 'failed',
                                                    'goal_not_defined': 'failed',
                                                    'arrived': 'RISE'})

            smach.StateMachine.add('RISE', RiseForInspect(robot, entityDes, searchArea),
                                   transitions={'succeeded': 'SEGMENT',
                                                'failed': 'SEGMENT'})

            smach.StateMachine.add('SEGMENT',
                                   SegmentObjects(robot, objectIDsDes.writeable, entityDes, searchArea,
                                                  unknown_threshold=unknown_threshold,
                                                  filter_threshold=filter_threshold,
                                                  fit_supporting_entity=fit_supporting_entity),
                                   transitions={'done': 'done'})


if __name__ == "__main__":

    from robot_skills import get_robot_from_argv
    from robot_smach_states.util.designators import EdEntityDesignator

    from robocup_knowledge import knowledge_loader

    common = knowledge_loader.load_knowledge("common")

    rospy.init_node('inspect_test')

    robot = get_robot_from_argv(index=1)

    sm = Inspect(robot=robot, entityDes=EdEntityDesignator(robot=robot, uuid="display_cabinet"),
                 navigation_area="in_front_of")
    print(sm.execute())
