from __future__ import print_function

# System
import time

# ROS
import PyKDL as kdl
import rospy
import smach

# TU/e Robotics
from robot_skills.classification_result import ClassificationResult
from robot_skills.util.entity import Entity
from robot_skills.util.kdl_conversions import VectorStamped
from robot_smach_states.navigation import NavigateToObserve, NavigateToSymbolic
import robot_smach_states.util.designators as ds


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

        if (not src_entity) or (not self._robot.ed.get_entity(src_entity.id)) or (not dst_entity):
            return "failed"

        if isinstance(dst_entity, Entity):
            dst_id = dst_entity.id
        else:
            dst_id = dst_entity

        self._robot.ed.update_entity(id=dst_id,
                                     frame_stamped=src_entity.pose)

        return "done"


class SegmentObjects(smach.State):
    """
    Look at an entity and segment objects within the area desired.
    """
    def __init__(self, robot, segmented_entity_ids_designator, entity_to_inspect_designator,
                 segmentation_area="on_top_of", unknown_threshold=0.0, filter_threshold=0.0):
        """ Constructor

        :param robot: robot object
        :param segmented_entity_ids_designator: designator that is used to store the segmented objects
        :param entity_to_inspect_designator: EdEntityDesignator indicating the (furniture) object to inspect
        :param segmentation_area: string defining where the objects are w.r.t. the entity, default = on_top_of
        :param unknown_threshold: Entities whose classification score is lower than this float are not marked with a type
        :param filter_threshold: Entities whose classification score is lower than this float are ignored
            (i.e. are not added to the segmented_entity_ids_designator)
        """
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot

        self.unknown_threshold = unknown_threshold
        self.filter_threshold = filter_threshold

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
        res = self.robot.ed.update_kinect("{} {}".format(segmentation_area, entity_to_inspect.id))
        segmented_object_ids = res.new_ids + res.updated_ids

        if segmented_object_ids:
            _color_info(">> Segmented %d objects!" % len(segmented_object_ids))

            # Classify and update IDs
            object_classifications = self.robot.ed.classify(ids=segmented_object_ids, unknown_threshold=self.unknown_threshold)

            if object_classifications:
                for idx, obj in enumerate(object_classifications):
                    _color_info("   - Object {} is a '{}' (ID: {})".format(idx, obj.type, obj.id))

                if self.filter_threshold:
                    over_threshold = [obj for obj in object_classifications if
                                      obj.probability >= self.filter_threshold]

                    dropped = {obj.id: obj.probability for obj in object_classifications if
                               obj.probability < self.filter_threshold}
                    rospy.loginfo("Dropping {ln} entities due to low class. score (< {th}): {dropped}"
                                  .format(th=self.filter_threshold, dropped=dropped, ln=len(dropped)))

                    object_classifications = over_threshold

                self.segmented_entity_ids_designator.write(object_classifications)
            else:
                rospy.logerr("    Classification failed, this should not happen!")
        else:
            rospy.logwarn(">> Tried to segment but no objects found")

        # Cancel the head goal
        self.robot.head.cancel_goal()

        return 'done'


class CheckEmpty(smach.State):
    """
    Check whether a volume of an entity is filled
    """
    def __init__(self, robot, segmented_entity_ids_designator, entity_designator, volume, threshold=None):
        """
        Constructor

        :param segmented_entity_ids_designator: designator containing the segmented objects in the volume
        :param entity_designator: EdEntityDesignator indicating the (furniture) object to check
        :param volume: string defining which volume of the entity is checked
        :param threshold: float [m^3] indicating the free volume above which the area is considered partially_occupied.
            (None means any entities filling the volume will result in 'occupied')
        """
        smach.State.__init__(self, outcomes=["occupied", "partially_occupied", "empty"])
        self.robot = robot
        self.seen_entities_des = segmented_entity_ids_designator
        self.entity_des = entity_designator
        self.volume = volume
        self.threshold = threshold

    def execute(self, userdata=None):
        entity = self.entity_des.resolve()  # type: Entity
        seen_entities = self.seen_entities_des.resolve()
        if seen_entities:
            if self.threshold:
                vol = entity.volumes[self.volume]  # type: Volume
                entities = [self.robot.ed.get_entity(id=seen_entity.id) for seen_entity in seen_entities]
                occupied_space = sum(entity.shape.size for entity in entities)
                remaining_space = vol.size - occupied_space
                rospy.loginfo('Occupied space is {}, remaining space is {}'.format(occupied_space, remaining_space))
                if remaining_space > self.threshold:
                    return 'partially_occupied'
            return 'occupied'
        else:
            return 'empty'


class Inspect(smach.StateMachine):
    """
    Class to navigate to a(n) (furniture) object and segment the objects on top of it.
    """
    def __init__(self, robot, entityDes, objectIDsDes=None, searchArea="on_top_of", navigation_area="",
                 unknown_threshold=0.0, filter_threshold=0.0):
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

            smach.StateMachine.add('SEGMENT',
                                   SegmentObjects(robot, objectIDsDes.writeable, entityDes, searchArea,
                                                  unknown_threshold=unknown_threshold,
                                                  filter_threshold=filter_threshold),
                                   transitions={'done': 'done'})


class CheckVolumeEmpty(smach.StateMachine):
    def __init__(self, robot, entity_des, volume="on_top_of", volume_threshold=0.0):
        """ Constructor

        :param robot: robot object
        :param entity_des: EdEntityDesignator indicating the (furniture) object to check
        :param volume: string defining volume of the entity to be checked, default = on_top_of
        :param volume_threshold: float [m^3] indicating the free volume above which the area is considered partially_occupied.
            (None means any entities filling the volume will result in 'occupied')
        """
        smach.StateMachine.__init__(self, outcomes=['empty', 'occupied',  'partially_occupied', 'failed'])

        seen_entities_des = ds.VariableDesignator([], resolve_type=[ClassificationResult])

        with self:
            smach.StateMachine.add('INSPECT',
                                   Inspect(robot, entity_des, searchArea=volume, objectIDsDes=seen_entities_des),
                                   transitions={"done": "CHECK",
                                                "failed": "failed"})

            smach.StateMachine.add('CHECK',
                                   CheckEmpty(robot, seen_entities_des, entity_des, volume, volume_threshold),
                                   transitions={'empty': 'empty',
                                                'partially_occupied': 'partially_occupied',
                                                'occupied': 'occupied'})


if __name__ == "__main__":

    from robot_skills import get_robot_from_argv
    from robot_smach_states.util.designators import EdEntityDesignator

    from robocup_knowledge import knowledge_loader

    common = knowledge_loader.load_knowledge("common")

    rospy.init_node('inspect_test')

    robot = get_robot_from_argv(index=1)

    sm = Inspect(robot=robot, entityDes=EdEntityDesignator(robot=robot, id="display_cabinet"),
                 navigation_area="in_front_of")
    print(sm.execute())
