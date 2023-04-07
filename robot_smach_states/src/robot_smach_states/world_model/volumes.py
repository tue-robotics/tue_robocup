from typing import Optional

import rospy
import smach

from ed.entity import Entity, Volume
from robot_skills.classification_result import ClassificationResult
from robot_smach_states.util import designators as ds
from .world_model import Inspect


# ToDo: This just checks if list has elements. So make more generic
class CheckEmpty(smach.State):
    """
    Check whether any entities are present in the entity designation
    """

    def __init__(self, robot, segmented_entity_ids_designator):
        """
        Constructor

        :param segmented_entity_ids_designator: designator containing the segmented objects in the volume
        """
        smach.State.__init__(self, outcomes=["occupied", "empty"])
        self.robot = robot
        self.seen_entities_des = segmented_entity_ids_designator

        ds.check_resolve_type(segmented_entity_ids_designator, [ClassificationResult])

    def execute(self, userdata=None):
        seen_entities = self.seen_entities_des.resolve()

        return "occupied" if seen_entities else "empty"


class CheckFreeSpaceVolume(smach.State):
    """
    Compare the free space in a volume to a threshold. Volume is determined as the sum of all entities within a
    designator
    """

    def __init__(self, robot, segmented_entity_ids_designator, entity_designator, volume, threshold_val: float):
        """
        Constructor

        :param segmented_entity_ids_designator: designator containing the segmented objects in the volume
        :param entity_designator: EdEntityDesignator indicating the (furniture) object to check
        :param volume: string or designator to a string, defining which volume of the entity is checked
        :param threshold_val: Indicating the free volume above which the area is considered
        occupied
        """
        smach.State.__init__(self, outcomes=["empty", "occupied", "failed"])
        self.robot = robot
        self.seen_entities_des = segmented_entity_ids_designator
        self.entity_des = entity_designator
        self.volume = volume
        self.threshold_val = threshold_val

        ds.check_resolve_type(segmented_entity_ids_designator, [ClassificationResult])
        ds.check_resolve_type(entity_designator, Entity)
        ds.check_type(volume, str)
        ds.check_type(threshold_val, float, None)  # Remove None, when moving the logic to StateMachine

    def execute(self, userdata=None):
        if self.threshold_val is None:
            return "occupied"

        entity = self.entity_des.resolve()  # type: Entity
        if entity is None:
            rospy.logerr("Entity is None")
            return "failed"

        seen_entities = self.seen_entities_des.resolve()

        volume = self.volume.resolve() if hasattr(self.volume, "resolve") else self.volume
        if volume not in entity.volumes:
            rospy.logerr(f"Entity {entity.uuid} has no volume {volume}")
            return "failed"
        vol = entity.volumes[volume]  # type: Volume

        entities = [self.robot.ed.get_entity(uuid=seen_entity.uuid) for seen_entity in seen_entities]
        occupied_space = sum(entity.shape.size for entity in entities if entity is not None)
        remaining_space = vol.size - occupied_space

        rospy.loginfo("Occupied space is {}, remaining space is {}".format(occupied_space, remaining_space))

        if remaining_space > self.threshold_val:
            return "empty"
        else:
            return "occupied"


class CheckFreeSpacePercentage(smach.State):
    """
    Compare the free space in a volume to a threshold. Volume is determined as the sum of all entities within a
    designator
    """

    def __init__(self, robot, segmented_entity_ids_designator, entity_designator, volume, threshold_perc: float):
        """
        Constructor

        :param segmented_entity_ids_designator: designator containing the segmented objects in the volume
        :param entity_designator: EdEntityDesignator indicating the (furniture) object to check
        :param volume: string or designator to a string, defining which volume of the entity is checked
        :param threshold_perc: Indicating the free volume percentage above which the area is considered occupied
        """
        smach.State.__init__(self, outcomes=["empty", "occupied", "failed"])
        self.robot = robot
        self.seen_entities_des = segmented_entity_ids_designator
        self.entity_des = entity_designator
        self.volume = volume
        self.threshold_perc = threshold_perc

        ds.check_resolve_type(segmented_entity_ids_designator, [ClassificationResult])
        ds.check_resolve_type(entity_designator, Entity)
        ds.check_type(volume, str)
        ds.check_type(threshold_perc, float, None)  # Remove None, when moving the logic to StateMachine

    def execute(self, userdata=None):
        if self.threshold_perc is None:
            return "occupied"

        entity = self.entity_des.resolve()  # type: Entity
        if entity is None:
            rospy.logerr("Entity is None")
            return "failed"

        seen_entities = self.seen_entities_des.resolve()

        volume = self.volume.resolve() if hasattr(self.volume, "resolve") else self.volume
        if volume not in entity.volumes:
            rospy.logerr(f"Entity {entity.uuid} has no volume {volume}")
            return "failed"
        vol = entity.volumes[volume]  # type: Volume

        entities = [self.robot.ed.get_entity(uuid=seen_entity.uuid) for seen_entity in seen_entities]
        occupied_space = sum(entity.shape.size for entity in entities if entity is not None)
        remaining_space = vol.size - occupied_space
        # TODO: the remaining space percentage can be negative, as overlap between entities is not checked.
        #  for now this does not break anything but fix it!
        remaining_space_perc = remaining_space / vol.size

        rospy.loginfo("Occupied space is {}, remaining space is {}".format(occupied_space, remaining_space))

        if remaining_space_perc > self.threshold_perc:
            return "empty"
        else:
            return "occupied"


class CheckVolumeEmpty(smach.StateMachine):
    def __init__(
        self,
        robot,
        entity_des,
        volume="on_top_of",
        volume_threshold_per: Optional[float] = None,
        volume_threshold_val: Optional[float] = None,
        fit_supporting_entity: bool = True,
    ):
        """
        Constructor

        :param robot: robot object
        :param entity_des: EdEntityDesignator indicating the (furniture) object to check
        :param volume: string or designator to a string, defining volume of the entity to be checked
        :param volume_threshold_per: Indicating the free volume percentage above which the area is considered
        partially_occupied (If both thresholds are None any entities filling the volume will result in 'occupied')
        :param volume_threshold_val: float [m^3] indicating the free volume above which the area is considered
        partially_occupied. (If both thresholds are None any entities filling the volume will result in 'occupied')
        :param fit_supporting_entity: Fit or not fit the supporting entity

        """
        # TODO implement logic for percent vs volume check in state machine rather than in the states themselves
        smach.StateMachine.__init__(self, outcomes=["empty", "occupied", "partially_occupied", "failed"])

        seen_entities_des = ds.VariableDesignator([], resolve_type=[ClassificationResult])

        with self:
            smach.StateMachine.add(
                "INSPECT",
                Inspect(
                    robot,
                    entity_des,
                    searchArea=volume,
                    objectIDsDes=seen_entities_des,
                    fit_supporting_entity=fit_supporting_entity,
                ),
                transitions={"done": "CHECK_EMPTY", "failed": "failed"},
            )

            smach.StateMachine.add(
                "CHECK_EMPTY",
                CheckEmpty(robot, seen_entities_des),
                transitions={"empty": "empty", "occupied": "CHECK_VOLUME"},
            )

            smach.StateMachine.add(
                "CHECK_VOLUME",
                CheckFreeSpaceVolume(robot, seen_entities_des, entity_des, volume, threshold_val=volume_threshold_val),
                transitions={"empty": "partially_occupied", "occupied": "CHECK_VOLUME_PERCENTAGE", "failed": "failed"},
            )

            smach.StateMachine.add(
                "CHECK_VOLUME_PERCENTAGE",
                CheckFreeSpacePercentage(
                    robot, seen_entities_des, entity_des, volume, threshold_perc=volume_threshold_per
                ),
                transitions={"empty": "partially_occupied", "occupied": "occupied", "failed": "failed"},
            )
