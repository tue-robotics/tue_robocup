# ROS
import smach

# TU/e
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_skills.util.kdl_conversions import FrameStamped


class dropPoseDesignator(ds.Designator):
    """
    Designator to resolver the drop pose
    """
    def __init__(self, robot, drop_height, name):
        """

        :param robot: robot object
        :param drop_height: height to drop the object from
        :param name: name of pose
        """
        super(dropPoseDesignator, self).__init__(resolve_type=FrameStamped, name=name)

        self._robot = robot
        self._drop_height = drop_height

    def _resolve(self):
        frame = None

        # Query ed
        try:
            frame = self._robot.ed.get_entity(id="drop_area")._pose
        except:
            return None

        frame.p.z(self._drop_height)

        return FrameStamped(frame, "/map")


class DropDownTrash(smach.StateMachine):
    """
    State that makes the robot go to the drop zone and drop the trash bag there
    """
    def __init__(self, robot, trash_designator, drop_designator):
        """
        :param robot: robot object
        :param trash_designator: EdEntityDesignator designating the trash
        :param drop_designator: EdEntityDesignator designating the collection zone
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        drop_area_pose = dropPoseDesignator(robot, 0.6, "drop_pose")
        arm_designator = ds.OccupiedArmDesignator(robot=robot, arm_properties={})
        self._trash_designator = trash_designator
        self._drop_designator = drop_designator

        with self:
            smach.StateMachine.add("GO_TO_COLLECTION_ZONE",
                                   states.NavigateToObserve(robot, self._drop_designator),
                                   transitions={"arrived": "PLACE_TRASH",
                                                "goal_not_defined": "aborted",
                                                "unreachable": "failed"})

            smach.StateMachine.add("PLACE_TRASH",
                                   states.Place(robot=robot, item_to_place=trash_designator,
                                                place_pose=drop_area_pose, place_volume="on_top_of",
                                                arm=arm_designator),
                                   transitions={"done": "succeeded",
                                                "failed": "HANDOVER"})

            smach.StateMachine.add("HANDOVER",
                                   states.HandoverToHuman(robot=robot, arm_designator=arm_designator),
                                   transitions={"succeeded": "succeeded",
                                                "failed": "failed"})
