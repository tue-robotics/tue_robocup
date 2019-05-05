# ROS
import rospy
import smach

# TU/e
import robot_skills
import robot_smach_states as states
import robot_smach_states.util.designators as ds
import robot_smach_states.manipulation as manipulation
from robot_skills.arms import PublicArm

from robot_skills.util.kdl_conversions import FrameStamped

class dropPoseDesignator(ds.Designator):
    def __init__(self, robot, drop_height, name):
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

class PlaceSingleItem(smach.State):
    """ Tries to place an object. A 'place' statemachine is constructed dynamically since this makes it easier to
     build a statemachine (have we succeeded in grasping the objects?)"""
    def __init__(self, robot, place_designator, item_designator):
        """ Constructor

        :param robot: robot object
        :param place_designator: Designator that resolves to the pose to place at. E.g. an EmptySpotDesignator
        """
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

        self._robot = robot
        self._place_designator = place_designator
        self._item_designator = item_designator

    def execute(self, userdata=None):

        # Try to place the object
        # ToDo: Change back when arm can be set to occupied in pick up
        arm_designator = ds.OccupiedArmDesignator(robot=self._robot, arm_properties={})
        # arm_designator = ds.UnoccupiedArmDesignator(self._robot, {}, name="empty_arm_designator")
        sm = states.Place(robot=self._robot, item_to_place=self._item_designator, place_pose=self._place_designator,
                          place_volume="on_top_of", arm=arm_designator)
        result = sm.execute()

        # If failed, do handover to human in order to continue
        if result != "done":
            sm = states.HandoverToHuman(robot=self._robot, arm_designator=arm_designator)
            sm.execute()

        return "succeeded" if result == "done" else "failed"


class DropDownTrash(smach.StateMachine):

    def __init__(self, robot, trash_designator, drop_designator):
        """

        :param robot: robot object
        :param trash_designator: EdEntityDesignator designating the trash
        :param drop_designator: EdEntityDesignator designating the collection zone
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        drop_area_pose = dropPoseDesignator(robot, 0.6, "drop_pose")

        with self:
            smach.StateMachine.add("GO_TO_COLLECTION_ZONE",
                                   states.NavigateToObserve(robot, drop_designator),
                                   transitions={"arrived": "PLACE_ITEM",
                                                "goal_not_defined": "aborted",
                                                "unreachable": "failed"})

            smach.StateMachine.add("PLACE_ITEM", PlaceSingleItem(robot=robot, place_designator=drop_area_pose,
                                                                 item_designator=trash_designator),
                                   transitions={"succeeded": "succeeded",
                                                "failed": "failed"})
