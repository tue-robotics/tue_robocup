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


class DefaultGrabDesignator(ds.Designator):
    """ Designator to pick the closest item on top of the table to grab. This is used for testing

    """
    def __init__(self, robot, surface_designator, area_description):
        """ Constructor

        :param robot: robot object
        :param surface_designator: designator for the object to grab from
        :param area_description: string with id of the area where the object should be located in
        """
        super(DefaultGrabDesignator, self).__init__(resolve_type=robot_skills.util.entity.Entity)

        self._robot = robot
        self._surface_designator = surface_designator
        self.area_description = area_description

    def _resolve(self):
        """ Resolves

        :return: entity in the <area_description> of the <surface_designator> that is closest to the robot
        """

        # Get the surface as an entity
        surface = self._surface_designator.resolve()
        if surface is None:
            rospy.logerror("Cannot resolve surface designator")
            return None

        # Get all entities and check which ones are on the table
        all_entities = self._robot.ed.get_entities()
        entities = []
        for e in all_entities:
            point = robot_skills.util.kdl_conversions.VectorStamped(frame_id=e.frame_id, vector=e._pose.p)
            if surface.in_volume(point=point, volume_id=self.area_description):
                entities.append(e)

        # Check if there are any
        if not entities:
            return None

        # Sort the entities and return the closest one
        base_pose = self._robot.base.get_location().frame
        entities = sorted(entities, key=lambda e: e.distance_to_2d(base_pose.p))
        rospy.loginfo(entities[0])
        return entities[0]


class GrabSingleItem(smach.StateMachine):
    """ Lock an object, announce it and grab it """
    def __init__(self, robot, grab_designator=None):
        """ Constructor

        :param robot: robot object
        :param grab_designator: EdEntityDesignator designating the item to grab. If not provided, a default one is
        constructed (grabs the closest object in the volume of the surface)
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        # Create designators
        self.empty_arm_designator = ds.UnoccupiedArmDesignator(robot, {}, name="empty_arm_designator")
        self.grab_designator = ds.LockToId(robot=robot, to_be_locked=grab_designator)

        with self:
            @smach.cb_interface(outcomes=["locked"])
            def lock(userdata=None):
                """ 'Locks' a locking designator """
                # This determines that self.current_item cannot not resolve to a new value until it is unlocked again.
                self.grab_designator.lock()
                if self.grab_designator.resolve():
                    rospy.loginfo("Current_item is now locked to {0}".format(self.grab_designator.resolve().id))

                return "locked"

            smach.StateMachine.add("LOCK_ITEM",
                                   smach.CBState(lock),
                                   transitions={'locked': 'GRAB_ITEM'})

            smach.StateMachine.add("GRAB_ITEM",
                                   states.Grab(robot, self.grab_designator, self.empty_arm_designator),
                                   transitions={'done': 'UNLOCK_ITEM_SUCCEED',
                                                'failed': 'UNLOCK_ITEM_FAIL'})

            @smach.cb_interface(outcomes=["unlocked"])
            def unlock(userdata=None):
                """ 'Unlocks' a locking designator """
                self.grab_designator.unlock()

                return "unlocked"

            smach.StateMachine.add("UNLOCK_ITEM_SUCCEED",
                                   smach.CBState(unlock),
                                   transitions={'unlocked': 'succeeded'})

            smach.StateMachine.add("UNLOCK_ITEM_FAIL",
                                   smach.CBState(unlock),
                                   transitions={'unlocked': 'failed'})


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
        arm_designator = ds.OccupiedArmDesignator(robot=self._robot, arm_properties={})
        sm = states.Place(robot=self._robot, item_to_place=self._item_designator, place_pose=self._place_designator,
                          place_volume="on_top_of", arm=arm_designator)
        result = sm.execute()

        # If failed, do handover to human in order to continue
        if result != "done":
            sm = states.HandoverToHuman(robot=self._robot, arm_designator=arm_designator)
            sm.execute()

        return "succeeded" if result == "done" else "failed"


class HandoverFromHuman(smach.StateMachine):
    '''
    State that enables low level grab reflex. Besides a robot object, needs
    an arm and an entity to grab, which is either one from ed through the
    grabbed_entity_designator or it is made up in the
    CloseGripperOnHandoverToRobot state and given the grabbed_entity_label
    as id.
    '''
    def __init__(self, robot, arm_designator, grabbed_entity_label="", grabbed_entity_designator=None, timeout=15, arm_configuration="handover_to_human"):
        """
        Hold up hand to accept an object and close hand once something is inserted
        :param robot: Robot with which to execute this behavior
        :param arm_designator: ArmDesignator resolving to arm accept item into
        :param grabbed_entity_label: What ID to give a dummy item in case no grabbed_entity_designator is supplied
        :param grabbed_entity_designator: EntityDesignator resolving to the accepted item. Can be a dummy
        :param timeout: How long to hold hand over before closing without anything
        :param arm_configuration: Which pose to put arm in when holding hand up for the item.
        """
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed','timeout'])

        ds.check_type(arm_designator, PublicArm)
        if not grabbed_entity_designator and grabbed_entity_label == "":
            rospy.logerr("No grabbed entity label or grabbed entity designator given")

        with self:
            smach.StateMachine.add("POSE", manipulation.ArmToJointConfig(robot, arm_designator, arm_configuration),
                                   transitions ={'succeeded': 'OPEN_BEFORE_INSERT','failed':'OPEN_BEFORE_INSERT'})

            smach.StateMachine.add(
                'OPEN_BEFORE_INSERT',
                manipulation.SetGripper(
                    robot=robot,
                    arm_designator=arm_designator,
                    gripperstate=manipulation.GripperState.OPEN),
                transitions={'succeeded': 'SAY1',
                             'failed': 'SAY1'}
            )

            smach.StateMachine.add("SAY1", states.Say(robot,'Please hand over the object by putting the top of the bag between'
                                                     ' my grippers and push firmly into my camera.'),
                            transitions={'spoken': 'CLOSE_AFTER_INSERT'})

            smach.StateMachine.add( 'CLOSE_AFTER_INSERT', manipulation.CloseGripperOnHandoverToRobot(robot,
                                                                                        arm_designator,
                                                                                        grabbed_entity_label=grabbed_entity_label,
                                                                                        grabbed_entity_designator=grabbed_entity_designator,
                                                                                        timeout=timeout),
                                transitions={'succeeded'    :   'succeeded',
                                             'timeout'      :   'timeout',
                                             'failed'       :   'failed'})

"""
class ForceGrabTrash:
    grabs the bag with in a hacky way

    def __init__(self):

        smach.State.__init__(self, outcomes=["done", "failed"])

        smach.StateMachine.add("SAY_TRY_AGAIN", states.Say("The thrash bag looks difficult to grab, let me try to grab")

"""

class TakeOut(smach.StateMachine):

    def __init__(self, robot, trashbin_designator, trash_designator, drop_designator, arm_designator):
        """

        :param robot: robot object
        :param trashbin_designator: EdEntityDesignator designating the trashbin
        :param trash_designator: EdEntityDesignator designating the trash
        :param drop_designator: EdEntityDesignator designating the collection zone
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        drop_area_pose = dropPoseDesignator(robot, 0.6, "drop_pose")

        with self:
            # Take Out 1
            smach.StateMachine.add("GO_TO_BIN",
                                   states.NavigateToObserve(robot, trashbin_designator, radius=0.5),
                                   transitions={"arrived": "INSPECT",
                                                "goal_not_defined": "aborted",
                                                "unreachable": "failed"})

            smach.StateMachine.add("INSPECT",
                                   states.Inspect(robot, trashbin_designator),
                                   transitions={"done": "FAILED_TO_SEE",
                                                "failed": "FAILED_TO_SEE"})

            # State is not used, because it can not grab bag
            smach.StateMachine.add("GRAB_TRASH", GrabSingleItem(robot=robot, grab_designator=trash_designator),
                                   transitions={"succeeded": "GO_TO_COLLECTION_ZONE",
                                                "failed": "FAILED_TO_GRAB"})

            smach.StateMachine.add("GO_TO_COLLECTION_ZONE",
                                   states.NavigateToObserve(robot, drop_designator),
                                   transitions={"arrived": "PLACE_ITEM",
                                                "goal_not_defined": "aborted",
                                                "unreachable": "failed"})

            # smach.StateMachine.add("PLACE_ITEM", PlaceSingleItem(robot=robot, place_designator=drop_designator,
            #                                                      item_designator=trash_designator),
            #                        transitions={"succeeded": "succeeded",
            #                                     "failed": "failed"})

            smach.StateMachine.add("PLACE_ITEM", PlaceSingleItem(robot=robot, place_designator=drop_area_pose,
                                                                 item_designator=trash_designator),
                                   transitions={"succeeded": "succeeded",
                                                "failed": "failed"})

            # if it fails with inspect or grabbing
            smach.StateMachine.add("FAILED_TO_SEE", states.Say(robot, "I cannot see the bag, so please hand over the "
                                                                      "bag to me when I say it and listen to the my "
                                                                      "instructions.",
                                              block=False),
                                   transitions={'spoken': 'ASK_HANDOVER'})

            smach.StateMachine.add("FAILED_TO_GRAB", states.Say(robot, "I cannot grab the bag, so please hand over the "
                                                                       "bag to me when I say it and listen to the my "
                                                                       "instructions.",
                                              block=False),
                                   transitions={'spoken': 'ASK_HANDOVER'})

            # Ask for handover from human
            smach.StateMachine.add("ASK_HANDOVER", HandoverFromHuman(robot=robot, arm_designator=arm_designator,
                                                                            grabbed_entity_label='thrash'),
                                   transitions={"succeeded": "LOWER_ARM",
                                                "failed": "failed",
                                                "timeout": "TIMEOUT"})

            arm_occupied_designator = ds.OccupiedArmDesignator(robot=robot, arm_properties={})

            smach.StateMachine.add("LOWER_ARM", states.ArmToJointConfig(robot=robot,
                                                                        arm_designator=arm_occupied_designator,
                                                                        configuration="reset"),
                                   transitions={"succeeded": "RECEIVED_TRASH_BAG",
                                                "failed": "RECEIVED_TRASH_BAG"})

            smach.StateMachine.add("RECEIVED_TRASH_BAG", states.Say(robot, "I received the thrash bag. I will throw"
                                                                           " it away, please move away.", block=True),
                                   transitions={'spoken': 'GO_TO_COLLECTION_ZONE'})

            smach.StateMachine.add("TIMEOUT", states.Say(robot, "I have not received anything, so I will stop",
                                                         block=False),
                                   transitions={'spoken': "failed"})

