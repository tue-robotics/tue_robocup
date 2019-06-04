# ROS
import rospy
import smach

# TU/e
import robot_smach_states as states
import robot_smach_states.util.designators as ds
import robot_smach_states.manipulation as manipulation
from robot_skills.arms import PublicArm

from ed_msgs.msg import EntityInfo
from robot_skills.util.kdl_conversions import FrameStamped


class GetTrashBin(smach.State):
    """
    Get the entity of the trashbin in FrameStamped and save it to get the orientation of the fixed point in world model
    """
    def __init__(self, robot, trashbin):
        """
        :param robot: robot object
        :param trashbin: EdEntityDesignator designating the trash bin
        """
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self._robot = robot
        self._trashbin = trashbin

    def execute(self, userdata=None):
        e = self._trashbin.resolve()

        # get original entity pose
        frame_original = self._robot.ed.get_entity(id=e.id)._pose

        # inspect and update entity
        states.look_at_segmentation_area(self._robot,  self._robot.ed.get_entity(id=e.id), 'on_top_of')
        self._robot.ed.update_kinect("{} {}".format('on_top_of', e.id))
        frame_updated = self._robot.ed.get_entity(id=e.id)._pose

        # update entity with original orientation
        frame_updated.M = frame_original.M
        new_frame = FrameStamped(frame_updated, "map")

        # new_frame.header.stamp
        print("{}".format(new_frame))
        self._robot.ed.update_entity(self._robot, e.id, frame_stamped=new_frame)
        if e:
            return "succeeded"
        else:
            return "failed"


class GrabTrash(smach.State):
    """
    Set the arm in the prepare grasp position so it can safely go to the grasp position
    """
    def __init__(self, robot, arm_designator):
        """
        :param robot: robot object
        :param arm_designator: arm designator resolving to the arm with which to grab
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        self._robot = robot
        self._arm_designator = arm_designator

    def execute(self, userdata=None):

        arm = self._arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        # Torso up (non-blocking)
        self._robot.torso.reset()

        # Arm to position in a safe way
        arm.send_joint_goal('handover')
        arm.wait_for_motion_done()

        # Force drive to get closer to bin
        self._robot.base.force_drive(0.15, 0, 0, 0.5)

        # Open gripper
        arm.send_gripper_goal('open')
        arm.wait_for_motion_done()

        # Send to grab trash pose
        arm.send_joint_goal('grab_bag_trash_bin')
        arm.wait_for_motion_done()

        # Go down and grab
        self._robot.torso.send_goal("grab_trash_down")
        self._robot.torso.wait_for_motion_done()
        arm.send_gripper_goal('close')
        arm.wait_for_motion_done()

        # Go up and back to pre grasp position
        self._robot.torso.send_goal("grab_trash_up")
        self._robot.torso.wait_for_motion_done()
        arm.send_joint_goal('handover')
        arm.wait_for_motion_done()

        # Go back and pull back arm
        self._robot.base.force_drive(-0.125, 0, 0, 2.0)
        arm.send_joint_goal('reset')
        arm.wait_for_motion_done()

        handed_entity = EntityInfo(id="trash")
        arm.occupied_by = handed_entity

        return "succeeded"


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


class PickUpTrash(smach.StateMachine):
    """
    State that makes the robot go to the correct position in front of trash bin and grabs the trash
    """
    def __init__(self, robot, trashbin_designator, arm_designator):
        """
        :param robot: robot object
        :param trashbin_designator: EdEntityDesignator designating the trashbin
        :param arm_designator: arm designator resolving to the arm with which to grab
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        with self:
            smach.StateMachine.add("GO_BIN",
                                   states.NavigateToSymbolic(robot, {trashbin_designator: 'in_front_of'},
                                                             trashbin_designator),
                                   transitions={"arrived": "GET_BIN_POSITION",
                                                "goal_not_defined": "aborted",
                                                "unreachable": "failed"})

            smach.StateMachine.add("GET_BIN_POSITION", GetTrashBin(robot=robot, trashbin=trashbin_designator),
                                   transitions={"succeeded": "GO_TO_NEW_BIN",
                                                "failed": "failed"})

            smach.StateMachine.add("GO_TO_NEW_BIN",
                                   states.NavigateToObserve(robot, trashbin_designator, radius=0.4),
                                   transitions={"arrived": "PREPARE_AND_GRAB",
                                                "goal_not_defined": "aborted",
                                                "unreachable": "ASK_HANDOVER"})

            smach.StateMachine.add("PREPARE_AND_GRAB", GrabTrash(robot=robot, arm_designator=arm_designator),
                                   transitions={"succeeded": "succeeded",
                                                "failed": "ASK_HANDOVER"})

            # Ask human to handover the trash bag
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
                                   transitions={'spoken': 'succeeded'})

            smach.StateMachine.add("TIMEOUT", states.Say(robot, "I have not received anything, so I will stop",
                                                         block=False),
                                   transitions={'spoken': "failed"})


