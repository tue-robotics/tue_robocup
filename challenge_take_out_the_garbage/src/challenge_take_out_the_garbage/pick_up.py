# ROS
import rospy
import smach

# TU/e
import robot_skills
import robot_smach_states as states
import robot_smach_states.util.designators as ds

from robot_skills.util.kdl_conversions import FrameStamped, kdl_frame_from_XYZRPY


class GetTrashBin(smach.State):
    """
    Get the entity of the trashbin in FrameStamped and save it to get the orientation of the fixed point in world model
    """
    def __init__(self, robot, trashbin):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self._robot = robot
        self._trashbin = trashbin

    def execute(self, userdata=None):
        e = self._trashbin.resolve()
        # get original entity pose
        print("{}".format(type(self._robot.ed.get_entity(id=e.id)._pose)))

        frame_original = self._robot.ed.get_entity(id=e.id)._pose

        # inspect and update entity
        states.look_at_segmentation_area(self._robot,  self._robot.ed.get_entity(id=e.id), 'on_top_of')
        self._robot.ed.update_kinect("{} {}".format('on_top_of', e.id))
        frame_updated = self._robot.ed.get_entity(id=e.id)._pose

        # update entity with original orientation
        test = frame_updated
        test.M = frame_original.M
        new_frame = FrameStamped(test, "map")

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
        arm.send_joint_goal('handover', timeout=0)
        arm.wait_for_motion_done()

        # Open gripper
        arm.send_gripper_goal('open', timeout=0.0)
        arm.wait_for_motion_done()

        # Send to grab trash pose
        arm.send_joint_goal('grab_bag_trash_bin', timeout=0)
        arm.wait_for_motion_done()

        # Go down and grab
        self._robot.torso.send_goal("grab_trash_down")
        self._robot.torso.wait_for_motion_done()
        arm.send_gripper_goal('close', timeout=0)
        arm.wait_for_motion_done()

        # Go up and back to pre grasp position
        self._robot.torso.send_goal("grab_trash_up")
        self._robot.torso.wait_for_motion_done()
        arm.send_joint_goal('prepare_grasp', timeout=0)
        arm.wait_for_motion_done()

        # Go back and pull back arm
        self._robot.base.force_drive(-0.125, 0, 0, 2.0)
        arm.send_joint_goal('reset', timeout=0)
        arm.wait_for_motion_done()


        return "succeeded"

class PickUpTrash(smach.StateMachine):

    def __init__(self, robot, trashbin_designator, arm_designator):
        """

        :param robot: robot object
        :param trashbin_designator: EdEntityDesignator designating the trashbin
        :param trash_designator: EdEntityDesignator designating the trash
        :param drop_designator: EdEntityDesignator designating the collection zone
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        with self:
            smach.StateMachine.add("GO_BIN",
                                   states.NavigateToObserve(robot, trashbin_designator, radius=0.5),
                                   transitions={"arrived": "PICK_UP_TRASH",
                                                "goal_not_defined": "aborted",
                                                "unreachable": "failed"})

            smach.StateMachine.add("PICK_UP_TRASH", GetTrashBin(robot=robot, trashbin=trashbin_designator),
                                   transitions={"succeeded": "GO_TO_NEW_BIN",
                                                "failed": "failed"})

            smach.StateMachine.add("GO_TO_NEW_BIN",
                                   states.NavigateToObserve(robot, trashbin_designator, radius=0.4),
                                   transitions={"arrived": "PREPARE_AND_GRAB",
                                                "goal_not_defined": "aborted",
                                                "unreachable": "failed"})

            smach.StateMachine.add("PREPARE_AND_GRAB", GrabTrash(robot=robot, arm_designator=arm_designator),
                                   transitions={"succeeded": "succeeded",
                                                "failed": "failed"})
