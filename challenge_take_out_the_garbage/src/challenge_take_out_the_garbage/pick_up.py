# ROS
import rospy
import numpy
import smach
from geometry_msgs.msg import WrenchStamped

# TU/e
from smach import cb_interface, CBState

import robot_smach_states as states
import robot_smach_states.manipulation as manipulation
from robot_skills.arms import PublicArm
from robot_smach_states.manipulation.place_designator import EmptySpotDesignator
from challenge_take_out_the_garbage.control_to_trash_bin import ControlToTrashBin

from ed_msgs.msg import EntityInfo

from robot_skills.force_sensor import TimeOutException
from robot_skills.util.kdl_conversions import FrameStamped


class MeasureForce(object):
    """
    Measure the three forces in the arm
    """

    def __init__(self, robot):
        """
        Constructor

        :param robot: robot object
        """
        self._robot = robot

    def get_force(self):
        # ft_sensor_topic = '/'+self._robot.robot_name+'/wrist_wrench/raw'
        ft_sensor_topic = '/' + self._robot.robot_name + '/wrist_wrench/compensated'

        force_grab = rospy.wait_for_message(ft_sensor_topic, WrenchStamped)

        force_data_x = force_grab.wrench.force.x
        force_data_y = force_grab.wrench.force.y
        force_data_z = force_grab.wrench.force.z

        return [force_data_x, force_data_y, force_data_z]


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
        return "succeeded"
        e = self._trashbin.resolve()

        if not e:
            raise Exception("trashbin designator is empty")

        # get original entity pose
        frame_original = self._robot.ed.get_entity(id=e.id)._pose

        # inspect and update entity
        states.look_at_segmentation_area(self._robot, self._robot.ed.get_entity(id=e.id), 'on_top_of')
        self._robot.ed.update_kinect("{} {}".format('on_top_of', e.id))
        frame_updated = self._robot.ed.get_entity(id=e.id)._pose

        # update entity with original orientation
        frame_updated.M = frame_original.M
        new_frame = FrameStamped(frame_updated, "map")

        # new_frame.header.stamp
        self._robot.ed.update_entity(self._robot, e.id, frame_stamped=new_frame)
        if e:
            return "succeeded"
        else:
            return "failed"


class GrabTrash(smach.State):
    """
    Set the arm in the prepare grasp position so it can safely go to the grasp position
    """

    def __init__(self, robot, arm_designator, try_num=1, minimal_weight=0.1):
        """
        :param robot: robot object
        :param arm_designator: arm designator resolving to the arm with which to grab
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        self._robot = robot
        self._arm_designator = arm_designator
        self._try_num = try_num
        self._minimal_weight = minimal_weight

    def execute(self, userdata=None):

        arm = self._arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        gravitation = 9.81
        try_current = 0

        measure_force = MeasureForce(self._robot)
        # To be able to determine the weight of the object the difference between the default weight and the weight with
        # the object needs to be taken. Note that initially the weight with the object is set to the default weight to
        # be able to enter the while loop below.
        arm_weight = measure_force.get_force()
        arm_with_object_weight = arm_weight
        weight_object = numpy.linalg.norm(numpy.subtract(arm_weight, arm_with_object_weight)) / gravitation

        while weight_object < self._minimal_weight and try_current < self._try_num:
            if try_current == 0:
                self._robot.speech.speak("Let me try to pick up the garbage")
            else:
                self._robot.speech.speak("I failed to pick up the trash, let me try again")
                rospy.loginfo("The weight I felt is %s", weight_object)
            try_current += 1

            # This opening and closing is to make sure that the gripper is empty and closed before measuring the forces
            # It is necessary to close the gripper since the gripper is also closed at the final measurement

            # arm.send_gripper_goal('open')
            # arm.wait_for_motion_done()
            # arm.send_gripper_goal('close', max_torque=1.0)
            # arm.wait_for_motion_done()

            arm_weight = measure_force.get_force()
            rospy.loginfo("Empty weight %s", arm_weight)

            # Open gripper
            arm.send_gripper_goal('open')
            arm.wait_for_motion_done()

            # Go down and grab
            try:
                arm.move_down_until_force_sensor_edge_up(timeout=7)
            except TimeOutException:
                rospy.logwarn("No forces were felt, however no action is taken!")
                pass

            # self._robot.torso.send_goal("grab_trash_down")
            # self._robot.torso.wait_for_motion_done()
            # rospy.sleep(3)
            arm.send_gripper_goal('close', max_torque=1.0)
            arm.wait_for_motion_done()

            # Go up and back to pre grasp position
            self._robot.torso.send_goal("grab_trash_up")
            self._robot.torso.wait_for_motion_done()

            arm_with_object_weight = measure_force.get_force()
            rospy.loginfo("Full weight %s", arm_with_object_weight)
            weight_object = numpy.linalg.norm(numpy.subtract(arm_weight, arm_with_object_weight)) / gravitation
            rospy.loginfo("weight_object = {}".format(weight_object))

        # Go back and pull back arm
        self._robot.head.look_up()
        self._robot.head.wait_for_motion_done()
        self._robot.base.force_drive(-0.5, 0, 0, 2.0)

        arm.send_joint_goal('handover')
        arm.wait_for_motion_done()
        arm.send_joint_goal('reset')
        arm.wait_for_motion_done()
        self._robot.head.reset()
        #
        # if weight_object < self._minimal_weight:
        #     return "failed"

        self._robot.speech.speak("Look at this I can pick up the trash!")
        handed_entity = EntityInfo(id="trash")
        arm.occupied_by = handed_entity

        return "succeeded"


class HandoverFromHumanFigure(smach.StateMachine):
    """
    State that enables low level grab reflex. Besides a robot object, needs
    an arm and an entity to grab, which is either one from ed through the
    grabbed_entity_designator or it is made up in the
    CloseGripperOnHandoverToRobot state and given the grabbed_entity_label
    as id.
    """

    def __init__(self, robot, arm_designator, grabbed_entity_label="", grabbed_entity_designator=None, timeout=15,
                 arm_configuration="handover_to_human"):
        """
        Hold up hand to accept an object and close hand once something is inserted

        :param robot: Robot with which to execute this behavior
        :param arm_designator: ArmDesignator resolving to arm accept item into
        :param grabbed_entity_label: What ID to give a dummy item in case no grabbed_entity_designator is supplied
        :param grabbed_entity_designator: EntityDesignator resolving to the accepted item. Can be a dummy
        :param timeout: How long to hold hand over before closing without anything
        :param arm_configuration: Which pose to put arm in when holding hand up for the item.
        """
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed', 'timeout'])

        ds.check_type(arm_designator, PublicArm)
        if not grabbed_entity_designator and grabbed_entity_label == "":
            rospy.logerr("No grabbed entity label or grabbed entity designator given")

        with self:
            smach.StateMachine.add("POSE", manipulation.ArmToJointConfig(robot, arm_designator, arm_configuration),
                                   transitions={'succeeded': 'OPEN_BEFORE_INSERT', 'failed': 'OPEN_BEFORE_INSERT'})

            smach.StateMachine.add('OPEN_BEFORE_INSERT', manipulation.SetGripper(robot=robot,
                                                                                 arm_designator=arm_designator,
                                                                                 gripperstate=manipulation.GripperState.
                                                                                 OPEN),
                                   transitions={'succeeded': 'SAY1', 'failed': 'SAY1'})

            smach.StateMachine.add("SAY1", states.Say(robot, 'Please hand over the trash by putting the top of the bag'
                                                             ' between my grippers and push firmly into my camera as'
                                                             ' will be shown on my screen.'),
                                   transitions={'spoken': 'SHOW_IMAGE'})

            smach.StateMachine.add("SHOW_IMAGE",
                                   states.ShowImageState(
                                        robot=robot,
                                        image_filename="~/ros/kinetic/system/src/challenge_take_out_the_garbage/src"
                                                       "/challenge_take_out_the_garbage/beun_picture.png",
                                        seconds=5),
                                   transitions={'succeeded': 'CLOSE_AFTER_INSERT'})

            smach.StateMachine.add('CLOSE_AFTER_INSERT', manipulation.CloseGripperOnHandoverToRobot(
                robot,
                arm_designator,
                grabbed_entity_label=grabbed_entity_label,
                grabbed_entity_designator=grabbed_entity_designator,
                timeout=timeout),
                                   transitions={'succeeded': 'succeeded',
                                                'timeout': 'failed',
                                                'failed': 'failed'})


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
        place_pose_designator = EmptySpotDesignator(robot, trashbin_designator, arm_designator)

        with self:
            # @cb_interface(outcomes=['done'])
            # def _joint_goal(_):
            #     arm = arm_designator.resolve()
            #     if not arm:
            #         rospy.logerr("Could not resolve arm")
            #         return "failed"
            #     # Send to grab trash pose
            #     arm.send_joint_goal('grab_trash_bag')
            #     arm.wait_for_motion_done()
            #     return 'done'
            #
            # smach.StateMachine.add("JOINT_GOAL",
            #                        CBState(_joint_goal),
            #                        transitions={'done': 'GO_BIN'})

            smach.StateMachine.add("GO_BIN",
                                   states.NavigateToPlace(robot=robot, place_pose_designator=place_pose_designator,
                                                          arm_designator=arm_designator),
                                   transitions={"arrived": "GET_BIN_POSITION",
                                                "goal_not_defined": "aborted",
                                                "unreachable": "failed"})

            smach.StateMachine.add("GET_BIN_POSITION", GetTrashBin(robot=robot, trashbin=trashbin_designator),
                                   transitions={"succeeded": "JOINT_PATH",
                                                "failed": "failed"})

            @cb_interface(outcomes=['done'])
            def _joint_path(_):
                robot.head.cancel_goal()
                arm = arm_designator.resolve()
                if not arm:
                    rospy.logerr("Could not resolve arm")
                    return "failed"  # ToDo: fix
                # Send to grab trash pose
                arm._arm._send_joint_trajectory(
                    [[0.01, 0.0, -1.57, -1.57, 0.0],
                     [0.69, 0.0, -1.57, -1.57, 0.0],
                     [0.65, -2.2, -1.57, -1.57, 0.],
                     [0.65, -2.2, 0.0, -0.85, 0.]
                     ]
                )
                arm.wait_for_motion_done()
                return 'done'

            smach.StateMachine.add("JOINT_PATH",
                                   CBState(_joint_path),
                                   transitions={'done': 'GO_TO_NEW_BIN'})

            smach.StateMachine.add("GO_TO_NEW_BIN",
                                   ControlToTrashBin(robot=robot, trashbin_id=trashbin_designator.id, radius=0.4,
                                                     yaw_offset=-0.2),
                                   transitions={"done": "PREPARE_AND_GRAB"})

            smach.StateMachine.add("PREPARE_AND_GRAB", GrabTrash(robot=robot, arm_designator=arm_designator),
                                   transitions={"succeeded": "succeeded",
                                                "failed": "ANNOUNCE_PICKUP_FAIL"})

            smach.StateMachine.add("ANNOUNCE_PICKUP_FAIL",
                                   states.Say(robot, "Unfortunately I could not pick up the trash myself, let's go to"
                                                     "plan B!",
                                              block=False),
                                   transitions={'spoken': 'ASK_HANDOVER'})

            # Ask human to handover the trash bag
            smach.StateMachine.add("ASK_HANDOVER", HandoverFromHumanFigure(robot=robot, arm_designator=arm_designator,
                                                                           grabbed_entity_label='thrash'),
                                   transitions={"succeeded": "LOWER_ARM",
                                                "failed": "failed",
                                                "timeout": "TIMEOUT"})

            arm_occupied_designator = ds.OccupiedArmDesignator(robot=robot, arm_properties={"required_goals": ["reset"
                                                                                                               ]})

            smach.StateMachine.add("LOWER_ARM", states.ArmToJointConfig(robot=robot,
                                                                        arm_designator=arm_occupied_designator,
                                                                        configuration="reset"),
                                   transitions={"succeeded": "RECEIVED_TRASH_BAG",
                                                "failed": "RECEIVED_TRASH_BAG"})

            smach.StateMachine.add("RECEIVED_TRASH_BAG", states.Say(robot, "I received the thrash bag. I will throw"
                                                                           " it away, please move away.", block=True),
                                   transitions={'spoken': 'succeeded'})

            smach.StateMachine.add("TIMEOUT", states.Say(robot, "I have not received anything, so I will just continue",
                                                         block=False),
                                   transitions={'spoken': "failed"})


if __name__ == '__main__':
    import os
    import robot_smach_states.util.designators as ds
    from robot_skills import Hero, arms

    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    hero = Hero()
    hero.reset()

    arm = ds.UnoccupiedArmDesignator(hero, {"required_goals": ["reset", "handover"], "force_sensor_required": True,
                                            "required_gripper_types": [arms.GripperTypes.GRASPING]})

    # sm = HandoverFromHumanFigure(hero, arm, grabbed_entity_label='trash')
    # sm.execute()

    GrabTrash(hero, arm, 100, 2).execute()
