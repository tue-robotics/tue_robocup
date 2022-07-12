# ROS
from pykdl_ros import FrameStamped
import rospy
import os
import rospkg
import numpy
import smach
from geometry_msgs.msg import WrenchStamped
import math

# TU/e
from smach import cb_interface, CBState

import robot_smach_states as states
from robot_smach_states.world_model import look_at_segmentation_area
import robot_smach_states.util.designators as ds
import robot_smach_states.manipulation as manipulation
from robot_skills.arm.arms import PublicArm
from robot_smach_states.manipulation.place_designator import EmptySpotDesignator
from challenge_take_out_the_garbage.control_to_trash_bin import ControlToTrashBin

from ed_msgs.msg import EntityInfo

from robot_skills.arm.force_sensor import TimeOutException


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
        e = self._trashbin.resolve()

        if not e:
            rospy.logerror("trashbin designator did not resolve")
            return 'failed'

        # Inspect and update entity
        look_at_segmentation_area(self._robot, self._robot.ed.get_entity(uuid=e.uuid), 'on_top_of')
        self._robot.ed.update_kinect("{} {}".format('on_top_of', e.uuid))

        return "succeeded"


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

            # arm.gripper.send_goal('open')
            # arm.wait_for_motion_done()
            # arm.gripper.send_goal('close', max_torque=1.0)
            # arm.wait_for_motion_done()

            arm_weight = measure_force.get_force()
            rospy.loginfo("Empty weight %s", arm_weight)

            # Open gripper
            arm.gripper.send_goal('open')
            arm.wait_for_motion_done()

            # Go down and grab
            try:
                arm.move_down_until_force_sensor_edge_up(timeout=5, distance_move_down=0.3)
            except TimeOutException:
                rospy.logwarn("No forces were felt, however no action is taken!")
                pass

            # self._robot.torso.send_goal("grab_trash_down")
            # self._robot.torso.wait_for_motion_done()
            # rospy.sleep(3)
            arm.gripper.send_goal('close', max_torque=1.0)
            arm.wait_for_motion_done()

            # Go up and back to pre grasp position
            self._robot.torso.send_goal("grab_trash_up")
            self._robot.torso.wait_for_motion_done()

            arm_with_object_weight = measure_force.get_force()
            rospy.loginfo("Full weight %s", arm_with_object_weight)
            weight_object = numpy.linalg.norm(numpy.subtract(arm_weight, arm_with_object_weight)) / gravitation
            rospy.loginfo("weight_object = {}".format(weight_object))

        # Make space for arm (So it doesn't hit the wall behind trashbin)
        arm._arm._send_joint_trajectory([[0.85, -2.2, 0.0, -0.85, 0.0]])
        self._robot.head.look_up()
        self._robot.head.wait_for_motion_done()
        self._robot.base.force_drive(-0.07, 0, 0, 2.0)

        # Lift bag up
        arm._arm._send_joint_trajectory(
            [
                [0.8, -2.2, 0.0, -0.85, 0.0],
                [0.8, -1, 0.0, -0.85, 0.0]
            ]
        )
        arm.wait_for_motion_done()

        # Go back and pull back arm
        self._robot.head.look_up()
        self._robot.head.wait_for_motion_done()
        self._robot.base.force_drive(-0.1, 0, 0, 2.0)

        arm.send_joint_goal('handover')
        arm.wait_for_motion_done()
        arm.send_joint_goal('reset')
        arm.wait_for_motion_done()
        self._robot.head.reset()
        #
        # if weight_object < self._minimal_weight:
        #     return "failed"

        self._robot.speech.speak("Look at this, I can pick up the trash!")
        handed_entity = ds.EntityByIdDesignator(robot=self._robot, uuid="trash").resolve()  # type: Entity
        arm.gripper.occupied_by = handed_entity

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

            smach.StateMachine.add("SAY1", states.human_interaction.Say(robot,
                                                                        'Please hand over the trash by putting the top of the bag'
                                                                        ' between my grippers and push firmly into my camera as'
                                                                        ' will be shown on my screen.'),
                                   transitions={'spoken': 'SHOW_IMAGE'})

            smach.StateMachine.add("SHOW_IMAGE",
                                   states.human_interaction.ShowImageState(
                                       robot=robot,
                                       image_filename=os.path.join(
                                           rospkg.RosPack().get_path("challenge_take_out_the_garbage"),
                                           "src",
                                           "challenge_take_out_the_garbage",
                                           "beun_picture.png"
                                       ),
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

        if not place_pose_designator:
            rospy.loginfo("Cannot resolve place_pose_designator")

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
                                   states.navigation.NavigateToSymbolic(robot=robot, entity_designator_area_name_map={
                                       trashbin_designator: "in_front_of"},
                                                                        entity_lookat_designator=trashbin_designator),
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
                    [
                        [0.01, 0.0, -1.57, -1.57, 0.0],
                        [0.59, 0.0, -1.57, -1.57, 0.0],
                        [0.55, -2.2, -1.57, -1.57, 0.],
                        [0.55, -2.2, 0.0, -0.85, 0.]
                    ]
                )
                arm.wait_for_motion_done()
                return 'done'

            smach.StateMachine.add("JOINT_PATH",
                                   CBState(_joint_path),
                                   transitions={'done': 'GO_TO_NEW_BIN'})

            smach.StateMachine.add("GO_TO_NEW_BIN",
                                   ControlToTrashBin(robot=robot, trashbin_id=trashbin_designator.uuid, radius=0.4,
                                                     yaw_offset=-0.2),
                                   transitions={"done": "PREPARE_AND_GRAB"})

            smach.StateMachine.add("PREPARE_AND_GRAB", GrabTrash(robot=robot, arm_designator=arm_designator),
                                   transitions={"succeeded": "succeeded",
                                                "failed": "ANNOUNCE_PICKUP_FAIL"})

            smach.StateMachine.add("ANNOUNCE_PICKUP_FAIL",
                                   states.human_interaction.Say(robot,
                                                                "Unfortunately I could not pick up the trash myself, let's go to"
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

            smach.StateMachine.add("LOWER_ARM", states.manipulation.ArmToJointConfig(robot=robot,
                                                                                     arm_designator=arm_occupied_designator,
                                                                                     configuration="reset"),
                                   transitions={"succeeded": "RECEIVED_TRASH_BAG",
                                                "failed": "RECEIVED_TRASH_BAG"})

            smach.StateMachine.add("RECEIVED_TRASH_BAG",
                                   states.human_interaction.Say(robot, "I received the thrash bag. I will throw"
                                                                       " it away, please move away.", block=True),
                                   transitions={'spoken': 'succeeded'})

            smach.StateMachine.add("TIMEOUT", states.human_interaction.Say(robot,
                                                                           "I have not received anything, so I will just continue",
                                                                           block=False),
                                   transitions={'spoken': "failed"})


if __name__ == '__main__':
    import os
    import robot_smach_states.util.designators as ds
    from robot_skills import get_robot
    from robot_skills.arm import arms

    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot_instance = get_robot("hero")
    robot_instance.reset()

    arm = ds.UnoccupiedArmDesignator(
        robot_instance,
        {
            "required_goals": ["reset", "handover"],
            "force_sensor_required": True,
            "required_gripper_types": [arms.GripperTypes.GRASPING]
        }
    )

    GrabTrash(robot_instance, arm, 100, 2).execute()
