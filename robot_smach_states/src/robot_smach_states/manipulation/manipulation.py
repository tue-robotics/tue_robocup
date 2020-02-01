# ROS
import rospy
import smach

# TU/e Robotics
from ed_msgs.msg import EntityInfo
from robot_skills.arms import PublicArm
from robot_skills.arms import GripperState
from robot_skills.util import transformations
from robot_smach_states.human_interaction import Say
from robot_smach_states.reset import ResetPart
from robot_smach_states.utility import LockDesignator, UnlockDesignator
from robot_smach_states.util.designators import LockingDesignator
from robot_smach_states.util.designators import check_type


class ArmToJointConfig(smach.State):
    def __init__(self, robot, arm_designator, configuration):
        """
        Put arm of robot in some joint configuration

        :param robot: robot to execute state with
        :param arm_designator: designator that resolves to arm to put in given configuration
        :param configuration: joint configuration to put arm in
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        self.robot = robot
        check_type(arm_designator, PublicArm)
        self.arm_designator = arm_designator
        self.configuration = configuration

    def execute(self, userdata=None):
        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"
        if arm.send_joint_goal(self.configuration):
            return 'succeeded'
        return "failed"


class HandOverTo(smach.State):
    def __init__(self, robot, arm_designator, timeout=10):
        """
        Handover the object in arm to a human.

        :param robot: robot to execute state with
        :param arm_designator: designator that resolves to arm holding an object
        :param timeout: float time the operation may take
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        self.robot = robot
        check_type(arm_designator, PublicArm)
        self.arm_designator = arm_designator
        self.timeout = timeout

    def execute(self, userdata=None):
        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        object_in_arm = arm.occupied_by
        if not object_in_arm:
            rospy.logerr("Unoccupied arm sent to HandOverToHuman, arm should be occupied when entering this state.")

        self.robot.speech.speak("Please take the object from my gripper.", block=False)

        attempt = 0

        while not arm.handover_to_human(timeout=10) and attempt < 2:
            self.robot.speech.speak("Please take it from my gripper.", block=False)
            attempt += 1

        arm.wait_for_motion_done()
        self.robot.ed.update_entity(id=object_in_arm.id, action='remove')

        self.robot.speech.speak("I will open my gripper now.", block=False)
        arm.send_gripper_goal('open')
        arm.reset()
        arm.wait_for_motion_done()

        arm.occupied_by = None

        return 'succeeded'


class HandoverFromHuman(smach.StateMachine):
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

        check_type(arm_designator, PublicArm)
        if not grabbed_entity_designator and grabbed_entity_label == "":
            rospy.logerr("No grabbed entity label or grabbed entity designator given")

        with self:
            smach.StateMachine.add('POSE', ArmToJointConfig(robot, arm_designator, arm_configuration),
                                   transitions={'succeeded': 'OPEN_BEFORE_INSERT', 'failed': 'OPEN_BEFORE_INSERT'})

            smach.StateMachine.add('OPEN_BEFORE_INSERT', SetGripper(robot, arm_designator,
                                                                    gripperstate=GripperState.OPEN),
                                   transitions={'succeeded': 'SAY',
                                                'failed': 'SAY'})

            smach.StateMachine.add('SAY', Say(robot, 'Please hand over the object by pushing it gently in my gripper'),
                                   transitions={'spoken': 'CLOSE_AFTER_INSERT'})

            smach.StateMachine.add('CLOSE_AFTER_INSERT',
                                   CloseGripperOnHandoverToRobot(robot, arm_designator,
                                                                 grabbed_entity_label=grabbed_entity_label,
                                                                 grabbed_entity_designator=grabbed_entity_designator,
                                                                 timeout=timeout),
                                   transitions={'succeeded': 'succeeded',
                                                'timeout': 'timeout',
                                                'failed': 'failed'})


class HandoverToHuman(smach.StateMachine):
    def __init__(self, robot, arm_designator, timeout=10):
        """
        State to hand over the object in the arm to a human operator

        :param robot: robot to execute state with
        :param arm_designator: designator that resolves to arm holding the object
        :param timeout: float amount of time the procedure may take
        """
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        # A designator can resolve to a different item every time its resolved. We don't want that here, so lock
        check_type(arm_designator, PublicArm)
        locked_arm = LockingDesignator(arm_designator)

        with self:
            smach.StateMachine.add("LOCK_ARM", LockDesignator(locked_arm),
                                   transitions={'locked': 'SPINDLE_MEDIUM'})

            smach.StateMachine.add("SPINDLE_MEDIUM", ResetPart(robot, robot.torso),
                                   transitions={'done': 'MOVE_HUMAN_HANDOVER_JOINT_GOAL'})

            smach.StateMachine.add("MOVE_HUMAN_HANDOVER_JOINT_GOAL", ArmToJointConfig(robot, locked_arm,
                                                                                      'handover_to_human'),
                                   transitions={'succeeded': 'SAY_DETECT_HANDOVER',
                                                'failed': 'SAY_DETECT_HANDOVER'})

            smach.StateMachine.add("SAY_DETECT_HANDOVER", Say(robot, ["I will handover the object now"
                                                                      "Please take it from my gripper."]),
                                   transitions={'spoken': 'DETECT_HANDOVER'})

            smach.StateMachine.add("DETECT_HANDOVER", HandOverTo(robot, locked_arm),
                                   transitions={'succeeded': 'SAY_CLOSE_NOW_GRIPPER',
                                                'failed': 'SAY_CLOSE_NOW_GRIPPER'})

            smach.StateMachine.add("SAY_CLOSE_NOW_GRIPPER", Say(robot, ["I will close my gripper now"]),
                                   transitions={'spoken': 'CLOSE_GRIPPER_HANDOVER'})

            smach.StateMachine.add('CLOSE_GRIPPER_HANDOVER', SetGripper(robot, locked_arm,
                                                                        gripperstate=GripperState.CLOSE, timeout=0),
                                   transitions={'succeeded': 'RESET_ARM',
                                                'failed': 'RESET_ARM'})

            smach.StateMachine.add('RESET_ARM', ArmToJointConfig(robot, locked_arm, 'reset'),
                                   transitions={'succeeded': 'UNLOCK_ARM',
                                                'failed': 'UNLOCK_ARM'})

            smach.StateMachine.add("UNLOCK_ARM", UnlockDesignator(locked_arm),
                                   transitions={'unlocked': 'succeeded'})


class CloseGripperOnHandoverToRobot(smach.State):
    def __init__(self, robot, arm_designator, grabbed_entity_label="", grabbed_entity_designator=None, timeout=10):
        """
        State to wait until the operator pushes an object into the gripper

        :param robot: robot to execute state with
        :param arm_designator: designator that resolves to arm receiving the object
        :param grabbed_entity_label: label to assign the dummy entity representing the received object
                                    (use this ore grabbed_entity_designator)
        :param grabbed_entity_designator: designator resolving to the entity which will be received
                                    (use this ore grabbed_entity_designator)
        :param timeout: float amount of time the procedure may take
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'timeout'])
        self.robot = robot
        self.arm_designator = arm_designator
        self.timeout = timeout
        self.item_label = grabbed_entity_label
        self.item_designator = grabbed_entity_designator

    def execute(self, userdata=None):
        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        if self.item_designator:
            arm.occupied_by = self.item_designator.resolve()
        else:
            if self.item_label != "":
                handed_entity = EntityInfo(id=self.item_label)
                arm.occupied_by = handed_entity
            else:
                rospy.logerr("No grabbed entity designator and no label for dummy entity given")
                return "failed"

        if arm.handover_to_robot(self.timeout):
            arm.send_gripper_goal('close')
            rospy.sleep(2.0)
            return "succeeded"
        else:
            rospy.logwarn("Eventhough no force is felt the gripper will close")
            arm.send_gripper_goal('close')
            rospy.sleep(2.0)
            return "timeout"


class SetGripper(smach.State):
    def __init__(self, robot, arm_designator, gripperstate=GripperState.OPEN, grab_entity_designator=None, timeout=10):
        """
        Instruct the gripper

        :param robot: robot to execute state with
        :param arm_designator: designator that resolves to arm corresponding to the gripper
        :param gripperstate: desired state of the gripper
        :param grab_entity_designator: Designator resolving to the entity to be attached to the gripper
        :param timeout: float amount of time the procedure may take
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        check_type(arm_designator, PublicArm)
        self.arm_designator = arm_designator
        self.robot = robot
        self.gripperstate = gripperstate
        self.grab_entity_designator = grab_entity_designator
        self.timeout = timeout

    def execute(self, userdata=None):
        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        # If needs attaching to gripper, the grab_entity_designator is used
        if self.grab_entity_designator:
            entity = self.grab_entity_designator.resolve()
            if not entity:
                rospy.logerr("Could not resolve {0}".format(self.grab_entity_designator))

            if self.gripperstate != GripperState.OPEN:
                arm.occupied_by = entity

        if arm.send_gripper_goal(self.gripperstate, timeout=self.timeout):
            return 'succeeded'
        else:
            return 'failed'


class TorsoToUserPos(smach.State):
    def __init__(self, robot, torso_pos, time_out=0.0):
        """
        State to set the pose of the torso

        :param robot: robot to execute state with
        :param torso_pos: float desired position of the torso
        :param time_out: float amount of time the procedure may take
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.robot = robot
        self.torso_pos = torso_pos
        self.time_out = time_out

    def execute(self, userdata=None):
        if self.robot.torso.send_goal(self.torso_pos, timeout=self.time_out):
            return 'succeeded'
        else:
            return 'failed'
