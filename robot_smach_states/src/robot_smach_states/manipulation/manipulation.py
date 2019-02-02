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



# TODO: poses to move to robot_description:
# carrying_pose: 0.18, y_home, 0.75, 0, 0, 0, 60
# handover_pose: 0.6, y_home, 0.966, 0, 0, 0, 30
# handover_to_human: -0.2, -0.7, 0.2, 2.0, 0, 0.5, 0.3
# prepare_grasp: -0.2, -0.044, 0.69, 1.4, -0.13, 0.38, 0.42
# retract: -0.1, 0.0, 0.0, 0.0, 0.0, 0.0
#
# TODO: trajectories to move to robot_description:
#
#
#
#
#
#

class ArmToJointConfig(smach.State):
    def __init__(self, robot, arm_designator, configuration):
        """
        Put arm of robot in some joint configuration
        :param robot: robot to execute state with
        :param arm_designator: designator that resolves to arm to put in given configuration
        :param configuration: joint configuration to put arm in
        """
        smach.State.__init__(self, outcomes=['succeeded','failed'])

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


class HandoverFromHuman(smach.StateMachine):
    '''
    State that enables low level grab reflex. Besides a robot object, needs
    an arm and an entity to grab, which is either one from ed through the
    grabbed_entity_designator or it is made up in the
    CloseGripperOnHandoverToRobot state and given the grabbed_entity_label
    as id.
    '''
    def __init__(self, robot, arm_designator, grabbed_entity_label="", grabbed_entity_designator=None, timeout=10, arm_configuration="handover_to_human"):
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

        check_type(arm_designator, PublicArm)
        if not grabbed_entity_designator and grabbed_entity_label == "":
            rospy.logerr("No grabbed entity label or grabbed entity designator given")

        with self:
            smach.StateMachine.add("POSE", ArmToJointConfig(robot, arm_designator, arm_configuration),
                            transitions={'succeeded':'OPEN_BEFORE_INSERT','failed':'OPEN_BEFORE_INSERT'})

            smach.StateMachine.add( 'OPEN_BEFORE_INSERT', SetGripper(robot, arm_designator, gripperstate=GripperState.OPEN),
                                transitions={'succeeded'    :   'SAY1',
                                             'failed'       :   'SAY1'})

            smach.StateMachine.add("SAY1", Say(robot,'Please hand over the object by pushing it gently in my gripper'),
                            transitions={'spoken':'CLOSE_AFTER_INSERT'})

            smach.StateMachine.add( 'CLOSE_AFTER_INSERT', CloseGripperOnHandoverToRobot(robot,
                                                                                        arm_designator,
                                                                                        grabbed_entity_label=grabbed_entity_label,
                                                                                        grabbed_entity_designator=grabbed_entity_designator,
                                                                                        timeout=timeout),
                                transitions={'succeeded'    :   'succeeded',
                                             'timeout'      :   'timeout',
                                             'failed'       :   'failed'})


class HandoverToHuman(smach.StateMachine):
    def __init__(self, robot, arm_designator, timeout=10):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        #A designator can resolve to a different item every time its resolved. We don't want that here, so lock
        check_type(arm_designator, PublicArm)
        locked_arm = LockingDesignator(arm_designator)

        with self:
            smach.StateMachine.add("LOCK_ARM",
                        LockDesignator(locked_arm),
                        transitions={'locked'         :'SPINDLE_MEDIUM'})

            smach.StateMachine.add("SPINDLE_MEDIUM",
                        ResetPart(robot, robot.torso),
                        transitions={'done'         :'MOVE_HUMAN_HANDOVER_JOINT_GOAL'})

            smach.StateMachine.add("MOVE_HUMAN_HANDOVER_JOINT_GOAL",
                        ArmToJointConfig(robot, locked_arm, 'handover_to_human'),
                        transitions={ 'succeeded'   :'SAY_OPEN_GRIPPER',
                                      'failed'      :'SAY_OPEN_GRIPPER'})

            smach.StateMachine.add("SAY_OPEN_GRIPPER",
                        Say(robot, [ "Watch out, I will open my gripper in one second. Please take it from me."]),
                        transitions={   'spoken'    :'OPEN_GRIPPER_HANDOVER'})

            smach.StateMachine.add('OPEN_GRIPPER_HANDOVER', SetGripper(robot, locked_arm, gripperstate=GripperState.OPEN, timeout=2.0),
                        transitions={'succeeded'    :   'SAY_CLOSE_NOW_GRIPPER',
                                     'failed'       :   'SAY_CLOSE_NOW_GRIPPER'})

            smach.StateMachine.add("SAY_CLOSE_NOW_GRIPPER",
                        Say(robot, [ "I will close my gripper now"]),
                        transitions={   'spoken'    :'CLOSE_GRIPPER_HANDOVER'})

            # smach.StateMachine.add('OPEN_GRIPPER_ON_HANDOVER', OpenGripperOnHandoverToHuman(robot, locked_arm, timeout=timeout),
            #             transitions={'succeeded'    :   'CLOSE_GRIPPER_HANDOVER',
            #                          'failed'       :   'SAY_I_WILL_KEEP_IT'})

            smach.StateMachine.add('CLOSE_GRIPPER_HANDOVER', SetGripper(robot, locked_arm, gripperstate=GripperState.CLOSE, timeout=0.0),
                        transitions={'succeeded'    :   'RESET_ARM',
                                     'failed'       :   'RESET_ARM'})

            # smach.StateMachine.add("SAY_I_WILL_KEEP_IT",
            #             Say(robot, [ "If you don't want it, I will keep it"]),
            #             transitions={   'spoken'    :'RESET_ARM'})

            smach.StateMachine.add('RESET_ARM',
                        ArmToJointConfig(robot, locked_arm, 'reset'),
                        transitions={'succeeded'    :'RESET_TORSO',
                                      'failed'      :'RESET_TORSO'    })

            smach.StateMachine.add('RESET_TORSO',
                        ResetPart(robot, robot.torso),
                        transitions={'done':'UNLOCK_ARM'})

            smach.StateMachine.add("UNLOCK_ARM",
                        UnlockDesignator(locked_arm),
                        transitions={'unlocked'         :'succeeded'})



class OpenGripperOnHandoverToHuman(smach.State):
    def __init__(self, robot, arm_designator, timeout=10):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot
        self.arm_designator = arm_designator
        self.timeout = timeout

    def execute(self,userdata=None):
        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        if arm.handover_to_human(self.timeout):
            arm.occupied_by = None
            return "succeeded"
        else:
            return "failed"


class CloseGripperOnHandoverToRobot(smach.State):
    def __init__(self, robot, arm_designator, grabbed_entity_label="", grabbed_entity_designator=None, timeout=10):
        smach.State.__init__(self, outcomes=['succeeded','failed','timeout'])
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
            return "succeeded"
        else:
            return "timeout"


class SetGripper(smach.State):
    def __init__(self, robot, arm_designator, gripperstate=GripperState.OPEN, drop_from_frame=None, grab_entity_designator=None, timeout=10):
        smach.State.__init__(self, outcomes=['succeeded','failed'])

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
                rospy.logerr("Could not resolve {0}: {1}".format(self.grab_entity_designator))

            arm.occupied_by = entity

        if arm.send_gripper_goal(self.gripperstate, timeout=self.timeout):
            result = True
        else:
            result = False

        # ToDo: make sure things can get attached to the gripper in this state. Use userdata?
        if self.gripperstate == GripperState.OPEN:
            arm.occupied_by = None

        # ToDo: check for failed in other states
        if result:
            return 'succeeded'
        else:
            return 'failed'


class ArmToUserPose(smach.State):
    def __init__(self, side, x, y, z, roll=0, pitch=0, yaw=0, time_out=20, pre_grasp=False, frame_id="/amigo/base_link", delta=False):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.side = side
        self.x = x
        self.y = y
        self.z = z
        self.roll  = roll
        self.pitch = pitch
        self.yaw   = yaw
        self.time_out   = time_out
        self.pre_grasp = pre_grasp
        self.frame_id  = frame_id
        self.delta = delta

    def execute(self, userdata=None):
        if not self.delta:
            rospy.logwarn("Transforming to baselink, should become obsolete but this is not yet the case")
            #target_position_bl = transformations.tf_transform(target_position, "/map","/base_link", tf_listener=self.robot.tf_listener)
            if self.side.send_goal(self.x, self.y, self.z, self.roll, self.pitch, self.yaw,
                timeout=self.time_out,
                pre_grasp=self.pre_grasp,
                frame_id=self.frame_id):
                return 'succeeded'
            else:
                return 'failed'
        else:
            if self.side.send_delta_goal(self.x, self.y, self.z, self.roll, self.pitch, self.yaw,
                timeout=self.time_out,
                pre_grasp=self.pre_grasp,
                frame_id=self.frame_id):
                return 'succeeded'
            else:
                return 'failed'

class ArmToQueryPoint(smach.State):
    def __init__(self, robot, side, point_designator, time_out=20, pre_grasp=False, first_joint_pos_only=False):
        """Move the arm to de designated point
        @param point_designator resolves to a PointStamped the gripper should move to"""
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot                = robot
        self.side                 = side
        self.point_designator     = point_designator
        self.time_out             = time_out
        self.pre_grasp            = pre_grasp
        self.first_joint_pos_only = first_joint_pos_only

    def execute(self, userdata=None):
        # ToDo: check point_designator?
        goal = self.point_designator.resolve()
        if not goal:
            rospy.loginfo("point_designator {0} cannot be resolved: {1}".format(self.point_designator))
            return 'failed'

        rospy.loginfo("ArmToQueryPoint: goal = {0}".format(goal))

        # Note: answers are typically in "map"frame, check whether this works out
        rospy.logwarn("Transforming to base_link frame for amigo_arm_navigation")
        goal_bl = transformations.tf_transform(goal.point, goal.header.frame_id, "/amigo/base_link", tf_listener=self.robot.tf_listener)
        if goal_bl == None:
            return 'failed'

        if self.side.send_goal(goal_bl.x, goal_bl.y, goal_bl.z, 0, 0, 0,
            frame_id="/amigo/base_link",
            timeout=self.time_out,
            pre_grasp=self.pre_grasp,
            first_joint_pos_only=self.first_joint_pos_only):
            return 'succeeded'
        else:
            rospy.logwarn("Goal unreachable: {0}".format(goal_bl).replace("\n", " "))
            return 'failed'

class TorsoToUserPos(smach.State):
    def __init__(self, robot, torso_pos, time_out=0.0):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot
        self.torso_pos = torso_pos
        self.time_out = time_out

    def execute(self, userdata=None):
        if self.robot.torso.send_goal(self.torso_pos,timeout=self.time_out):
            return 'succeeded'
        else:
            return 'failed'
