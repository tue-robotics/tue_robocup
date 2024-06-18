from __future__ import absolute_import
import numpy as np

# ROS
import PyKDL as kdl
from pykdl_ros import VectorStamped, FrameStamped
import rospy
import smach
import tf2_ros

# TU/e Robotics
from challenge_restaurant.Arucomarker import get_aruco_pos
from robot_skills.robot import Robot
from robot_skills.arm.arms import PublicArm, GripperTypes
from robot_smach_states.utility  import check_arm_requirements, ResolveArm
from robot_smach_states.util.designators import check_type
from robot_smach_states.util.designators.arm import ArmDesignator
from robot_smach_states.util.designators.core import Designator
from smach import cb_interface, CBState


class PickUpArucomarker(smach.State):
    REQUIRED_ARM_PROPERTIES = {"required_goals": ["carrying_pose"], }

    def __init__(self, robot: Robot, arm: ArmDesignator) -> None:
        """
        Pick up an item given an arm and an entity to be picked up

        :param robot: robot to execute this state with
        :param arm: Designator that resolves to the arm to grasp with
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        # Assign member variables
        self.robot = robot
        self.arm_designator = arm

        assert self.robot.get_arm(**self.REQUIRED_ARM_PROPERTIES) is not None, \
            "None of the available arms meets all this class's requirements: {}".format(self.REQUIRED_ARM_PROPERTIES)

    def execute(self, userdata=None) -> str:
        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr('Transformation of goal to base failed: {0}'.format(tfe))
            return "failed"

        # Make sure the torso and the arm are done
        self.robot.torso.wait_for_motion_done(cancel=True)
        arm.wait_for_motion_done(cancel=True)

        # This is needed because the head is not entirely still when the
        # look_at_point function finishes
        rospy.sleep(rospy.Duration(0.5))


### TO DO: dit ergens anders
        aruco_type = "DICT_5X5_50"

        intrinsic_camera = np.array(((568.09415192, 0, 343.88872714), (0,  568.09415192, 211.47931024), (0, 0, 1)))
        distortion = np.array((1.79783341e+01, -1.56647194e+02, -1.27060586e-02, 3.36172848e-02,
        9.10987308e+02, 1.77606753e+01, -1.54458073e+02, 8.87397178e+02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        vector = get_aruco_pos(self.robot, aruco_type, intrinsic_camera, distortion)
        if vector is False:
            return 'failed'
####
        goal_map = FrameStamped.from_xyz_rpy(vector.x, vector.y, vector.z, vector.roll * np.pi / 180,
                                             (vector.pitch * np.pi / 180) - np.pi / 2,
                                             (vector.yaw * np.pi / 180) - np.pi / 2, rospy.Time.now(),
                                             "head_rgbd_sensor_rgb_frame")

        try:
            # Transform to base link frame
            goal_bl = self.robot.tf_buffer.transform(goal_map, "map", timeout=rospy.Duration(1.0))
            if goal_bl is None:
                rospy.logerr('Transformation of goal to (0,0) failed')
                return 'failed'
        except tf2_ros.TransformException as tfe:
            rospy.logerr('Transformation of goal to base failed: {0}'.format(tfe))
            return 'failed'

        # Grasp
        rospy.loginfo('Start grasping')
        if not arm.send_goal(goal_bl, timeout=20, pre_grasp=True,):
            self.robot.speech.speak('I am sorry but I cannot move my arm to the object position', block=False)
            rospy.logerr('Grasp failed')
            arm.reset()
            arm.gripper.send_goal('close', timeout=0.0)
            return 'failed'

        # Close gripper
        arm.gripper.send_goal('close')

        goal_bl.frame.p.x(goal_bl.frame.p.x() - 0.1)  # Retract 10 cm
        goal_bl.frame.p.z(goal_bl.frame.p.z() + 0.1)  # Go 10 cm higher
        rospy.loginfo("Start retract")
        if not arm.send_goal(goal_bl, timeout=0.0):
            rospy.logerr('Failed retract')
        arm.wait_for_motion_done()
        self.robot.base.force_drive(-0.125, 0, 0, 2.0)

        arm.wait_for_motion_done(cancel=True)

        # Carrying pose
        rospy.loginfo('start moving to carrying pose')
        arm.send_joint_goal('carrying_pose', timeout=0.0)

        result = 'succeeded'

        # Reset head
        self.robot.head.cancel_goal()

        return result


class ResetOnFailure(smach.State):
    """ Class to reset the robot after a grab has failed """

    REQUIRED_ARM_PROPERTIES = {"required_gripper_types": [GripperTypes.GRASPING], }

    def __init__(self, robot, arm):
        """
        Constructor

        :param robot: robot object
        :param arm: arm designator
        """
        smach.State.__init__(self, outcomes=['done'])

        self._robot = robot
        self.arm_designator = arm

    def execute(self, userdata=None):
        """ Execute hook """
        arm = self.arm_designator.resolve()
        arm.reset()

        if self._robot.robot_name == "amigo":
            self._robot.torso.reset()  # Move up to make resetting of the arm safer.
        if arm is not None:
            arm.gripper.send_goal('close')
        self._robot.head.reset()  # Sends a goal
        self._robot.head.cancel_goal()  # And cancels it...
        if arm is not None:
            arm.reset()
        self._robot.torso.reset()
        return 'done'


class GrabBasket(smach.StateMachine):
    REQUIRED_ARM_PROPERTIES = {"required_trajectories": ["prepare_grasp"], }

    def __init__(self, robot: Robot, arm: ArmDesignator):
        """
        Let the given robot grab an object with an arucomarker on it

        :param robot: Robot to use
        :param arm: Designator that resolves to the arm to use for grabbing. E.g. UnoccupiedArmDesignator

        """
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        # Check types or designator resolve types

        check_type(arm, PublicArm)
        self.robot = robot

        assert self.robot.get_arm(**self.REQUIRED_ARM_PROPERTIES) is not None, \
            "None of the available arms meets all this class's requirements: {}".format(self.REQUIRED_ARM_PROPERTIES)

        @cb_interface(outcomes=['succeeded'])
        def prepare_grasp(ud):

            #Torso up (non-blocking)
            #robot.torso.high()

            # Arm to position in a safe way
            #arm.resolve().send_joint_trajectory('prepare_grasp', timeout=0)

            # Open gripper
            arm.resolve().gripper.send_goal('open', timeout=0.0)
            return 'succeeded'

        with self:
            smach.StateMachine.add('RESOLVE_ARM', ResolveArm(arm, self),
                                   transitions={'succeeded': 'PREPARE_GRASP',
                                                'failed': 'failed'})

            smach.StateMachine.add("PREPARE_GRASP", CBState(prepare_grasp),
                                   transitions={'succeeded': 'GRAB'})

            ## To do: add navigate to grasp to define contraints

            smach.StateMachine.add('GRAB', PickUpArucomarker(robot, arm),
                                   transitions={'succeeded': 'done',
                                                'failed': 'RESET_FAILURE'})

            smach.StateMachine.add("RESET_FAILURE", ResetOnFailure(robot, arm),
                                   transitions={'done': 'failed'})

if __name__ == '__main__':
    from robot_skills import get_robot
    from robot_smach_states.util.designators import ArmDesignator, LockingDesignator

    rospy.init_node("test_grab_basket")

    robot = get_robot('hero')
    arm_des = ArmDesignator(robot)
    lock_arm_des = LockingDesignator(arm_des)
    sm = GrabBasket(robot, lock_arm_des)
    sm.execute()
