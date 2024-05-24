from __future__ import absolute_import

# ROS
import PyKDL as kdl
from pykdl_ros import VectorStamped, FrameStamped
import rospy
import smach
import tf2_ros
from geometry_msgs.msg import Twist
import time

# TU/e Robotics
from ed.entity import Entity
from robot_skills.robot import Robot
from robot_skills.arm.arms import PublicArm, GripperTypes
from robot_smach_states.utility import check_arm_requirements, ResolveArm
from robot_smach_states.util.designators import check_type
from robot_smach_states.navigation.navigate_to_grasp import NavigateToGrasp
#from robot_smach_states.manipulation.grasp_point_determination import GraspPointDeterminant
from robot_smach_states.util.designators.arm import ArmDesignator
from robot_smach_states.util.designators.core import Designator

from robot_smach_states.manipulation.cutlery_detector import YoloSegmentor
from robot_skills.util.exceptions import TimeOutException



class PrepareGrasp(smach.State):
    REQUIRED_ARM_PROPERTIES = {"required_gripper_types": [GripperTypes.GRASPING],}

    def __init__(self, robot: Robot, arm: ArmDesignator) -> None:
        """
        Set the arm in the appropriate position before actually grabbing

        :param robot: robot to execute state with
        :param arm: Designator that resolves to the arm to grasp with
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        # Assign member variables
        self.robot = robot
        self.arm_designator = arm

    def execute(self, userdata=None) -> str:
        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"        
        #pre-grasp position in two steps to limit possible collisions with the table
        pre_grasp_joint_goal_upwards = [0.69, # arm lift joint. ranges from 0.0 to 0.7m
                                        0.0, # arm flex joint. lower values move the arm downwards ranges from -2 to 0.0 radians
                                        0.0, # arm roll joint
                                        -1.57, # wrist flex joint. lower values move the hand down
                                        0.0] # wrist roll joint. 
        arm._arm._send_joint_trajectory([pre_grasp_joint_goal_upwards]) # send the command to the robot.
        arm.wait_for_motion_done()
        #only lifting the arm flex joint
        pre_grasp_joint_goal_outwards = [0.69, 
                                        -1.57,
                                        0.0, 
                                        -1.57, 
                                        0.0] 
        arm._arm._send_joint_trajectory([pre_grasp_joint_goal_outwards]) 
        # Open gripper
        arm.gripper.send_goal('open', timeout=0.0)
        arm.wait_for_motion_done()
        return 'succeeded'


class TopGrasp(smach.State):
    REQUIRED_ARM_PROPERTIES = {"required_gripper_types": [GripperTypes.GRASPING],
                               "required_goals": ["carrying_pose"], }

    def __init__(self, robot: Robot, arm: ArmDesignator, grab_entity: Designator) -> None:
        """
        Pick up an item given an arm and an entity to be picked up

        :param robot: robot to execute this state with
        :param arm: Designator that resolves to the arm to grasp with
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        # Assign member variables
        self.robot = robot
        self.arm_designator = arm

        assert self.robot.get_arm(**self.REQUIRED_ARM_PROPERTIES) is not None,\
            "None of the available arms meets all this class's requirements: {}".format(self.REQUIRED_ARM_PROPERTIES)
        check_type(grab_entity, Entity)
        self.grab_entity_designator = grab_entity
        self.yolo_segmentor = YoloSegmentor()

    def execute(self, userdata=None) -> str:
        grab_entity = self.grab_entity_designator.resolve()
        if not grab_entity:
            rospy.logerr("Could not resolve grab_entity")
            return "failed"

        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"
        
#### examples, this part can be commented out      
        # define a goal with respect to the entity.
        goal_map = FrameStamped.from_xyz_rpy(0, 0, 0, 0, 0, 0, rospy.Time(), frame_id=grab_entity.uuid)

        try:
            # Transform to goal to the base link frame
            goal_bl = self.robot.tf_buffer.transform(goal_map, self.robot.base_link_frame)
            if goal_bl is None:
                rospy.logerr('Transformation of goal to base failed')
                return 'failed'
        except tf2_ros.TransformException as tfe:
            rospy.logerr('Transformation of goal to base failed: {0}'.format(tfe))
            return 'failed'
        #rospy.loginfo(f"goal_bl = {goal_bl}")

        grasp_succeeded=False
        rate = rospy.Rate(10) # loop rate in hz

        # define a desired transformation from base to gripper
        base_to_gripper = self.frame_from_xyzrpy(0.5, # x distance to the robot
                                                0.08, # y distance off center from the robot (fixed if rpy=0)
                                                0.7, # z height of the gripper
                                                0, 0, 0) # Roll pitch yaw. 0,0,0 for a horizontal gripper.
# start segmentation
        self.yolo_segmentor.start()

        class_id = self.yolo_segmentor.data_class()

        #wait until cutlery is detected
        while class_id not in {42, 43, 44}: 
            rospy.sleep()


        #Obtain needed data from the cutlery detector
        x_center, y_center = self.yolo_segmentor.data_center()
 
        print(f"DATA FROM DETECTION: x_center: {x_center}, y_center: {y_center}, slope: {slope}, upwards: {upwards}")

    #rewrite camera frame into cooridnate frame

    
    #what is the orientation of the gripper coordinate frame

#move towards coordinates (with arm or with base?) will this even work with base or get stuck because too close to table

#rotate wrist according to orientation


#Moving arm downwards
        move_arm = True
        while not grasp_succeeded and not rospy.is_shutdown():
            # control loop

            #TODO get grasp pose wrt wrist

            #TODO force sensor does not provide a good interface for this.
            if (move_arm):
                downward_joint_goal = [0.0, # arm lift joint. ranges from 0.0 to 0.7m
                                       -1.57, # arm flex joint. lower values move the arm downwards ranges from -2 to 0.0 radians
                                       0.0, # arm roll joint
                                       -1.57, # wrist flex joint. lower values move the hand down
                                       0.0] # wrist roll joint. 
                arm._arm._send_joint_trajectory([downward_joint_goal]) # send the command to the robot.
                #Move arm downwards, don't wait until motion is done, but until a force is detected
                try:
                    arm._arm.force_sensor.wait_for_edge_up(3.0)  # wait 3 seconds for a force detection
                except TimeOutException:
                    rospy.loginfo("No edge up detected within timeout")

                #After force detection, make sure arm moves upwards
                grasp_joint_goal = [0.63, #change this in a position relative to obtained coordinates or table height
                                    -1.57, 
                                    0.0, 
                                    -1.57, 
                                    0.0]
                arm._arm._send_joint_trajectory([grasp_joint_goal]) # send the command to the robot.
                arm.wait_for_motion_done() # wait until the motion is complete
                move_arm = False # reset flag to move the arm.
                continue # dont wait for the rest of the loop.

#Closing the gripper = grasping
            arm.gripper.send_goal('close', timeout=0.0, max_torque = 0.1) # option given by max_torque to close the gripper with more force
            arm.wait_for_motion_done()

                        # check if done
            if False: # check if the grasp has succeeded
                grasp_succeeded = True
    #create a way out of the while loop by grasp_succeeded

#Move towards able edge, with base or with gripper
#TAKE MARGINS INTO ACCOUNT

# open gripper

            
#move gripper towards end point cutlery or choose a good position

#rotate wrist around the cutlery, pre-grasp

#close gripper

#lift up, go to original stance




            # example base command
            v = Twist()
            v.linear.x = 0 # forward
            v.linear.y = 0 # linear left
            v.angular.z = 0 # rotation speed to the left
            self.robot.base._cmd_vel.publish(v) # send command to the robot

            # example arm pose command
            if (move_arm):
                pose_goal = FrameStamped(base_to_gripper,
                                        rospy.Time.now(), #timestamp when this pose was created
                                        "base_link" # the frame in which the pose is expressed. base link lies in the center of the robot at the height of the floor.
                                        )
                arm.send_goal(pose_goal) # send the command to the robot.
                arm.wait_for_motion_done() # wait until the motion is complete
                move_arm = False # reset flag to move the arm.
                #continue # dont wait for the rest of the loop.

                # control the arm in joint space
                joint_goal = [0.5, # arm lift joint. ranges from 0.0 to 0.7m
                              -1.57, # arm flex joint. lower values move the arm downwards ranges from -2 to 0.0 radians
                              0.0, # arm roll joint
                              -1.57, # wrist flex joint. lower values move the hand down
                              0.0] # wrist roll joint. 
                arm._arm._send_joint_trajectory([joint_goal]) # send the command to the robot.
                arm.wait_for_motion_done() # wait until the motion is complete
                move_arm = False # reset flag to move the arm.
                continue # dont wait for the rest of the loop.

            # get base-gripper transform
            gripper_id = FrameStamped.from_xyz_rpy(0, 0, 0, 0, 0, 0, rospy.Time(), frame_id="hand_palm_link")
            gripper_bl = self.robot.tf_buffer.transform(gripper_id, self.robot.base_link_frame)
            gripper_bl.frame = arm._arm.offset.Inverse() * gripper_bl.frame  # compensate for the offset in hand palm link
            rospy.loginfo(f"gripper_bl = {gripper_bl}")

            # check if done
            if False: # check if the grasp has succeeded
                grasp_succeeded = True

            rospy.loginfo(f"print a message to show we are still running.")
            rate.sleep()
        # stop segmentation to preserve computation power
        self.yolo_segmentor.stop()

        return "succeeded"

    @staticmethod
    def frame_from_xyzrpy(x, y, z, roll, pitch, yaw):
        """
        Helper function to create a kdl frame based on an x,y,z position and a RPY rotation
        """
        return kdl.Frame(kdl.Rotation.RPY(roll, pitch, yaw), kdl.Vector(x, y, z))


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
            arm.gripper.send_goal('open')
        self._robot.head.reset()  # Sends a goal
        self._robot.head.cancel_goal()  # And cancels it...
        if arm is not None:
            arm.reset()
        self._robot.torso.reset()
        return 'done'


class TopGrab(smach.StateMachine):
    def __init__(self, robot: Robot, item: Designator, arm: ArmDesignator):
        """
        Let the given robot move to an entity and grab that entity using some arm

        :param robot: Robot to use
        :param item: Designator that resolves to the item to grab. E.g. EntityByIdDesignator
        :param arm: Designator that resolves to the arm to use for grabbing. E.g. UnoccupiedArmDesignator
        """
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        # Check types or designator resolve types
        check_type(arm, PublicArm)

        with self:
            smach.StateMachine.add('RESOLVE_ARM', ResolveArm(arm, self),
                                   transitions={'succeeded': 'NAVIGATE_TO_GRAB',
                                                'failed': 'failed'})

            smach.StateMachine.add('NAVIGATE_TO_GRAB', NavigateToGrasp(robot, arm, item),
                                   transitions={'unreachable': 'RESET_FAILURE',
                                                'goal_not_defined': 'RESET_FAILURE',
                                                'arrived': 'PREPARE_GRASP'})

            smach.StateMachine.add('PREPARE_GRASP', PrepareGrasp(robot, arm),
                                   transitions={'succeeded': 'GRAB',
                                                'failed': 'RESET_FAILURE'})

            smach.StateMachine.add('GRAB', TopGrasp(robot, arm, item),
                                   transitions={'succeeded': 'done',
                                                'failed': 'RESET_FAILURE'})

            smach.StateMachine.add("RESET_FAILURE", ResetOnFailure(robot, arm),
                                   transitions={'done': 'failed'})

        check_arm_requirements(self, robot)
