#! /usr/bin/env python
import rospy
import smach
import tf

import ed.msg
import robot_skills.util.msg_constructors as msgs
import robot_skills.util.transformations as transformations

from robot_skills.arms import Arm
from robot_smach_states.state import State
from robot_smach_states.util.designators import check_type

from robot_smach_states.navigation import NavigateToGrasp
from robot_smach_states.manipulation.grasp_point_determination import GraspPointDeterminant

class PrepareEdGrasp(State):
    def __init__(self, robot, arm, grab_entity):
        # Check that the entity_designator resolves to an Entity or is an entity
        check_type(grab_entity, ed.msg.EntityInfo)

        # Check that the arm is a designator that resolves to an Arm or is an Arm
        check_type(arm, Arm)

        State.__init__(self, locals(), outcomes=['succeeded', 'failed'])

    def run(self, robot, arm, grab_entity):
        if not grab_entity:
            rospy.logerr("Could not resolve grab_entity")
            return "failed"
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        # Arm to position in a safe way
        arm.send_joint_trajectory('prepare_grasp')

        # Open gripper
        arm.send_gripper_goal('open', timeout=0.0)

        return 'succeeded'



class PickUp(State):
    def __init__(self, robot, arm, grab_entity):
        # Check that the entity_designator resolves to an Entity or is an entity
        check_type(grab_entity, ed.msg.EntityInfo)

        # Check that the arm is a designator that resolves to an Arm or is an Arm
        check_type(arm, Arm)

        State.__init__(self, locals(), outcomes=['succeeded', 'failed'])

    def run(self, robot, arm, grab_entity):
        if not grab_entity:
            rospy.logerr("Could not resolve grab_entity")
            return "failed"
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        rospy.loginfo('PickUp!')

        # Trigger perception once again to update object pose
        # robot.ed.segment_kinect(max_sensor_range=2)

        # goal in map frame
        goal_map = msgs.Point(0, 0, 0)

        try:
            # Transform to base link frame
            goal_bl = transformations.tf_transform(
                goal_map, grab_entity.id, robot.robot_name+'/base_link',
                tf_listener=robot.tf_listener
            )
            if goal_bl is None:
                rospy.logerr('Transformation of goal to base failed')
                return 'failed'
        except tf.Exception, tfe:
            rospy.logerr('Transformation of goal to base failed: {0}'.format(tfe))
            return 'failed'

        rospy.loginfo(goal_bl)

        # # Arm to position in a safe way
        # arm.send_joint_trajectory('prepare_grasp')

        # # Open gripper
        # arm.send_gripper_goal('open', timeout=0.0)

        # Pre-grasp
        rospy.loginfo('Starting Pre-grasp')
        if not arm.send_goal(goal_bl.x, goal_bl.y, goal_bl.z+0.1, 0, 0, 0,
                             frame_id='/'+robot.robot_name+'/base_link',
                             timeout=20, pre_grasp=True, first_joint_pos_only=True
                             ):
            rospy.logerr('Pre-grasp failed:')

            arm.reset()
            arm.send_gripper_goal('close', timeout=None)
            return 'failed'

        # Grasp
        if not arm.send_goal(goal_bl.x, goal_bl.y, goal_bl.z+0.08, 0, 0, 0,
                             frame_id='/'+robot.robot_name+'/base_link',
                             timeout=120, pre_grasp=True,
                             allowed_touch_objects=[grab_entity.id]
                             ):
            robot.speech.speak('I am sorry but I cannot move my arm to the object position', block=False)
            rospy.logerr('Grasp failed')
            arm.reset()
            arm.send_gripper_goal('close', timeout=None)
            return 'failed'

        # Close gripper
        arm.send_gripper_goal('close')

        arm.occupied_by = grab_entity

        # Lift
        if not arm.send_goal(goal_bl.x, goal_bl.y, goal_bl.z + 0.1, 0.0, 0.0, 0.0,
                             frame_id='/'+robot.robot_name+'/base_link',
                             timeout=20, allowed_touch_objects=[grab_entity.id]
                             ):
            rospy.logerr('Failed lift')

        robot.base.force_drive(-0.125,0,0,3)

        # Retract
        if not arm.send_goal(goal_bl.x - 0.1, goal_bl.y, goal_bl.z + 0.15, 0.0, 0.0, 0.0,
                             frame_id='/'+robot.robot_name+'/base_link',
                             timeout=20, allowed_touch_objects=[grab_entity.id]
                             ):
            rospy.logerr('Failed retract')

        # Carrying pose
        if arm.side == 'left':
            y_home = 0.2
        else:
            y_home = -0.2

        rospy.loginfo('y_home = ' + str(y_home))

        rospy.loginfo('start moving to carrying pose')
        if not arm.send_goal(0.18, y_home, 0.6, 0, 0.3, 0,
                             timeout=60
                             ):
            rospy.logerr('Failed carrying pose')

        return 'succeeded'


class Grab(smach.StateMachine):
    def __init__(self, robot, item, arm):
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        # Check types or designator resolve types
        check_type(item, ed.msg.EntityInfo)
        check_type(arm, Arm)

        with self:
            smach.StateMachine.add('PREPARE_GRASP', PrepareEdGrasp(robot, arm, item),
                                   transitions={ 'succeeded'    : 'NAVIGATE_TO_GRAB',
                                                 'failed'       : 'failed'})

            smach.StateMachine.add('NAVIGATE_TO_GRAB', NavigateToGrasp(robot, item, arm),
                                   transitions={'unreachable':      'failed',
                                                'goal_not_defined': 'failed',
                                                'arrived':          'GRAB'})

            smach.StateMachine.add('GRAB', PickUp(robot, arm, item),
                                   transitions={'succeeded': 'done',
                                                'failed':    'failed'})

# ----------------------------------------------------------------------------------------------------

class SjoerdsGrab(smach.State):

    def __init__(self, robot, item_des, arm_des):
        smach.State.__init__(self, outcomes=["done", 'failed'])
        self._robot = robot
        self.item_des = item_des
        self.arm_des = arm_des
        self._gpd = GraspPointDeterminant(robot)

    def execute(self, userdata=None):
        entity = self.item_des.resolve()

        self.fsm = NavigateToGrasp(self._robot, self.item_des, self.arm_des)
        self.fsm.execute()

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # Resolve entity and arm

        entity = self.item_des.resolve()
        arm = self.arm_des.resolve()

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        # Open gripper (non-blocking)
        arm.send_gripper_goal('open', timeout=0)

        # Torso up (non-blocking)
        self._robot.torso.high()

        # Arm to position in a safe way
        # arm._send_joint_trajectory([
        #     [-0.1,-0.6,0.1,1.2,0.0,0.1,0.0],
        #     [-0.1,-0.8,0.1,1.6,0.0,0.2,0.0],
        #     [-0.1,-1.0,0.1,2.0,0.0,0.3,0.0],
        #     [-0.1,-0.5,0.1,2.0,0.0,0.3,0.0],
        #     ])#, timeout=20)      

        arm._send_joint_trajectory([
            [-0.1,-1.0,0.1,2.0,0.0,0.3,0.0]], timeout=rospy.Duration(0))  

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # Update the pose of the entity

        # Make sure the head looks at the entity
        pos = entity.pose.position
        self._robot.head.look_at_point(msgs.PointStamped(pos.x, pos.y, pos.z, "/map"), timeout=10)

        # This is needed because the head is not entirely still when the
        # look_at_point function finishes
        import time
        time.sleep(1)

        # Inspect the entity
        segm_res = self._robot.ed.update_kinect("%s" % entity.id)

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # Grasp point determination
        grasp_pose = self._gpd.get_grasp_pose(entity, arm)        

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        # goal in map frame
        goal_map = msgs.Point(0, 0, 0)

        # Transform to base link frame
        goal_bl = transformations.tf_transform(goal_map, entity.id, "/amigo/base_link", tf_listener=self._robot.tf_listener)
        if goal_bl == None:
            return 'failed'

        # Pre-grasp
        if not arm.send_goal(goal_bl.x, goal_bl.y, goal_bl.z, 0, 0, 0, frame_id="/amigo/base_link", timeout=20, pre_grasp=True, first_joint_pos_only=True):
            print "Pre-grasp failed"
            arm.reset()
            arm.send_gripper_goal('close', timeout=0.01)
            self._robot.head.cancel_goal()
            return

        # Grasp
        if not arm.send_goal(goal_bl.x, goal_bl.y, goal_bl.z, 0, 0, 0, frame_id="/amigo/base_link", timeout=120, pre_grasp = True):
            self._robot.speech.speak("I am sorry but I cannot move my arm to the object position", block=False)
            print "Grasp failed"
            arm.reset()
            arm.send_gripper_goal('close', timeout=0.01)
            self._robot.head.cancel_goal()
            return "failed"

        # Close gripper
        arm.send_gripper_goal('close', timeout=5)

        # Cancel the head goal
        self._robot.head.cancel_goal()

        # Lift
        if not arm.send_goal( goal_bl.x, goal_bl.y, goal_bl.z + 0.1, 0.0, 0.0, 0.0, timeout=20, pre_grasp=False, frame_id="/amigo/base_link"):
            print "Failed lift"

        # Retract
        if not arm.send_goal(max(0.18, goal_bl.x - 0.25), goal_bl.y, goal_bl.z + 0.1, 0.0, 0.0, 0.0, timeout=20, pre_grasp=False, frame_id="/amigo/base_link"):
            print "Failed retract"

        # Carrying pose
        arm._send_joint_trajectory([[-0.1, -0.6, 0.2, 1.7, 0, 0.4, 0]])

        return "done"

