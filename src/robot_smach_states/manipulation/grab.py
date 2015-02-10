#! /usr/bin/env python
import rospy
import smach

# from geometry_msgs.msg import *
import robot_skills.util.msg_constructors as msgs
import robot_skills.util.transformations as transformations

from robot_smach_states.state import State

# from cb_planner_msgs_srvs.srv import *
# from cb_planner_msgs_srvs.msg import *

from robot_smach_states.navigation import NavigateToGrasp

# ----------------------------------------------------------------------------------------------------

class PickUp(State):
    def __init__(self, robot, arm, grab_entity):
        State.__init__(self, locals(), outcomes=['succeeded','failed'])

    def run(self, robot, arm, grab_entity):
        rospy.loginfo('PickUp!')

        # goal in map frame
        goal_map = msgs.Point(0, 0, 0)

        # Transform to base link frame
        goal_bl = transformations.tf_transform(goal_map, grab_entity.id, robot.robot_name+'/base_link', tf_listener=robot.tf_listener)
        if goal_bl == None:
            rospy.logerr('Transformation of goal to base failed')
            return 'failed'

        rospy.loginfo(goal_bl)

        # Arm to position in a safe way
        arm.send_joint_trajectory('prepare_grasp')

        # Open gripper
        arm.send_gripper_goal('open')

        # Pre-grasp
        rospy.loginfo('Starting Pre-grasp')
        if not arm.send_goal(goal_bl.x, goal_bl.y, goal_bl.z, 0, 0, 0,
                             frame_id='/'+robot.robot_name+'/base_link', timeout=20, pre_grasp=True, first_joint_pos_only=True):
            rospy.logerr('Pre-grasp failed:')

            arm.reset()
            arm.send_gripper_goal('close', timeout=None)
            return 'failed'

        # Grasp
        if not arm.send_goal(goal_bl.x, goal_bl.y, goal_bl.z, 0, 0, 0, frame_id='/'+robot.robot_name+'/base_link', timeout=120, pre_grasp = True):
            robot.speech.speak('I am sorry but I cannot move my arm to the object position', block=False)
            rospy.logerr('Grasp failed')
            arm.reset()
            arm.send_gripper_goal('close', timeout=None)
            return 'failed'

        # Close gripper
        arm.send_gripper_goal('close')

        # Lift
        if not arm.send_goal( goal_bl.x, goal_bl.y, goal_bl.z + 0.1, 0.0, 0.0, 0.0, timeout=20, pre_grasp=False, frame_id='/'+robot.robot_name+'/base_link'):
            rospy.logerr('Failed lift')

        # Retract
        if not arm.send_goal( goal_bl.x - 0.1, goal_bl.y, goal_bl.z + 0.1, 0.0, 0.0, 0.0, timeout=20, pre_grasp=False, frame_id='/'+robot.robot_name+'/base_link'):
            rospy.logerr('Failed retract')

        # Carrying pose
        if arm.side == 'left':
            y_home = 0.2
        else:
            y_home = -0.2

        rospy.loginfo('y_home = ' + str(y_home))

        rospy.loginfo('start moving to carrying pose')
        if not arm.send_goal(0.18, y_home, goal_bl.z + 0.1, 0, 0, 0, 60):
            rospy.logerr('Failed carrying pose')

        return 'succeeded'

# ----------------------------------------------------------------------------------------------------

class Grab(smach.StateMachine):
    def __init__(self, robot, item, arm):
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        with self:
            smach.StateMachine.add('NAVIGATE_TO_GRAB', NavigateToGrasp(robot, item, arm_designator),
                transitions={ 'unreachable' : 'failed',
                              'goal_not_defined' : 'failed',
                              'arrived' : 'GRAB'})

            smach.StateMachine.add('GRAB', PickUp(robot, arm_designator, item_designator),
                transitions={'succeeded' :   'done',
                             'failed' :   'failed'})
