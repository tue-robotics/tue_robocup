import math

import PyKDL as kdl
import rospy
import tf2_geometry_msgs
from cb_planner_msgs_srvs.msg import PositionConstraint, OrientationConstraint
from challenge_dishwasher.custom_find_cup import CustomFindCup
from geometry_msgs.msg import PointStamped, Point
from robot_skills.amigo import Amigo
from robot_skills.util.kdl_conversions import frame_stamped
from robot_smach_states import NavigateTo
from robot_smach_states.util.designators import Designator
from smach import StateMachine, State, cb_interface, CBState
from tf2_ros import Header

_ = tf2_geometry_msgs


class SimpleNavigateToGrasp(NavigateTo):
    def __init__(self, robot, arm):
        super(SimpleNavigateToGrasp, self).__init__(robot, reset_head=False, input_keys=['position'])
        self.arm = arm
        self.point = None

    def execute(self, ud):
        self.point = ud.position
        return super(SimpleNavigateToGrasp, self).execute(ud)

    def generateConstraint(self):
        arm = self.arm.resolve()

        if arm == self.robot.arms['left']:
            angle_offset = math.atan2(-self.robot.grasp_offset.y, self.robot.grasp_offset.x)
        elif arm == self.robot.arms['right']:
            angle_offset = math.atan2(self.robot.grasp_offset.y, self.robot.grasp_offset.x)
        else:
            raise IndexError('Arm not found')
        radius = math.hypot(self.robot.grasp_offset.x, self.robot.grasp_offset.y)

        x = self.point.point.x
        y = self.point.point.y
        frame_id = self.point.header.frame_id

        # Outer radius
        radius -= 0.1
        ro = "(x-%f)^2+(y-%f)^2 < %f^2" % (x, y, radius + 0.075)
        ri = "(x-%f)^2+(y-%f)^2 > %f^2" % (x, y, radius - 0.075)
        pc = PositionConstraint(constraint=ri + " and " + ro, frame=frame_id)
        oc = OrientationConstraint(look_at=Point(x, y, 0.0), frame=frame_id, angle_offset=angle_offset)

        return pc, oc


class SimplePickup(State):
    def __init__(self, robot, arm):
        State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['position'])
        self.robot = robot
        self.arm_designator = arm

    def execute(self, ud):
        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        # Make sure the torso and the arm are done
        self.robot.torso.wait_for_motion_done(cancel=True)
        arm.wait_for_motion_done(cancel=True)

        # This is needed because the head is not entirely still when the
        # look_at_point function finishes
        rospy.sleep(rospy.Duration(0.5))

        # Take the last available, not at detection timestamp
        ud.position.header.stamp = rospy.Time(0)
        goal_point_base_link = self.robot.tf_listener.transformPoint('/amigo/base_link', ud.position)

        x = goal_point_base_link.point.x
        y = goal_point_base_link.point.y
        z = goal_point_base_link.point.z
        goal_bl = frame_stamped(goal_point_base_link.header.frame_id, x, y, z, roll=0, pitch=0, yaw=-0.0463)

        # Grasp
        rospy.loginfo('Start grasping')
        if not arm.send_goal(goal_bl, timeout=20, pre_grasp=True):
            self.robot.speech.speak('I am sorry but I cannot move my arm to the object position', block=False)
            rospy.logerr('Grasp failed')
            arm.reset()
            arm.send_gripper_goal('close', timeout=0.0)
            return 'failed'

        # Close gripper
        arm.send_gripper_goal('close')

        arm.occupied_by = None  # TODO: what to do if we don't have an entity

        # Lift
        rospy.loginfo('Start lifting')
        if arm.side == "left":
            roll = 0.3
        else:
            roll = -0.3

        goal_bl.frame.p.z(goal_bl.frame.p.z() + 0.05)  # Add 5 cm
        goal_bl.frame.M = kdl.Rotation.RPY(roll, 0, 0)  # Update the roll
        rospy.loginfo("Start lift")
        if not arm.send_goal(goal_bl, timeout=20):
            rospy.logerr('Failed lift')

        # Retract
        rospy.loginfo('Start retracting')
        if arm.side == "left":
            roll = 0.6
        else:
            roll = -0.6

        goal_bl.frame.p.x(goal_bl.frame.p.x() - 0.1)  # Retract 10 cm
        goal_bl.frame.p.z(goal_bl.frame.p.z() + 0.05)  # Go 5 cm higher
        goal_bl.frame.M = kdl.Rotation.RPY(roll, 0.0, 0.0)  # Update the roll
        rospy.loginfo("Start retract")
        if not arm.send_goal(goal_bl, timeout=0.0):
            rospy.logerr('Failed retract')

        self.robot.base.force_drive(-0.125, 0, 0, 2.0)

        arm.wait_for_motion_done(cancel=True)

        # Carrying pose
        rospy.loginfo('start moving to carrying pose')
        arm.send_joint_goal('carrying_pose', timeout=0.0)

        # Reset head
        self.robot.head.cancel_goal()

        return 'succeeded'


class SimpleGrab(StateMachine):
    def __init__(self, robot, arm):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"], input_keys=['position'])

        @cb_interface(outcomes=['succeeded'])
        def prepare_grasp(ud):
            # Open gripper (non-blocking)
            arm.resolve().send_gripper_goal('open', timeout=0)

            # Torso up (non-blocking)
            robot.torso.high()

            # Arm to position in a safe way
            arm.resolve().send_joint_trajectory('prepare_grasp', timeout=0)

            # Open gripper
            arm.resolve().send_gripper_goal('open', timeout=0.0)
            return 'succeeded'

        with self:
            StateMachine.add("PREPARE_GRASP", CBState(prepare_grasp),
                             transitions={'succeeded': 'SIMPLE_NAVIGATE_TO_GRASP'})

            StateMachine.add("SIMPLE_NAVIGATE_TO_GRASP", SimpleNavigateToGrasp(robot, arm),
                             transitions={'unreachable': 'failed',
                                          'arrived': 'SIMPLE_PICKUP',
                                          'goal_not_defined': 'failed'})

            StateMachine.add("SIMPLE_PICKUP", SimplePickup(robot, arm),
                             transitions={'succeeded': 'succeeded',
                                          'failed': 'failed'})


class TestSimpleGrab(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        @cb_interface(outcomes=['succeeded'], output_keys=['position'])
        def send_point(ud):
            ud.position = PointStamped(header=Header(frame_id='/amigo/base_link'), point=Point(0.5, 0, 0.7))
            return 'succeeded'

        arm = Designator(robot.leftArm)

        with self:
            StateMachine.add("SEND_POINT", CBState(send_point),
                             transitions={'succeeded': 'FIND_CUP'})

            StateMachine.add("FIND_CUP", SimpleGrab(robot, arm),
                             transitions={'succeeded': 'succeeded',
                                          'failed': 'failed'})


class FindAndGrab(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        arm = Designator(robot.leftArm)

        with self:
            StateMachine.add("FIND_CUP", CustomFindCup(robot),
                             transitions={'succeeded': 'GRAB',
                                          'failed': 'failed'})

            StateMachine.add("GRAB", SimpleGrab(robot, arm),
                             transitions={'succeeded': 'succeeded',
                                          'failed': 'failed'})


if __name__ == '__main__':
    rospy.init_node('test_open_dishwasher')

    robot = Amigo()
    robot.ed.reset()
    robot.leftArm.reset()
    robot.torso.reset()

    sm = FindAndGrab(robot)
    sm.execute()
