import PyKDL as kdl
import rospy
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, Point
from robot_skills.amigo import Amigo
from robot_skills.util.kdl_conversions import frame_stamped
from robot_smach_states.util.designators import Designator
from smach import StateMachine, State, cb_interface, CBState
from tf2_ros import Header

_ = tf2_geometry_msgs


class SimpleGrab(State):
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

        goal_point = ud.position
        x = goal_point.point.x
        y = goal_point.point.y
        z = goal_point.point.z
        goal_bl = frame_stamped(goal_point.header.frame_id, x, y, z, roll=0, pitch=0, yaw=0)

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


if __name__ == '__main__':
    rospy.init_node('test_open_dishwasher')

    robot = Amigo()
    robot.ed.reset()
    robot.leftArm.reset()
    robot.torso.reset()

    sm = TestSimpleGrab(robot)
    sm.execute()
