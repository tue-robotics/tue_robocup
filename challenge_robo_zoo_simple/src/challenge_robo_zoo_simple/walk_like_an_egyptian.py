#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_robo_zoo_simple')

import rospy
import smach
import robot_skills.util.msg_constructors as msgs
import robot_smach_states as states

egyptian_pose_start     = [-1.570, 0.000, -1.50, 1.800, 0.000, -0.55, 0.000]
egyptian_pose_center    = [-1.570, 0.000, -1.50, 1.570, 0.000, -0.30, 0.000]
egyptian_pose_end       = [-1.570, 0.000, -1.50, 1.177, 0.000, -0.15, 0.000]

egyptian_motion_left = [egyptian_pose_center, egyptian_pose_start, egyptian_pose_center, egyptian_pose_end, egyptian_pose_center]
egyptian_motion_right= [egyptian_pose_center, egyptian_pose_end, egyptian_pose_center, egyptian_pose_start, egyptian_pose_center]

def walk_like_an_egyptian(robot):
    for i in range(2):
        robot.head.send_goal(msgs.PointStamped(0,0,0, frame_id="/amigo/grippoint_left"), pan_vel=5.0, tilt_vel=5.0)
        robot.head.send_goal(msgs.PointStamped(0,0,0, frame_id="/amigo/grippoint_left"), keep_tracking=True, pan_vel=5.0, tilt_vel=5.0)
        robot.leftArm.send_joint_trajectory(egyptian_motion_left, timeout=10)

        robot.head.send_goal(msgs.PointStamped(0,0,0, frame_id="/amigo/grippoint_right"), pan_vel=5.0, tilt_vel=5.0)
        robot.head.send_goal(msgs.PointStamped(0,0,0, frame_id="/amigo/grippoint_right"), keep_tracking=True, pan_vel=5.0, tilt_vel=5.0)
        robot.rightArm.send_joint_trajectory(egyptian_motion_right, timeout=10)

    robot.rightArm.reset_arm()
    robot.leftArm.reset_arm()
    robot.head.reset_position()

class WalkLikeAnEgyptian(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["Done"])
        self.robot = robot

    def execute(self, userdata=None):
        walk_like_an_egyptian(self.robot)
        return "Done"

if __name__ == "__main__":
    rospy.init_node("walk_like_an_egyptian")
    import robot_skills.amigo
    amigo = robot_skills.amigo.Amigo(wait_services=True)

    walk_like_an_egyptian(amigo)
