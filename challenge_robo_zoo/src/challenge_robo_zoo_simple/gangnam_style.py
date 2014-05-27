#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_robo_zoo')

import rospy
import smach
import robot_skills.util.msg_constructors as msgs

import os

from musicmanager import music
import threading

gangnam_poseA_left_start     = [-0.050, 1.500, 1.500, 0.800, -0.15, 0.000, 0.000]
gangnam_poseA_left_end       = [-0.050, 1.500, 1.500, 0.800, 0.150, 0.000, 0.000]
gangnam_poseA_right_start    = [-0.050, 1.500, 1.500, 0.100, 0.150, 0.000, 0.000]
gangnam_poseA_right_end      = [-0.050, 1.500, 1.500, 0.100, -0.15, 0.000, 0.000]

gangnam_poseB_left_start     = [-0.050, 1.500, 1.500, 1.300, -0.15, 0.000, 0.000]
gangnam_poseB_left_end       = [-0.050, 1.500, 1.500, 1.300, 0.150, 0.000, 0.000]
gangnam_poseB_right_start    = [-0.050, 1.500, 0.250, 1.570, 0.250, 0.000, 0.000]
gangnam_poseB_right_end      = [-0.050, 1.500, -0.25, 1.570, -0.25, 0.000, 0.000]
                                                                                                                                 
gangnam_motionA_left = [gangnam_poseA_left_start, gangnam_poseA_left_end, gangnam_poseA_left_start, gangnam_poseA_left_end, gangnam_poseA_left_start, gangnam_poseA_left_end]
gangnam_motionA_right= [gangnam_poseA_right_start, gangnam_poseA_right_end, gangnam_poseA_right_start, gangnam_poseA_right_end, gangnam_poseA_right_start, gangnam_poseA_right_end]
gangnam_motionB_left = [gangnam_poseB_left_start, gangnam_poseB_left_end, gangnam_poseB_left_start, gangnam_poseB_left_end, gangnam_poseB_left_start, gangnam_poseB_left_end]
gangnam_motionB_right= [gangnam_poseB_right_start, gangnam_poseB_right_end, gangnam_poseB_right_start, gangnam_poseB_right_end, gangnam_poseB_right_start, gangnam_poseB_right_end]


def spindle_up_down(robot, lower, upper, stopEvent):
    """Loop the robot's spindle between the lower and upper heights given here"""
    while not rospy.is_shutdown() and not stopEvent.is_set():
        robot.spindle.send_goal(lower, timeout=4.0)
        robot.spindle.send_goal(upper, timeout=4.0)

def gangnam_style(robot):    
    stopEvent = threading.Event()

    up_and_down_spindle = threading.Thread(target=spindle_up_down, args=(robot, 0.3, 0.4, stopEvent))
    up_and_down_spindle.start()

    robot.rightArm.send_joint_goal(-0.050, 1.500, 1.500, 0.100, 0.150, 0.000, 0.000)
    for i in range(1):
        robot.head.send_goal(msgs.PointStamped(0,0,0, frame_id="/amigo/grippoint_left"), pan_vel=1.0, tilt_vel=1.0)
        robot.head.send_goal(msgs.PointStamped(0,0,0, frame_id="/amigo/grippoint_left"), keep_tracking=True, pan_vel=1.0, tilt_vel=1.0)
        robot.leftArm.send_joint_trajectory(gangnam_motionA_left, timeout=10)
        robot.rightArm.send_joint_trajectory(gangnam_motionA_right, timeout=10)
    
    robot.leftArm.reset_arm()
    rospy.sleep(5)
    #robot.rightArm.send_joint_goal(-0.050, 1.500, 0.000, 1.570, 0.150, 0.000, 0.000)

    for i in range(1):
        robot.leftArm.send_joint_goal(-0.050, 1.500, 1.5300, 1.300, -0.15, 0.000, 0.000)
        robot.head.send_goal(msgs.PointStamped(0,0,0, frame_id="/amigo/grippoint_right"), pan_vel=1.0, tilt_vel=1.0)
        robot.head.send_goal(msgs.PointStamped(0,0,0, frame_id="/amigo/grippoint_right"), keep_tracking=True, pan_vel=1.0, tilt_vel=1.0)
        robot.rightArm.send_joint_trajectory(gangnam_motionB_right, timeout=10)
        
    stopEvent.set()
    robot.rightArm.reset_arm()
    robot.leftArm.reset_arm()
    robot.head.reset_position()

class GangNamStyle(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["Done"])
        self.robot = robot

        abspath = os.path.abspath(__file__)
        dname = os.path.dirname(abspath)
        os.chdir(dname)
        musicfile = "psygangnam.mp3"
        self.musicfile = os.path.join(dname, musicfile)

    def execute(self, userdata=None):
        with music(self.musicfile):
            gangnam_style(self.robot)
        return "Done"

if __name__ == "__main__":
    rospy.init_node("gangnam_style")
    import robot_skills.amigo
    amigo = robot_skills.amigo.Amigo(wait_services=True)

    gns = GangNamStyle(amigo)
    gns.execute()