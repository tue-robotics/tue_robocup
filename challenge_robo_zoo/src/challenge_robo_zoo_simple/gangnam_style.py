#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_robo_zoo_simple')

import rospy
import smach
import robot_skills.util.msg_constructors as msgs

import os
import signal

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

def gangnam_style(robot):
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
        

    robot.rightArm.reset_arm()
    robot.leftArm.reset_arm()
    robot.head.reset_position()

def music():
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname)
    os.system("mpg123 'psygangnam.mp3' &")

def start_music():
    import subprocess
    
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname)
    musicfile = "psygangnam.mp3"
    musicfile = os.path.join(dname, musicfile)
    rospy.loginfo("Playing music: {0}".format(musicfile))
    # The os.setsid() is passed in the argument preexec_fn so
    # it's run after the fork() and before  exec() to run the shell.
    music_process = subprocess.Popen("mpg123 '{0}'".format(musicfile), stdout=subprocess.PIPE, 
                           shell=True, preexec_fn=os.setsid) 
    rospy.loginfo("If the music keeps going somehow, its PID is: {0}".format(music_process.pid))
    return music_process

def stop_music(music_process):
    os.killpg(music_process.pid, signal.SIGTERM)  # Send the signal to all the process groups

class GangNamStyle(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["Done"])
        self.robot = robot

    def execute(self, userdata=None):
        proc = start_music()
        try:
            gangnam_style(self.robot)
        finally:
            stop_music(proc)
        return "Done"

if __name__ == "__main__":
    rospy.init_node("gangnam_style")
    import robot_skills.amigo
    amigo = robot_skills.amigo.Amigo(wait_services=True)

    gangnam_style(amigo)
