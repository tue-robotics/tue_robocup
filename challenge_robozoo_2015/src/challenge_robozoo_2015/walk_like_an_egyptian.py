#! /usr/bin/env python
import rospy
import smach
import robot_skills.util.msg_constructors as msgs

import os
import signal

egyptian_pose_start     = [-1.470, 0.000, -1.45, 1.800, 0.000, -0.55, 0.000]
egyptian_pose_center    = [-1.470, 0.000, -1.45, 1.570, 0.000, -0.30, 0.000]
egyptian_pose_end       = [-1.470, 0.000, -1.45, 1.177, 0.000, -0.15, 0.000]
egyptian_pose_down      = [-1.470, 0.000, 1.45, 1.177, 0.000, -0.30, 0.000]
                                                                                                                                 #Also moving the arms down is way too slow
egyptian_motion_left = [egyptian_pose_center, egyptian_pose_start, egyptian_pose_center, egyptian_pose_end, egyptian_pose_center]#, egyptian_pose_down]
egyptian_motion_right= [egyptian_pose_center, egyptian_pose_end, egyptian_pose_center, egyptian_pose_start, egyptian_pose_center]#, egyptian_pose_down]

def walk_like_an_egyptian(robot):
    for i in range(2):
        robot.head.look_at_point(msgs.PointStamped(0,0,0, frame_id="/amigo/grippoint_left"))
        robot.head.look_at_point(msgs.PointStamped(0,0,0, frame_id="/amigo/grippoint_left"))
        robot.leftArm._send_joint_trajectory(egyptian_motion_left, timeout=rospy.Duration(10))  # TODO: Make work with different robots.

        robot.head.look_at_point(msgs.PointStamped(0,0,0, frame_id="/amigo/grippoint_right"))
        robot.head.look_at_point(msgs.PointStamped(0,0,0, frame_id="/amigo/grippoint_right"))
        robot.rightArm._send_joint_trajectory(egyptian_motion_right, timeout=rospy.Duration(10))  # TODO: Make work with different robots.

    robot.rightArm.reset()
    robot.leftArm.reset()
    robot.head.reset()

def music():
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname)
    os.system("mpg123 '01 Walk Like An Egyptian.mp3' &")

def start_music():
    import subprocess
    
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname)
    musicfile = "01 Walk Like An Egyptian.mp3"
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

class WalkLikeAnEgyptian(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["Done"])
        self.robot = robot

    def execute(self, userdata=None):
        proc = start_music()
        try:
            walk_like_an_egyptian(self.robot)
        finally:
            stop_music(proc)
        return "Done"

if __name__ == "__main__":
    rospy.init_node("walk_like_an_egyptian")
    import robot_skills.amigo
    amigo = robot_skills.amigo.Amigo(wait_services=True)

    walk_like_an_egyptian(amigo)
