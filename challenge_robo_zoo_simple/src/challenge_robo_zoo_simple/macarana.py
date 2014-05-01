#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_robo_zoo_simple')

import rospy
import smach

import os
import signal

#0. Arms are in reset pose
#1. Right arm straight forward and hand up
arm_straight_1 =                [0.000, 1.570, 0.000, 0.000, 0.000, 0.000, 0.000]
#2. Move up and down (spindle)
#3. Turn the right hand upside down,  move the left arm up as wel
arm_straight_2 =                [0.000, 1.570, 1.570, 0.000, 1.570, 0.000, 0.000]
#4. Right hand to the left shoulder
right_hand_to_left_shoulder =   [0.000, 1.750, 1.370, 1.700, 1.570, 0.000, 0.000]
#5. Left hand to right shoulder
left_hand_to_right_shoulder =   [0.000, 1.290, 1.570, 1.700, 1.570, 0.000, 0.000]
#6. Right arm to head
right_arm_to_head =             [0.000, 1.750, 0.000, 1.900, 1.570, 0.500, 0.000]
#6a. Left arm to head
left_arm_to_head_1 =            [0.000, 1.290, 1.570, 1.300, 1.570, 0.000, 0.000]
left_arm_to_head_2 =            [0.000, 1.750, 1.570, 1.300, 1.570, 0.000, 0.000]
left_arm_to_head_3 =            [0.000, 1.750, 0.000, 1.300, 1.570, 0.000, 0.000]
left_arm_to_head_4 =            [0.000, 1.750, 0.000, 1.900, 1.570, 0.500, 0.000]
#7. Right arm down to the hips
arm_to_hips_1 =                 [-0.200, -0.200, 0.200, 0.800, 0.000, 0.000, 0.000] 
arm_to_hips_2 =                 [-0.400, 0.000, 1.570, 0.600, 0.000, 0.000, 0.000]

zero =                          [-0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000]
#7a. Left arm down to the hips

def macarena(robot):
    _left =  robot.leftArm.send_joint_goal #The undersore makes the outlining work (below). OCD
    right = robot.rightArm.send_joint_goal
    for i in range(1):
        right(*arm_straight_1, timeout=10)
        _left(*arm_straight_1, timeout=10)

        right(*arm_straight_2, timeout=10)
        _left(*arm_straight_2, timeout=10)

        right(*right_hand_to_left_shoulder, timeout=10)
        _left(*left_hand_to_right_shoulder, timeout=10)

        right(*right_arm_to_head, timeout=10)

        _left(*left_arm_to_head_1, timeout=10)
        _left(*left_arm_to_head_2, timeout=10)
        _left(*left_arm_to_head_3, timeout=10)
        _left(*left_arm_to_head_4, timeout=10)

        right(*arm_to_hips_1, timeout=10)
        _left(*arm_to_hips_1, timeout=10)
        right(*arm_to_hips_2, timeout=10)
        _left(*arm_to_hips_2, timeout=10)
        right(*arm_to_hips_1, timeout=10)
        _left(*arm_to_hips_1, timeout=10)
        
        right(*zero, timeout=10)
        _left(*zero, timeout=10)

    robot.rightArm.reset_arm()
    robot.leftArm.reset_arm()
    robot.head.reset_position()

def start_music():
    import subprocess
    
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname)
    musicfile = "01 Walk Like An macarena.mp3"
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

class Macarena(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["Done"])
        self.robot = robot

    def execute(self, userdata=None):
        proc = start_music()
        macarena(self.robot)
        stop_music(proc)
        return "Done"

if __name__ == "__main__":
    rospy.init_node("macarena")
    import robot_skills.amigo
    amigo = robot_skills.amigo.Amigo(wait_services=True)

    macarena(amigo)
