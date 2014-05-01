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
arm_to_hips_3 =                 [-0.400, 0.000, 1.570, 0.000, 0.000, 0.000, 0.000]
arm_to_hips_4 =                 [-0.400, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000]

zero =                          [-0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000]
#7a. Left arm down to the hips

import threading

class StoppableThread(threading.Thread):
    """Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition."""

    def __init__(self):
        super(StoppableThread, self).__init__()
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

def spindle_up_down(robot, lower, upper, stopEvent):
    """Loop the robot's spindle between the lower and upper heights given here"""
    while not rospy.is_shutdown() and not stopEvent.is_set():
        robot.spindle.send_goal(lower, timeout=2.0)
        robot.spindle.send_goal(upper, timeout=2.0)

def macarena(robot):
    stopEvent = threading.Event()

    up_and_down = threading.Thread(target=spindle_up_down, args=(robot, 0.3, 0.4, stopEvent))
    up_and_down.start()
    #robot.spindle.send_goal(0.3)
    def _left(*args, **kwargs): #The underscore  makes the outlining below easier to read
        if not robot.leftArm.send_joint_goal(*args, **kwargs):
            raise Exception("Arms dit not reach goal,  need help")
            robot.speech.speak("Guys, could you help me, i'm stuck in the macarena")
    
    def right(*args, **kwargs): 
        if not robot.rightArm.send_joint_goal(*args, **kwargs):
            raise Exception("Arms dit not reach goal,  need help")
            robot.speech.speak("Guys, could you help me, i'm stuck in the macarena")

    for i in range(1):
        right(*arm_straight_1, timeout=5)
        _left(*arm_straight_1, timeout=5)

        right(*arm_straight_2, timeout=5)
        _left(*arm_straight_2, timeout=5)

        right(*right_hand_to_left_shoulder, timeout=5)
        _left(*left_hand_to_right_shoulder, timeout=5)

        right(*right_arm_to_head, timeout=5)

        _left(*left_arm_to_head_1, timeout=5)
        _left(*left_arm_to_head_2, timeout=5)
        _left(*left_arm_to_head_3, timeout=5)
        _left(*left_arm_to_head_4, timeout=5)

        right(*arm_to_hips_1, timeout=5)
        _left(*arm_to_hips_1, timeout=5)
        right(*arm_to_hips_2, timeout=5)
        _left(*arm_to_hips_2, timeout=5)
        right(*arm_to_hips_3, timeout=5)
        _left(*arm_to_hips_3, timeout=5)
        right(*arm_to_hips_4, timeout=5)
        _left(*arm_to_hips_4, timeout=5)
        right(*arm_to_hips_1, timeout=5)
        _left(*arm_to_hips_1, timeout=5)
        
        # right(*zero, timeout=10)
        # _left(*zero, timeout=10)

    stopEvent.set()
    up_and_down.join()
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
