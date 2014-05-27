#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_robo_zoo')

import rospy
import smach
import math

import os

from musicmanager import music

TURNSPEED = 1.0

#2. Move up and down (spindle)
#4. Right hand to the left shoulder
right_hand_to_left_shoulder =   [0.000, 1.500, 1.200, 1.500, 1.500, 0.000, 0.000]
#5. Left hand to right shoulder
left_hand_to_right_shoulder =   [0.000, 1.500, 1.000, 1.000, 0.000, 0.000, 0.000]
#6. Right arm to head
right_arm_to_head_1 =           [0.000, 1.400, 0.000, 2.100, 1.500, 0.500, 0.000]
#6a. Left arm to head
left_arm_to_head_1 =            [0.000, 1.400, 0.000, 2.100, 1.500, 0.500, 0.000]
#6. Right arm to eyes
right_arm_to_eyes_1 =           [0.000, 1.400, 0.350, 1.800, 0.0, 0.0, 0.500]
left_arm_to_eyes_1 =            [0.000, 1.400, 0.350, 1.800, 0.0, 0.0, 0.500]
#6. Right arm to nose
right_arm_to_nose_1 =           [0.000, 1.100, 0.350, 2.000, 0.000, 0.300, 0.500]
arm_to_hips_1 =                 [-0.200, -0.200, 0.200, 0.800, 0.000, 0.000, 0.000] 

zero =                          [-0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000]
#7a. Left arm down to the hips


def spindle_up_down(robot, lower, upper, stopEvent):
    """Loop the robot's spindle between the lower and upper heights given here"""
    while not rospy.is_shutdown() and not stopEvent.is_set():
        robot.spindle.send_goal(lower, timeout=4.0)
        robot.spindle.send_goal(upper, timeout=4.0)


def head_up_down(robot, stopEvent):
    """Loop the robot's spindle between the lower and upper heights given here"""
    while not rospy.is_shutdown() and not stopEvent.is_set():
        robot.head.look_down(tilt_vel=0.5)
        robot.head.look_up(tilt_vel=0.5)
    robot.head.reset_position()

def hoofdschoudersknieteen(robot):
    #TODO: Look at hand at given times

    def _left(*args, **kwargs): #The underscore  makes the outlining below easier to read
        if not robot.leftArm.send_joint_goal(*args, **kwargs):
            raise Exception("Arms did not reach goal,  need help")
    
    def right(*args, **kwargs): 
        if not robot.rightArm.send_joint_goal(*args, **kwargs):
            raise Exception("Arms did not reach goal,  need help")
    #Defined shortcuts above

    def hoofd(speak=True):
        if speak: robot.speech.speak("Hoofd", language='nl', block=False)
        robot.spindle.send_goal(0.4, timeout=4.0)
        right(*right_arm_to_head_1, timeout=0) #Dont wait, both arms should move in sync
        _left(*left_arm_to_head_1,  timeout=10)

    def schouders(speak=True):
        if speak: robot.speech.speak("Schouders", language='nl', block=False)
        robot.spindle.send_goal(0.4, timeout=4.0)
        right(*right_hand_to_left_shoulder, timeout=10) #Wait, otherwise arms collide
        _left(*left_hand_to_right_shoulder, timeout=10)

    def knie(speak=True): #TODO: tune poses
        if speak: robot.speech.speak("Knie", language='nl', block=False)
        robot.spindle.send_goal(0.3, timeout=4.0)
        right(*arm_to_hips_1, timeout=0)#Dont wait, both arms should move in sync
        _left(*arm_to_hips_1, timeout=10)
    
    def teen(speak=True): #TODO: tune poses
        if speak: robot.speech.speak("En teen", language='nl', block=False)
        robot.spindle.send_goal(0.1, timeout=4.0)
        right(*arm_to_hips_1, timeout=0)#Dont wait, both arms should move in sync
        _left(*arm_to_hips_1, timeout=10)

    def oren(speak=True):
        if speak: robot.speech.speak("Oren", language='nl', block=False)
        robot.spindle.send_goal(0.4, timeout=4.0)
        right(*right_arm_to_head_1, timeout=0) #Dont wait, both arms should move in sync
        _left(*left_arm_to_head_1,  timeout=10)

    def ogen(speak=True):
        if speak: robot.speech.speak("Ogen", language='nl', block=False)
        robot.spindle.send_goal(0.4, timeout=4.0)
        right(*right_arm_to_eyes_1, timeout=0) #Dont wait, both arms should move in sync
        _left(*left_arm_to_eyes_1,  timeout=10)

    def puntje_van_je_neus(turn=False, speak=True):
        if speak: robot.speech.speak("Puntje van je neus!", language='nl', block=False)
        if turn:
            robot.base.force_drive(0, 0, TURNSPEED, (2*math.pi)/TURNSPEED) #Turn a full circle at TURNSPEED rad/sec
        robot.spindle.send_goal(0.4, timeout=4.0)
        right(*right_arm_to_head_1, timeout=0) #Dont wait, both arms should move in sync
        robot.leftArm.reset_arm() #Move left arm down, only use right arm to point at 'nose'

    try:
        hoofd(); schouders(); knie(); teen(); knie(); teen()
        hoofd(); schouders(); knie(); teen(); knie(); teen()
        oren(); ogen(); puntje_van_je_neus()
        hoofd(); schouders(); knie(); teen(); knie(); teen()
    except Exception, e:
        robot.speech.speak("Guys, could you help me, my dance stopped suddenly")
        rospy.logerr(e)

class HoofdSchouderKnieTeen(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["Done"])
        self.robot = robot

        abspath = os.path.abspath(__file__)
        dname = os.path.dirname(abspath)
        os.chdir(dname)
        musicfile = "hoofd_schouders_knie_teen.mp3"
        self.musicfile = os.path.join(dname, musicfile)

    def execute(self, userdata=None):
        with music(self.musicfile):
            hoofdschoudersknieteen(self.robot)
        return "Done"

if __name__ == "__main__":
    rospy.init_node("hoofd_schouders_knie_teen")
    import robot_skills.amigo
    amigo = robot_skills.amigo.Amigo(wait_services=True)

    hskt = HoofdSchouderKnieTeen(amigo)
    hskt.execute()
