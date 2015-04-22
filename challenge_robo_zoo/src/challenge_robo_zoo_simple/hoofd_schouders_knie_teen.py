#! /usr/bin/env python
import rospy
import smach
import math

import os

from musicmanager import music

TURNSPEED = 1.0

#2. Move up and down (torso)
#4. Right hand to the left shoulder
right_hand_to_left_shoulder =   [[0.000, 0.400, 0.000, 2.100, 1.500, 0.500, 0.000]]
#5. Left hand to right shoulder
left_hand_to_right_shoulder =   [[0.000, 0.400, 0.000, 2.100, 1.500, 0.500, 0.000]]
#6. Right arm to head
right_arm_to_head_1 =           [[0.000, 1.400, 0.000, 2.100, 1.500, 0.500, 0.000]]
#6a. Left arm to head
left_arm_to_head_1 =            [[0.000, 1.400, 0.000, 2.100, 1.500, 0.500, 0.000]]
#6. Right arm to eyes
right_arm_to_eyes_1 =           [[0.000, 1.400, 0.350, 1.800, 0.0, 0.0, 0.500]]
left_arm_to_eyes_1 =            [[0.000, 1.400, 0.350, 1.800, 0.0, 0.0, 0.500]]
#6. Right arm to nose
right_arm_to_nose_1 =           [[0.000, 1.100, 0.350, 2.000, 0.000, 0.300, 0.500]]
arm_to_knees_1 =                [[-0.200, -0.200, 0.200, 0.800, 0.000, 0.000, 0.000] ]
arm_to_toes_1 =                [[-0.200, -0.200, 0.200, 0.800, 0.000, 0.000, 0.000] ]

zero =                          [[-0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000]]
#7a. Left arm down to the hips

def hoofdschoudersknieteen(robot):
    #TODO: Look at hand at given times

    def _left(trajectory, timeout=10.0): #The underscore  makes the outlining below easier to read
        robot.leftArm._send_joint_trajectory(trajectory, timeout=rospy.Duration(7))

    def right(trajectory, timeout=10.0):
        robot.rightArm._send_joint_trajectory(trajectory, timeout=rospy.Duration(7))

    #Defined shortcuts above
    robot.head.look_at_hand("right")

    def hoofd(speak=False):
        rospy.loginfo("hoofd")
        robot.torso.send_goal('upper', timeout=4.0) # timeout=0.0
        right(right_arm_to_head_1) #Dont wait, both arms should move in sync # timeout=0.0
        if speak: robot.speech.speak("Head", language='en', block=True)
        _left(left_arm_to_head_1, timeout=10)

    def schouders(speak=False):
        rospy.loginfo("schouders")
        robot.torso.send_goal('upper',timeout=4.0) # timeout=0.0
        right(right_hand_to_left_shoulder) #Wait, otherwise arms collide # timeout=0.0
        if speak: robot.speech.speak("Shoulders", language='en', block=True)
        _left(left_hand_to_right_shoulder, timeout=10)

    def knie(speak=False):
        rospy.loginfo("knie") #TODO: tune poses
        robot.torso.send_goal('lower',  timeout=1.0)
        right(arm_to_knees_1)#Dont wait, both arms should move in sync # timeout=0.0
        if speak: robot.speech.speak("Knees", language='en', block=True)
        _left(arm_to_knees_1, timeout=10)

    def teen(speak=False):
        rospy.loginfo("teen") #TODO: tune poses
        robot.torso.send_goal('lower',  timeout=4.0)
        right(arm_to_knees_1)#Dont wait, both arms should move in sync # timeout=0.0
        if speak: robot.speech.speak("and Toes", language='en', block=True)
        _left(arm_to_knees_1, timeout=10)

    def oren(speak=False):
        rospy.loginfo("oren")
        robot.torso.send_goal('upper', timeout=4.0) # timeout=0.0
        right(right_arm_to_head_1) #Dont wait, both arms should move in sync # timeout=0.0
        if speak: robot.speech.speak("Ears", language='en', block=True)
        _left(left_arm_to_head_1, timeout=10)

    def ogen(speak=False):
        rospy.loginfo("ogen")
        robot.torso.send_goal('upper', timeout=4.0) # timeout=0.0
        right(right_arm_to_eyes_1) #Dont wait, both arms should move in sync # timeout=0.0
        if speak: robot.speech.speak("and Eyes", language='en', block=True)
        _left(left_arm_to_eyes_1, timeout=10)

    def puntje_van_je_neus(turn=False, speak=False):
        rospy.loginfo("puntje_van_je_neus")
        if speak: robot.speech.speak("and mouth and nose!", language='en', block=False)
        if turn:
            robot.base.force_drive(0, 0, TURNSPEED, (2*math.pi)/TURNSPEED) #Turn a full circle at TURNSPEED rad/sec
        robot.torso.send_goal('upper', timeout=4.0) # timeout=0.0
        right(right_arm_to_nose_1) #Dont wait, both arms should move in syn, timeout=10c
        robot.leftArm.reset() #Move left arm down, only use right arm to point at 'nose'

    try:
        hoofd(speak=False); schouders(speak=False); knie(speak=False); teen(speak=False); knie(speak=False); teen(speak=False)
        hoofd(speak=False); schouders(speak=False); knie(speak=False); teen(speak=False); knie(speak=False); teen(speak=False)
        oren(speak=False); ogen(speak=False); puntje_van_je_neus(speak=False, turn=False)
        #hoofd(speak=False); schouders(speak=True); knie(speak=True); teen(speak=True); knie(speak=True); teen(speak=True)

        robot.torso.reset()
        robot.head.reset()
        robot.rightArm.reset()
        robot.leftArm.reset()
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
        musicfile = "kopfschulternknieundfuss.mp3"
        self.musicfile = os.path.join(dname, musicfile)

    def execute(self, userdata=None):
        with music(self.musicfile, volume=100):
            hoofdschoudersknieteen(self.robot)
        return "Done"

if __name__ == "__main__":
    rospy.init_node("hoofd_schouders_knie_teen")
    import robot_skills.amigo
    amigo = robot_skills.amigo.Amigo(wait_services=True)

    hskt = HoofdSchouderKnieTeen(amigo)
    hskt.execute()
