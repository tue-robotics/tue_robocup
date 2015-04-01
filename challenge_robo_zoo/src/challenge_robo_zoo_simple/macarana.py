#! /usr/bin/env python
import rospy
import smach

import os
import signal
import robot_skills.util.msg_constructors as msgs

from musicmanager import music

#0. Arms are in reset pose
#1. Right arm straight forward and hand up
arm_straight_1 =                [[0.000, 1.500, 0.000, 0.000, 0.000, 0.000, 0.000]]
#2. Move up and down (torso)
#3. Turn the right hand upside down,  move the left arm up as wel
arm_straight_2 =                [[0.000, 1.500, 1.500, 0.000, 1.500, 0.000, 0.000]]
#4. Right hand to the left shoulder
right_hand_to_left_shoulder =   [[0.000, 1.500, 1.200, 1.500, 1.500, 0.000, 0.000]]
#5. Left hand to right shoulder
left_hand_to_right_shoulder =   [[0.000, 1.500, 1.000, 1.000, 0.000, 0.000, 0.000]]
#6. Right arm to head
right_arm_to_head_1 =           [[0.000, 1.500, 0.000, 1.500, 1.500, 0.500, 0.000]]
right_arm_to_head_2 =           [[0.000, 1.500, 0.000, 1.500, 1.500, 0.500, 0.000]]
#6a. Left arm to head
left_arm_to_head_1 =            [[0.000, 1.290, 1.500, 1.300, 1.500, 0.000, 0.000]]
left_arm_to_head_2 =            [[0.000, 1.500, 1.500, 1.300, 1.500, 0.000, 0.000]]
left_arm_to_head_3 =            [[0.000, 1.500, 0.000, 1.300, 1.500, 0.000, 0.000]]
left_arm_to_head_4 =            [[0.000, 1.500, 0.000, 1.500, 1.500, 0.500, 0.000]]
#7. Right arm down to the hips
arm_to_hips_1 =                 [[-0.200, -0.200, 0.200, 0.800, 0.000, 0.000, 0.000] ]
arm_to_hips_2 =                 [[-0.400, 0.000, 1.500, 0.600, 0.000, 0.000, 0.000]]
arm_to_hips_3 =                 [[-0.400, 0.000, 1.500, 0.000, 0.000, 0.000, 0.000]]
arm_to_hips_4 =                 [[-0.400, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000]]

zero =                          [[-0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000]]
#7a. Left arm down to the hips

import threading

def torso_up_down(robot, lower, upper, stopEvent):
    """Loop the robot's torso between the lower and upper heights given here"""
    while not rospy.is_shutdown() and not stopEvent.is_set():
        robot.torso.send_goal("lower") #, timeout=4.0
        rospy.sleep(1.)
        robot.torso.send_goal("upper") #, timeout=4.0


def head_up_down(robot, stopEvent):
    """Loop the robot's torso between the lower and upper heights given here"""
    while not rospy.is_shutdown() and not stopEvent.is_set():
        robot.head.reset()
        rospy.sleep(1.)
        robot.head.look_at_standing_person()
    robot.head.reset()

def macarena(robot):
    stopEvent = threading.Event()

    up_and_down_torso = threading.Thread(target=torso_up_down, args=(robot, "lower", "upper", stopEvent))
    up_and_down_torso.start()
    #robot.torso.send_goal(0.3)
    up_and_down_head = threading.Thread(target=head_up_down, args=(robot, stopEvent))
    #up_and_down_head.start()
    robot.head.look_at_point(msgs.PointStamped(0.2, 0, 1.3, frame_id="/amigo/base"))
    
    def _left(trajectory, timeout=10.0): #The underscore  makes the outlining below easier to read
        if not robot.leftArm._send_joint_trajectory(trajectory):
            raise Exception("Arms did not reach goal,  need help")
    
    def right(trajectory, timeout=10.0): 
        if not robot.rightArm._send_joint_trajectory(trajectory):
            raise Exception("Arms did not reach goal,  need help")
    #Defined shortcuts above

    try:
        for i in range(1):
            right(arm_straight_1, timeout=rospy.Duration(10))
            _left(arm_straight_1, timeout=rospy.Duration(10))

            right(arm_straight_2, timeout=rospy.Duration(5))
            _left(arm_straight_2, timeout=rospy.Duration(5))

            right(right_hand_to_left_shoulder, timeout=rospy.Duration(10))
            _left(left_hand_to_right_shoulder, timeout=rospy.Duration(10))

            right(right_arm_to_head_1, timeout=rospy.Duration(10))
            #right(*right_arm_to_head_2,] timeout=rospy.Duration(10))

            #_left(*left_arm_to_head_1) ], timeout=rospy.Duration(5))
            #_left(*left_arm_to_head_2) ], timeout=rospy.Duration(5))
            #_left(*left_arm_to_head_,] timeout=rospy.Duration(5))
            _left(left_arm_to_head_4, timeout=rospy.Duration(5))

            right(arm_to_hips_1, timeout=rospy.Duration(10))
            _left(arm_to_hips_1, timeout=rospy.Duration(10))
            right(arm_to_hips_2, timeout=rospy.Duration(5))
            _left(arm_to_hips_2, timeout=rospy.Duration(5))
            right(arm_to_hips_3, timeout=rospy.Duration(5))
            _left(arm_to_hips_3, timeout=rospy.Duration(5))
            right(arm_to_hips_4, timeout=rospy.Duration(5))
            _left(arm_to_hips_4, timeout=rospy.Duration(5))
            right(arm_to_hips_1, timeout=rospy.Duration(15))
            _left(arm_to_hips_1, timeout=rospy.Duration(15))

        stopEvent.set()
        up_and_down_torso.join()
        #up_and_down_head.join()
        robot.rightArm.reset()
        robot.leftArm.reset()
        robot.head.reset()
    except Exception, e:
        robot.speech.speak("Guys, could you help me, i'm stuck in the maca-rena")
        rospy.logerr(e)
            # right(*zero, timeout=rospy.Duration(10))
            # _left(*zero, timeout=rospy.Duration(10))

class Macarena(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["Done"])
        self.robot = robot

        abspath = os.path.abspath(__file__)
        dname = os.path.dirname(abspath)
        os.chdir(dname)
        musicfile = "macarena.mp3"
        self.musicfile = os.path.join(dname, musicfile)

    def execute(self, userdata=None):
        with music(self.musicfile):
            macarena(self.robot)
        return "Done"

if __name__ == "__main__":
    rospy.init_node("macarena")
    import robot_skills.amigo
    amigo = robot_skills.amigo.Amigo(wait_services=True)

    macarena(amigo)
