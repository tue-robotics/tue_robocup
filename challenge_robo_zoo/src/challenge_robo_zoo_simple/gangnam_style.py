#! /usr/bin/env python
import rospy
import smach
import robot_skills.util.msg_constructors as msgs

import os

from musicmanager import music
import threading

gangnam_poseA_left_pre_start = [-0.050, 0.200, 0.200, 0.800, -0.15, 0.000, 0.000]
gangnam_poseA_left_start     = [-0.050, 1.500, 1.500, 0.800, -0.15, 0.000, 0.000]
gangnam_poseA_left_end       = [-0.050, 1.500, 1.500, 0.800, 0.150, 0.000, 0.000]

gangnam_poseA_right_pre_start= [-0.050, 0.200, 0.200, 0.800, -0.15, 0.000, 0.000]
gangnam_poseA_right_start    = [-0.050, 1.500, 1.500, 0.100, 0.150, 0.000, 0.000]
gangnam_poseA_right_end      = [-0.050, 1.500, 1.500, 0.100, -0.15, 0.000, 0.000]

gangnam_poseB_left_pre_start = [-0.050, 0.200, 0.200, 1.300, -0.15, 0.000, 0.000]
gangnam_poseB_left_start     = [-0.050, 1.500, 1.500, 1.300, -0.15, 0.000, 0.000]
gangnam_poseB_left_end       = [-0.050, 1.500, 1.500, 1.300, 0.150, 0.000, 0.000]

gangnam_poseB_right_pre_start= [-0.050, 0.200, 0.250, 1.570, 0.250, 0.000, 0.000]
gangnam_poseB_right_start    = [-0.050, 1.500, 0.250, 1.570, 0.250, 0.000, 0.000]
gangnam_poseB_right_end      = [-0.050, 1.500, -0.25, 1.570, -0.25, 0.000, 0.000]
                                                                                                                                 
gangnam_motionA_left = [gangnam_poseA_left_pre_start, gangnam_poseA_left_start, gangnam_poseA_left_end, gangnam_poseA_left_start, gangnam_poseA_left_end, gangnam_poseA_left_start, gangnam_poseA_left_end]
gangnam_motionA_right= [gangnam_poseA_right_pre_start, gangnam_poseA_right_start, gangnam_poseA_right_end, gangnam_poseA_right_start, gangnam_poseA_right_end, gangnam_poseA_right_start, gangnam_poseA_right_end]
gangnam_motionB_left = [gangnam_poseB_left_pre_start, gangnam_poseB_left_start, gangnam_poseB_left_end, gangnam_poseB_left_start, gangnam_poseB_left_end, gangnam_poseB_left_start, gangnam_poseB_left_end]
gangnam_motionB_right= [gangnam_poseB_right_pre_start, gangnam_poseB_right_start, gangnam_poseB_right_end, gangnam_poseB_right_start, gangnam_poseB_right_end, gangnam_poseB_right_start, gangnam_poseB_right_end]


def torso_up_down(robot, lower, upper, stopEvent):
    """Loop the robot's torso between the lower and upper heights given here"""
    while not rospy.is_shutdown() and not stopEvent.is_set():
        robot.torso.send_goal("lower", timeout=4.0)
        robot.torso.send_goal("upper", timeout=4.0)

def motionA(robot):
    robot.rightArm._send_joint_trajectory([[-0.050, 1.500, 1.500, 0.100, 0.150, 0.000, 0.000]], timeout=rospy.Duration(1))
    for i in range(1):
        robot.head.look_at_point(msgs.PointStamped(0,0,0, frame_id="/amigo/grippoint_left"))
        robot.head.look_at_point(msgs.PointStamped(0,0,0, frame_id="/amigo/grippoint_left"))
        # robot.leftArm.send_joint_trajectory([gangnam_motionA_left], timeout=rospy.Duration(10))
        # robot.rightArm.send_joint_trajectory([gangnam_motionA_right], timeout=rospy.Duration(10))

        robot.leftArm._send_joint_trajectory([gangnam_poseA_left_pre_start], timeout=rospy.Duration(1))  # TODO: Make work with different robots.
        robot.rightArm._send_joint_trajectory([gangnam_poseA_right_pre_start], timeout=rospy.Duration(1))  # TODO: Make work with different robots.
        
        robot.leftArm._send_joint_trajectory([gangnam_poseA_left_start], timeout=rospy.Duration(1))  # TODO: Make work with different robots.
        robot.rightArm._send_joint_trajectory([gangnam_poseA_right_start], timeout=rospy.Duration(1))  # TODO: Make work with different robots.
        
        robot.leftArm._send_joint_trajectory([gangnam_poseA_left_end], timeout=rospy.Duration(1))  # TODO: Make work with different robots.
        robot.rightArm._send_joint_trajectory([gangnam_poseA_right_end], timeout=rospy.Duration(1))  # TODO: Make work with different robots.
        
        robot.leftArm._send_joint_trajectory([gangnam_poseA_left_start], timeout=rospy.Duration(1))  # TODO: Make work with different robots.
        robot.rightArm._send_joint_trajectory([gangnam_poseA_right_start], timeout=rospy.Duration(1))  # TODO: Make work with different robots.
        
        robot.leftArm._send_joint_trajectory([gangnam_poseA_left_end], timeout=rospy.Duration(1))  # TODO: Make work with different robots.
        robot.rightArm._send_joint_trajectory([gangnam_poseA_right_end], timeout=rospy.Duration(1))  # TODO: Make work with different robots.
        
        robot.leftArm._send_joint_trajectory([gangnam_poseA_left_start], timeout=rospy.Duration(1))  # TODO: Make work with different robots.
        robot.rightArm._send_joint_trajectory([gangnam_poseA_right_start], timeout=rospy.Duration(1))  # TODO: Make work with different robots.
        
        robot.leftArm._send_joint_trajectory([gangnam_poseA_left_end], timeout=rospy.Duration(1))  # TODO: Make work with different robots.
        robot.rightArm._send_joint_trajectory([gangnam_poseA_right_end], timeout=rospy.Duration(1))  # TODO: Make work with different robots.
    
    robot.leftArm.reset()

def motionB(robot):
    #import ipdb; ipdb.set_trace()
    for i in range(1):
        robot.leftArm._send_joint_trajectory([[-0.050, 0.500, 0.7150, 1.300, -0.15, 0.000, 0.000]], timeout=rospy.Duration(10))
        robot.leftArm._send_joint_trajectory([[-0.050, 0.500, 1.5300, 0.700, -0.15, 0.000, 0.000]], timeout=rospy.Duration(1))
        robot.leftArm._send_joint_trajectory([[-0.050, 1.500, 1.5300, 0.700, -0.15, 0.000, 0.000]], timeout=rospy.Duration(1))
        robot.head.look_at_point(msgs.PointStamped(0,0,0, frame_id="/amigo/grippoint_right"), pan_vel=1.0, tilt_vel=1.0)
        robot.head.look_at_point(msgs.PointStamped(0,0,0, frame_id="/amigo/grippoint_right"), pan_vel=1.0, tilt_vel=1.0)

        #robot.rightArm._send_joint_trajectory([gangnam_poseB_right_pre_start], timeout=rospy.Duration(1))  # TODO: Make work with different robots.
        #Right is now in [             -0.050, 1.500, 1.500, 0.100, -0.15, 0.000, 0.000]
        robot.rightArm._send_joint_trajectory([[-0.050, 1.500, 0.000, 0.100, 0.250, 0.000, 0.000]], timeout=rospy.Duration(10))
        robot.rightArm._send_joint_trajectory([[-0.050, 1.500, 0.000, 1.570, 0.250, 0.000, 0.000]], timeout=rospy.Duration(1))
        
        robot.leftArm._send_joint_trajectory([[-0.050, 1.500, 1.5300, 1.300, -0.15, 0.000, 0.000]])
        
        robot.rightArm._send_joint_trajectory([gangnam_poseB_right_end], timeout=rospy.Duration(1))  # TODO: Make work with different robots.
        robot.rightArm._send_joint_trajectory([gangnam_poseB_right_start], timeout=rospy.Duration(1))  # TODO: Make work with different robots.
        robot.rightArm._send_joint_trajectory([gangnam_poseB_right_end], timeout=rospy.Duration(1))  # TODO: Make work with different robots.
        robot.rightArm._send_joint_trajectory([gangnam_poseB_right_start], timeout=rospy.Duration(1))  # TODO: Make work with different robots.
        robot.rightArm._send_joint_trajectory([gangnam_poseB_right_end], timeout=rospy.Duration(1))  # TODO: Make work with different robots.
        
        robot.leftArm._send_joint_trajectory([[-0.050, 1.500, 1.5300, 0.800, -0.15, 0.000, 0.000]], timeout=rospy.Duration(1))

def gangnam_style(robot):    
    stopEvent = threading.Event()

    up_and_down_torso = threading.Thread(target=torso_up_down, args=(robot, 0.3, 0.4, stopEvent))
    up_and_down_torso.start() 
    #import ipdb; ipdb.set_trace()
    motionA(robot)

    rospy.sleep(5)

    motionB(robot)
        
    stopEvent.set()
    robot.leftArm.reset()
    robot.rightArm.reset()
    robot.head.reset()

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
