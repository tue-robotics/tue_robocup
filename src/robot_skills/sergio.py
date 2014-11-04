#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import robot

import rospy

''' Arms '''
#import arms
import arms_dummy

''' Spindle '''
import sergio_torso
        
class Sergio(robot.Robot):
    """docstring for Sergio"""
    def __init__(self, wait_services=False):
        super(Sergio, self).__init__(robot_name="sergio", wait_services=False,
            armClass=arms_dummy.DummyArms, 
            torsoClass=sergio_torso.SergioTorso)
        
if __name__ == "__main__":
    print "Starting sergio console"
    rospy.init_node('robot_executioner', anonymous=True)

    ''' Initialize robot object '''
    robot  = Sergio('sergio')
    sergio = robot

    ''' Import console functions '''
    #from console import *
    #set_robot(robot)

    #robot.spindle.send_goal(0.35)
