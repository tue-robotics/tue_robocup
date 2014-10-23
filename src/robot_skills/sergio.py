#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import robot

import rospy

import arms
import arms_dummy
import spindle # ToDo: get rid of this (we only need the interface for now)

class SergioTorso(spindle.Spindle):
    def __init__(self, wait_service=False):
        super(SergioTorso, self).__init__()

    def close(self):
        return True
    
    def send_goal(self, spindle_pos,spindle_vel=0.0,spindle_acc=0.0,spindle_stop=0.0,timeout=0.0):
        return False
            
    def high(self):
        return False
        #return self.send_goal(self.upper_limit)
        
    def medium(self):
        return False
        #return self.send_goal((self.upper_limit + self.lower_limit)/2)
        
    def low(self):
        return False
        #return self.send_goal(self.lower_limit)
        
    def reset(self):
        return False
        #return self.send_goal(0.35)
    
    def wait(self, timeout=10):
        return False
        #self.ac_move_spindle.wait_for_result(rospy.Duration(timeout))
        #if self.ac_move_spindle.get_state() == GoalStatus.SUCCEEDED:
        #    rospy.loginfo("Spindle target reached")
        #    return True
        #else:
        #    rospy.loginfo("Reaching spindle target failed")
        #    return False

    def send_laser_goal(self, laser_height, timeout=0.0):
        return False
        #return self.send_goal(laser_height - self.laser_offset,timeout=timeout)
            
    def cancel_goal(self):
        return True
        #self.ac_move_spindle.cancel_all_goals()
        #return True

    def get_position(self):
        return 0.0
        
class Sergio(robot.Robot):
    """docstring for Sergio"""
    def __init__(self, wait_services=False):
        super(Sergio, self).__init__(robot_name="sergio", wait_services=False,armClass=arms_dummy.DummyArms, torsoClass=SergioTorso)
        
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
