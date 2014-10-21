#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import robot

import arms
import spindle # ToDo: get rid of this (we only need the interface for now)

class SergioArms(arms.Arms):
    def __init__(self):
        super(SergioArms, self).__init__()

class SergioTorso(spindle.Spindle):
    def __init__(self):
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
    def __init__(self, arg):
        super(Sergio, self).__init__(
            armClass=SergioArm,
            perceptionClass=PerceptionED)
        
if __name__ == "__main__":

    ''' Import console functions '''
    import console

    rospy.logerr("Sergio-console not yet implemented")

    #robot.spindle.send_goal(0.35)
