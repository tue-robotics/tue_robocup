#! /usr/bin/env python
import roslib; roslib.load_manifest('tue_execution_pack')
import rospy
import geometry_msgs.msg
import amigo_msgs.msg
import actionlib
from actionlib_msgs.msg import GoalStatus
import amigo_actions.msg

class Spindle(object):
    """Interface to Amigo's spindle or spine"""
    
    def __init__(self, wait_service=True):
        #TODO: Make use of the action client for the spindle
        self.spindle_setpoint = rospy.Publisher("/spindle_controller/spindle_coordinates",amigo_msgs.msg.spindle_setpoint)
        
        self.ac_move_spindle = actionlib.SimpleActionClient('/spindle_server', amigo_actions.msg.AmigoSpindleCommandAction)
        if wait_service:
            rospy.loginfo("waiting for spindle action server")
            self.ac_move_spindle.wait_for_server(timeout=rospy.Duration(2.0))

    def close(self):
        rospy.loginfo("Spindle cancelling all goals on close")
        self.ac_move_spindle.cancel_all_goals()
    
    def send_goal(self, spindle_pos,spindle_vel=0.0,spindle_acc=0.0,spindle_stop=0.0,waittime=0.0):
        rospy.loginfo("Send spindle goal")
        
        ''' Publish directly to spindle controller '''
        #self.spindle_setpoint.publish(spindle_pos,spindle_vel,spindle_acc,spindle_stop)
        #rospy.sleep(rospy.Duration(waittime))
        #return True
        
        ''' Using actionlib interface '''
        spindle_goal = amigo_actions.msg.AmigoSpindleCommandGoal()
        spindle_goal.spindle_height = spindle_pos
        
        self.ac_move_spindle.send_goal(spindle_goal)
        
        if waittime == 0.0:
            return True
        else:
            return self.wait(waittime)
            
    def high(self):
        return self.send_goal(0.4)
        
    def medium(self):
        return self.send_goal(0.2)
        
    def low(self):
        return self.send_goal(0.05)
        
    def reset(self):
        return self.send_goal(0.35)
    
    def wait(self, wait_time=10):
        self.ac_move_spindle.wait_for_result(rospy.Duration(wait_time))
        if self.ac_move_spindle.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("Spindle target reached")
            return True
        else:
            rospy.loginfo("Reaching spindle target failed")
            return False
            
    def cancel_goal(self):
        self.ac_move_spindle.cancel_all_goals()
        return True

if __name__ == "__main__":
    rospy.init_node('amigo_spindle_executioner', anonymous=True)
    spindle = Spindle()
