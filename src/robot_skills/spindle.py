#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
import amigo_actions.msg
import threading
import util.concurrent_util
from sensor_msgs.msg import JointState

class Spindle(object):
    """Interface to Amigo's spindle or spine"""
    joint_name = 'torso_joint'
    
    def __init__(self, wait_service=True):
        self.ac_move_spindle = actionlib.SimpleActionClient('/spindle_server', amigo_actions.msg.AmigoSpindleCommandAction)
        if wait_service:
            rospy.loginfo("waiting for spindle action server")
            self.ac_move_spindle.wait_for_server(timeout=rospy.Duration(2.0))

        ''' Keeps track of the current spindle position '''
        self.spindle_sub = rospy.Subscriber("/amigo/torso/controller/measurements", JointState, self._receive_spindle_measurement)
        self.current_position = 0.35

        # Offset parameter to send the laser to a specific height
        # For spindle_position = 0.35 the laser_heigt = 1.02, hence offset = 1.02 - 0.35 = 0.67 (this works in simulation)
        self.laser_offset = 0.67
        self.lower_limit = 0.070
        self.upper_limit = 0.4

    def close(self):
        rospy.loginfo("Spindle cancelling all goals on close")
        self.ac_move_spindle.cancel_all_goals()
    
    def send_goal(self, spindle_pos,spindle_vel=0.0,spindle_acc=0.0,spindle_stop=0.0,waittime=0.0):
        rospy.loginfo("Send spindle goal {0}, waittime = {1}".format(spindle_pos, waittime))
        
        ''' Using actionlib interface '''
        spindle_goal = amigo_actions.msg.AmigoSpindleCommandGoal()
        spindle_goal.spindle_height = spindle_pos
        if (spindle_pos < self.lower_limit or spindle_pos > self.upper_limit):
            rospy.logwarn("Spindle target {0} outside spindle range".format(spindle_pos))
            return False

        self.ac_move_spindle.send_goal(spindle_goal)
        
        if waittime == 0.0:
            return True
        else:
            return self.wait(waittime)
            
    def high(self):
        return self.send_goal(self.upper_limit)
        
    def medium(self):
        return self.send_goal((self.upper_limit + self.lower_limit)/2)
        
    def low(self):
        return self.send_goal(self.lower_limit)
        
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

    def send_laser_goal(self, laser_height, timeout=0.0):
        return self.send_goal(laser_height - self.laser_offset,waittime=timeout)
            
    def cancel_goal(self):
        self.ac_move_spindle.cancel_all_goals()
        return True

    _lock = threading.RLock()

    @util.concurrent_util.synchronized(_lock)
    def _receive_spindle_measurement(self, jointstate):
        """jointstate is of type JointState"""
        self.current_position = jointstate.position[0]

    def get_position(self):
        return self.current_position

if __name__ == "__main__":
    rospy.init_node('amigo_spindle_executioner', anonymous=True)
    spindle = Spindle()
