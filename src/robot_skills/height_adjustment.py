#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy

import threading
import util.concurrent_util

class HeightAdjustment(object):
    """Interface for height adjustment parts"""
    joint_name = 'torso_joint'
    
    def __init__(self, wait_service=True):
        pass

    def close(self):
        """Cancel all goals and close connection to component"""
        raise NotImplementedError("Implement interface-method in subclass")
    
    def send_goal(self, pos,spindle_vel=0.0,spindle_acc=0.0,spindle_stop=0.0,timeout=0.0):
        """end to height adjustment component to some height"""
        raise NotImplementedError("Implement interface-method in subclass")
            
    def high(self):
        raise NotImplementedError("Implement interface-method in subclass")
        
    def medium(self):
        raise NotImplementedError("Implement interface-method in subclass")
        
    def low(self):
        raise NotImplementedError("Implement interface-method in subclass")
        
    def reset(self):
        """Bring the component to its default/reset position"""
        raise NotImplementedError("Implement interface-method in subclass")
    
    def wait(self, timeout=10):
        """Wait for the component to reach its goal"""
        raise NotImplementedError("Implement interface-method in subclass")

    def send_laser_goal(self, laser_height, timeout=0.0):
        """Assume that a laser's height can be adjusted by this height adjustment component and bring it to the given height"""
        raise NotImplementedError("Implement interface-method in subclass")
            
    def cancel_goal(self):
        """Cancel all pending goals"""
        raise NotImplementedError("Implement interface-method in subclass")

    _lock = threading.RLock()

    @util.concurrent_util.synchronized(_lock)
    def _receive_spindle_measurement(self, jointstate):
        """Set the current height adjustment joint state"""
        raise NotImplementedError("Implement interface-method in subclass")

    def get_position(self):
        """Current position of the height adjustment component"""
        raise NotImplementedError("Implement interface-method in subclass")
