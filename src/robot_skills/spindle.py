#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
import amigo_actions.msg
import control_msgs.msg
import trajectory_msgs.msg
import threading
import util.concurrent_util
from sensor_msgs.msg import JointState

class Spindle(object):
    """Interface to Amigo's spindle or spine"""
    joint_name = 'torso_joint'
    
    def __init__(self, wait_service=True):
        ac_move_spindle = actionlib.SimpleActionClient('/spindle_server', amigo_actions.msg.AmigoSpindleCommandAction)
        ac_joint_trajectory_action = actionlib.SimpleActionClient('/joint_trajectory_action', control_msgs.msg.FollowJointTrajectoryAction)
        rospy.loginfo("waiting for spindle action server")

	self.wbc = False
        if ac_move_spindle.wait_for_server(timeout=rospy.Duration(0.5)):
            self.ac_move_spindle = ac_move_spindle
            self.wbc = False
        elif ac_joint_trajectory_action.wait_for_server(timeout=rospy.Duration(0.5)):
            self.ac_move_spindle = ac_joint_trajectory_action
            self.wbc = True
        else:
            rospy.logwarn("Cannot find spindle action server")


        ''' Keeps track of the current spindle position '''
        self.spindle_sub = rospy.Subscriber("/amigo/torso/measurements", JointState, self._receive_spindle_measurement)
        self.current_position = 0.35

        # Offset parameter to send the laser to a specific height
        # For spindle_position = 0.35 the laser_heigt = 1.02, hence offset = 1.02 - 0.35 = 0.67 (this works in simulation)
        self.laser_offset = 0.67
        self.lower_limit = 0.070
        self.upper_limit = 0.4

    def close(self):
        rospy.loginfo("Spindle cancelling all goals on close")
        self.ac_move_spindle.cancel_all_goals()
    
    def send_goal(self, spindle_pos,spindle_vel=0.0,spindle_acc=0.0,spindle_stop=0.0,timeout=0.0):
        rospy.loginfo("Send spindle goal {0}, timeout = {1}".format(spindle_pos, timeout))
        
        ''' Using actionlib interface '''
        if (spindle_pos < self.lower_limit or spindle_pos > self.upper_limit):
            rospy.logwarn("Spindle target {0} outside spindle range".format(spindle_pos))
            return False

        if not self.wbc:
            spindle_goal = amigo_actions.msg.AmigoSpindleCommandGoal()
            spindle_goal.spindle_height = spindle_pos
        elif self.wbc:
            spindle_goal = control_msgs.msg.FollowJointTrajectoryGoal()
            spindle_goal_point = trajectory_msgs.msg.JointTrajectoryPoint()
            rospy.loginfo("Goal = {0}".format(spindle_goal))
            rospy.loginfo("Goalpoint = {0}".format(spindle_goal_point))
            spindle_goal.trajectory.joint_names.append("torso_joint")
            spindle_goal_point.positions.append(spindle_pos)
            spindle_goal.trajectory.points.append(spindle_goal_point)

        self.ac_move_spindle.send_goal(spindle_goal)
        
        if timeout == 0.0:
            return True
        else:
            return self.wait(timeout)
            
    def high(self):
        return self.send_goal(self.upper_limit)
        
    def medium(self):
        return self.send_goal((self.upper_limit + self.lower_limit)/2)
        
    def low(self):
        return self.send_goal(self.lower_limit)
        
    def reset(self):
        return self.send_goal(0.35)
    
    def wait(self, timeout=10):
        self.ac_move_spindle.wait_for_result(rospy.Duration(timeout))
        if self.ac_move_spindle.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("Spindle target reached")
            return True
        else:
            rospy.loginfo("Reaching spindle target failed")
            return False

    def send_laser_goal(self, laser_height, timeout=0.0):
        return self.send_goal(laser_height - self.laser_offset,timeout=timeout)
            
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
