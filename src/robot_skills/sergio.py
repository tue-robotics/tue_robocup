#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import robot

import rospy

''' Arms '''
#import arms
import arms_dummy

''' Spindle '''
import spindle # ToDo: get rid of this (we only need the interface for now)
import control_msgs.msg
import trajectory_msgs.msg
from sensor_msgs.msg import JointState
import threading
import util.concurrent_util
import actionlib
from actionlib_msgs.msg import GoalStatus

class SergioTorso(spindle.Spindle):
    def __init__(self, wait_service=False):
        super(SergioTorso, self).__init__()
        self.joint_names = ['ankle_joint', 'knee_joint', 'hip_joint']

        ''' Temp: overwrite action client: needs to be made generic ''' 
        self.ac_move_torso = actionlib.SimpleActionClient('/sergio/torso_server', control_msgs.msg.FollowJointTrajectoryAction)
        ''' Keeps track of the current spindle position '''
        self.torso_sub = rospy.Subscriber("/sergio/torso/measurements", JointState, self._receive_torso_measurement)

        # Properties: can we get these from parameter server or urdf?
        # ToDo: what do we do with constraints???
        # ToDo: update numbers
        self.lower_limit = [0.0, 0.0, 0.85]
        self.upper_limit = [1.059, 2.158, 2.36]
        self.default_position = [0.87, 2.0, 2.36]
        self.default_tolerance = [0.1, 3.14, 0.1] #Knee joint has a very high tolerance since it depends on ankle joint

    def close(self):
        rospy.loginfo("Spindle cancelling all goals on close")
        self.ac_move_torso.cancel_all_goals()
    
    def send_goal(self, torso_pos, spindle_vel=0.0, spindle_acc=0.0, spindle_stop=0.0, timeout=0.0, tolerance = []):
        rospy.loginfo("Send torso goal {0}, timeout = {1}".format(torso_pos, timeout))

        if (spindle_vel != 0.0 or spindle_acc != 0.0):
            rospy.logwarn('Spindle_vel {0} and spindle_acc {1} are not supported'.format(spindle_vel, spindle_acc))

        if (len(torso_pos) != len(self.joint_names)):
            rospy.logwarn('Length of desired torso pos {0} does not correspond with number of joints {1}'.format(len(torso_pos), len(self.joint_names)))
            return False
        
        ''' Check limits '''
        # ToDo: make nice
        for i in range(0, len(self.joint_names)):
            if (torso_pos[i] < self.lower_limit[i] or torso_pos[i] > self.upper_limit):
                rospy.logwarn("Desired position {0} for joint {1} exceeds limits [{2}, {3}]".format(torso_pos[i], self.joint_names[i], self.lower_limit[i], self.upper_limit[i]))
                return False

        #if not self.wbc:
        #    spindle_goal = amigo_actions.msg.AmigoSpindleCommandGoal()
        #    spindle_goal.spindle_height = spindle_pos
        #elif self.wbc:
        torso_goal = control_msgs.msg.FollowJointTrajectoryGoal()
        torso_goal_point = trajectory_msgs.msg.JointTrajectoryPoint()
        torso_goal.trajectory.joint_names = self.joint_names
        torso_goal_point.positions = torso_pos
        torso_goal.trajectory.points.append(torso_goal_point)
        
        # ToDo: make nice
        for i in range(0,len(self.joint_names)):
            goal_tolerance = control_msgs.msg.JointTolerance()
            goal_tolerance.name = self.joint_names[i]
            if (len(tolerance) == len(self.joint_names)):
                goal_tolerance.position = tolerance[i]
            else:
                goal_tolerance.position = self.default_tolerance[i]
            torso_goal.goal_tolerance.append(goal_tolerance)

        self.ac_move_torso.send_goal(torso_goal)
        
        if timeout == 0.0:
            return True
        else:
            return self.wait(timeout)
            
    def high(self):
        return self.send_goal(self.upper_limit)
        
    def medium(self):
        goal = []
        # ToDo: make nice
        for i in range(0, len(self.joint_names)):
            goal.append(self.lower_limit[i]+(self.upper_limit[i]-self.lower_limit[i])/2)
        return self.send_goal(goal)
        
    def low(self):
        return self.send_goal(self.lower_limit)
        
    def reset(self):
        return self.send_goal(self.default_position)
    
    def wait(self, timeout=10):
        self.ac_move_torso.wait_for_result(rospy.Duration(timeout))
        if self.ac_move_torso.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("Torso target reached")
            return True
        else:
            rospy.loginfo("Reaching torso target failed")
            return False

    def send_laser_goal(self, laser_height, timeout=0.0):
        rospy.logwarn("Sending laser goal not implemented")
        return False
        #return self.send_goal(laser_height - self.laser_offset,timeout=timeout)
            
    def cancel_goal(self):
        self.ac_move_torso.cancel_all_goals()
        #return True

    _lock = threading.RLock()

    @util.concurrent_util.synchronized(_lock)
    def _receive_torso_measurement(self, jointstate):
        """jointstate is of type JointState"""
        self.current_position = jointstate

    def get_position(self):
        return self.current_position.position
        
class Sergio(robot.Robot):
    """docstring for Sergio"""
    def __init__(self, wait_services=False):
        super(Sergio, self).__init__(robot_name="sergio", wait_services=False,
            armClass=arms_dummy.DummyArms, 
            torsoClass=SergioTorso)
        
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
