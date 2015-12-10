#! /usr/bin/env python
import threading

import actionlib
import control_msgs.msg
import rospy
import trajectory_msgs.msg
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState

from .util import concurrent_util


class Torso(object):
    def __init__(self, robot_name, wait_service=False):

        self.robot_name  = robot_name
        self.joint_names = rospy.get_param('/'+self.robot_name+'/skills/torso/joint_names')
        self.default_configurations = rospy.get_param('/'+self.robot_name+'/skills/torso/default_configurations')
        self.default_tolerance = rospy.get_param('/'+self.robot_name+'/skills/torso/default_tolerance')
        self.lower_limit = self.default_configurations['lower_limit']
        self.upper_limit = self.default_configurations['upper_limit']

        # Init action client
        #self.ac_move_torso = actionlib.SimpleActionClient('/'+self.robot_name+'/torso_server', control_msgs.msg.FollowJointTrajectoryAction)
        self.ac_move_torso = actionlib.SimpleActionClient('/'+self.robot_name+'/body/joint_trajectory_action', control_msgs.msg.FollowJointTrajectoryAction)

        # Init joint measurement subscriber
        self.torso_sub = rospy.Subscriber('/'+self.robot_name+'/sergio/torso/measurements', JointState, self._receive_torso_measurement)

    def close(self):
        rospy.loginfo("Torso cancelling all goals on close")
        self.ac_move_torso.cancel_all_goals()

    def send_goal(self, configuration, timeout=0.0, tolerance = []):
        if configuration in self.default_configurations:
            return self._send_goal(self.default_configurations[configuration], timeout=timeout, tolerance=tolerance)
        else:
            rospy.logwarn('Default configuration {0} does not exist'.format(configuration))
            return False

    def _send_goal(self, torso_pos, timeout=0.0, tolerance = []):
        rospy.logdebug("Send torso goal {0}, timeout = {1}".format(torso_pos, timeout))

        if len(torso_pos) != len(self.joint_names):
            rospy.logwarn('Length of desired torso pos {0} does not correspond with number of joints {1}'.format(len(torso_pos), len(self.joint_names)))
            return False

        ''' Check limits '''
        for i in range(0, len(self.joint_names)):
            if torso_pos[i] < self.lower_limit[i] or torso_pos[i] > self.upper_limit:
                rospy.logwarn("Desired position {0} for joint {1} exceeds limits [{2}, {3}]".format(torso_pos[i], self.joint_names[i], self.lower_limit[i], self.upper_limit[i]))
                return False

        torso_goal = control_msgs.msg.FollowJointTrajectoryGoal()
        torso_goal_point = trajectory_msgs.msg.JointTrajectoryPoint()
        torso_goal.trajectory.joint_names = self.joint_names
        torso_goal_point.positions = torso_pos
        torso_goal.trajectory.points.append(torso_goal_point)

        for i in range(0,len(self.joint_names)):
            goal_tolerance = control_msgs.msg.JointTolerance()
            goal_tolerance.name = self.joint_names[i]
            if len(tolerance) == len(self.joint_names):
                goal_tolerance.position = tolerance[i]
            else:
                goal_tolerance.position = self.default_tolerance[i]
            torso_goal.goal_tolerance.append(goal_tolerance)

        rospy.logdebug("Sending torso_goal: {0}".format(torso_goal))

        import time; time.sleep(0.001)  # This is necessary: the rtt_actionlib in the hardware seems
                                        # to only have a queue size of 1 and runs at 1000 hz. This
                                        # means that if two goals are send approximately at the same
                                        # time (e.g. an arm goal and a torso goal), one of the two
                                        # goals probably won't make it. This sleep makes sure the
                                        # goals will always arrive in different update hooks in the
                                        # hardware TrajectoryActionLib server.

        self.ac_move_torso.send_goal(torso_goal)

        if timeout == 0.0:
            return True
        else:
            return self.wait(timeout)

    def high(self):
        return self._send_goal(self.upper_limit)

    def medium(self):
        goal = []
        # ToDo: make nice
        for i in range(0, len(self.joint_names)):
            goal.append(self.lower_limit[i]+(self.upper_limit[i]-self.lower_limit[i])/2)
        return self._send_goal(goal)

    def low(self):
        return self._send_goal(self.lower_limit)

    def reset(self):
        return self.send_goal('reset')

    def cancel_goal(self):
        self.ac_move_torso.cancel_goal()
        #return True

    def wait_for_motion_done(self, timeout=10):
        if self.ac_move_torso.gh:
            self.ac_move_torso.wait_for_result(rospy.Duration(timeout))
            if self.ac_move_torso.get_state() == GoalStatus.SUCCEEDED:
                rospy.logdebug("Torso target reached")
                return True
            else:
                rospy.loginfo("Reaching torso target failed")
                return False

    def wait(self, timeout=10):
        import warnings
        warnings.warn("Please use wait_for_motion_done instead", Warning)
        self.wait_for_motion_done(timeout)

    _lock = threading.RLock()

    @concurrent_util.synchronized(_lock)
    def _receive_torso_measurement(self, jointstate):
        """@param jointstate JointState"""
        self.current_position = jointstate

    def get_position(self):
        return self.current_position.position
