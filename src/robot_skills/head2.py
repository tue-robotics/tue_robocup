#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy
import actionlib

from geometry_msgs.msg import Point
from head_ref.msg import HeadReferenceAction, HeadReferenceGoal

class Head(object):
    
    def __init__(self):
        self._ac_head_ref_action = actionlib.SimpleActionClient("/head_reference",  HeadReferenceAction)
        self._goal = None
        self._at_setpoint = False

    def setPanTiltGoal(self, pan, tilt, end_time=0, pan_vel=0.2, tilt_vel=0.2):
        self._setHeadReferenceGoal(1, pan_vel, tilt_vel, end_time, pan=pan, tilt=tilt)

    def setLookAtGoal(self, frame, point=Point(), end_time=0, pan_vel=0.2, tilt_vel=0.2):
        self._setHeadReferenceGoal(0, pan_vel, tilt_vel, end_time, frame=frame, point=point)

    def cancelGoal(self):
        self._ac_head_ref_action.cancel_goal()
        self._goal = None
        self._at_setpoint = False

    def getGoal(self):
        return self._goal

    def atGoal(self):
        return self._at_setpoint

    # ----

    def _setHeadReferenceGoal(self, goal_type, pan_vel, tilt_vel, end_time, frame="", point=Point(), pan=0, tilt=0):
        self._goal = HeadReferenceGoal()
        self._goal.goal_type = goal_type
        self._goal.priority = 1 # Executives get prio 1
        self._goal.pan_vel = pan_vel
        self._goal.tilt_vel = tilt_vel
        self._goal.target_point.header.frame_id = frame
        self._goal.target_point.header.stamp = rospy.Time.now()
        self._goal.target_point.point = point
        self._goal.pan = pan
        self._goal.tilt = tilt
        self._goal.end_time = end_time
        self._ac_head_ref_action.send_goal(self._goal, done_cb = self.__doneCallback, feedback_cb = self.__feedbackCallback) 

    def __feedbackCallback(self, feedback):
        self._at_setpoint = feedback.at_setpoint

    def __doneCallback(self, terminal_state, result):
        self._goal = None
        self._at_setpoint = False
