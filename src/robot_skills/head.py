#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy

import actionlib

from geometry_msgs.msg import Point, PointStamped
import robot_skills.util.msg_constructors as msgs
from head_ref.msg import HeadReferenceAction, HeadReferenceGoal

class Head():
    def __init__(self, robot_name):
        self.robot_name = robot_name
        self._ac_head_ref_action = actionlib.SimpleActionClient("/head_reference",  HeadReferenceAction)
        self._goal = None
        self._at_setpoint = False

    def close(self):
        self._ac_head_ref_action.cancel_all_goals()

    #Maps HeadBaseclass method names to already used names here
    def set_pan_tilt(self, pan = 0.0, tilt = 0.2, pan_vel=0, tilt_vel=0, timeout=0.0):
        #TODO: also add end_time and wait_for_setpoint to interface
        self.setPanTiltGoal(pan, tilt, pan_vel=pan_vel, tilt_vel=tilt_vel)
        return True

    #Maps HeadBaseclass method names to already used names here
    def send_goal(self, point_stamped, timeout=4.0, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0, pan_vel=0, tilt_vel=0):
        """
        Send a goal for the head, Executes a HeadRefAction
        Expects a position which is a geometry_msgs.msg.Point(). Should become geometry_msgs.msg.PointStamped, so we don't need the frame_id-param anymore
        And optional frame_id and timeout, frame_id defaults to /map
        By default, it does not keep tracking
        """
        self.setLookAtGoal(point_stamped, pan_vel=pan_vel, tilt_vel=tilt_vel)
        return True

    def cancel_goal(self):
        self.cancelGoal()

    def reset(self, timeout=0.01):
        """
        Reset head position
        """
        reset_goal = PointStamped()
        reset_goal.header.stamp = rospy.Time.now()
        reset_goal.header.frame_id = "/"+self.robot_name+"/base_link"
        reset_goal.point.x = 10
        reset_goal.point.y = 0.0
        reset_goal.point.z = 0.0

        return self.send_goal(reset_goal, keep_tracking=False, timeout=timeout, pan_vel=0.75, tilt_vel=0.75)

    def look_at_hand(self, side, keep_tracking=True):
        """
        Look at the left or right hand, expects string "left" or "right"
        Optionally, keep tracking can be disabled (keep_tracking=False)
        """
        if (side == "left"):
            return self.setLookAtGoal(msgs.PointStamped(0,0,0,frame_id="/"+self.robot_name+"/grippoint_left"))
        elif (side == "right"):
            return self.setLookAtGoal(msgs.PointStamped(0,0,0,frame_id="/"+self.robot_name+"/grippoint_right"))
        else:
            rospy.logerr("No side specified for look_at_hand. Give me 'left' or 'right'")
            return False

    def wait(self, timeout=10):
        self._ac_head_ref_action.wait_for_result(rospy.Duration(timeout))

        if self._ac_head_ref_action.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("Head target reached")
            return True
        else:
            rospy.loginfo("Reaching head target failed")
            return False

    def getGoal(self):
        return self._goal

    def atGoal(self):
        return self._at_setpoint

    def lookAtStandingPerson(self, timeout=0):
        """
        Gives a target at z = 1.75 at 1 m in front of the robot
        """
        goal = PointStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "/"+self.robot_name+"/base_link"
        goal.point.x = 1.0
        goal.point.y = 0.0
        goal.point.z = 1.75

        return self._setHeadReferenceGoal(goal_type=0,
            pan_vel=0.75,
            tilt_vel=0.75,
            end_time=0,
            frame=goal.header.frame_id,
            point=goal.point,
            wait_for_setpoint=False)

    # -- Functionality --

    def setPanTiltGoal(self, pan, tilt, end_time=0, pan_vel=0.2, tilt_vel=0.2, wait_for_setpoint=False):
        self._setHeadReferenceGoal(1, pan_vel, tilt_vel, end_time, pan=pan, tilt=tilt, wait_for_setpoint=wait_for_setpoint)

    def setLookAtGoal(self, point_stamped, end_time=0, pan_vel=0.2, tilt_vel=0.2, wait_for_setpoint=False):
        self._setHeadReferenceGoal(0, pan_vel, tilt_vel, end_time, point_stamped, wait_for_setpoint=wait_for_setpoint)

    def cancelGoal(self):
        self._ac_head_ref_action.cancel_goal()
        self._goal = None
        self._at_setpoint = False


    # ---- INTERFACING THE NODE ---

    def _setHeadReferenceGoal(self, goal_type, pan_vel, tilt_vel, end_time, point_stamped, pan=0, tilt=0, wait_for_setpoint=False):
        self._goal = HeadReferenceGoal()
        self._goal.goal_type = goal_type
        self._goal.priority = 1 # Executives get prio 1
        self._goal.pan_vel = pan_vel
        self._goal.tilt_vel = tilt_vel
        self._goal.target_point = point_stamped
        self._goal.pan = pan
        self._goal.tilt = tilt
        self._goal.end_time = end_time
        self._ac_head_ref_action.send_goal(self._goal, done_cb = self.__doneCallback, feedback_cb = self.__feedbackCallback)
        if wait_for_setpoint:
            print "HEAD2.py wait for setpoint -- THIS IS NOT YET SUPPORTED"

    def __feedbackCallback(self, feedback):
        self._at_setpoint = feedback.at_setpoint

    def __doneCallback(self, terminal_state, result):
        self._goal = None
        self._at_setpoint = False

if __name__ == "__main__":
    rospy.init_node('amigo_head_executioner', anonymous=True)
    head = Head()
