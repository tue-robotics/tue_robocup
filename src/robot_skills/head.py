#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy

import head_baseclass

import geometry_msgs.msg
import actionlib
from actionlib_msgs.msg import GoalStatus
from amigo_head_ref.msg import HeadRefAction, HeadRefGoal
import random
from sensor_msgs.msg import JointState
import robot_skills.util.msg_constructors as msgs

class Head(head_baseclass.HeadBaseclass):
    """
    An interface to amigo's head

    Example:
    #Send head to position with a 10 second time out
    >>> head.send_goal(msgs.PointStamped(0.65, 0.0, 0.9, '/amigo/base_link'))

    #Equivalent
    >>> head.set_position(msgs.PointStamped(0.65, 0.0, 0.9, '/amigo/base_link'))

    #Reset head
    >>> head.reset_position()

    Topics and message setup:
    /amigo/head/measurements
    /amigo/head/references

    name: ['neck_pan_joint', 'neck_tilt_joint']
    """
    joint_names = ['neck_pan_joint', 'neck_tilt_joint'] 
    #This way, we have some sort of link between the stringily typed message and strongly, dynamically typed Python
    NECK_PAN_JOINT = 0
    NECK_TILT_JOINT = 1

    def __init__(self):
        super(Head, self).__init__()
        self._ac_head_ref_action = actionlib.SimpleActionClient("head_ref_action",  HeadRefAction)
        self._search_movement_random_timer = rospy.Time.now()
        self._search_movement_random_offsets = [0,0,0]
        self._measurement_subscriber = rospy.Subscriber("/amigo/neck/measurements", JointState, self._measurement_listener)

        self._position = (0, 0)

    def close(self):
        self._ac_head_ref_action.cancel_all_goals()

    def send_goal(self, point_stamped, timeout=4.0, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0, pan_vel=0, tilt_vel=0):
        """
        Send a goal for the head, Executes a HeadRefAction
        Expects a position which is a geometry_msgs.msg.Point(). Should become geometry_msgs.msg.PointStamped, so we don't need the frame_id-param anymore
        And optional frame_id and timeout, frame_id defaults to /map
        By default, it does not keep tracking
        """

        head_goal = HeadRefGoal()

        # goal_type = 0 --> meaning the action wants to "look at"
        head_goal.goal_type = 0
        head_goal.keep_tracking = keep_tracking

        if isinstance(point_stamped, geometry_msgs.msg.PointStamped):
            head_goal.target_point = point_stamped
        elif isinstance(point_stamped, geometry_msgs.msg.Point):
            head_goal.target_point.header.stamp = rospy.get_rostime()
            head_goal.target_point.header.frame_id = "/map"
            head_goal.target_point.point = point_stamped #This goes wrong when position is a raw geometry_msgs.Point, which has no header.

        head_goal.pan_vel  =pan_vel
        head_goal.tilt_vel =tilt_vel
        head_goal.min_pan  = min_pan
        head_goal.max_pan  = max_pan
        head_goal.min_tilt = min_tilt
        head_goal.max_tilt = max_tilt

        #rospy.loginfo(head_goal)

        # send goal:
        rospy.logdebug("Head_goal is ({0})".format(head_goal.target_point.point))
        self._ac_head_ref_action.send_goal(head_goal)
        rospy.logdebug("Waiting {0} secs for head goal".format(timeout))

        ''' Alternative '''
        if (timeout == 0 or keep_tracking):
            return True
        else:
            self._ac_head_ref_action.wait_for_result(rospy.Duration(timeout))
            if self._ac_head_ref_action.get_state() == GoalStatus.SUCCEEDED:
                return True
            else:
                rospy.logwarn("Cannot reach head target {0}".format(head_goal.target_point.point))
                ''' Cancel goal to stop publishing reference values '''
                self.cancel_goal()
                return False

    def cancel_goal(self):
        self._ac_head_ref_action.cancel_all_goals()

    def set_pan_tilt(self, pan = 0.0, tilt = 0.2, pan_vel=0, tilt_vel=0, timeout=0.0):
        """Amigo rotate head based on pan/tilt, both in radius"""
        head_goal = HeadRefGoal()
        head_goal.goal_type = 1
        head_goal.pan       = pan
        head_goal.tilt      = tilt
        head_goal.pan_vel   = pan_vel
        head_goal.tilt_vel  = tilt_vel

        #return self._ac_head_ref_action.send_goal(head_goal)
        
        # send goal:
        rospy.logdebug("Head_goal is ({0})".format(head_goal.target_point.point))
        self._ac_head_ref_action.send_goal(head_goal)
        rospy.logdebug("Waiting {0} secs for head goal".format(timeout))

        if timeout == 0.0:
            return True
        else:
            self._ac_head_ref_action.wait_for_result(rospy.Duration(timeout))
            if self._ac_head_ref_action.get_state() == GoalStatus.SUCCEEDED:
                return True
            else:
                rospy.logwarn("Cannot reach head target {0}".format(head_goal))
                ''' Cancel goal to stop publishing reference values '''
                self.cancel_goal()
                return False

    def reset_position(self, timeout=2.0):
        """
        Reset head position
        """
        """reset_head_goal = self.point_stamped(0.214, 0.0, 1.0, '/amigo/torso')
        # Added 3 second timeout instead of standard 10
        self.send_goal(reset_head_goal.point,
                       reset_head_goal.header.frame_id, 3.0)

        return True"""
        reset_head_goal = self.point_stamped(0.214, 0.0, 1.0, '/amigo/torso')

        return self.send_goal(reset_head_goal, keep_tracking=False, timeout=timeout, pan_vel=0.75, tilt_vel=0.75)
    
    def look_at_hand(self, side, keep_tracking=True):
        """
        Look at the left or right hand, expects string "left" or "right"
        Optionally, keep tracking can be disabled (keep_tracking=False)
        """
        if (side == "left"):
            return self.set_position(msgs.PointStamped(0,0,0,frame_id="/amigo/grippoint_left"), keep_tracking=keep_tracking)
        elif (side == "right"):
            return self.set_position(msgs.PointStamped(0,0,0,frame_id="/amigo/grippoint_right"), keep_tracking=keep_tracking)
        else:
            rospy.logerr("No side specified for look_at_hand. Give me 'left' or 'right'")
            return False

    def _measurement_listener(self, jointstate):
        self._position = jointstate.position

    @property
    def position(self):
        return self._position

    def wait(self, timeout=10):
        self._ac_head_ref_action.wait_for_result(rospy.Duration(timeout))
        
        if self._ac_head_ref_action.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("Head target reached")
            return True
        else:
            rospy.loginfo("Reaching head target failed")
            return False



if __name__ == "__main__":
    rospy.init_node('amigo_head_executioner', anonymous=True)
    head = Head()
