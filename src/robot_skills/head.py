#! /usr/bin/env python
import roslib; roslib.load_manifest('tue_execution_pack')
import rospy
import geometry_msgs.msg
import amigo_msgs.msg
import actionlib
from actionlib_msgs.msg import GoalStatus
from amigo_head_ref.msg import HeadRefAction, HeadRefGoal
from math import sqrt
from math import sin
from math import cos
from components import message_helper

class Head(object):
    """
    An interface to amigo's head

    Example:
    #Send head to position with a 10 second time out
    >>> head.send_goal(message_helper.point(0.65, 0.0, 0.9,), '/base_link')

    #Equivalent
    >>> head.set_position(0.65, 0.0, 0.9, '/base_link')

    #Reset head
    >>> head.reset_position()
    """

    def __init__(self):
        self._ac_head_ref_action = actionlib.SimpleActionClient("head_ref_action",  HeadRefAction)
        self._head_topic = rospy.Publisher("/head_target", geometry_msgs.msg.PointStamped)

    def close(self):
        self._ac_head_ref_action.cancel_all_goals()

    def send_goal(self, position, frame_id="/map", timeout=4.0, keep_tracking=False):
        """
        Send a goal for the head, Executes a HeadRefAction
        Expects a position which is a geometry_msgs.msg.Point().
        And optional frame_id and timeout, frame_id defaults to /map
        By default, it does not keep tracking
        """
        # create goal:
        #rospy.loginfo(position.point)

        head_goal = HeadRefGoal()#geometry_msgs.msg.PointStamped()

        # goal_type = 0 --> meaning the action wants to "look at"
        head_goal.goal_type = 0
        head_goal.keep_tracking = keep_tracking

        if isinstance(position, geometry_msgs.msg.PointStamped):
            head_goal.target_point = position
        elif isinstance(position, geometry_msgs.msg.Point):
            head_goal.target_point.header.stamp = rospy.get_rostime();
            head_goal.target_point.header.frame_id = frame_id
            head_goal.target_point.point = position #This goes wrong when position is a raw geometry_msgs.Point, which has no header.

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
                ''' Cancel goal to stop publishing reference values '''
                self.cancel_goal()
                return False


    def send_goal_topic(self, position, frame_id='/map'):
         """
         Send goal on a topic, does not use the action client and therefore does not
         wait on return
         """
         """
         ps = message_helper.stamp_point(position, frame_id)
         self._head_topic.publish(ps.header, ps.point)
         """
         rospy.logwarn("head.send_goal_topic will become deprecated soon!")
         self.send_goal(position, frame_id, timeout=0, keep_tracking=True)

         return True

    def cancel_goal(self):
        self._ac_head_ref_action.cancel_all_goals()

    def point(self, x, y, z):
        """
        Helper method for creating a geometry_msgs Point
        """
        goal = geometry_msgs.msg.Point(x, y, z)
        return goal

    def point_stamped(self, x, y, z, frame_id='/map'):
        """
        Helper method for creating a geometry_msgs Point
        """
        head_goal = geometry_msgs.msg.PointStamped()
        head_goal.header.frame_id = frame_id
        head_goal.point.x = x
        head_goal.point.y = y
        head_goal.point.z = z
        return head_goal

    def set_pan_tilt(self, pan = 0.0, tilt = 0.2):
        """Amigo rotate head based on pan/tilt, both in radius"""
        head_goal = HeadRefGoal()
        head_goal.goal_type = 1
        head_goal.pan = pan
        head_goal.tilt = tilt

        return self._ac_head_ref_action.send_goal(head_goal)

    def look_down(self):
        """
        Amigo looks down at a predefined position
        """
        # previous implementation
        # head_goal = self.point_stamped(1.0, 0.0, 0.8, '/base_link')
        # return self.send_goal(head_goal.point, head_goal.header.frame_id, keep_tracking=False)

        return self.set_pan_tilt(0.0, 0.2)

    def look_up(self):
        """
        AMIGO looks up at a predefined tilt angle
        """
        return self.set_pan_tilt(0.0, -0.2)

    def reset_position(self, timeout=2.0):
        """
        Reset head position
        """
        """reset_head_goal = self.point_stamped(0.214, 0.0, 1.0, '/torso')
        # Added 3 second timeout instead of standard 10
        self.send_goal(reset_head_goal.point,
                       reset_head_goal.header.frame_id, 3.0)

        return True"""
        reset_head_goal = self.point_stamped(0.214, 0.0, 1.0, '/torso')

        return self.send_goal(reset_head_goal.point, reset_head_goal.header.frame_id, keep_tracking=False, timeout=timeout)

    def set_position(self, x, y, z, frame_id='/map', keep_tracking=False):
        """
        Set head goal at a specified position
        Expects: x,y,z coordinates, and optional frame_id
        """

        manual_head_goal = geometry_msgs.msg.PointStamped()

        manual_head_goal.header.frame_id = frame_id
        manual_head_goal.point.x = x
        manual_head_goal.point.y = y
        manual_head_goal.point.z = z

        return self.send_goal(manual_head_goal.point,
                       manual_head_goal.header.frame_id,
                       keep_tracking=keep_tracking)

    def set_position_topic(self, x, y, z, frame_id="/maps"):
        """
        Set head goal on specified position, uses topic does not use
        action client. Expects x,y,z coordinates and optional frame_id
        """
        """
        self.send_goal_topic(message_helper.head_ref_action(message_helper.point(x, y, z), frame_id))
        """
        manual_head_goal = geometry_msgs.msg.PointStamped()

        manual_head_goal.header.frame_id = frame_id
        manual_head_goal.point.x = x
        manual_head_goal.point.y = y
        manual_head_goal.point.z = z

        rospy.logwarn("head.set_position_topic will become deprecated soon!")

        self.send_goal(manual_head_goal.point,
                       manual_head_goal.header.frame_id,
                       timeout=0,
                       keep_tracking=True)

        return True

    def search_movement(self):
        """
        Look around and search for movement
        """
        search_head_goal = geometry_msgs.msg.PointStamped()

        search_head_goal.header.frame_id = "/base_link"

        time = rospy.Time.now()
        sec = time.secs

        search_head_goal.point.x = sqrt(sin(sec)*sin(sec))
        search_head_goal.point.y = cos(sec)
        search_head_goal.point.z = 1.2

        self.send_goal(search_head_goal.point,
                       search_head_goal.header.frame_id)
        rospy.logwarn("head.search_movement has not been updated yet")
        
    def look_at_hand(self, side, keep_tracking=True):
        """
        Look at the left or right hand, expects string "left" or "right"
        Optionally, keep tracking can be disabled (keep_tracking=False)
        """
        if (side == "left"):
            return self.set_position(0,0,0,frame_id="/grippoint_left",keep_tracking=keep_tracking)
        elif (side == "right"):
            return self.set_position(0,0,0,frame_id="/grippoint_right",keep_tracking=keep_tracking)
        else:
            rospy.logerr("No side specified for look_at_hand")
            return False

if __name__ == "__main__":
    rospy.init_node('amigo_head_executioner', anonymous=True)
    head = Head()
