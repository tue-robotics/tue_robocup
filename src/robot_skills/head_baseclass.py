#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy
import geometry_msgs.msg
import actionlib
from actionlib_msgs.msg import GoalStatus
from amigo_head_ref.msg import HeadRefAction, HeadRefGoal
import random
from sensor_msgs.msg import JointState
import robot_skills.util.msg_constructors as msgs

class HeadBaseclass(object):
    """
    An interface to a robot's head

    Example:
    #Send head to position with a 10 second time out
    >>> head.send_goal(msgs.PointStamped(0.65, 0.0, 0.9, '/amigo/base_link'))

    #Equivalent
    >>> head.set_position(msgs.PointStamped(0.65, 0.0, 0.9, '/amigo/base_link'))

    #Reset head
    >>> head.reset_position()
    """

    def __init__(self):
        pass

    def close(self):
        raise NotImplementedError("Implement interface-method in subclass")

    def send_goal(self, point_stamped, timeout=4.0, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0, pan_vel=0, tilt_vel=0):
        """
        Send a goal for the head, Executes a HeadRefAction
        Expects a position which is a geometry_msgs.msg.Point(). Should become geometry_msgs.msg.PointStamped, so we don't need the frame_id-param anymore
        And optional frame_id and timeout, frame_id defaults to /map
        By default, it does not keep tracking
        """
        raise NotImplementedError("Implement interface-method in subclass")

    def set_pan_tilt(self, pan = 0.0, tilt = 0.2, pan_vel=0, tilt_vel=0, timeout=0.0):
        """Set the head's desired pan and tilt in radians"""
        raise NotImplementedError("Implement interface-method in subclass")

    def cancel_goal(self):
        raise NotImplementedError("Implement interface-method in subclass")

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
        rospy.logwarn("Please use generic helper function")
        head_goal = geometry_msgs.msg.PointStamped()
        head_goal.header.frame_id = frame_id
        head_goal.point.x = x
        head_goal.point.y = y
        head_goal.point.z = z
        return head_goal

    def look_down(self, pan_vel=0, tilt_vel=0):
        """
        Look down at a predefined position
        """
        return self.set_pan_tilt(0.0, 0.4, pan_vel=pan_vel, tilt_vel=tilt_vel)

    def look_up(self, pan_vel=0, tilt_vel=0):
        """
        Look up at a predefined tilt angle
        """
        return self.set_pan_tilt(0.0, -0.2, pan_vel=pan_vel, tilt_vel=tilt_vel)

    def reset_position(self, timeout=2.0):
        """
        Reset head position
        """
        raise NotImplementedError("Implement interface-method in subclass")

    def set_position(self, point_stamped, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0):
        """
        Set head goal at a specified position
        Expects: a PointStamped
        """
        rospy.logerr("Obsolete, please replace by send_goal")
        return self.send_goal(point_stamped,
                       keep_tracking=keep_tracking, 
                       min_pan=min_pan, max_pan=max_pan, min_tilt=min_tilt, max_tilt=max_tilt)

    def search_movement(self, target_point, cube_size=1.0, step_time=1.5, min_pan=0.0, max_pan=0.0, min_tilt=0.0, max_tilt=0.0):
        """search_movement: the robot will look at all 8 corners of a cube around the obstacle, and will then look at the obstacle again
        @param cube_size: the size of the cube around the obstacle
        @param step_time: the max amount of time per head movement (1 head movement = look at 1 corner)"""
        tx = target_point.point.x
        ty = target_point.point.y
        tz = target_point.point.z
        search_head_goal = geometry_msgs.msg.Point()

        # first scan around the obstacle at a heigher height
        # next, scan the ground plane
        points = [  (tx - cube_size, ty - cube_size, tz + cube_size),
                    (tx + cube_size, ty - cube_size, tz + cube_size),
                    (tx + cube_size, ty + cube_size, tz + cube_size),
                    (tx - cube_size, ty + cube_size, tz + cube_size),
                    (tx - cube_size, ty + cube_size, tz),
                    (tx + cube_size, ty + cube_size, tz),
                    (tx + cube_size, ty - cube_size, tz),
                    (tx - cube_size, ty - cube_size, tz),
                    (tx, ty, tz) ]

        for index, p in enumerate(points):
            search_head_goal.x = p[0]
            search_head_goal.y = p[1]
            search_head_goal.z = p[2]

            print "Search movement step {0}: looking at ".format(index) + str(p[0]) + ", " + str(p[1]) + ", " + str(p[2])

            self.send_goal(search_head_goal, keep_tracking=False, timeout=step_time, min_pan=min_pan, max_pan=max_pan, min_tilt=min_tilt, max_tilt=max_tilt)

        return True

    def look_at_hand(self, side, keep_tracking=True):
        """
        Look at the left or right hand, expects string "left" or "right"
        Optionally, keep tracking can be disabled (keep_tracking=False)
        """
        raise NotImplementedError("Implement interface-method in subclass")

    def _measurement_listener(self, jointstate):
        self._position = jointstate.position

    @property
    def position(self):
        return self._position

    def wait(self, timeout=10):
        raise NotImplementedError("Implement interface-method in subclass")


if __name__ == "__main__":
    rospy.init_node('amigo_head_executioner', anonymous=True)
    head = HeadBaseclass()
