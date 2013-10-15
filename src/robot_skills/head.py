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

class Head(object):
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
        self._ac_head_ref_action = actionlib.SimpleActionClient("head_ref_action",  HeadRefAction)
        self._search_movement_random_timer = rospy.Time.now()
        self._search_movement_random_offsets = [0,0,0]
        self._measurement_subscriber = rospy.Subscriber("/amigo/head/measurements", JointState, self._measurement_listener)

        self._position = (0, 0)

    def close(self):
        self._ac_head_ref_action.cancel_all_goals()

    def send_goal(self, point_stamped, timeout=4.0, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0):
        """
        Send a goal for the head, Executes a HeadRefAction
        Expects a position which is a geometry_msgs.msg.Point(). Should become geometry_msgs.msg.PointStamped, so we don't need the frame_id-param anymore
        And optional frame_id and timeout, frame_id defaults to /map
        By default, it does not keep tracking
        """
        # create goal:
        #rospy.loginfo(position.point)

        head_goal = HeadRefGoal()#geometry_msgs.msg.PointStamped()

        # goal_type = 0 --> meaning the action wants to "look at"
        head_goal.goal_type = 0
        head_goal.keep_tracking = keep_tracking

        if isinstance(point_stamped, geometry_msgs.msg.PointStamped):
            head_goal.target_point = point_stamped
        elif isinstance(point_stamped, geometry_msgs.msg.Point):
            head_goal.target_point.header.stamp = rospy.get_rostime()
            head_goal.target_point.header.frame_id = "/map"
            head_goal.target_point.point = point_stamped #This goes wrong when position is a raw geometry_msgs.Point, which has no header.

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
                ''' Cancel goal to stop publishing reference values '''
                self.cancel_goal()
                return False


    def send_goal_topic(self, point_stamped):
         """
         Send goal on a topic, does not use the action client and therefore does not
         wait on return
         """
         rospy.logwarn("head.send_goal_topic will become deprecated soon!")
         self.send_goal(point_stamped, timeout=0, keep_tracking=True)

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
        # head_goal = self.point_stamped(1.0, 0.0, 0.8, '/amigo/base_link')
        # return self.send_goal(head_goal.point, head_goal.header.frame_id, keep_tracking=False)

        return self.set_pan_tilt(0.0, 0.4)

    def look_up(self):
        """
        AMIGO looks up at a predefined tilt angle
        """
        return self.set_pan_tilt(0.0, -0.2)

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

        return self.send_goal(reset_head_goal, keep_tracking=False, timeout=timeout)

    def set_position(self, point_stamped, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0):
        """
        Set head goal at a specified position
        Expects: a PointStamped
        """

        return self.send_goal(point_stamped,
                       keep_tracking=keep_tracking, 
                       min_pan=min_pan, max_pan=max_pan, min_tilt=min_tilt, max_tilt=max_tilt)

    def set_position_topic(self, x, y, z, frame_id="/map"):
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

        self.send_goal(manual_head_goal,
                       timeout=0,
                       keep_tracking=True)

        return True

    def search_movement_old(self, target_point, updatetime, x_min=0, y_min=0, z_min=0, x_max=0, y_max=0, z_max=0):
        """
        Look around and search for movement
        """
        '''
        search_head_goal = geometry_msgs.msg.PointStamped()

        search_head_goal.header.frame_id = "/amigo/base_link"

        time = rospy.Time.now()
        sec = time.secs

        search_head_goal.point.x = sqrt(sin(sec)*sin(sec))
        search_head_goal.point.y = cos(sec)
        search_head_goal.point.z = 1.2

        self.send_goal(search_head_goal.point,
                       search_head_goal.header.frame_id)
        rospy.logwarn("head.search_movement has not been updated yet")
        '''
        #import ipdb; ipdb.set_trace()
        # Update only after a fixed timing interval
        if ((rospy.Time.now() - self._search_movement_random_timer)) > rospy.Duration(updatetime):
            xoffset = x_min + random.random() * (x_max - x_min)
            yoffset = y_min + random.random() * (y_max - y_min)
            zoffset = z_min + random.random() * (z_max - z_min)
            self._search_movement_random_offsets = [xoffset,yoffset,zoffset]
            self._search_movement_random_timer = rospy.Time.now()

        target_point.point.x += self._search_movement_random_offsets[0]
        target_point.point.y += self._search_movement_random_offsets[1]
        target_point.point.z += self._search_movement_random_offsets[2]

        return self.send_goal(target_point, keep_tracking=False)

    # search_movement: the robot will look at all 8 corners of a cube around the obstacle, and will then look at the obstacle again
    #    cube_size: the size of the cube around the obstacle
    #    step_time: the max amount of time per head movement (1 head movement = look at 1 corner)
    def search_movement(self, target_point, cube_size=1.0, step_time=1.5, min_pan=0.0, max_pan=0.0, min_tilt=0.0, max_tilt=0.0):
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
        if (side == "left"):
            return self.set_position(0,0,0,frame_id="/amigo/grippoint_left",keep_tracking=keep_tracking)
        elif (side == "right"):
            return self.set_position(0,0,0,frame_id="/amigo/grippoint_right",keep_tracking=keep_tracking)
        else:
            rospy.logerr("No side specified for look_at_hand")
            return False

    def _measurement_listener(self, jointstate):
        self._position = jointstate.position

    @property
    def position(self):
        return self._position



if __name__ == "__main__":
    rospy.init_node('amigo_head_executioner', anonymous=True)
    head = Head()
